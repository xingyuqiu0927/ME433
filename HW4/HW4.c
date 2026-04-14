#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "font.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "ssd1306.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define I2C_BAUDRATE 2000000

#define ADC_INPUT 0
#define ADC_PIN 26
#define ADC_REF_MV 3300
#define ADC_MAX_COUNTS 4095

#define OLED_WIDTH 128
#define OLED_HEIGHT 32
#define CHAR_WIDTH 5
#define CHAR_HEIGHT 8
#define CHAR_SPACING 1

#define HEARTBEAT_HALF_PERIOD_MS 500
#define HEARTBEAT_PIXEL_X 64
#define HEARTBEAT_PIXEL_Y 18
#define HEARTBEAT_PIXEL_SIZE 4

static void drawChar(int x, int y, char letter) {
    unsigned char glyph = '?';

    if ((unsigned char) letter >= 0x20 && (unsigned char) letter <= 0x7F) {
        glyph = (unsigned char) letter;
    }

    for (int col = 0; col < CHAR_WIDTH; col++) {
        unsigned char column_bits = (unsigned char) ASCII[glyph - 0x20][col];

        for (int row = 0; row < CHAR_HEIGHT; row++) {
            unsigned char pixel_on = (column_bits >> row) & 0x01u;
            ssd1306_drawPixel((unsigned char) (x + col), (unsigned char) (y + row), pixel_on);
        }
    }
}

static void drawString(int x, int y, const char *message) {
    int cursor_x = x;
    int cursor_y = y;

    while (*message != '\0') {
        if (*message == '\n') {
            cursor_x = x;
            cursor_y += CHAR_HEIGHT;
        } else {
            drawChar(cursor_x, cursor_y, *message);
            cursor_x += CHAR_WIDTH + CHAR_SPACING;
        }

        message++;
    }
}

static void drawHeartbeatPixel(bool heartbeat_on) {
    for (int x = 0; x < HEARTBEAT_PIXEL_SIZE; x++) {
        for (int y = 0; y < HEARTBEAT_PIXEL_SIZE; y++) {
            ssd1306_drawPixel((unsigned char) (HEARTBEAT_PIXEL_X + x),
                              (unsigned char) (HEARTBEAT_PIXEL_Y + y),
                              heartbeat_on ? 1 : 0);
        }
    }
}

int main(void) {
    stdio_init_all();

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(ADC_INPUT);

    ssd1306_setup();

    bool heartbeat_on = false;
    absolute_time_t next_heartbeat = make_timeout_time_ms(HEARTBEAT_HALF_PERIOD_MS);
    uint32_t fps_tenths = 0;
    uint64_t last_frame_end_us = to_us_since_boot(get_absolute_time());

    while (true) {
        if (absolute_time_diff_us(get_absolute_time(), next_heartbeat) <= 0) {
            heartbeat_on = !heartbeat_on;
            next_heartbeat = make_timeout_time_ms(HEARTBEAT_HALF_PERIOD_MS);
        }

#ifdef PICO_DEFAULT_LED_PIN
        gpio_put(PICO_DEFAULT_LED_PIN, heartbeat_on);
#endif

        uint16_t adc_raw = adc_read();
        uint32_t millivolts = ((uint32_t) adc_raw * ADC_REF_MV) / ADC_MAX_COUNTS;

        char voltage_text[32];
        char footer_text[32];
        snprintf(voltage_text,
                 sizeof(voltage_text),
                 "ADC0: %lu.%03lu V",
                 (unsigned long) (millivolts / 1000u),
                 (unsigned long) (millivolts % 1000u));
        snprintf(footer_text,
                 sizeof(footer_text),
                 "FPS:%lu.%01lu",
                 (unsigned long) (fps_tenths / 10u),
                 (unsigned long) (fps_tenths % 10u));

        ssd1306_clear();
        drawString(0, 0, "I2C: GP12/GP13");
        drawString(0, 8, voltage_text);
        drawString(0, 16, "HEARTBEAT:");
        drawHeartbeatPixel(heartbeat_on);
        drawString(0, 24, footer_text);
        ssd1306_update();

        uint64_t frame_end_us = to_us_since_boot(get_absolute_time());
        uint64_t frame_time_us = frame_end_us - last_frame_end_us;
        last_frame_end_us = frame_end_us;

        if (frame_time_us > 0) {
            fps_tenths = (uint32_t) ((10000000ull + (frame_time_us / 2ull)) / frame_time_us);
        }
    }
}
