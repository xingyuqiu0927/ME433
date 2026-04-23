#include <string.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "ssd1306.h"

static const uint8_t ssd1306_address = 0x3C;
static uint8_t ssd1306_buffer[513];
static i2c_inst_t *ssd1306_i2c = i2c0;

void ssd1306_use_i2c(i2c_inst_t *i2c_instance) {
    ssd1306_i2c = i2c_instance;
}

void ssd1306_setup(void) {
    ssd1306_buffer[0] = 0x40;

    sleep_ms(20);
    ssd1306_command(SSD1306_DISPLAYOFF);
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);
    ssd1306_command(0x80);
    ssd1306_command(SSD1306_SETMULTIPLEX);
    ssd1306_command(0x1F);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);
    ssd1306_command(0x00);
    ssd1306_command(SSD1306_SETSTARTLINE);
    ssd1306_command(SSD1306_CHARGEPUMP);
    ssd1306_command(0x14);
    ssd1306_command(SSD1306_MEMORYMODE);
    ssd1306_command(0x00);
    ssd1306_command(SSD1306_SEGREMAP | 0x01);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);
    ssd1306_command(0x02);
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(0x8F);
    ssd1306_command(SSD1306_SETPRECHARGE);
    ssd1306_command(0xF1);
    ssd1306_command(SSD1306_SETVCOMDETECT);
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYON);
    ssd1306_clear();
    ssd1306_update();
}

void ssd1306_command(unsigned char c) {
    uint8_t buffer[2] = {0x00, c};
    i2c_write_blocking(ssd1306_i2c, ssd1306_address, buffer, 2, false);
}

void ssd1306_update(void) {
    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0);
    ssd1306_command(3);
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);
    ssd1306_command(127);

    i2c_write_blocking(ssd1306_i2c, ssd1306_address, ssd1306_buffer, sizeof(ssd1306_buffer), false);
}

void ssd1306_drawPixel(unsigned char x, unsigned char y, unsigned char color) {
    if (x >= 128 || y >= 32) {
        return;
    }

    if (color == 1) {
        ssd1306_buffer[1 + x + (y / 8) * 128] |= (1u << (y & 7u));
    } else {
        ssd1306_buffer[1 + x + (y / 8) * 128] &= (uint8_t) ~(1u << (y & 7u));
    }
}

void ssd1306_clear(void) {
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
    ssd1306_buffer[0] = 0x40;
}
