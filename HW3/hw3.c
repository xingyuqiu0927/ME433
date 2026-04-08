#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define I2C_PORT i2c0
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define I2C_BAUDRATE 100000
#define I2C_TIMEOUT_US 1000

#define MCP23008_ADDR 0x20
#define MCP23008_IODIR 0x00
#define MCP23008_GPPU 0x06
#define MCP23008_GPIO 0x09
#define MCP23008_OLAT 0x0A

#define MCP23008_BUTTON_PIN 0
#define MCP23008_LED_PIN 7
#define MCP23008_IODIR_VALUE 0x7F
#define MCP23008_GPPU_VALUE 0x01

#define HEARTBEAT_PERIOD_MS 250
#define POLL_DELAY_MS 10

static bool set_pin(uint8_t address, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    int written = i2c_write_timeout_us(I2C_PORT, address, buffer, 2, false, I2C_TIMEOUT_US);
    return written == 2;
}

static bool read_pin(uint8_t address, uint8_t reg, uint8_t *value) {
    int written = i2c_write_timeout_us(I2C_PORT, address, &reg, 1, true, I2C_TIMEOUT_US);
    if (written != 1) {
        return false;
    }

    int read = i2c_read_timeout_us(I2C_PORT, address, value, 1, false, I2C_TIMEOUT_US);
    return read == 1;
}

static bool init_mcp23008(void) {
    return set_pin(MCP23008_ADDR, MCP23008_IODIR, MCP23008_IODIR_VALUE) &&
           set_pin(MCP23008_ADDR, MCP23008_GPPU, MCP23008_GPPU_VALUE) &&
           set_pin(MCP23008_ADDR, MCP23008_OLAT, 0x00);
}

int main() {
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

    bool mcp_ready = init_mcp23008();
    absolute_time_t next_heartbeat = make_timeout_time_ms(HEARTBEAT_PERIOD_MS);
    bool heartbeat_state = false;
    uint8_t output_latch = 0x00;

    while (true) {
#ifdef PICO_DEFAULT_LED_PIN
        if (absolute_time_diff_us(get_absolute_time(), next_heartbeat) <= 0) {
            heartbeat_state = !heartbeat_state;
            gpio_put(PICO_DEFAULT_LED_PIN, heartbeat_state);
            next_heartbeat = make_timeout_time_ms(HEARTBEAT_PERIOD_MS);
        }
#endif

        if (mcp_ready) {
            uint8_t gpio_state = 0;

            if (read_pin(MCP23008_ADDR, MCP23008_GPIO, &gpio_state)) {
                if ((gpio_state & (1u << MCP23008_BUTTON_PIN)) == 0u) {
                    output_latch |= (1u << MCP23008_LED_PIN);
                } else {
                    output_latch &= (uint8_t) ~(1u << MCP23008_LED_PIN);
                }

                mcp_ready = set_pin(MCP23008_ADDR, MCP23008_OLAT, output_latch);
            } else {
                mcp_ready = false;
            }
        } else {
            mcp_ready = init_mcp23008();
        }

        sleep_ms(POLL_DELAY_MS);
    }
}
