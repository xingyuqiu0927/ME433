#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define DAC_SPI_PORT spi0
#define DAC_SPI_BAUDRATE 12000

// HW5/HW6 used GP0 and GP12-GP16, so this keeps the DAC on free pins.
#define DAC_CS_PIN 17
#define DAC_SCK_PIN 18
#define DAC_SDI_PIN 19

#define DAC_UPDATE_RATE_HZ 200
#define SINE_TABLE_SIZE 100
#define TRIANGLE_PERIOD_US 1000000u

#define MCP4912_CHANNEL_A 0u
#define MCP4912_CHANNEL_B 1u

static const uint16_t sine_table[SINE_TABLE_SIZE] = {
     512,  544,  576,  607,  639,  670,  700,  729,  758,  786,
     812,  838,  862,  884,  906,  925,  943,  960,  974,  987,
     998, 1007, 1014, 1019, 1022, 1023, 1022, 1019, 1014, 1007,
     998,  987,  974,  960,  943,  925,  906,  884,  862,  838,
     812,  786,  758,  729,  700,  670,  639,  607,  576,  544,
     512,  479,  447,  416,  384,  353,  323,  294,  265,  237,
     211,  185,  161,  139,  117,   98,   80,   63,   49,   36,
      25,   16,    9,    4,    1,    0,    1,    4,    9,   16,
      25,   36,   49,   63,   80,   98,  117,  139,  161,  185,
     211,  237,  265,  294,  323,  353,  384,  416,  447,  479
};

static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

static void mcp4912_write(uint channel, uint16_t value) {
    value &= 0x03ff;

    uint16_t command = (channel ? 0x8000 : 0x0000) |
                       0x3000 |
                       (value << 2);
    uint8_t data[] = {
        (uint8_t)(command >> 8),
        (uint8_t)(command & 0xff)
    };

    cs_select(DAC_CS_PIN);
    spi_write_blocking(DAC_SPI_PORT, data, 2);
    cs_deselect(DAC_CS_PIN);
}

static uint16_t triangle_value(uint64_t time_us) {
    uint32_t phase_us = time_us % TRIANGLE_PERIOD_US;

    if (phase_us < TRIANGLE_PERIOD_US / 2u) {
        return (uint16_t)((phase_us * 1023u) / (TRIANGLE_PERIOD_US / 2u));
    }

    return (uint16_t)(((TRIANGLE_PERIOD_US - phase_us) * 1023u) /
                      (TRIANGLE_PERIOD_US / 2u));
}

static void dac_init(void) {
    spi_init(DAC_SPI_PORT, DAC_SPI_BAUDRATE);
    gpio_set_function(DAC_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DAC_SDI_PIN, GPIO_FUNC_SPI);

    gpio_init(DAC_CS_PIN);
    gpio_set_dir(DAC_CS_PIN, GPIO_OUT);
    gpio_put(DAC_CS_PIN, 1);
}

int main() {
    stdio_init_all();
    dac_init();

    const uint32_t update_interval_us = 1000000u / DAC_UPDATE_RATE_HZ;
    absolute_time_t next_update = get_absolute_time();

    while (true) {
        uint64_t time_us = to_us_since_boot(get_absolute_time());
        uint sine_index = (time_us * 2u * SINE_TABLE_SIZE / 1000000u) %
                          SINE_TABLE_SIZE;

        mcp4912_write(MCP4912_CHANNEL_A, sine_table[sine_index]);
        mcp4912_write(MCP4912_CHANNEL_B, triangle_value(time_us));

        next_update = delayed_by_us(next_update, update_interval_us);
        sleep_until(next_update);
    }
}
