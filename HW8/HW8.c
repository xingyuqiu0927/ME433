#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"

#define SPI_PORT spi0
#define SPI_BAUDRATE 1000000

// Shared SPI0 pins. Wire the MCP4912 DAC SDI and 23K256 SI to GP19,
// both SCK pins to GP18, and the 23K256 SO pin to GP16.
#define SPI_RX_PIN 16
#define DAC_CS_PIN 17
#define SPI_SCK_PIN 18
#define SPI_TX_PIN 19
#define RAM_CS_PIN 20

#define SINE_TABLE_SIZE 1000
#define SAMPLE_DELAY_MS 1

#define RAM_WRITE 0x02
#define RAM_READ 0x03
#define RAM_WRITE_STATUS 0x01
#define RAM_MODE_SEQUENTIAL 0x40

#define MCP4912_CHANNEL_A 0u
#define MCP4912_BUFFERED 0x4000u
#define MCP4912_GAIN_1X 0x2000u
#define MCP4912_ACTIVE 0x1000u

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

static void spi_bus_init(void) {
    spi_init(SPI_PORT, SPI_BAUDRATE);
    gpio_set_function(SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);

    gpio_init(DAC_CS_PIN);
    gpio_set_dir(DAC_CS_PIN, GPIO_OUT);
    gpio_put(DAC_CS_PIN, 1);

    gpio_init(RAM_CS_PIN);
    gpio_set_dir(RAM_CS_PIN, GPIO_OUT);
    gpio_put(RAM_CS_PIN, 1);
}

static void spi_ram_init(void) {
    uint8_t data[] = {RAM_WRITE_STATUS, RAM_MODE_SEQUENTIAL};

    cs_select(RAM_CS_PIN);
    spi_write_blocking(SPI_PORT, data, sizeof(data));
    cs_deselect(RAM_CS_PIN);
}

static void spi_ram_write(uint16_t address, const uint8_t *data, size_t length) {
    uint8_t header[] = {
        RAM_WRITE,
        (uint8_t)(address >> 8),
        (uint8_t)(address & 0xff)
    };

    cs_select(RAM_CS_PIN);
    spi_write_blocking(SPI_PORT, header, sizeof(header));
    spi_write_blocking(SPI_PORT, data, length);
    cs_deselect(RAM_CS_PIN);
}

static void spi_ram_read(uint16_t address, uint8_t *data, size_t length) {
    uint8_t header[] = {
        RAM_READ,
        (uint8_t)(address >> 8),
        (uint8_t)(address & 0xff)
    };

    cs_select(RAM_CS_PIN);
    spi_write_blocking(SPI_PORT, header, sizeof(header));
    spi_read_blocking(SPI_PORT, 0, data, length);
    cs_deselect(RAM_CS_PIN);
}

static uint16_t dac_command(uint channel, uint16_t value) {
    value &= 0x03ff;

    return (channel ? 0x8000u : 0x0000u) |
           MCP4912_BUFFERED |
           MCP4912_GAIN_1X |
           MCP4912_ACTIVE |
           (uint16_t)(value << 2);
}

static void dac_write_bytes(const uint8_t data[2]) {
    cs_select(DAC_CS_PIN);
    spi_write_blocking(SPI_PORT, data, 2);
    cs_deselect(DAC_CS_PIN);
}

static void load_sine_wave_to_ram(void) {
    for (uint16_t i = 0; i < SINE_TABLE_SIZE; i++) {
        float radians = 2.0f * (float)M_PI * (float)i / (float)SINE_TABLE_SIZE;
        float voltage = 1.65f + (1.65f * sinf(radians));
        uint16_t dac_value = (uint16_t)((voltage * 1023.0f / 3.3f) + 0.5f);
        uint16_t command = dac_command(MCP4912_CHANNEL_A, dac_value);
        uint8_t bytes[] = {
            (uint8_t)(command >> 8),
            (uint8_t)(command & 0xff)
        };

        spi_ram_write((uint16_t)(2u * i), bytes, sizeof(bytes));
    }
}

int main()
{
    stdio_init_all();
    spi_bus_init();
    spi_ram_init();
    load_sine_wave_to_ram();

    uint16_t sample = 0;

    while (true) {
        uint8_t dac_bytes[2];
        spi_ram_read((uint16_t)(2u * sample), dac_bytes, sizeof(dac_bytes));
        dac_write_bytes(dac_bytes);

        sample++;
        if (sample >= SINE_TABLE_SIZE) {
            sample = 0;
        }

        sleep_ms(SAMPLE_DELAY_MS);
    }
}
