#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_REG 0x0C

#define ENCODER_I2C i2c1
#define ENCODER_SDA_PIN 26
#define ENCODER_SCL_PIN 27

static int read_as5600_raw_angle(void)
{
    uint8_t reg = AS5600_RAW_ANGLE_REG;
    uint8_t data[2];

    int written = i2c_write_blocking(ENCODER_I2C, AS5600_ADDR, &reg, 1, true);
    if (written != 1) {
        return -1;
    }

    int read = i2c_read_blocking(ENCODER_I2C, AS5600_ADDR, data, 2, false);
    if (read != 2) {
        return -1;
    }

    return ((data[0] & 0x0F) << 8) | data[1];
}

int main()
{
    stdio_init_all();

    i2c_init(ENCODER_I2C, 400 * 1000);
    gpio_set_function(ENCODER_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(ENCODER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ENCODER_SDA_PIN);
    gpio_pull_up(ENCODER_SCL_PIN);

    sleep_ms(2000);

    while (true) {
        int raw_angle = read_as5600_raw_angle();

        if (raw_angle < 0) {
            printf("encoder_error\n");
        } else {
            float angle_degrees = raw_angle * 360.0f / 4096.0f;
            printf("%d,%.2f\n", raw_angle, angle_degrees);
        }

        sleep_ms(20);
    }
}
