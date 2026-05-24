#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "mpu6050.h"

#define IMU_I2C_PORT i2c1
#define IMU_I2C_SDA_PIN 14
#define IMU_I2C_SCL_PIN 15
#define IMU_I2C_BAUDRATE 400000

#define STATUS_LED_PIN 0
#define STATUS_LED_ON_LEVEL 1

#define SAMPLE_RATE_HZ 30u
#define SAMPLE_PERIOD_US (1000000u / SAMPLE_RATE_HZ)
#define ACCEL_FILTER_SHIFT 2

static void set_status_led(bool on) {
    gpio_put(STATUS_LED_PIN, on ? STATUS_LED_ON_LEVEL : !STATUS_LED_ON_LEVEL);
}

static void init_status_led(void) {
    gpio_init(STATUS_LED_PIN);
    gpio_set_dir(STATUS_LED_PIN, GPIO_OUT);
    set_status_led(false);
}

static void init_imu_i2c(void) {
    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);
}

static void blink_error_forever(void) {
    while (true) {
        set_status_led(true);
        sleep_ms(100);
        set_status_led(false);
        sleep_ms(100);
    }
}

static void update_filter(
    const mpu6050_sample_t *sample,
    bool *filter_ready,
    int32_t *accel_x,
    int32_t *accel_y,
    int32_t *accel_z
) {
    if (!*filter_ready) {
        *accel_x = sample->accel_x;
        *accel_y = sample->accel_y;
        *accel_z = sample->accel_z;
        *filter_ready = true;
        return;
    }

    *accel_x += ((int32_t) sample->accel_x - *accel_x) >> ACCEL_FILTER_SHIFT;
    *accel_y += ((int32_t) sample->accel_y - *accel_y) >> ACCEL_FILTER_SHIFT;
    *accel_z += ((int32_t) sample->accel_z - *accel_z) >> ACCEL_FILTER_SHIFT;
}

int main(void) {
    stdio_init_all();
    init_status_led();
    init_imu_i2c();

    sleep_ms(2000);

    mpu6050_t imu = {0};
    if (!mpu6050_init_auto(&imu, IMU_I2C_PORT)) {
        printf("error,imu_not_found\r\n");
        blink_error_forever();
    }

    printf("# HW10 IMU stream: ax,ay,az,gx,gy,gz\r\n");
    printf("# GPIO: MPU6050 SDA=GP14 SCL=GP15 on i2c1\r\n");

    bool filter_ready = false;
    int32_t accel_x = 0;
    int32_t accel_y = 0;
    int32_t accel_z = 0;
    uint sample_count = 0;
    absolute_time_t next_sample = get_absolute_time();

    while (true) {
        mpu6050_sample_t sample = {0};

        if (!mpu6050_read_sample(&imu, &sample)) {
            printf("error,read_failed\r\n");
            blink_error_forever();
        }

        update_filter(&sample, &filter_ready, &accel_x, &accel_y, &accel_z);

        printf("%" PRId32 ",%" PRId32 ",%" PRId32 ",%d,%d,%d\r\n",
               accel_x,
               accel_y,
               accel_z,
               sample.gyro_x,
               sample.gyro_y,
               sample.gyro_z);

        sample_count++;
        if (sample_count >= SAMPLE_RATE_HZ / 2u) {
            sample_count = 0;
            gpio_put(STATUS_LED_PIN, !gpio_get(STATUS_LED_PIN));
        }

        next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
        sleep_until(next_sample);
    }
}
