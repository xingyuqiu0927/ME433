#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_sample_t;

typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
} mpu6050_t;

bool mpu6050_init_auto(mpu6050_t *imu, i2c_inst_t *i2c);
bool mpu6050_read_sample(mpu6050_t *imu, mpu6050_sample_t *sample);

#endif
