#include "mpu6050.h"

#define MPU6050_ADDR_LOW 0x68
#define MPU6050_ADDR_HIGH 0x69

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

static int16_t combine_bytes(uint8_t high_byte, uint8_t low_byte) {
    return (int16_t) (((uint16_t) high_byte << 8) | low_byte);
}

static bool mpu6050_write_register(mpu6050_t *imu, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    int written = i2c_write_blocking(imu->i2c, imu->address, buffer, 2, false);
    return written == 2;
}

static bool mpu6050_read_registers(mpu6050_t *imu, uint8_t reg, uint8_t *buffer, size_t length) {
    int written = i2c_write_blocking(imu->i2c, imu->address, &reg, 1, true);
    if (written != 1) {
        return false;
    }

    int read = i2c_read_blocking(imu->i2c, imu->address, buffer, length, false);
    return read == (int) length;
}

static bool mpu6050_probe(i2c_inst_t *i2c, uint8_t address) {
    uint8_t reg = WHO_AM_I;
    uint8_t who_am_i = 0;

    int written = i2c_write_blocking(i2c, address, &reg, 1, true);
    if (written != 1) {
        return false;
    }

    int read = i2c_read_blocking(i2c, address, &who_am_i, 1, false);
    if (read != 1) {
        return false;
    }

    return who_am_i == 0x68 || who_am_i == 0x98;
}

bool mpu6050_init_auto(mpu6050_t *imu, i2c_inst_t *i2c) {
    imu->i2c = i2c;

    if (mpu6050_probe(i2c, MPU6050_ADDR_LOW)) {
        imu->address = MPU6050_ADDR_LOW;
    } else if (mpu6050_probe(i2c, MPU6050_ADDR_HIGH)) {
        imu->address = MPU6050_ADDR_HIGH;
    } else {
        return false;
    }

    if (!mpu6050_write_register(imu, PWR_MGMT_1, 0x00)) {
        return false;
    }
    if (!mpu6050_write_register(imu, ACCEL_CONFIG, 0x00)) {
        return false;
    }
    if (!mpu6050_write_register(imu, GYRO_CONFIG, 0x18)) {
        return false;
    }

    return true;
}

bool mpu6050_read_sample(mpu6050_t *imu, mpu6050_sample_t *sample) {
    uint8_t data[14];

    if (!mpu6050_read_registers(imu, ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    sample->accel_x = combine_bytes(data[0], data[1]);
    sample->accel_y = combine_bytes(data[2], data[3]);
    sample->accel_z = combine_bytes(data[4], data[5]);
    sample->temp = combine_bytes(data[6], data[7]);
    sample->gyro_x = combine_bytes(data[8], data[9]);
    sample->gyro_y = combine_bytes(data[10], data[11]);
    sample->gyro_z = combine_bytes(data[12], data[13]);

    return true;
}
