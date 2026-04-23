#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "ssd1306.h"

#define OLED_I2C_PORT i2c0
#define OLED_I2C_SDA_PIN 12
#define OLED_I2C_SCL_PIN 13
#define OLED_I2C_BAUDRATE 2000000

#define IMU_I2C_PORT i2c1
#define IMU_I2C_SDA_PIN 14
#define IMU_I2C_SCL_PIN 15
#define IMU_I2C_BAUDRATE 400000

#define OLED_WIDTH 128
#define OLED_HEIGHT 32
#define OLED_CENTER_X (OLED_WIDTH / 2)
#define OLED_CENTER_Y (OLED_HEIGHT / 2)

#define ARROW_LENGTH_PIXELS 12.0f
#define ARROW_HEAD_LENGTH_PIXELS 4.0f
#define ARROW_HEAD_HALF_WIDTH_PIXELS 3.0f
#define ARROW_MIN_VALID_G 0.10f

#define SAMPLE_PERIOD_MS 10
#define DISPLAY_PERIOD_MS 20

#define MPU6050_ADDR_LOW 0x68
#define MPU6050_ADDR_HIGH 0x69

#define ACCEL_SCALE_COUNTS 16384
#define TEMP_OFFSET_C_X100 3653
#define TEMP_SCALE_COUNTS_PER_C 340

#define DISPLAY_SWAP_AXES 0
#define DISPLAY_INVERT_X 1
#define DISPLAY_INVERT_Y 0

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_sample_t;

static uint8_t imu_address = MPU6050_ADDR_LOW;

static int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t combine_bytes(uint8_t high_byte, uint8_t low_byte) {
    return (int16_t) (((uint16_t) high_byte << 8) | low_byte);
}

static int round_to_int(float value) {
    if (value >= 0.0f) {
        return (int) (value + 0.5f);
    }
    return (int) (value - 0.5f);
}

static void draw_line(int x0, int y0, int x1, int y1) {
    int dx = x1 > x0 ? (x1 - x0) : (x0 - x1);
    int sx = x0 < x1 ? 1 : -1;
    int dy = y0 < y1 ? -(y1 - y0) : -(y0 - y1);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        ssd1306_drawPixel((unsigned char) x0, (unsigned char) y0, 1);
        if (x0 == x1 && y0 == y1) {
            break;
        }

        int err2 = 2 * err;
        if (err2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (err2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void draw_arrow(int tail_x, int tail_y, int tip_x, int tip_y) {
    float dx = (float) (tip_x - tail_x);
    float dy = (float) (tip_y - tail_y);
    float length = sqrtf(dx * dx + dy * dy);

    if (length < 1.0f) {
        ssd1306_drawPixel((unsigned char) tail_x, (unsigned char) tail_y, 1);
        return;
    }

    float unit_x = dx / length;
    float unit_y = dy / length;
    float base_x = (float) tip_x - unit_x * ARROW_HEAD_LENGTH_PIXELS;
    float base_y = (float) tip_y - unit_y * ARROW_HEAD_LENGTH_PIXELS;
    float left_x = base_x - unit_y * ARROW_HEAD_HALF_WIDTH_PIXELS;
    float left_y = base_y + unit_x * ARROW_HEAD_HALF_WIDTH_PIXELS;
    float right_x = base_x + unit_y * ARROW_HEAD_HALF_WIDTH_PIXELS;
    float right_y = base_y - unit_x * ARROW_HEAD_HALF_WIDTH_PIXELS;

    int shaft_end_x = round_to_int(base_x);
    int shaft_end_y = round_to_int(base_y);
    int left_point_x = round_to_int(left_x);
    int left_point_y = round_to_int(left_y);
    int right_point_x = round_to_int(right_x);
    int right_point_y = round_to_int(right_y);

    draw_line(tail_x, tail_y, shaft_end_x, shaft_end_y);
    draw_line(tip_x, tip_y, left_point_x, left_point_y);
    draw_line(tip_x, tip_y, right_point_x, right_point_y);
    draw_line(left_point_x, left_point_y, right_point_x, right_point_y);
}

static void imu_error_halt(const char *message) {
#ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
    printf("%s\n", message);
    while (true) {
        tight_loop_contents();
    }
}

static bool imu_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    int written = i2c_write_blocking(IMU_I2C_PORT, imu_address, buffer, 2, false);
    return written == 2;
}

static bool imu_read_registers(uint8_t reg, uint8_t *buffer, size_t length) {
    int written = i2c_write_blocking(IMU_I2C_PORT, imu_address, &reg, 1, true);
    if (written != 1) {
        return false;
    }

    int read = i2c_read_blocking(IMU_I2C_PORT, imu_address, buffer, length, false);
    return read == (int) length;
}

static bool imu_probe(uint8_t address, uint8_t *who_am_i_value) {
    uint8_t reg = WHO_AM_I;
    uint8_t value = 0;
    int written = i2c_write_blocking(IMU_I2C_PORT, address, &reg, 1, true);
    if (written != 1) {
        return false;
    }

    int read = i2c_read_blocking(IMU_I2C_PORT, address, &value, 1, false);
    if (read != 1) {
        return false;
    }

    if (value == 0x68 || value == 0x98) {
        imu_address = address;
        *who_am_i_value = value;
        return true;
    }

    return false;
}

static void imu_init(void) {
    uint8_t who_am_i_value = 0;
    bool found =
        imu_probe(MPU6050_ADDR_LOW, &who_am_i_value) || imu_probe(MPU6050_ADDR_HIGH, &who_am_i_value);

    if (!found) {
        imu_error_halt("MPU6050 not found at 0x68 or 0x69");
    }

    if (!imu_write_register(PWR_MGMT_1, 0x00)) {
        imu_error_halt("Failed to wake MPU6050");
    }
    if (!imu_write_register(ACCEL_CONFIG, 0x00)) {
        imu_error_halt("Failed to configure accelerometer");
    }
    if (!imu_write_register(GYRO_CONFIG, 0x18)) {
        imu_error_halt("Failed to configure gyroscope");
    }

    printf("MPU6050 ready at 0x%02X, WHO_AM_I=0x%02X\n", imu_address, who_am_i_value);
}

static bool imu_read_sample(imu_sample_t *sample) {
    uint8_t data[14];
    if (!imu_read_registers(ACCEL_XOUT_H, data, sizeof(data))) {
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

static void print_sample(const imu_sample_t *sample) {
    int32_t ax_mg = ((int32_t) sample->accel_x * 1000) / ACCEL_SCALE_COUNTS;
    int32_t ay_mg = ((int32_t) sample->accel_y * 1000) / ACCEL_SCALE_COUNTS;
    int32_t az_mg = ((int32_t) sample->accel_z * 1000) / ACCEL_SCALE_COUNTS;
    int32_t temp_c_x100 = TEMP_OFFSET_C_X100 + (((int32_t) sample->temp * 100) / TEMP_SCALE_COUNTS_PER_C);
    int32_t temp_frac = temp_c_x100 % 100;

    if (temp_frac < 0) {
        temp_frac = -temp_frac;
    }

    printf("ax=%6d ay=%6d az=%6d mg | gx=%6d gy=%6d gz=%6d | temp=%ld.%02ld C\n",
           (int) ax_mg,
           (int) ay_mg,
           (int) az_mg,
           (int) sample->gyro_x,
           (int) sample->gyro_y,
           (int) sample->gyro_z,
           (long) (temp_c_x100 / 100),
           (long) temp_frac);
}

static void draw_gravity_arrow(const imu_sample_t *sample) {
    static float last_dir_x = 0.0f;
    static float last_dir_y = 1.0f;

    int16_t raw_x = sample->accel_x;
    int16_t raw_y = sample->accel_y;

#if DISPLAY_SWAP_AXES
    int16_t temp = raw_x;
    raw_x = raw_y;
    raw_y = temp;
#endif
#if DISPLAY_INVERT_X
    raw_x = (int16_t) -raw_x;
#endif
#if DISPLAY_INVERT_Y
    raw_y = (int16_t) -raw_y;
#endif

    float gravity_x = (float) raw_x / (float) ACCEL_SCALE_COUNTS;
    float gravity_y = (float) raw_y / (float) ACCEL_SCALE_COUNTS;
    float projection_magnitude = sqrtf(gravity_x * gravity_x + gravity_y * gravity_y);

    if (projection_magnitude > ARROW_MIN_VALID_G) {
        last_dir_x = gravity_x / projection_magnitude;
        last_dir_y = gravity_y / projection_magnitude;
    }

    int x_offset = round_to_int(last_dir_x * ARROW_LENGTH_PIXELS);
    int y_offset = round_to_int(last_dir_y * ARROW_LENGTH_PIXELS);

    int end_x = clamp_int(OLED_CENTER_X + x_offset, 0, OLED_WIDTH - 1);
    int end_y = clamp_int(OLED_CENTER_Y + y_offset, 0, OLED_HEIGHT - 1);

    ssd1306_clear();
    draw_arrow(OLED_CENTER_X, OLED_CENTER_Y, end_x, end_y);
    ssd1306_update();
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif

    i2c_init(OLED_I2C_PORT, OLED_I2C_BAUDRATE);
    gpio_set_function(OLED_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_I2C_SDA_PIN);
    gpio_pull_up(OLED_I2C_SCL_PIN);

    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);

    ssd1306_use_i2c(OLED_I2C_PORT);
    ssd1306_setup();
    imu_init();

    imu_sample_t latest_sample = {0};
    absolute_time_t next_sample = delayed_by_ms(get_absolute_time(), SAMPLE_PERIOD_MS);
    absolute_time_t next_display = get_absolute_time();

    while (true) {
        sleep_until(next_sample);
        next_sample = delayed_by_ms(next_sample, SAMPLE_PERIOD_MS);

        if (!imu_read_sample(&latest_sample)) {
            imu_error_halt("Lost communication with MPU6050");
        }

        print_sample(&latest_sample);

        if (absolute_time_diff_us(get_absolute_time(), next_display) <= 0) {
            draw_gravity_arrow(&latest_sample);
            next_display = delayed_by_ms(next_display, DISPLAY_PERIOD_MS);
        }
    }
}
