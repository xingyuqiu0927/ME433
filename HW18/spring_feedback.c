#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#define X_I2C_PORT i2c0
#define X_I2C_SDA 4
#define X_I2C_SCL 5

#define Y_I2C_PORT i2c1
#define Y_I2C_SDA 6
#define Y_I2C_SCL 7

#define X_MOTOR_PWM_A 10
#define X_MOTOR_PWM_B 11
#define Y_MOTOR_PWM_A 12
#define Y_MOTOR_PWM_B 13

#define AS5600_ADDR 0x36
#define AS5600_ANGLE_REG 0x0E
#define AS5600_COUNTS_PER_REV 4096

#define X_ENCODER_SIGN -1
#define Y_ENCODER_SIGN 1
#define X_MOTOR_INVERT true
#define Y_MOTOR_INVERT false

#define PWM_WRAP 1000
#define PWM_CLKDIV 1.0f
#define CONTROL_DT_MS 10
#define PRINT_DT_MS 250

#define CENTER_KP_PWM_PER_COUNT 0.85f
#define CENTER_KD_PWM_PER_COUNT_PER_S 0.10f
#define VELOCITY_FILTER_ALPHA 0.30f
#define CENTER_DEADBAND_COUNTS 4
#define MOTOR_MIN_PWM 95
#define MOTOR_MIN_RAMP_COUNTS 90.0f
#define MOTOR_MAX_PWM 650
#define ENCODER_STALE_LIMIT_MS 120

typedef struct {
    const char *name;
    i2c_inst_t *i2c;
    uint sda_pin;
    uint scl_pin;
    int32_t sign;
    uint16_t zero_raw;
    uint16_t raw;
    int32_t position;
    bool have_position;
    uint32_t misses;
    absolute_time_t last_ok_time;
} encoder_axis_t;

typedef struct {
    uint pwm_a_pin;
    uint pwm_b_pin;
    bool invert;
    int32_t last_position;
    bool have_last_position;
    float velocity_counts_s;
} motor_axis_t;

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool as5600_read_raw(encoder_axis_t *axis, uint16_t *raw)
{
    uint8_t reg = AS5600_ANGLE_REG;
    uint8_t data[2];

    if (i2c_write_blocking(axis->i2c, AS5600_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    if (i2c_read_blocking(axis->i2c, AS5600_ADDR, data, 2, false) != 2) {
        return false;
    }

    *raw = ((data[0] << 8) | data[1]) & 0x0FFF;
    return true;
}

static int32_t shortest_delta_counts(uint16_t raw, uint16_t zero_raw)
{
    int32_t delta = (int32_t) raw - (int32_t) zero_raw;

    if (delta > AS5600_COUNTS_PER_REV / 2) {
        delta -= AS5600_COUNTS_PER_REV;
    } else if (delta < -AS5600_COUNTS_PER_REV / 2) {
        delta += AS5600_COUNTS_PER_REV;
    }

    return delta;
}

static uint16_t wrap_raw_count(int32_t count)
{
    count %= AS5600_COUNTS_PER_REV;
    if (count < 0) {
        count += AS5600_COUNTS_PER_REV;
    }
    return (uint16_t) count;
}

static bool update_encoder_axis(encoder_axis_t *axis)
{
    uint16_t raw = 0;

    if (!as5600_read_raw(axis, &raw)) {
        if (axis->misses < UINT32_MAX) {
            axis->misses++;
        }
        return axis->have_position;
    }

    axis->raw = raw;
    axis->position = axis->sign * shortest_delta_counts(raw, axis->zero_raw);
    axis->have_position = true;
    axis->misses = 0;
    axis->last_ok_time = get_absolute_time();
    return true;
}

static bool encoder_fresh(const encoder_axis_t *axis, absolute_time_t now)
{
    return axis->have_position &&
           absolute_time_diff_us(axis->last_ok_time, now) <= ENCODER_STALE_LIMIT_MS * 1000;
}

static void init_encoder_bus(encoder_axis_t *axis)
{
    i2c_init(axis->i2c, 400 * 1000);
    gpio_set_function(axis->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(axis->scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(axis->sda_pin);
    gpio_pull_up(axis->scl_pin);
}

static void init_motor_axis(motor_axis_t *motor)
{
    gpio_set_function(motor->pwm_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(motor->pwm_b_pin, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(motor->pwm_a_pin);
    uint slice_b = pwm_gpio_to_slice_num(motor->pwm_b_pin);

    pwm_set_wrap(slice_a, PWM_WRAP);
    pwm_set_wrap(slice_b, PWM_WRAP);
    pwm_set_clkdiv(slice_a, PWM_CLKDIV);
    pwm_set_clkdiv(slice_b, PWM_CLKDIV);
    pwm_set_gpio_level(motor->pwm_a_pin, 0);
    pwm_set_gpio_level(motor->pwm_b_pin, 0);
    pwm_set_enabled(slice_a, true);
    pwm_set_enabled(slice_b, true);
}

static void drive_motor_axis(const motor_axis_t *motor, int32_t command)
{
    command = clamp_i32(command, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);

    if (motor->invert) {
        command = -command;
    }

    if (command > 0) {
        pwm_set_gpio_level(motor->pwm_a_pin, (uint16_t) command);
        pwm_set_gpio_level(motor->pwm_b_pin, 0);
    } else if (command < 0) {
        pwm_set_gpio_level(motor->pwm_a_pin, 0);
        pwm_set_gpio_level(motor->pwm_b_pin, (uint16_t) -command);
    } else {
        pwm_set_gpio_level(motor->pwm_a_pin, 0);
        pwm_set_gpio_level(motor->pwm_b_pin, 0);
    }
}

static void stop_motors(const motor_axis_t *x_motor, const motor_axis_t *y_motor)
{
    drive_motor_axis(x_motor, 0);
    drive_motor_axis(y_motor, 0);
}

static bool calibrate_zero(encoder_axis_t *axis)
{
    uint16_t base_raw = 0;
    int32_t delta_sum = 0;
    int32_t samples = 0;

    if (!as5600_read_raw(axis, &base_raw)) {
        return false;
    }

    for (int i = 0; i < 32; i++) {
        uint16_t raw = 0;
        if (as5600_read_raw(axis, &raw)) {
            delta_sum += shortest_delta_counts(raw, base_raw);
            samples++;
        }
        sleep_ms(5);
    }

    if (samples == 0) {
        return false;
    }

    axis->zero_raw = wrap_raw_count((int32_t) base_raw + (delta_sum / samples));
    axis->raw = axis->zero_raw;
    axis->position = 0;
    axis->have_position = true;
    axis->last_ok_time = get_absolute_time();
    return true;
}

static int32_t center_spring_command(motor_axis_t *motor, int32_t position)
{
    float raw_velocity = 0.0f;

    if (motor->have_last_position) {
        raw_velocity = (float) (position - motor->last_position) * (1000.0f / (float) CONTROL_DT_MS);
        motor->velocity_counts_s =
            (VELOCITY_FILTER_ALPHA * raw_velocity) +
            ((1.0f - VELOCITY_FILTER_ALPHA) * motor->velocity_counts_s);
    }

    motor->last_position = position;
    motor->have_last_position = true;

    if (abs_i32(position) <= CENTER_DEADBAND_COUNTS) {
        return 0;
    }

    float force = (CENTER_KP_PWM_PER_COUNT * (float) position) +
                  (CENTER_KD_PWM_PER_COUNT_PER_S * motor->velocity_counts_s);
    int32_t command = (int32_t) -force;
    int32_t magnitude = abs_i32(command);
    float ramp = clamp_f32(((float) abs_i32(position) - (float) CENTER_DEADBAND_COUNTS) /
                               MOTOR_MIN_RAMP_COUNTS,
                           0.0f,
                           1.0f);
    int32_t minimum = (int32_t) ((float) MOTOR_MIN_PWM * ramp);

    if (magnitude > 0 && magnitude < minimum) {
        magnitude = minimum;
    }

    command = command < 0 ? -magnitude : magnitude;
    return clamp_i32(command, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1500);

    encoder_axis_t x_axis = {
        .name = "X",
        .i2c = X_I2C_PORT,
        .sda_pin = X_I2C_SDA,
        .scl_pin = X_I2C_SCL,
        .sign = X_ENCODER_SIGN,
    };
    encoder_axis_t y_axis = {
        .name = "Y",
        .i2c = Y_I2C_PORT,
        .sda_pin = Y_I2C_SDA,
        .scl_pin = Y_I2C_SCL,
        .sign = Y_ENCODER_SIGN,
    };
    motor_axis_t x_motor = {
        .pwm_a_pin = X_MOTOR_PWM_A,
        .pwm_b_pin = X_MOTOR_PWM_B,
        .invert = X_MOTOR_INVERT,
    };
    motor_axis_t y_motor = {
        .pwm_a_pin = Y_MOTOR_PWM_A,
        .pwm_b_pin = Y_MOTOR_PWM_B,
        .invert = Y_MOTOR_INVERT,
    };

    init_encoder_bus(&x_axis);
    init_encoder_bus(&y_axis);
    init_motor_axis(&x_motor);
    init_motor_axis(&y_motor);
    stop_motors(&x_motor, &y_motor);

    printf("\nHW18 simple center spring\n");
    printf("Hold joystick centered during boot. Calibrating zero...\n");

    while (!calibrate_zero(&x_axis) || !calibrate_zero(&y_axis)) {
        stop_motors(&x_motor, &y_motor);
        printf("encoder not ready; retrying zero calibration\n");
        sleep_ms(500);
    }

    printf("zero set: X raw=%u Y raw=%u\n", x_axis.zero_raw, y_axis.zero_raw);
    printf("center spring auto-started\n\n");

    int32_t x_command = 0;
    int32_t y_command = 0;
    absolute_time_t last_control_time = get_absolute_time();
    absolute_time_t last_print_time = get_absolute_time();

    while (true) {
        update_encoder_axis(&x_axis);
        update_encoder_axis(&y_axis);
        absolute_time_t now = get_absolute_time();

        if (absolute_time_diff_us(last_control_time, now) >= CONTROL_DT_MS * 1000) {
            last_control_time = now;

            if (encoder_fresh(&x_axis, now) && encoder_fresh(&y_axis, now)) {
                x_command = center_spring_command(&x_motor, x_axis.position);
                y_command = center_spring_command(&y_motor, y_axis.position);
                drive_motor_axis(&x_motor, x_command);
                drive_motor_axis(&y_motor, y_command);
            } else {
                x_command = 0;
                y_command = 0;
                stop_motors(&x_motor, &y_motor);
            }
        }

        if (absolute_time_diff_us(last_print_time, now) >= PRINT_DT_MS * 1000) {
            last_print_time = now;
            printf("center-spring raw X=%4u Y=%4u pos X=%5ld Y=%5ld vel X=%6ld Y=%6ld cmd X=%4ld Y=%4ld miss X=%lu Y=%lu\n",
                   x_axis.raw,
                   y_axis.raw,
                   (long) x_axis.position,
                   (long) y_axis.position,
                   (long) x_motor.velocity_counts_s,
                   (long) y_motor.velocity_counts_s,
                   (long) x_command,
                   (long) y_command,
                   (unsigned long) x_axis.misses,
                   (unsigned long) y_axis.misses);
            fflush(stdout);
        }

        sleep_ms(1);
    }
}
