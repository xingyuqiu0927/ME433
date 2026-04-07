#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define SERVO_PIN 15
#define SERVO_MIN_PULSE_US 300.0f
#define SERVO_MAX_PULSE_US 2400.0f
#define SERVO_PERIOD_US 20000.0f
#define SERVO_STEP_DEG 2.0f
#define SERVO_STEP_DELAY_MS 20

static void servo_init(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, (uint16_t)(SERVO_PERIOD_US - 1));
    pwm_init(slice_num, &config, true);
}

static void servo_set_angle(uint gpio_pin, float angle_deg) {
    if (angle_deg < 0.0f) {
        angle_deg = 0.0f;
    } else if (angle_deg > 180.0f) {
        angle_deg = 180.0f;
    }

    float pulse_width_us = SERVO_MIN_PULSE_US +
                           (angle_deg / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    pwm_set_gpio_level(gpio_pin, (uint16_t)pulse_width_us);
}

int main() {
    stdio_init_all();
    servo_init(SERVO_PIN);

    while (true) {
        for (float angle = 0.0f; angle <= 180.0f; angle += SERVO_STEP_DEG) {
            servo_set_angle(SERVO_PIN, angle);
            sleep_ms(SERVO_STEP_DELAY_MS);
        }

        for (float angle = 180.0f; angle >= 0.0f; angle -= SERVO_STEP_DEG) {
            servo_set_angle(SERVO_PIN, angle);
            sleep_ms(SERVO_STEP_DELAY_MS);
        }
    }
}
