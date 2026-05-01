#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "ssd1306.h"
#include "tusb.h"

#include "mpu6050.h"
#include "usb_descriptors.h"

#define OLED_I2C_PORT i2c0
#define OLED_I2C_SDA_PIN 12
#define OLED_I2C_SCL_PIN 13
#define OLED_I2C_BAUDRATE 400000

#define IMU_I2C_PORT i2c1
#define IMU_I2C_SDA_PIN 14
#define IMU_I2C_SCL_PIN 15
#define IMU_I2C_BAUDRATE 400000

#define MODE_BUTTON_PIN 16
#define BUTTON_ACTIVE_LEVEL 0
#define BUTTON_DEBOUNCE_MS 25

#define MODE_LED_PIN 0
#define MODE_LED_ON_LEVEL 1

#define IMU_SAMPLE_PERIOD_MS 10
#define MOUSE_REPORT_PERIOD_MS 10
#define DISPLAY_PERIOD_MS 40

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_CENTER_X (DISPLAY_WIDTH / 2)
#define DISPLAY_CENTER_Y (DISPLAY_HEIGHT / 2)

#define MOUSE_SWAP_AXES 0
#define MOUSE_INVERT_X 1
#define MOUSE_INVERT_Y 0

#define ACCEL_FILTER_SHIFT 2
#define ACCEL_DEADZONE_COUNTS 1200
#define ACCEL_LEVEL1_COUNTS 2800
#define ACCEL_LEVEL2_COUNTS 5200
#define ACCEL_LEVEL3_COUNTS 8500

#define MOUSE_SPEED_LEVEL1 2
#define MOUSE_SPEED_LEVEL2 5
#define MOUSE_SPEED_LEVEL3 8
#define MOUSE_SPEED_LEVEL4 12

#define CIRCLE_RADIUS_PIXELS 220.0f
#define CIRCLE_STEP_RAD 0.008f
#define DISPLAY_CIRCLE_RADIUS_PIXELS 10.0f
#define ARROW_LENGTH_PIXELS 12.0f
#define ARROW_HEAD_LENGTH_PIXELS 4.0f
#define ARROW_HEAD_HALF_WIDTH_PIXELS 3.0f

typedef enum {
    MODE_IMU_MOUSE = 0,
    MODE_REMOTE_CIRCLE = 1,
} mouse_mode_t;

typedef struct {
    bool last_raw_pressed;
    bool stable_pressed;
    uint32_t last_change_ms;
} button_state_t;

static mouse_mode_t current_mode = MODE_IMU_MOUSE;
static button_state_t mode_button = {0};
static mpu6050_t imu = {0};
static mpu6050_sample_t latest_sample = {0};
static bool have_sample = false;
static int32_t filtered_accel_x = 0;
static int32_t filtered_accel_y = 0;
static float circle_angle = 0.0f;
static float previous_circle_x = CIRCLE_RADIUS_PIXELS;
static float previous_circle_y = 0.0f;
static float circle_residual_x = 0.0f;
static float circle_residual_y = 0.0f;

static int clamp_int(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int round_to_int(float value) {
    if (value >= 0.0f) {
        return (int) (value + 0.5f);
    }
    return (int) (value - 0.5f);
}

static void set_mode_led(bool on) {
    gpio_put(MODE_LED_PIN, on ? MODE_LED_ON_LEVEL : !MODE_LED_ON_LEVEL);
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

static void draw_circle_outline(int center_x, int center_y, int radius) {
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        ssd1306_drawPixel((unsigned char) (center_x + x), (unsigned char) (center_y + y), 1);
        ssd1306_drawPixel((unsigned char) (center_x + y), (unsigned char) (center_y + x), 1);
        ssd1306_drawPixel((unsigned char) (center_x - y), (unsigned char) (center_y + x), 1);
        ssd1306_drawPixel((unsigned char) (center_x - x), (unsigned char) (center_y + y), 1);
        ssd1306_drawPixel((unsigned char) (center_x - x), (unsigned char) (center_y - y), 1);
        ssd1306_drawPixel((unsigned char) (center_x - y), (unsigned char) (center_y - x), 1);
        ssd1306_drawPixel((unsigned char) (center_x + y), (unsigned char) (center_y - x), 1);
        ssd1306_drawPixel((unsigned char) (center_x + x), (unsigned char) (center_y - y), 1);

        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        }
        if (err > 0) {
            x--;
            err -= 2 * x + 1;
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

static void draw_usb_status(void) {
    if (tud_mounted()) {
        for (int x = 0; x < 6; x++) {
            ssd1306_drawPixel((unsigned char) x, 0, 1);
            ssd1306_drawPixel((unsigned char) x, 1, 1);
        }
    } else {
        for (int y = 0; y < 6; y++) {
            ssd1306_drawPixel(0, (unsigned char) y, 1);
            ssd1306_drawPixel(1, (unsigned char) y, 1);
        }
    }
}

static void fatal_halt(void) {
    while (true) {
        set_mode_led(true);
        sleep_ms(100);
        set_mode_led(false);
        sleep_ms(100);
    }
}

static void init_mode_io(void) {
    gpio_init(MODE_BUTTON_PIN);
    gpio_set_dir(MODE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MODE_BUTTON_PIN);

    gpio_init(MODE_LED_PIN);
    gpio_set_dir(MODE_LED_PIN, GPIO_OUT);
    set_mode_led(false);

    bool raw_pressed = gpio_get(MODE_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL;
    mode_button.last_raw_pressed = raw_pressed;
    mode_button.stable_pressed = raw_pressed;
    mode_button.last_change_ms = to_ms_since_boot(get_absolute_time());
}

static void init_oled_i2c(void) {
    i2c_init(OLED_I2C_PORT, OLED_I2C_BAUDRATE);
    gpio_set_function(OLED_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_I2C_SDA_PIN);
    gpio_pull_up(OLED_I2C_SCL_PIN);
}

static void init_imu_i2c(void) {
    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);
    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);
}

static void reset_circle_motion(void) {
    circle_angle = 0.0f;
    previous_circle_x = CIRCLE_RADIUS_PIXELS;
    previous_circle_y = 0.0f;
    circle_residual_x = 0.0f;
    circle_residual_y = 0.0f;
}

static void reset_imu_filter(void) {
    if (!have_sample) {
        filtered_accel_x = 0;
        filtered_accel_y = 0;
        return;
    }

    filtered_accel_x = latest_sample.accel_x;
    filtered_accel_y = latest_sample.accel_y;
}

static void set_mode(mouse_mode_t new_mode) {
    current_mode = new_mode;
    set_mode_led(current_mode == MODE_REMOTE_CIRCLE);
    reset_circle_motion();
    reset_imu_filter();
}

static void render_imu_display(void) {
    int32_t raw_x = filtered_accel_x;
    int32_t raw_y = filtered_accel_y;

#if MOUSE_SWAP_AXES
    int32_t temp = raw_x;
    raw_x = raw_y;
    raw_y = temp;
#endif
#if MOUSE_INVERT_X
    raw_x = -raw_x;
#endif
#if MOUSE_INVERT_Y
    raw_y = -raw_y;
#endif

    draw_line(DISPLAY_CENTER_X - 5, DISPLAY_CENTER_Y, DISPLAY_CENTER_X + 5, DISPLAY_CENTER_Y);
    draw_line(DISPLAY_CENTER_X, DISPLAY_CENTER_Y - 5, DISPLAY_CENTER_X, DISPLAY_CENTER_Y + 5);

    float direction_x = (float) raw_x;
    float direction_y = (float) raw_y;
    float magnitude = sqrtf(direction_x * direction_x + direction_y * direction_y);

    if (magnitude >= (float) ACCEL_DEADZONE_COUNTS) {
        int x_offset = round_to_int((direction_x / magnitude) * ARROW_LENGTH_PIXELS);
        int y_offset = round_to_int((direction_y / magnitude) * ARROW_LENGTH_PIXELS);
        int tip_x = clamp_int(DISPLAY_CENTER_X + x_offset, 0, DISPLAY_WIDTH - 1);
        int tip_y = clamp_int(DISPLAY_CENTER_Y + y_offset, 0, DISPLAY_HEIGHT - 1);

        draw_arrow(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, tip_x, tip_y);
    } else {
        draw_circle_outline(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, 2);
    }
}

static void render_circle_display(void) {
    int dot_x = DISPLAY_CENTER_X + round_to_int(cosf(circle_angle) * DISPLAY_CIRCLE_RADIUS_PIXELS);
    int dot_y = DISPLAY_CENTER_Y + round_to_int(sinf(circle_angle) * DISPLAY_CIRCLE_RADIUS_PIXELS);

    draw_circle_outline(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, (int) DISPLAY_CIRCLE_RADIUS_PIXELS);
    draw_line(DISPLAY_CENTER_X, DISPLAY_CENTER_Y, dot_x, dot_y);
    draw_circle_outline(dot_x, dot_y, 2);
}

static void update_display(void) {
    ssd1306_clear();
    draw_usb_status();

    if (current_mode == MODE_IMU_MOUSE) {
        render_imu_display();
    } else {
        render_circle_display();
    }

    ssd1306_update();
}

static bool update_mode_button(uint32_t now_ms) {
    bool raw_pressed = gpio_get(MODE_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL;

    if (raw_pressed != mode_button.last_raw_pressed) {
        mode_button.last_raw_pressed = raw_pressed;
        mode_button.last_change_ms = now_ms;
    }

    if ((uint32_t) (now_ms - mode_button.last_change_ms) < BUTTON_DEBOUNCE_MS) {
        return false;
    }

    if (raw_pressed != mode_button.stable_pressed) {
        mode_button.stable_pressed = raw_pressed;
        return raw_pressed;
    }

    return false;
}

static int8_t quantize_axis(int32_t accel_counts) {
    int32_t magnitude = accel_counts >= 0 ? accel_counts : -accel_counts;
    int8_t speed = 0;

    if (magnitude < ACCEL_DEADZONE_COUNTS) {
        return 0;
    }
    if (magnitude < ACCEL_LEVEL1_COUNTS) {
        speed = MOUSE_SPEED_LEVEL1;
    } else if (magnitude < ACCEL_LEVEL2_COUNTS) {
        speed = MOUSE_SPEED_LEVEL2;
    } else if (magnitude < ACCEL_LEVEL3_COUNTS) {
        speed = MOUSE_SPEED_LEVEL3;
    } else {
        speed = MOUSE_SPEED_LEVEL4;
    }

    return accel_counts >= 0 ? speed : (int8_t) -speed;
}

static void compute_imu_mouse_delta(int8_t *dx, int8_t *dy) {
    int32_t raw_x = filtered_accel_x;
    int32_t raw_y = filtered_accel_y;

#if MOUSE_SWAP_AXES
    int32_t temp = raw_x;
    raw_x = raw_y;
    raw_y = temp;
#endif
#if MOUSE_INVERT_X
    raw_x = -raw_x;
#endif
#if MOUSE_INVERT_Y
    raw_y = -raw_y;
#endif

    *dx = quantize_axis(raw_x);
    *dy = quantize_axis(raw_y);
}

static void compute_circle_mouse_delta(int8_t *dx, int8_t *dy) {
    circle_angle += CIRCLE_STEP_RAD;

    float x = CIRCLE_RADIUS_PIXELS * cosf(circle_angle);
    float y = CIRCLE_RADIUS_PIXELS * sinf(circle_angle);

    float delta_x = x - previous_circle_x;
    float delta_y = y - previous_circle_y;

    previous_circle_x = x;
    previous_circle_y = y;

    circle_residual_x += delta_x;
    circle_residual_y += delta_y;

    int rounded_x = (int) circle_residual_x;
    int rounded_y = (int) circle_residual_y;

    circle_residual_x -= (float) rounded_x;
    circle_residual_y -= (float) rounded_y;

    if (rounded_x > 127) {
        rounded_x = 127;
    } else if (rounded_x < -127) {
        rounded_x = -127;
    }

    if (rounded_y > 127) {
        rounded_y = 127;
    } else if (rounded_y < -127) {
        rounded_y = -127;
    }

    *dx = (int8_t) rounded_x;
    *dy = (int8_t) rounded_y;
}

static void sample_imu(void) {
    if (!mpu6050_read_sample(&imu, &latest_sample)) {
        fatal_halt();
    }

    if (!have_sample) {
        filtered_accel_x = latest_sample.accel_x;
        filtered_accel_y = latest_sample.accel_y;
        have_sample = true;
        return;
    }

    filtered_accel_x += ((int32_t) latest_sample.accel_x - filtered_accel_x) >> ACCEL_FILTER_SHIFT;
    filtered_accel_y += ((int32_t) latest_sample.accel_y - filtered_accel_y) >> ACCEL_FILTER_SHIFT;
}

static void send_mouse_report(void) {
    if (!tud_hid_ready()) {
        return;
    }

    int8_t dx = 0;
    int8_t dy = 0;

    if (current_mode == MODE_IMU_MOUSE) {
        if (!have_sample) {
            return;
        }
        compute_imu_mouse_delta(&dx, &dy);
    } else {
        compute_circle_mouse_delta(&dx, &dy);
    }

    if (dx == 0 && dy == 0) {
        return;
    }

    tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, dx, dy, 0, 0);
}

int main(void) {
    board_init();
    init_mode_io();
    init_oled_i2c();
    init_imu_i2c();

    ssd1306_use_i2c(OLED_I2C_PORT);
    ssd1306_setup();

    if (!mpu6050_init_auto(&imu, IMU_I2C_PORT)) {
        fatal_halt();
    }

    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    set_mode(MODE_IMU_MOUSE);

    uint32_t next_imu_sample_ms = board_millis();
    uint32_t next_mouse_report_ms = board_millis();
    uint32_t next_display_ms = board_millis();
    update_display();

    while (true) {
        tud_task();

        uint32_t now_ms = board_millis();

        if (update_mode_button(now_ms)) {
            mouse_mode_t next_mode = current_mode == MODE_IMU_MOUSE ? MODE_REMOTE_CIRCLE : MODE_IMU_MOUSE;
            set_mode(next_mode);
        }

        if ((int32_t) (now_ms - next_imu_sample_ms) >= 0) {
            next_imu_sample_ms += IMU_SAMPLE_PERIOD_MS;
            sample_imu();
        }

        if ((int32_t) (now_ms - next_mouse_report_ms) >= 0) {
            next_mouse_report_ms += MOUSE_REPORT_PERIOD_MS;
            send_mouse_report();
        }

        if ((int32_t) (now_ms - next_display_ms) >= 0) {
            next_display_ms += DISPLAY_PERIOD_MS;
            update_display();
        }
    }
}

uint16_t tud_hid_get_report_cb(
    uint8_t instance,
    uint8_t report_id,
    hid_report_type_t report_type,
    uint8_t *buffer,
    uint16_t reqlen
) {
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
    return 0;
}

void tud_hid_set_report_cb(
    uint8_t instance,
    uint8_t report_id,
    hid_report_type_t report_type,
    uint8_t const *buffer,
    uint16_t bufsize
) {
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;
}
