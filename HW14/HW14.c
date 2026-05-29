#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"

#define HX711_SCK_PIN 16
#define HX711_DT_PIN 17

#define MAX_SAMPLES 4096
#define IIR_DIVISOR 8
#define HX711_READY_TIMEOUT_MS 500

static uint32_t sample_time_ms[MAX_SAMPLES];
static int32_t raw_samples[MAX_SAMPLES];
static int32_t filtered_samples[MAX_SAMPLES];

static void hx711_init(void) {
    gpio_init(HX711_SCK_PIN);
    gpio_set_dir(HX711_SCK_PIN, GPIO_OUT);
    gpio_put(HX711_SCK_PIN, 0);

    gpio_init(HX711_DT_PIN);
    gpio_set_dir(HX711_DT_PIN, GPIO_IN);
    gpio_pull_up(HX711_DT_PIN);
}

static bool hx711_read(int32_t *value) {
    uint32_t raw = 0;
    absolute_time_t deadline = make_timeout_time_ms(HX711_READY_TIMEOUT_MS);

    while (gpio_get(HX711_DT_PIN)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
        tight_loop_contents();
    }

    for (int i = 0; i < 24; i++) {
        gpio_put(HX711_SCK_PIN, 1);
        sleep_us(1);
        raw = (raw << 1) | (uint32_t)gpio_get(HX711_DT_PIN);
        gpio_put(HX711_SCK_PIN, 0);
        sleep_us(1);
    }

    gpio_put(HX711_SCK_PIN, 1);
    sleep_us(1);
    gpio_put(HX711_SCK_PIN, 0);
    sleep_us(1);

    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

    *value = (int32_t)raw;
    return true;
}

static int read_sample_count(void) {
    int requested = 0;

    printf("Enter number of samples, 1-%d:\n", MAX_SAMPLES);
    fflush(stdout);

    while (scanf("%d", &requested) != 1) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            sleep_ms(10);
        }
    }

    if (requested < 1) {
        requested = 1;
    } else if (requested > MAX_SAMPLES) {
        requested = MAX_SAMPLES;
    }

    return requested;
}

static bool collect_samples(int sample_count) {
    int32_t filtered = 0;

    for (int i = 0; i < sample_count; i++) {
        int32_t raw = 0;

        if (!hx711_read(&raw)) {
            printf("error,HX711 DT stayed high on GPIO %d\n", HX711_DT_PIN);
            fflush(stdout);
            return false;
        }

        if (i == 0) {
            filtered = raw;
        } else {
            filtered += (raw - filtered) / IIR_DIVISOR;
        }

        sample_time_ms[i] = to_ms_since_boot(get_absolute_time());
        raw_samples[i] = raw;
        filtered_samples[i] = filtered;
    }

    return true;
}

static void print_samples(int sample_count) {
    printf("time_ms,raw,filtered\n");
    for (int i = 0; i < sample_count; i++) {
        printf("%lu,%ld,%ld\n",
               (unsigned long)sample_time_ms[i],
               (long)raw_samples[i],
               (long)filtered_samples[i]);
    }
    printf("done\n");
    fflush(stdout);
}

int main(void) {
    stdio_init_all();
    hx711_init();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    printf("HW14 HX711 force sensor reader\n");
    printf("SCK GPIO %d, DT GPIO %d\n", HX711_SCK_PIN, HX711_DT_PIN);

    while (true) {
        int sample_count = read_sample_count();
        printf("collecting,%d\n", sample_count);
        fflush(stdout);

        if (collect_samples(sample_count)) {
            print_samples(sample_count);
        }
    }
}
