#include <stdio.h>
#include "hardware/uart.h"
#include "pico/stdlib.h"

#define PICO_UART uart0
#define PICO_UART_TX_PIN 0
#define PICO_UART_RX_PIN 1
#define BAUD_RATE 115200


int main()
{
    stdio_init_all();

    uart_init(PICO_UART, BAUD_RATE);
    gpio_set_function(PICO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_UART_RX_PIN, GPIO_FUNC_UART);

    while (true) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uart_putc_raw(PICO_UART, (char)c);
        }

        while (uart_is_readable(PICO_UART)) {
            putchar_raw(uart_getc(PICO_UART));
        }

        stdio_flush();
        sleep_ms(1);
    }
}
