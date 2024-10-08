#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "string.h"

#include "polaris.h"

#define GPIO_SERIAL_TX CONFIG_GPIO_SERIAL_TX // IO33 on wESP
#define GPIO_SERIAL_RX CONFIG_GPIO_SERIAL_RX // IO39 on wESP

#define UART_NUM UART_NUM_2

static const char *TAG = "eth_example_polaris";

void setup_uart(int baud_rate) {
    const uart_port_t uart_num = UART_NUM;
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins. RTS and CTS pins are not set
    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM, GPIO_SERIAL_TX, GPIO_SERIAL_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)
    );

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));

    ESP_LOGI(TAG, "UART SETUP");
}

void set_baudrate(int baud_rate) {
    uart_set_baudrate(UART_NUM, baud_rate);
}

void uart_break(const char* str, int duration) {
    uart_write_bytes_with_break(UART_NUM, (const char*)str, strlen(str), duration);
}

void uart_write(const char* str) {
     uart_write_bytes(UART_NUM, (const char*)str, strlen(str));
}

int uart_read(char* buf, int length) {
    uart_read_bytes(UART_NUM, buf, length, 1000);
    *(buf + length) = '\0';
    return length;
}

int uart_read_until(char* buf, const char* match_str) {
    int match_str_len = strlen(match_str);
    int read_bytes = 0;
    do {
        uart_read_bytes(UART_NUM, buf + read_bytes, 1, 1000);
        read_bytes += 1;
    } while (strncmp(buf + read_bytes - match_str_len, match_str, match_str_len));

    *(buf + read_bytes) = '\0';
    return read_bytes;
}
