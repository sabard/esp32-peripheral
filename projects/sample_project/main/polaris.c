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

void polaris_setup_uart(int baud_rate) {
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

void polaris_set_baudrate(int baud_rate) {
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

void polaris_send_init_seq(char *read_buf){
    int bytes_read;

    ESP_LOGI(TAG, "POLARIS INIT");

    // Write data to UART.
    uart_write("This is a test string.\n");


    // send break with beep (need to send something)
    uart_break("BEEP 1\r", 100);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // reset Polaris to intial state
    uart_write("RESET 0\r");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "reset: %s, %d", read_buf, bytes_read);
    
    // sanity beep
    uart_write("BEEP 1\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "beep: %s, %d", read_buf, bytes_read);

    // initialze
    uart_write("INIT \r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "init: %s, %d", read_buf, bytes_read);

    uart_write("PHRQ *********1****\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "phrq: %s, %d", read_buf, bytes_read);

    // load dummy tool file
    uart_write("PVWR 0100004E4449009D080000010000000100000201000000018C9932B400000003000000030000000000C03F000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 010040000020410000000000000000000000000000000000000000000000000000484200000000000000000000D2420000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 01008000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 0100C000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 01010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 01014000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 01018000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 0101C000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 0102000000000000000000000000000000000000000000000000000000000000000000000000000000000000010200000000000000000000000000000000001F1F1F1F\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 010240090000004E4449000000000000000000383730303234380000000000000000000000000009010101000000000000000000000000000000000001010100000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 01028000000000000000000000000000800029000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);
    uart_write("PVWR 0102C000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pvwr: %s, %d", read_buf, bytes_read);

    // initial tool port
    uart_write("PINIT 01\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pinit: %s, %d", read_buf, bytes_read);

    // start tool port
    uart_write("PENA 01S\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "pena: %s, %d", read_buf, bytes_read);

    // turn on IR illuminators
    uart_write("IRATE 2\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "irate: %s, %d", read_buf, bytes_read);

    // set larger capture volume
    uart_write("VSEL 1\r");
    uart_read((char *)read_buf, 100);
    ESP_LOGI(TAG, "vsel: %s", read_buf);

    // enter tracking mode
    uart_write("TSTART \r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "tstart: %s, %d", read_buf, bytes_read);

    // set serial port speed to 115200
    uart_write("COMM 50001\r");
    bytes_read = uart_read_until((char *)read_buf, "\r");
    ESP_LOGI(TAG, "baudrate: %s, %d", read_buf, bytes_read);


    ESP_LOGI(TAG, "POLARIS INIT END");
}

void polaris_read(Polaris_frame *frame, char *read_buf) {
    unsigned short reply_len;
    unsigned short header_id;
    
    ESP_LOGI(TAG, "POLARIS READ");

    uart_write("BX 1000\r");

    // read header
    uart_read(read_buf, 4);

    while(!(read_buf[0] == 0xc4 && read_buf[1] == 0xa5)) {
        // should be 0xc4a5
        printf("%02x ", *read_buf);
        printf("%02x\n", *(read_buf+1));

        uart_write("BX 1000\r");

        // read header
        uart_read(read_buf, 1);
        uart_read(read_buf, 4);
    }

    // should be 0xc4a5
    header_id = *(unsigned short *)read_buf;
    assert(header_id == 0xa5c4);

    reply_len = *(unsigned short *)(read_buf+2);

    // TODO this was +4 in lico code but maybe -4 makes more sense?
    uart_read(read_buf, reply_len + 4);

    frame->num_markers = (int)(*((unsigned char *)(read_buf + 5)));

    frame->mx = *(float *)(read_buf + 7);
    frame->my = *(float *)(read_buf + 11);
    frame->mz = *(float *)(read_buf + 15);

    printf(
        "num_markers: %d, mx: %f, my: %f, mz: %f \n",
        frame->num_markers, frame->mx, frame->my, frame->mz
    );


    ESP_LOGI(TAG, "POLARIS READ END");
}
