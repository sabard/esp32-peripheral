#ifndef POLARIS_H
#define POLARIS_H

#include "freertos/FreeRTOS.h"

typedef struct {
    int num_markers;
    float mx;
    float my;
    float mz;
} Polaris_frame;

void polaris_setup_uart();

void polaris_set_baudrate(int baudrate);

void polaris_send_init_seq();

void polaris_read(Polaris_frame *frame, char *read_buf);

#endif
