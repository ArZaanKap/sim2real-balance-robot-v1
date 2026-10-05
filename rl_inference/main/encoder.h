#pragma once
#include "driver/pulse_cnt.h"
#include <stdint.h>


// encoder
#define COUNTS_PER_REV 1976.0f // measured ourselves
#define ENC_L_A_GPIO 33
#define ENC_L_B_GPIO 32
#define ENC_R_A_GPIO 4
#define ENC_R_B_GPIO 13


// overflow = big running total, edited by ISR (Interrupt Service Routine)
// when something counter hits +/-3000
typedef struct {
    pcnt_unit_handle_t unit; // handle
    volatile int64_t overflow; // volatile - standard for variables accessed by interrupts and main?
} encoder_t;

// on watch point only used internally
//bool IRAM_ATTR on_watch_point(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
int64_t read_position(encoder_t *e);
void encoder_init(encoder_t *e, int a_gpio, int b_gpio);