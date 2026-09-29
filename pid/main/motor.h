#pragma once
#include "driver/ledc.h"


// motor
#define MOT_L_IN1_GPIO 27
#define MOT_L_IN2_GPIO 14
#define MOT_R_IN1_GPIO 25
#define MOT_R_IN2_GPIO 26



typedef struct{
    ledc_channel_t in1_ch, in2_ch;
} motor_t;


void ledc_common_init(void);
void motor_init(motor_t *m, int in1_gpio, int in2_gpio);
void motor_drive(motor_t *m, int duty);