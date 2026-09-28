#pragma once


// motor
#define MOT_L_IN1_GPIO 27
#define MOT_L_IN2_GPIO 14
#define MOT_R_IN1_GPIO 25
#define MOT_R_IN2_GPIO 26

#define PWM_FREQ_HZ 20000                        
#define PWM_RES  LEDC_TIMER_8_BIT     // 8-bit -> duty range 0..255 (same as Arduino)


void ledc_common_init(void);
void motor_init(motor_t *m, int in1_gpio, int in2_gpio);
void motor_drive(motor_t *m, int duty);