#pragma once


typedef struct {
    float kp, ki, kd;
    float target; // target point to stabilise around
    float out_max; // for clamp
    float dt;
    float integral;
} pid_ctrl_t;


void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float target, float out_max, float dt);
float pid_controller(pid_ctrl_t *p, const float input, const float input_dt);
void pid_reset(pid_ctrl_t *p);