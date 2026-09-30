#include "pid.h"


void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float target, float out_max, float dt){
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->target = target;
    p->out_max = out_max;
    p->dt = dt;
    p->integral = 0;

}

float pid_controller(pid_ctrl_t *p, const float input, const float input_rate){
    
    float error = p->target - input;
    float error_rate = -input_rate; // e = t - i  ->  de/dt = -i
    p->integral += error * p->dt;
    

    float u = p->kp * error + p->kd * error_rate + p->ki * p->integral;
    if (u > p->out_max){
        u = p->out_max;
    }
    else if (u < -p->out_max){
        u = -p->out_max;
    }

    return u;
}

void pid_reset(pid_ctrl_t *p){
    p->integral = 0.0f;
}