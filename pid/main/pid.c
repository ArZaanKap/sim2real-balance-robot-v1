#include "pid.h"


float pid_controller(const float theta_y, const float w_y){ 
    
    static float I = 0;
    I += theta_y * dt;

    float u = Kp * theta_y + Kd * w_y + Ki * I;
    if (abs(u) > 255.0){u = 255.0;}
    
    return u;
}