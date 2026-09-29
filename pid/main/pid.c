#include "pid.h"


#define Kp 2550
#define Kd 0.0
#define Ki 0.0
//#define dt 0.01 // 100Hz?

float pid_controller(const float theta_y, const float w_y, const float dt){ // this dt never changes, but should be sourced from main - better way to make distiction? 
    
    static float I = 0;
    I += theta_y * dt;

    float u = Kp * theta_y + Kd * w_y + Ki * I;
    if (u > 255.0){
        u = 255.0;
    }
    else if (u < -255.0){
        u = -255.0;
    }

    return u;
}