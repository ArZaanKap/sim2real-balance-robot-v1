#pragma once

#define Kp 2550
#define Kd 0.0
#define Ki 0.0
#define dt 0.01 // 100Hz?

// RAD - for 0.1rad i want max speed 255 for now?
float pid_controller(const float theta_y, const float w_y);