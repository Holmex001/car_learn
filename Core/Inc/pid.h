//
// Created by ThinkBook on 25-9-20.
//

#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prev_error;
}PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Compute(PID_Controller *pid, float current_value);
float PID_Stand_UP(PID_Controller *pid, float current_value, float delta_gyro);

#endif //PID_H
