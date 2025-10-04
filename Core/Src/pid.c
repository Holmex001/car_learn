
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prev_error;

}PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Compute(PID_Controller *pid, float current_value)
{
    float error = pid->setpoint - current_value;
    pid->integral += error ;
    float derivative = error - pid->prev_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->prev_error = error;

    return output;
}

float PID_Stand_UP(PID_Controller *pid, float current_value, float delta_gyro)
{
    float error = current_value - pid->setpoint;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    // float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * delta_gyro;
    pid->prev_error = error;

    return output;
}
