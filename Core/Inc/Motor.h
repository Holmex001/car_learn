#ifndef Motor_H
#define Motor_H

#include "main.h"
#include "tim.h"

void Motor_Init();
void Motor_Set_Mode(int motor,int mode);
void Motor_Set_PWM(int motor,int PWMvar);

#endif