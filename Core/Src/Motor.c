#include "Motor.h"



// motor
//  0 A
//  1 B
// mode
// 0 制动 IN1 0 IN2 0
// 1 正转 IN1 0 IN2 1
// -1 反转 IN1 1 IN2 0

void Motor_Init()
{
    HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Motor_Set_Mode(int motor,int mode) {
    if (motor == 0)
    {
        if (mode == 0)
        {
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
        }
        else if (mode == 1)
        {
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        }
        else if (mode == -1)
        {
            HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        }
    }else if (motor == 1)
    {
        if (mode == 0)
        {
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
        }
        else if (mode == -1)
        {
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        }
        else if (mode == 1)
        {
            HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        }
    }
}

void Motor_Set_PWM(int motor,int PWMvar) {
    if (PWMvar >= 0)
    {
        PWMvar = PWMvar * 1.07;
        Motor_Set_Mode(motor,1);
    }else
    {
        Motor_Set_Mode(motor,-1);
        PWMvar = -PWMvar;
    }


    if (motor == 0)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, PWMvar);
    }else if (motor == 1)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, PWMvar);
    }
}

void Motor_Set_v(int motor,int v)
{

}
