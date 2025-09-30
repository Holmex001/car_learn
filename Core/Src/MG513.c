//我是编码器 MG513
//我用的TIM4 5 编码器模式
//0 tim4
//1 tim5

//0 + 1 -
#include "MG513.h"


void MG513_Init()
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
}

int Get_Direction(int idx)
{
    int Direction = 0;
    if (idx == 0)
    {
        Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
        return Direction;
    }
    if (idx == 1)
    {
        Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5);
        return Direction;
    }
    return Direction;
}

int Get_CaptureNumber(int idx)
{
    int CaptureNumber = 0;
    if (idx == 0)
    {
        CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim4);
        return CaptureNumber;
    }
    if (idx == 1)
    {
        CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim5);
        return CaptureNumber;
    }
    return CaptureNumber;
}

float Get_Speed(int CaptureNumber0, int CaptureNumber1)
{
    float Speed = 0;
    float distance = (CaptureNumber1 - CaptureNumber0) * METERS_PER_PULSE_CD;
    Speed = distance / Circle_Time; //cm/s

    return Speed;
}

void MG513_Counter_Reset()
{
    __HAL_TIM_GET_COUNTER(&htim4) = 0;
    __HAL_TIM_GET_COUNTER(&htim5) = 0;
}


void MG513_To_String(int DirectionA, int CaptureNumberA, int DirectionB, int CaptureNumberB, float SpeedA, float SpeedB) //dma可能出问题
{
    char msg[1000];
    sprintf(msg, "Direction A : %d , CaptureNumber A : %d , Speed A : %s \r\nDirection B : %d , CaptureNumber B : %d ,Speed B : %s \r\n", DirectionA, CaptureNumberA, Float2String(SpeedA), DirectionB, CaptureNumberB, Float2String(SpeedB));
    HAL_UART_Transmit_IT(&huart4,msg,strlen(msg));

    // sprintf(msg, "Direction B : %d , CaptureNumber B : %d \r\n", DirectionB, CaptureNumberB);
    // HAL_UART_Transmit_IT(&huart4,msg,strlen(msg));
}

