//
// Created by ThinkBook on 25-9-20.
//

#ifndef MG513_H
#define MG513_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define wheel 6.5f
#define Circle_Time 0.1f
#define ENCODER_RESOLUTION_CD (500.0f * 4.0f * 30.0f)
#define METERS_PER_PULSE_CD ((wheel * 3.1415926535f) / ENCODER_RESOLUTION_CD)

void MG513_Init();
int Get_Direction(int idx);
int Get_CaptureNumber(int idx);
float Get_Speed(int CaptureNumber0, int CaptureNumber1);
void MG513_Counter_Reset();
void MG513_To_String(int DirectionA, int CaptureNumberA, int DirectionB, int CaptureNumberB, float SpeedA, float SpeedB);

#endif //MG513_H
