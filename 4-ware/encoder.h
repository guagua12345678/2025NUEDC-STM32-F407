#include "stm32f4xx.h"                  // Device header

#ifndef __ENCODER_H__
#define __ENCODER_H__

extern volatile float motor_rpm_L;
extern volatile float motor_rpm_R;

extern volatile float pos_x; // X坐标（cm）
extern volatile float pos_y; // Y坐标（cm）
extern volatile float theta; // 当前角度（0-360度）

void Encoder_Init(void);
void encoder_get_rpm(void);
	
#endif
