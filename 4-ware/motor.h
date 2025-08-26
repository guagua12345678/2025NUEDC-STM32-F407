#include "stm32f4xx.h"

#ifndef __Motor_H__
#define __Motor_H__

void Motor_Init(void);
void Left_Wheel_Ctrl(int16_t set_speed);//±600
void Right_Wheel_Ctrl(int16_t set_speed);//±600
	
#endif
