#include <stm32f4xx.h>

#ifndef __STEPPER_H__
#define __STEPPER_H__

float Servo_Y_PID_Control(int16_t x_diff,uint8_t pid_num);
float Servo_P_PID_Control(int16_t y_diff,uint8_t pid_num);

#endif
