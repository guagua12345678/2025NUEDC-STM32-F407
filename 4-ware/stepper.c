//基于张大头步进电机，使用绝对位置模式

#include <stm32f4xx.h>
#include "math.h"

// 全局PID参数
float Kp_Stepper1 = 0.0f;
float Ki_Stepper1 = 0.06f;
float Kd_Stepper1 = 0.006f;

float Kp_Stepper2 = 0.0f;
float Ki_Stepper2 = 0.08f;
float Kd_Stepper2 = 0.01f;

#define MAX_DELTA 10.0f

// 电机历史误差
float P_prev_error = 0.0f;
float P_prev_prev_error = 0.0f;
float Y_prev_error = 0.0f;
float Y_prev_prev_error = 0.0f;

// 像素差值到角度的转换系数
#define PIXEL_TO_ANGLE_YAW 0.0385f   // 水平：65°/800像素
#define PIXEL_TO_ANGLE_PITCH 0.036f // 垂直：40°/480像素

float Servo_Y_PID_Control(int16_t x_diff,uint8_t pid_num)
{
	float Kp_Stepper,Ki_Stepper,Kd_Stepper;
	if(pid_num == 1)
	{
		Kp_Stepper = Kp_Stepper1;
		Ki_Stepper = Ki_Stepper1;
		Kd_Stepper = Kd_Stepper1;
	}
	else if(pid_num == 2)
	{
		Kp_Stepper = Kp_Stepper2;
		Ki_Stepper = Ki_Stepper2;
		Kd_Stepper = Kd_Stepper2;
	}
	
	float yaw_error = x_diff * PIXEL_TO_ANGLE_YAW;
	
    float delta_Y = Kp_Stepper * (yaw_error - Y_prev_error)
                  + Ki_Stepper *  yaw_error
                  + Kd_Stepper * (yaw_error - 2*Y_prev_error + Y_prev_prev_error);
    
    if(delta_Y > MAX_DELTA) delta_Y = MAX_DELTA;
    else if(delta_Y < -MAX_DELTA) delta_Y = -MAX_DELTA;
    
    Y_prev_prev_error = Y_prev_error;
    Y_prev_error = yaw_error;
	
	return delta_Y;
}

float Servo_P_PID_Control(int16_t y_diff,uint8_t pid_num)
{
	float Kp_Stepper,Ki_Stepper,Kd_Stepper;
	if(pid_num == 1)
	{
		Kp_Stepper = Kp_Stepper1;
		Ki_Stepper = Ki_Stepper1;
		Kd_Stepper = Kd_Stepper1;
	}
	else if(pid_num == 2)
	{
		Kp_Stepper = Kp_Stepper2;
		Ki_Stepper = Ki_Stepper2;
		Kd_Stepper = Kd_Stepper2;
	}
	
	float pitch_error = y_diff * PIXEL_TO_ANGLE_PITCH;
	
    float delta_P = Kp_Stepper * (pitch_error - P_prev_error)
                  + Ki_Stepper *  pitch_error
                  + Kd_Stepper * (pitch_error - 2*P_prev_error + P_prev_prev_error);
    
    if(delta_P > MAX_DELTA) delta_P = MAX_DELTA;
    else if(delta_P < -MAX_DELTA) delta_P = -MAX_DELTA;
    
    P_prev_prev_error = P_prev_error;
    P_prev_error = pitch_error;

	return delta_P;
}

