//use encoder in UTF-8 without signature
#include <stm32f4xx.h>
#include "lcd_init.h"
#include "lcd_touch.h"
#include "lcd.h"
#include "delay.h"
#include "icm42688.h"
#include "timer_clock.h"
#include "task_ctrl.h"
#include "key_beep_led.h"
#include "ahrs.h"
#include "stepper_yaw.h"
#include "stepper_pitch.h"
#include "stepper_ctrl.h"
#include "stepper.h"
#include "k230_serial.h"
#include "laser.h"

//参数：设置的绝对角度
float Set_Pitch_Theta = 0.0f;
float Set_Yaw_Theta = 0.0f;

//参数：设置的速度
uint16_t Pitch_RPM = 0;
uint16_t Yaw_RPM = 0;

//参数：设置的加速度
uint8_t set_aac = 0;

void Reset_Stepper(void) 
{
	Set_Pitch_Theta = 0.0f;
	Set_Yaw_Theta = 0.0f;
	Pitch_RPM = 0;
	Yaw_RPM = 0;
	set_aac = 0;
}

int16_t Diff_X = 0;
int16_t Diff_Y = 0;

int16_t R_Diff_X = 0;
int16_t R_Diff_Y = 0;

uint8_t flag_10ms = 0;
uint8_t task_num = 0;

uint8_t Get_which_direction(void)//left:1 behind:2 right:3 front:4 
{
	static uint8_t direction = 0;
	if(ahrs_yaw < -85 && ahrs_yaw > -95 && (direction == 0 || direction == 4)) direction = 1;
	if((ahrs_yaw > 175 || ahrs_yaw < -175) && direction == 1) direction = 2;
	if(ahrs_yaw > 85 && ahrs_yaw < 95 && direction == 2) direction = 3;
	if(ahrs_yaw < 5 && ahrs_yaw > -5 && direction == 3) direction = 4;
	return direction;
}

void basic_task0(void)//屏幕显示
{
	static uint8_t num = 0;
	num ++;
	if(num == 1) 
	{
		LCD_DrawRectangle(10,40,100,80,DARKBLUE);
		LCD_ShowStr(30,50,RED,WHITE,"task1",24,0);
	}
	if(num == 2) 
	{
		LCD_DrawRectangle(120,40,210,80,DARKBLUE);
		LCD_ShowStr(140,50,RED,WHITE,"task2",24,0);
	}
	if(num == 3) 
	{
		LCD_DrawRectangle(230,40,320,80,DARKBLUE);
		LCD_ShowStr(250,50,RED,WHITE,"task3",24,0);
		num = 0;
	}

	TP_Scan();
	if(Lcd_Touch_Flag)
	{
		if(Touch_Y > 10  && Touch_Y < 100 ) task_num = 1;
		if(Touch_Y > 120 && Touch_Y < 160)  task_num = 2;
		if(Touch_Y > 170 && Touch_Y < 210)  task_num = 3;
		if(Touch_Y > 230 && Touch_Y < 320)  task_num = 4;
	}
}

void basic_task1(void)//自选打靶
{
	static uint8_t count = 0;
	
    float pid_yaw_add = Servo_Y_PID_Control(-Diff_X,1);
	float pid_pitch_add = Servo_P_PID_Control(-Diff_Y,1);
	
	Set_Pitch_Theta += pid_pitch_add;
	Set_Yaw_Theta += pid_yaw_add;
	
	Pitch_RPM = 1000;
	Yaw_RPM = 1000;
	
	set_aac = 200;
	
	if((Diff_X < 3 && Diff_X > -3) && (Diff_Y < 3 && Diff_Y > -3)) count ++;
	else count = 0;
	if(count == 10) Laser_3V3_OnOff(1); 
	if(count == 100) {Laser_3V3_OnOff(0); task_num = 0;Reset_Stepper();}
}

void basic_task2(void)//旋转打靶
{
	static uint8_t tracing_flag = 0;
	if(Diff_X != 0 || Diff_Y != 0) tracing_flag = 1;
	if(tracing_flag == 1)
	{
		basic_task1();
	}
	else
	{
		if(task_num == 2) Set_Yaw_Theta += 0.7f;
		if(task_num == 3) Set_Yaw_Theta -= 0.7f;
		Yaw_RPM = 1000;
		set_aac = 0;
	}
}

void basic_task3(void)//N=1/N=2运动打靶
{
	static float last_imu_theta = 0.0f;
	static uint8_t first_flag = 0;
	if(!first_flag)
	{
		Reset_Stepper();
		Laser_3V3_OnOff(1);
		Set_Yaw_Theta = -120.0f;
		first_flag = 1;
	}
	
	if(Get_which_direction() == 1) 
	{
		static uint16_t count = 0;if(count < 500)count++;
		if(count > 50 && count < 270) Diff_X -= 5;
	}
	else if(Get_which_direction() == 2) 
	{
		static uint16_t count = 0;if(count < 500)count++;
		if(count > 50 && count < 270) Diff_X -= 10;
	}
	else if(Get_which_direction() == 3) 
	{
		static uint16_t count = 0;if(count < 500)count++;
		if(count > 50 && count < 270) Diff_X -= 5;
	}
	else if(Get_which_direction() == 4) 
	{
		static uint16_t count = 0;if(count < 500)count++;
		if(count > 50 && count < 270) Diff_X += 25;
	}
	
	Set_Yaw_Theta = Set_Yaw_Theta + ahrs_total_yaw - last_imu_theta;
	last_imu_theta = ahrs_total_yaw; 
	
	float pid_yaw_add = Servo_Y_PID_Control(-Diff_X,2);
	float pid_pitch_add = Servo_P_PID_Control(-Diff_Y,2);
	
	Set_Pitch_Theta += pid_pitch_add;
	Set_Yaw_Theta += pid_yaw_add;
	
	Pitch_RPM = 50;
	Yaw_RPM = 50;
	
	set_aac = 0;
}

void all_task_ctrl(void)
{
	if(task_num == 0) 
	{
		basic_task0();
	}
	if(task_num == 1)
	{
		basic_task1();
	}
	if(task_num == 2 || task_num == 3)
	{
		basic_task2();
	}
	if(task_num == 4)
	{
		basic_task3();
	}
}

int main(void)
{
	All_Nvic_Init();

	LCD_Init();
	TP_Init();

	ICM42688_Init(2,2,8,8,3,3,2,1,1);//4g 500dps 100hz
	
	TimClock_Init();

	Key_Beep_Led_Init();

	Task_TIM6_Init();

	USART3_DMA_Init();
	USART2_DMA_Init();
	
	Delay_ms(1000);
	Emm_V5_Origin_Trigger_Return(0x01,0,0);
	Emm_V5_Origin_Trigger_Return(0x02,0,0);
	Delay_ms(1000);
	Emm_V5_Reset_CurPos_To_Zero(0x01);
	Emm_V5_Reset_CurPos_To_Zero(0x02);
	Delay_ms(1000);
	
	Serial_Init();
	Init_Laser();
	
	while(1)
	{
		if(flag_10ms)
		{
			all_task_ctrl();
				
			if (Serial_RxFlag4) 
			{
				char msg1[10], msg2[10];
				
				if (Serial_ParsePercentPacket(msg1, msg2))
				{
					Diff_X = (int16_t)strtol(msg1, NULL, 10);
					Diff_Y = (int16_t)strtol(msg2, NULL, 10);
				}
			}
			
			Key_Read();
			if(key1_flag) {Laser_3V3_OnOff(0); key1_flag=0;}
			if(key2_flag) {Laser_3V3_OnOff(1); key2_flag=0;}
			
			Set_Pitch(set_aac,Pitch_RPM,Set_Pitch_Theta);
			Set_Yaw(set_aac,Yaw_RPM,Set_Yaw_Theta);
					
			flag_10ms = 0;
		}
	}
}

//定时器6中断服务函数
void TIM6_DAC_IRQHandler(void)//10ms
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
		flag_10ms = 1;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}
