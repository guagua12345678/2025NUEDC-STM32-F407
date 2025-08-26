#include "stm32f4xx.h"

#ifndef __PID_LOCATION_H__
#define __PID_LOCATION_H__

void reset_pid_go_straight(void);
void pid_go_straight(float set_speed, float now_theta, float set_theta);
void pid_turn_angle(float now_theta, float turn_angle);
void pid_set_location(float set_length, float now_x, float now_y, float now_theta, float num);
	
#endif
