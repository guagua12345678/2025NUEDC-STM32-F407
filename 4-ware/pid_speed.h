#include "stm32f4xx.h"

#ifndef __PID_SPEED_H__
#define __PID_SPEED_H__

void reset_pid_speed(void);
void pid_set_left_speed(float set_rpm_L);
void pid_set_right_speed(float set_rpm_R);

#endif
