#include <stm32f4xx.h>

#ifndef __AHRS_H__
#define __AHRS_H__

extern float ahrs_yaw, ahrs_pitch, ahrs_roll;

extern float ahrs_total_yaw;

void ahrs_init(void);
void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
