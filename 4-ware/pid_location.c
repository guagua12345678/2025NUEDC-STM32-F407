//位置环

#include "stm32f4xx.h"
#include "motor.h"
#include "encoder.h"
#include "pid_speed.h"
#include "math.h"

// 直线行驶PID参数
float Kp_Angle = 10.0f;       // 角度比例系数
float Ki_Angle = 10.0f;       // 角度积分系数
float Kd_Angle = 0.1f;        // 角度微分系数

#define max_angle_adjust 30

// 角度历史误差
float prev_angle_error = 0.0f;
float prev_prev_angle_error = 0.0f;

void reset_pid_go_straight(void)
{
	prev_angle_error = 0.0f;
	prev_prev_angle_error = 0.0f;
}

void pid_go_straight(float set_speed, float now_theta, float set_theta)
{
    // 1. 计算角度误差 (考虑角度环状特性)
    float error = set_theta - now_theta;
    
    // 将角度误差归一化到[-180, 180]范围
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    
    // 2. 计算PID增量（增量式算法）
    float delta = Kp_Angle * (error - prev_angle_error)
				+ Ki_Angle *  error
				+ Kd_Angle * (error - 2*prev_angle_error + prev_prev_angle_error);
    
    // 3. 修正量限幅
    if (delta > max_angle_adjust) delta = max_angle_adjust;
    else if (delta < -max_angle_adjust) delta = -max_angle_adjust;
    
    // 4. 更新历史误差
    prev_prev_angle_error = prev_angle_error;
    prev_angle_error = error;
    
    // 5. 计算左右轮目标速度
    float left_target = set_speed + delta;
    float right_target = set_speed - delta;
    
    // 6. 设置电机速度（调用速度环PID）
    pid_set_left_speed(left_target);
	pid_set_right_speed(right_target);
}

// 旋转PID参数
float Kp_Turn = 8.0f;         // 旋转比例系数
float Ki_Turn = 3.0f;         // 旋转积分系数
float Kd_Turn = 0.1f;         // 旋转微分系数

#define Angle_Tolerance 0.1f   // 角度容差
#define MAX_TURN_SPEED  150.0f // 最大旋转速度 (RPM)

// 旋转状态变量
float prev_turn_error = 0.0f;
float prev_prev_turn_error = 0.0f;

// 原地旋转函数 (+-180度范围)
void pid_turn_angle(float now_theta, float set_theta)
{
    // 1.计算角度误差并归一化到[-180, 180]
    float error = now_theta - set_theta;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    
    // 2.检查是否到达目标角度
    if (fabs(error) <= Angle_Tolerance) 
	{
        pid_set_left_speed(0);
        pid_set_right_speed(0);
        prev_turn_error = 0;
        prev_prev_turn_error = 0;
        return;
    }
    
    // 3.计算PID增量
    float delta = Kp_Turn * (error - prev_turn_error)
                + Ki_Turn *  error
                + Kd_Turn * (error - 2*prev_turn_error + prev_prev_turn_error);
    
    // 4.限幅处理
    if (delta > MAX_TURN_SPEED) delta = MAX_TURN_SPEED;
    else if (delta < -MAX_TURN_SPEED) delta = -MAX_TURN_SPEED;
    
    // 5.更新历史误差
    prev_prev_turn_error = prev_turn_error;
    prev_turn_error = error;
    
    // 6.执行旋转策略
    if (error < 0) 
	{ 
        pid_set_left_speed(fabs(delta));
        pid_set_right_speed(0);
    } 
	else 
	{ 
        pid_set_left_speed(0);
        pid_set_right_speed(fabs(delta));
    }
}

// 位置控制PID参数
#define Kp_Pos 10.0f      // 位置比例系数
#define Ki_Pos 15.0f      // 位置积分系数
#define Kd_Pos 0.1f       // 位置微分系数

#define MAX_SPEED 150.0f  // 最大前进速度 (RPM)
#define POS_TOLERANCE 0.5f // 位置容差 (cm)

float prev_location_error = 0.0f;
float prev_prev_location_error = 0.0f;

void pid_set_location(float set_length, float now_x, float now_y, float now_theta, float num)
{
    static float start_x = 0.0f, start_y = 0.0f, start_theta = 0.0f;
    static uint8_t last_num = 0;

    // 1. 首次调用时计算目标点坐标
    if (last_num != num) 
	{
		last_num = num;
        start_x = now_x;
        start_y = now_y;
		start_theta = now_theta;
    }

    // 2. 计算当前点到目标点的距离误差
    float dx = now_x - start_x;
    float dy = now_y - start_y;
    float error = set_length - sqrtf(dx*dx + dy*dy);

    // 3. 检查是否到达目标
    if (fabs(error) <= POS_TOLERANCE) 
	{
		pid_set_left_speed(0);
		pid_set_right_speed(0);
		prev_location_error = 0.0f;
		prev_prev_location_error = 0.0f;
        return;
    }
	
    // 4. 计算PID输出（位置式PID）
    float delta = Kp_Pos * (error - prev_location_error)
				+ Ki_Pos *  error
				+ Kd_Pos * (error - 2*prev_location_error + prev_prev_location_error);

    // 5. 速度限幅
    if (delta > MAX_SPEED) delta = MAX_SPEED;
    else if (delta < 0) delta = 0;

    // 6. 保持直线行驶（使用当前方向作为目标角度）
    pid_go_straight(delta,now_theta, start_theta);

    // 7. 更新历史误差
    prev_prev_location_error = prev_location_error;
	prev_location_error = error;
}

