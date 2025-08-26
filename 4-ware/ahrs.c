//icm42688的ahrs解算，参考https://github.com/AzureHalcyon/icm42688_hal

#include <stm32f4xx.h>
#include "math.h"

//最终结果
float ahrs_yaw = 0.0f, ahrs_pitch = 0.0f, ahrs_roll = 0.0f;

#define AHRS_PI 3.1415926535f
	
// 为不同轴设置不同的PI参数
#define KP_ROLL     3.2f    /**< Roll轴比例增益 */
#define KP_PITCH    2.95f   /**< Pitch轴比例增益 */
#define KP_YAW      1.0f    /**< Yaw轴比例增益（通常要小一些）*/

#define KI_ROLL     0.095f  /**< Roll轴积分增益 */
#define KI_PITCH    0.085f  /**< Pitch轴积分增益 */
#define KI_YAW      0.065f  /**< Yaw轴积分增益（通常要小一些）*/

// 积分限幅，防止积分饱和
#define INT_LIMIT 0.1f          /**< 基础积分限幅值 */
#define INT_LIMIT_FACTOR 0.5f   /**< 动态限幅系数 */

// 死区参数
#define GYRO_DEADBAND    0.002f  /**< 陀螺仪死区（弧度/秒）*/
#define ERROR_DEADBAND   0.005f /**< 误差死区 */

// 四元数和积分误差
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static float limitIntegral(float value, float limit, float gx, float gy, float gz)
{
    float dynamic_limit = limit + fabsf(gx + gy + gz) * INT_LIMIT_FACTOR;
    return fminf(fmaxf(value, -dynamic_limit), dynamic_limit);
}

static float applyDeadband(float value, float deadband) 
{
    if(fabs(value) < deadband) return 0.0f;
    return value;
}

// 新增变量
static float last_yaw = 0.0f;        // 上一次的瞬时航向角
float ahrs_total_yaw = 0.0f;         // 无限累加的航向角（全局可访问）

void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) 
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez, halfT;
    float tempq0, tempq1, tempq2, tempq3;
    
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    
    gx = applyDeadband(gx, GYRO_DEADBAND);
    gy = applyDeadband(gy, GYRO_DEADBAND);
    gz = applyDeadband(gz, GYRO_DEADBAND);
	
	halfT = (dt / 2.0f);

    // 加速度计归一化
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    // 根据当前四元数计算出重力方向
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    // 计算误差，估计重力方向和测量重力方向的叉积
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);
	
	// 应用误差死区
    ex = applyDeadband(ex, ERROR_DEADBAND);
    ey = applyDeadband(ey, ERROR_DEADBAND);
    ez = applyDeadband(ez, ERROR_DEADBAND);

    // PI控制器，分别处理三个轴
    if(ex != 0.0f) {
        exInt = exInt + ex * KI_ROLL * halfT;
        exInt = limitIntegral(exInt, INT_LIMIT, gx, gy, gz);
        gx = gx + KP_ROLL * ex + exInt;
    }
    
    if(ey != 0.0f) {
        eyInt = eyInt + ey * KI_PITCH * halfT;
        eyInt = limitIntegral(eyInt, INT_LIMIT, gx, gy, gz);
        gy = gy + KP_PITCH * ey + eyInt;
    }
    
    if(ez != 0.0f) {
        ezInt = ezInt + ez * KI_YAW * halfT;
        ezInt = limitIntegral(ezInt, INT_LIMIT, gx, gy, gz);
        gz = gz + KP_YAW * ez + ezInt;
    }
    
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
    
    // 四元数归一化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;

    // 计算欧拉角
    float current_yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1) * 180/AHRS_PI;
    ahrs_pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2) * 180/AHRS_PI;
    ahrs_roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180/AHRS_PI;
	
	// 1. 计算当前帧与上一帧的角度差
    float delta_yaw = current_yaw - last_yaw;
    
    // 2. 处理360°边界跳变（关键步骤！）
    if (delta_yaw > 180.0f) {
        delta_yaw -= 360.0f;  // 正方向跳变补偿
    } else if (delta_yaw < -180.0f) {
        delta_yaw += 360.0f;  // 负方向跳变补偿
    }
    
    // 3. 累加到总航向角
    ahrs_total_yaw += delta_yaw;
    
    // 4. 更新上一次的航向角
    last_yaw = current_yaw;
    
    // 5. 可选：将瞬时值赋给ahrs_yaw（保持原始行为）
    ahrs_yaw = current_yaw;
}

void ahrs_init(void) 
{
	q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
	ahrs_yaw = 0.0f;
	ahrs_pitch = 0.0f;
	ahrs_roll = 0.0f;
}
