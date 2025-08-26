//轮趣科技大电机（gmr磁编码版），编码器读取

#include "stm32f4xx.h"
#include <math.h>

// 定义小车参数
#define PI 						3.1415926535f
#define WHEEL_DIAMETER_L_CM     6.58f		   	// 左轮直径
#define WHEEL_DIAMETER_R_CM     6.56f  		    // 右轮直径
#define PULSE_PER_REVOLUTION    56000.0f	    // PPR
#define TRACK_WIDTH_CM      	12.17f   		// 左右轮间距
#define RAD_TO_DEG 				(180.0f/PI) 	// 弧度转角度系数

const float PULSE_TO_CM_L = (PI * WHEEL_DIAMETER_L_CM) / PULSE_PER_REVOLUTION;
const float PULSE_TO_CM_R = (PI * WHEEL_DIAMETER_R_CM) / PULSE_PER_REVOLUTION;

volatile float pos_x = 0.0f; // X坐标（cm）
volatile float pos_y = 0.0f; // Y坐标（cm）
volatile float theta = 0.0f; // 当前角度（-180-180度）

volatile int32_t delta_TIM1 = 0;  // 左轮（TIM1）
volatile int32_t delta_TIM5 = 0;  // 右轮（TIM5）

volatile float motor_rpm_L = 0.0f;     // 左后转速
volatile float motor_rpm_R = 0.0f;     // 右后转速

void Encoder_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct = {0};
    TIM_ICInitTypeDef TIM_ICInitStruct = {0};

    //配置TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);

    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitStruct.TIM_ICFilter = 10;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM1, &TIM_ICInitStruct);

    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM1, &TIM_ICInitStruct);
    
    TIM_Cmd(TIM1, ENABLE);

    //配置TIM5
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

    TIM_TimeBaseStruct.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStruct.TIM_Prescaler = 0;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
    
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_ICInitStruct.TIM_ICFilter = 10;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM5, &TIM_ICInitStruct);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM5, &TIM_ICInitStruct);
    
    TIM_Cmd(TIM5, ENABLE);
}

void encoder_get_rpm(void) 
{
    static uint16_t last_TIM1 = 0;  // 左轮（TIM1，16位）
    static uint32_t last_TIM5 = 0;  // 右轮（TIM5，32位）
	
    // 读取 TIM1（左轮，16位），正确处理溢出
    uint16_t current_TIM1 = TIM_GetCounter(TIM1);
    int16_t diff = (int16_t)(current_TIM1 - last_TIM1); // 有符号差值
    delta_TIM1 = (int32_t)(0-diff);
    last_TIM1 = current_TIM1;
	motor_rpm_L = (delta_TIM1 * 60.0f) / 0.01f / PULSE_PER_REVOLUTION;

    // 读取 TIM5（右轮，32位）
    uint32_t current_TIM5 = TIM_GetCounter(TIM5);
    delta_TIM5 = (int32_t)(current_TIM5 - last_TIM5);
    last_TIM5 = current_TIM5;
	motor_rpm_R = (delta_TIM5 * 60.0f) / 0.01f / PULSE_PER_REVOLUTION;
	
    // 计算左右轮位移（cm）
    float delta_s_A = delta_TIM1 * PULSE_TO_CM_L; // 左轮
    float delta_s_B = delta_TIM5 * PULSE_TO_CM_R; // 右轮

    // 计算线速度（cm/s）
    float v_left = delta_s_A / 0.01f;
    float v_right = delta_s_B / 0.01f;

    // 计算角速度（rad/s）
    float omega = (v_left - v_right) / TRACK_WIDTH_CM;

    // 更新角度（0-360度）
    theta += omega * 0.01f * RAD_TO_DEG;
	
    if (theta > 180.0f) theta -= 360.0f;
	if (theta < -180.0f) theta += 360.0f;

    // 计算质心线速度（cm/s）
    float v_center = (v_left + v_right) / 2.0f;

    // 更新坐标（使用弧度）
    float theta_rad = theta * PI / 180.0f;
    pos_y += v_center * cosf(theta_rad) * 0.01f;
    pos_x += v_center * sinf(theta_rad) * 0.01f;
}
