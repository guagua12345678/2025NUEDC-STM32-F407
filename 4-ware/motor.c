//轮趣科技大电机（gmr磁编码版），电机驱动

#include "stm32f4xx.h"

//freq : 84MHZ / PSC / ARR
//duty : CCR / ARR
uint16_t PWM_PSC = 7;     // 预分频值
uint16_t PWM_ARR = 600;   // 自动重装载值

void TIM8_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    // 2. 配置GPIO为复用功能
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 3. 配置引脚复用映射
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8); // 通道2
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8); // 通道3

    // 4. 配置TIM8基本参数
    TIM_TimeBaseInitStruct.TIM_Period = PWM_ARR - 1;       // ARR
    TIM_TimeBaseInitStruct.TIM_Prescaler = PWM_PSC - 1;    // PSC
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

    // 5. 配置PWM模式
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

    // 配置通道1和2
    TIM_OC1Init(TIM8, &TIM_OCInitStruct);
    TIM_OC2Init(TIM8, &TIM_OCInitStruct);

    // 使能预装载寄存器
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // 6. 启用主输出
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    // 7. 启动定时器
    TIM_Cmd(TIM8, ENABLE);
}

// PWM占空比设置函数 0-600级
void PWM_SetDuty_L(uint16_t CCR) { TIM8->CCR1 = CCR; } // 左轮PC7（通道2）
void PWM_SetDuty_R(uint16_t CCR) { TIM8->CCR2 = CCR; } // 右轮PC8（通道3）


// 修改后的引脚分配：
// 左轮：PC11（IN1） PC13（IN2）
// 右轮：PC4（IN1）  PD7（IN2）
void Motor_Init(void)
{
    TIM8_PWM_Init();
    
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 使能GPIOE和GPIOC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

    // 配置左轮引脚（PE4，PC3）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 配置右轮和左轮引脚（PC0，PC4，PC3）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 初始化所有引脚为低电平
    GPIO_ResetBits(GPIOC, GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_4);
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

/* 左轮控制函数 */
void Left_Wheel_Ctrl(int16_t set_speed)
{
    set_speed = (set_speed > 600) ? 600 : (set_speed < -600) ? -600 : set_speed;

    if(set_speed == 0)  // 刹车
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
    else if(set_speed > 0)  // 正转
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        PWM_SetDuty_L(set_speed);
    }
    else  // 反转
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_11);
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        PWM_SetDuty_L(-set_speed);
    }
}

/* 右轮控制函数 */
void Right_Wheel_Ctrl(int16_t set_speed)
{
    set_speed = (set_speed > 600) ? 600 : (set_speed < -600) ? -600 : set_speed;

    if(set_speed == 0)  // 刹车
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_4);
        GPIO_SetBits(GPIOD, GPIO_Pin_7);
    }
    else if(set_speed > 0)  // 正转
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_4);
        GPIO_ResetBits(GPIOD, GPIO_Pin_7);
        PWM_SetDuty_R(set_speed);
    }
    else  // 反转
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);
        GPIO_SetBits(GPIOD, GPIO_Pin_7);
        PWM_SetDuty_R(-set_speed);
    }
}
