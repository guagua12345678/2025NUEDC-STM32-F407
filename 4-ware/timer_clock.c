//任务耗时计时器

#include "stm32f4xx.h"

#define MAX_TIMER_NUM  10

volatile uint32_t tim7_overflow_count = 0;
static uint64_t start_ticks[MAX_TIMER_NUM] = {0};

void TimClock_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;   // 84MHz/84 = 1MHz (1us)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;      // 最大计数值65535
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
    TIM_Cmd(TIM7, ENABLE);
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
    {
        tim7_overflow_count++;
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
}

static uint64_t get_current_tick(void)
{
    uint32_t overflow1, overflow2;
    uint16_t counter;
    do 
	{
        overflow1 = tim7_overflow_count;
        counter = TIM_GetCounter(TIM7);
        overflow2 = tim7_overflow_count;
    } 
	while (overflow1 != overflow2);
    
    return ((uint64_t)overflow1 << 16) + counter;
}

void TimClock_Start(uint8_t num)
{
    if (num < MAX_TIMER_NUM) 
		start_ticks[num] = (uint32_t)get_current_tick();
}


float TimeClock_Stop(uint8_t num)
{
    if (num >= MAX_TIMER_NUM) return 0.0f;
    uint64_t end_tick = get_current_tick();
    uint64_t start_tick = start_ticks[num];
    uint64_t elapsed_us = end_tick - start_tick;
    return (float)elapsed_us / 1000.0f;
}
