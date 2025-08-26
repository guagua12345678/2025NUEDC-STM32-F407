//串口控制步进电机的底层实现

#include "stm32f4xx.h"
#include "stepper_ctrl.h"
#include <string.h>
#include "math.h"

// USART2 for addr 0x01 as pitch
float now_pitch = 0.0f;
	
#define PITCH_RX_BUFFER_SIZE  64
volatile uint8_t p_rx_buf[PITCH_RX_BUFFER_SIZE] = {0};
volatile uint8_t p_rx_flag = 0;
volatile uint8_t p_rx_cnt = 0;

#define PITCH_TX_BUFFER_SIZE  128
volatile uint8_t p_tx_buf[PITCH_TX_BUFFER_SIZE];
volatile uint16_t p_tx_read = 0;
volatile uint16_t p_tx_write = 0;
volatile uint16_t p_tx_cnt = 0;

void USART2_DMA_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    
    /* 1. 时钟使能 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    /* 2. GPIO配置 (PD5:TX, PD6:RX) */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    
    /* 3. USART2参数配置 */
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);
    
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    
    /* 4. DMA接收配置 (使用DMA1 Stream5) */
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)p_rx_buf;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = PITCH_RX_BUFFER_SIZE;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_InitStruct);
    
    /* 5. DMA发送配置 (使用DMA1 Stream6) */
    DMA_DeInit(DMA1_Stream6);
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(DMA1_Stream6, &DMA_InitStruct);
    
    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
    
    USART_Cmd(USART2, ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

static void USART2_DMA_StartSend(void)
{
    uint16_t len = 0;
    
    if(p_tx_read < p_tx_write) {
        len = p_tx_write - p_tx_read;
    } else if(p_tx_read > p_tx_write) {
        len = PITCH_TX_BUFFER_SIZE - p_tx_read;
    }
    
    if(len == 0) return;
    
    DMA_Cmd(DMA1_Stream6, DISABLE);
    DMA1_Stream6->NDTR = len;
    DMA1_Stream6->M0AR = (uint32_t)(p_tx_buf + p_tx_read);
    DMA1_Stream6->PAR = (uint32_t)&USART2->DR;
    
    DMA_ClearFlag(DMA1_Stream6, 
               DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | 
               DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | 
               DMA_FLAG_FEIF6);
    
    p_tx_read = (p_tx_read + len) % PITCH_TX_BUFFER_SIZE;
    p_tx_cnt -= len;
    
    DMA_Cmd(DMA1_Stream6, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

void USART2_DMA_Send(uint8_t* data, uint16_t len)
{
    uint16_t freeSpace;
    uint16_t firstPart;

    freeSpace = PITCH_TX_BUFFER_SIZE - p_tx_cnt;
    if(freeSpace >= len) 
    {
        if(p_tx_write + len <= PITCH_TX_BUFFER_SIZE) 
        {
            memcpy((void*)(p_tx_buf + p_tx_write), data, len);
            p_tx_write += len;
        }
        else 
        {
            firstPart = PITCH_TX_BUFFER_SIZE - p_tx_write;
            memcpy((void*)(p_tx_buf + p_tx_write), data, firstPart);
            memcpy((void*)p_tx_buf, data + firstPart, len - firstPart);
            p_tx_write = len - firstPart;
        }
        
        p_tx_cnt += len;
        if(DMA_GetCmdStatus(DMA1_Stream6) == DISABLE) 
        {
            USART2_DMA_StartSend();
        }
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t tmp = 0;
        tmp = USART2->SR;
        tmp = USART2->DR;
        
        p_rx_cnt = PITCH_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);
        p_rx_flag = 1;
		
		DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA1_Stream5->NDTR = PITCH_RX_BUFFER_SIZE;
		DMA1_Stream5->M0AR = (uint32_t)p_rx_buf;
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5|DMA_FLAG_HTIF5|DMA_FLAG_TEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_FEIF5);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}

void DMA1_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
    {
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        if(p_tx_cnt > 0) 
        {
            USART2_DMA_StartSend();
        }
    }
}

//发送相对/绝对位置 --- 等待完成标志
// dir ：方向        ，0为CW，其余值为CCW
// rpm ：速度(RPM)   ，范围0 - 5000RPM
// acc ：加速度      ，范围0 - 255，注意：0是直接启动
// clk ：脉冲数      ，范围0- (2^32 - 1)个

float max_pitch = 45.0f;
float min_pitch = -30.0f;

void Set_Pitch(uint8_t aac,uint16_t rpm,float theta)
{
	static uint8_t pitch_task_count = 0;
	if(pitch_task_count == 1)
	{
		if(p_rx_flag)
		{
			if(p_rx_cnt==4&&p_rx_buf[0]==0x01&&p_rx_buf[1]==0xFD&&p_rx_buf[2]==0x9F&&p_rx_buf[3]==0x6B)
			{ 
				p_rx_flag = 0;
				now_pitch = theta;
				pitch_task_count = 0; 
			}
		}
	}
	if(pitch_task_count == 0)
	{
		if(theta > max_pitch) theta = max_pitch;
		else if(theta < min_pitch) theta = min_pitch;
		uint8_t dir = 0;
		uint32_t clk = fabs(theta)*142.22222f;
		if(theta > 0) dir = 0; else dir = 1;
		Emm_V5_Pos_Control(0x01,dir,rpm,aac,clk,1,0);
        pitch_task_count = 1;
	}
}
