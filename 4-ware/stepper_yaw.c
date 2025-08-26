//串口控制步进电机的底层实现

#include "stm32f4xx.h"
#include "stepper_ctrl.h"
#include <string.h>
#include "math.h"

//usart3 for addr 0x02 as yaw
float now_yaw = 0.0f;

// 定义接收缓冲区大小
#define YAW_RX_BUFFER_SIZE  64
volatile uint8_t y_rx_buf[YAW_RX_BUFFER_SIZE] = {0};
volatile uint8_t y_rx_flag = 0;
volatile uint8_t y_rx_cnt = 0;

// 添加发送缓冲区及控制变量
#define YAW_TX_BUFFER_SIZE  128
volatile uint8_t y_tx_buf[YAW_TX_BUFFER_SIZE];
volatile uint16_t y_tx_read = 0;
volatile uint16_t y_tx_write = 0;
volatile uint16_t y_tx_cnt = 0;

void USART3_DMA_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
	
    /* 1. 时钟使能 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    /* 2. GPIO配置 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 引脚复用映射 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    
    /* 3. USART3参数配置 */
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStruct);
    
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    
    /* 4. DMA接收配置 */
    DMA_DeInit(DMA1_Stream1);
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)y_rx_buf;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = YAW_RX_BUFFER_SIZE;
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
    DMA_Init(DMA1_Stream1, &DMA_InitStruct);
    
    /* 5. DMA发送配置 */
    DMA_DeInit(DMA1_Stream3);
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = 0;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_Init(DMA1_Stream3, &DMA_InitStruct);
    
    // 使能DMA发送完成中断
    DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    
    USART_Cmd(USART3, ENABLE);
    DMA_Cmd(DMA1_Stream1, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

// 启动DMA发送（内部函数）
static void USART3_DMA_StartSend(void)
{
    uint16_t len = 0;
    
    // 计算连续可发送数据长度
    if(y_tx_read < y_tx_write) {
        // 数据在缓冲区中连续
        len = y_tx_write - y_tx_read;
    } else if(y_tx_read > y_tx_write) {
        // 数据在缓冲区尾部+头部
        len = YAW_TX_BUFFER_SIZE - y_tx_read;
    }
    
    if(len == 0) return;
    
    // 配置DMA
    DMA_Cmd(DMA1_Stream3, DISABLE);
    DMA1_Stream3->NDTR = len;
    DMA1_Stream3->M0AR = (uint32_t)(y_tx_buf + y_tx_read);
    DMA1_Stream3->PAR = (uint32_t)&USART3->DR;
    
    // 清除标志位
    DMA_ClearFlag(DMA1_Stream3, 
               DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | 
               DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | 
               DMA_FLAG_FEIF3);
    
    // 更新缓冲区索引
    y_tx_read = (y_tx_read + len) % YAW_TX_BUFFER_SIZE;
    y_tx_cnt -= len;
    
    // 启动DMA传输
    DMA_Cmd(DMA1_Stream3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

// 非阻塞发送函数
void USART3_DMA_Send(uint8_t* data, uint16_t len)
{
    uint16_t freeSpace;
    uint16_t firstPart;

    freeSpace = YAW_TX_BUFFER_SIZE - y_tx_cnt;
    if(freeSpace >= len) 
	{
		// 写入缓冲区
		if(y_tx_write + len <= YAW_TX_BUFFER_SIZE) 
		{
			memcpy((void*)(y_tx_buf + y_tx_write), data, len);
			y_tx_write += len;
		}
		else 
		{
			firstPart = YAW_TX_BUFFER_SIZE - y_tx_write;
			memcpy((void*)(y_tx_buf + y_tx_write), data, firstPart);
			memcpy((void*)y_tx_buf, data + firstPart, len - firstPart);
			y_tx_write = len - firstPart;
		}
		
		y_tx_cnt += len;

		if(DMA_GetCmdStatus(DMA1_Stream3) == DISABLE) USART3_DMA_StartSend();
	}
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t tmp = 0;
        tmp = USART3->SR;
        tmp = USART3->DR;
        
        y_rx_cnt = YAW_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
        y_rx_flag = 1;
		
		DMA_Cmd(DMA1_Stream1, DISABLE); // 禁用DMA
		DMA1_Stream1->NDTR = YAW_RX_BUFFER_SIZE; // 重置传输数据量
		DMA1_Stream1->M0AR = (uint32_t)y_rx_buf; // 重置内存地址
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1|DMA_FLAG_HTIF1|DMA_FLAG_TEIF1|DMA_FLAG_DMEIF1|DMA_FLAG_FEIF1);
		DMA_Cmd(DMA1_Stream1, ENABLE); // 重新使能DMA
    }
}

void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        if(y_tx_cnt > 0) USART3_DMA_StartSend();
    }
}

//发送相对/绝对位置  ---  等待完成标志 --- 发送读取位置 --- 等待返回位置
// dir ：方向        ，0为CW，其余值为CCW
// rpm ：速度(RPM)   ，范围0 - 5000RPM
// acc ：加速度      ，范围0 - 255，注意：0是直接启动
// clk ：脉冲数      ，范围0- (2^32 - 1)个
//设置的是云台的绝对值，可以重复设置相同的值

float max_yaw = 900.0f;
float min_yaw = -900.0f;

void Set_Yaw(uint8_t aac,uint16_t rpm,float theta)
{
	static uint8_t yaw_task_count = 0;
	if(yaw_task_count == 1)
	{
		if(y_rx_flag)
		{
			if(y_rx_cnt==4&&y_rx_buf[0]==0x02&&y_rx_buf[1]==0xFD&&y_rx_buf[2]==0x9F&&y_rx_buf[3]==0x6B)
			{ 
				y_rx_flag = 0;
				now_yaw = theta;
				yaw_task_count = 0; 
			}
		}
	}
	if(yaw_task_count == 0)
	{
		if(theta > max_yaw) theta = max_yaw;
		else if(theta < min_yaw) theta = min_yaw;

		uint8_t dir = 0;
		uint32_t clk = fabs(theta)*142.22222f;
		if(theta > 0) dir = 0; else dir = 1;
		Emm_V5_Pos_Control(0x02,dir,rpm,aac,clk,1,0);
		yaw_task_count = 1;
	}
}
