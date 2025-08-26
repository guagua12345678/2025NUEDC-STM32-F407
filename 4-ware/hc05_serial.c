//hc05蓝牙串口配合vofa+上位机调参（just float模式）

#include "stm32f4xx.h"
#include <string.h>    

void hc05_init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
    GPIO_InitTypeDef  gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9 , GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    USART_InitTypeDef usart;
    USART_StructInit(&usart);
    usart.USART_BaudRate   = 9600;     
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits   = USART_StopBits_1;
    usart.USART_Parity     = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_Cmd(USART1, ENABLE);
}

static inline void USART1_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

static void USART1_SendBuf(const uint8_t *buf, uint16_t len)
{
    while (len--) USART1_SendByte(*buf++);
}

typedef union {float f; uint8_t b[4];} fbytes_t;

void VOFA_Send(float a, float b, float c, float d)
{
    uint8_t frame[20];
    fbytes_t fb;

    fb.f = a;
    memcpy(&frame[0], fb.b, 4);

    fb.f = b;
    memcpy(&frame[4], fb.b, 4);

    fb.f = c;
    memcpy(&frame[8], fb.b, 4);
	
	fb.f = d;
    memcpy(&frame[12], fb.b, 4);

    frame[16] = 0x00;
    frame[17] = 0x00;
    frame[18] = 0x80;
    frame[19] = 0x7F;

    USART1_SendBuf(frame, sizeof(frame));
}
