#include "stm32f4xx.h"                  // Device header

#ifndef __K230_SERIAL_H__
#define __K230_SERIAL_H__

extern char Serial_RxPacket1[10];    // 定义接收数据包数组，格式为"@MSG\r\n"
extern uint8_t Serial_RxFlag1;      // 定义@格式接收数据包标志位

extern char Serial_RxPacket2[10];   // 定义接收数据包数组，格式为"#MSG\r\n"
extern uint8_t Serial_RxFlag2;      // 定义#格式接收数据包标志位

extern char Serial_RxPacket3[10];   // 定义接收数据包数组，格式为"$MSG\r\n"
extern uint8_t Serial_RxFlag3;      // 定义$格式接收数据包标志位

extern char Serial_RxPacket4[20];   // 增大缓冲区以容纳逗号分隔的两个字段
extern uint8_t Serial_RxFlag4;      // %格式接收标志位

extern char Serial_TxPacket[10];    // 定义发送数据包数组，格式为"@MSG\r\n"

void Serial_Init(void);
void Serial_SendPacket(const char* data);
uint8_t Serial_ParsePercentPacket(char* msg1, char* msg2);

#endif
