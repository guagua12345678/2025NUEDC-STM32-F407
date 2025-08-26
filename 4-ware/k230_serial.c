//与k230通信，使用数据包

#include "stm32f4xx.h"      // 设备头文件
#include <string.h>         // 用于memset

char Serial_RxPacket1[20];    // 定义接收数据包数组，格式为"@MSG\r\n"
uint8_t Serial_RxFlag1;      // 定义@格式接收数据包标志位

char Serial_RxPacket2[20];   // 定义接收数据包数组，格式为"#MSG\r\n"
uint8_t Serial_RxFlag2;      // 定义#格式接收数据包标志位

char Serial_RxPacket3[20];   // 定义接收数据包数组，格式为"$MSG\r\n"
uint8_t Serial_RxFlag3;      // 定义$格式接收数据包标志位

char Serial_RxPacket4[20];   // 增大缓冲区以容纳逗号分隔的两个字段
uint8_t Serial_RxFlag4;      // %格式接收标志位

char Serial_TxPacket[10];    // 定义发送数据包数组，格式为"@MSG\r\n"

void Serial_Init(void)
{
    /* 使能时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); // USART5属于APB1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE); //使能GPIOC和GPIOD时钟

    /* 初始化接收缓冲区 */
    memset(Serial_RxPacket1, 0, sizeof(Serial_RxPacket1));
    memset(Serial_RxPacket2, 0, sizeof(Serial_RxPacket2));
    memset(Serial_RxPacket3, 0, sizeof(Serial_RxPacket3));
	memset(Serial_RxPacket4, 0, sizeof(Serial_RxPacket4));
	
    /* GPIO配置 */
    GPIO_InitTypeDef GPIO_InitStruct;
    // 配置TX引脚（PC12）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5); // 复用为UART5_TX

    // 配置RX引脚（PD2）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // 上拉输入
    GPIO_Init(GPIOD, &GPIO_InitStruct);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); // 复用为UART5_RX

    /* USART配置 */
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStruct); // 使用UART5

    /* 中断配置 */
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    /* 启用UART5 */
    USART_Cmd(UART5, ENABLE);
}

void Serial_SendPacket(const char* data)
{
    uint8_t i = 0;

    /* 发送数据包头 */
    while (!(UART5->SR & USART_SR_TXE)); // 等待发送数据寄存器为空
    USART_SendData(UART5, '@');

    /* 发送数据内容 */
    while (data[i] != '\0' && i < sizeof(Serial_TxPacket) - 3) // 留空间给@、\r、\n
    {
        while (!(UART5->SR & USART_SR_TXE));
        USART_SendData(UART5, data[i]);
        i++;
    }

    /* 发送数据包尾 */
    while (!(UART5->SR & USART_SR_TXE));
    USART_SendData(UART5, '\r');

    while (!(UART5->SR & USART_SR_TXE));
    USART_SendData(UART5, '\n');

    /* 等待发送完成 */
    while (!(UART5->SR & USART_SR_TC));
}

void UART5_IRQHandler(void) 
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;
    static uint8_t PacketType = 0; // 1=@, 2=#, 3=$, 4=%

    // 新增：错误标志检测和处理
    if(USART_GetFlagStatus(UART5, USART_FLAG_ORE) || 
       USART_GetFlagStatus(UART5, USART_FLAG_FE) ||
       USART_GetFlagStatus(UART5, USART_FLAG_NE))
    {
        // 清除错误标志：顺序读SR和DR寄存器
        volatile uint16_t temp = UART5->SR;
        temp = UART5->DR;
        (void)temp;
    }

    if (USART_GetITStatus(UART5, USART_IT_RXNE)) 
	{
        uint8_t RxData = USART_ReceiveData(UART5);

        switch (RxState) {
            // 状态0：等待包头
            case 0:
                if (RxData == '@' && Serial_RxFlag1 == 0) {
                    PacketType = 1;
                    pRxPacket = 0;
                    RxState = 1;
                } 
                else if (RxData == '#' && Serial_RxFlag2 == 0) {
                    PacketType = 2;
                    pRxPacket = 0;
                    RxState = 1;
                } 
                else if (RxData == '$' && Serial_RxFlag3 == 0) {
                    PacketType = 3;
                    pRxPacket = 0;
                    RxState = 1;
                }
                else if (RxData == '%' && Serial_RxFlag4 == 0) {
                    PacketType = 4;
                    pRxPacket = 0;
                    RxState = 1;
                }
                break;

            // 状态1：接收数据内容
            case 1:
                if (RxData == '\r') {
                    RxState = 2;
                } 
                else {
                    char* buffer = NULL;
                    uint8_t maxLen = 0;
                    switch (PacketType) {
                        case 1:
                            buffer = Serial_RxPacket1;
                            maxLen = sizeof(Serial_RxPacket1) - 1;
                            break;
                        case 2:
                            buffer = Serial_RxPacket2;
                            maxLen = sizeof(Serial_RxPacket2) - 1;
                            break;
                        case 3:
                            buffer = Serial_RxPacket3;
                            maxLen = sizeof(Serial_RxPacket3) - 1;
                            break;
                        case 4:
                            buffer = Serial_RxPacket4;
                            maxLen = sizeof(Serial_RxPacket4) - 1;
                            break;
                    }

                    if (pRxPacket < maxLen) 
					{
                        buffer[pRxPacket++] = RxData;
                    } 
					else 
					{
                        RxState = 0;
                        PacketType = 0;
                    }
                }
                break;

            // 状态2：验证包尾
            case 2:
                if (RxData == '\n') {
                    switch (PacketType) {
                        case 1:
                            Serial_RxPacket1[pRxPacket] = '\0';
                            Serial_RxFlag1 = 1;
                            break;
                        case 2:
                            Serial_RxPacket2[pRxPacket] = '\0';
                            Serial_RxFlag2 = 1;
                            break;
                        case 3:
                            Serial_RxPacket3[pRxPacket] = '\0';
                            Serial_RxFlag3 = 1;
                            break;
                        case 4:
                            Serial_RxPacket4[pRxPacket] = '\0';
                            Serial_RxFlag4 = 1;
                            break;
                    }
                }
                RxState = 0;
                PacketType = 0;
                break;
        }
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    }
}

uint8_t Serial_ParsePercentPacket(char* msg1, char* msg2)
{
    if (Serial_RxFlag4 == 0) return 0;
    
    char* rxData = Serial_RxPacket4;
    uint8_t commaPos = 0;
    uint8_t len = 0;
    
    // 查找逗号位置
    while (rxData[len] != '\0' && len < sizeof(Serial_RxPacket4)-1) {
        if (rxData[len] == ',') {
            commaPos = len;
            break;
        }
        len++;
    }
    
    // 修改校验逻辑：确保逗号位置有效
    if (commaPos == 0 || commaPos >= sizeof(Serial_RxPacket4)-1) {
        Serial_RxFlag4 = 0;
        return 0;
    }
    
    // 提取第一个消息 (限制长度)
    uint8_t i;
    for (i = 0; i < commaPos && i < 9; i++) {
        msg1[i] = rxData[i];
    }
    msg1[i] = '\0';
    
    // 提取第二个消息 (限制长度)
    uint8_t j = 0;
    for (i = commaPos+1; rxData[i] != '\0' && j < 9; i++, j++) {
        msg2[j] = rxData[i];
    }
    msg2[j] = '\0';
    
    Serial_RxFlag4 = 0;
    return 1;
}
