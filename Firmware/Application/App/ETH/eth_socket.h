#ifndef __ETH_SOCKET_H
#define __ETH_SOCKET_H


#include "stm32f4xx.h"


#define TCP_TxBUF_SIZE			512
struct TCP_SendBuf_TypeDef
{
    uint8_t Buffer[TCP_TxBUF_SIZE];     // 发送暂存缓冲区
    volatile uint16_t Counter;          // 发送数据个数
};

typedef enum
{
	Client_Port,
	Server_PortA
}TCPPort_TypeDef;


void TCP_Send(TCPPort_TypeDef Port, const void *data, uint16_t size);
int TCPIP_Errno(void);

void vEthernet_Task( void *pvParameters );


#endif /* __ETH_SOCKET_H */

