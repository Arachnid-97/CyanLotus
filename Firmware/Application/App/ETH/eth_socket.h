#ifndef __ETH_SOCKET_H
#define __ETH_SOCKET_H


#include "stm32f4xx.h"


#define ETH_SPEED_TEST          0

#define IF_NAMESIZE             16

#define TCP_TxBUF_SIZE			1540
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

extern char remote_server_ip[IF_NAMESIZE];
extern int remote_server_port;


void TCP_Send(TCPPort_TypeDef Port, const void *data, uint16_t size);
int TCPIP_Errno(void);

void Ethernet_Init(void);


void vTCPClient_Task(void *pvParameters);
void vTCPServer_Task(void *pvParameters);
void vUDP_Task(void *pvParameters);

#if ETH_SPEED_TEST
int ETH_Speed_Detection(int sockfd, char *ptr, int len, int (*fun)(int, void *, unsigned int));
#endif /* ETH_SPEED_TEST */


#endif /* __ETH_SOCKET_H */

