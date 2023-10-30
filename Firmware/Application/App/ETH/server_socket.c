#include "eth_socket.h"

#include "lwip/mem.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/sys.h"
#include <stdlib.h>
#include <string.h>

#include "bsp_uart.h"
#include "sys_guard.h"
#include "netconf.h"

#include "FreeRTOS.h"
#include "task.h"


#define SERVER_RXBUF_SIZE       (4*1024)

int SocketServer;


static void Server_Build(int *pSockfd)
{
	struct sockaddr_in servaddr;
	int sockfd;
	int err;

	do	{
		if (!netif_is_link_up(&xnetif))
        { /* waiting for link up */
			vTaskDelay(800 / portTICK_RATE_MS);
            continue;
        }
		else
			break;
	}while(1);	
	
	// 在 server 中的 client 连接套接字
	do	{
		sockfd = socket(AF_INET,  SOCK_STREAM, 0);
		if (sockfd < 0) {
			INFO_PRINTF("TCP Server create socket failed!\r\n");
			vTaskDelay(1500 / portTICK_RATE_MS);
		}	
	}while(sockfd < 0);
	INFO_PRINTF("TCP Server create socket OK!\r\n");
	
	bzero(&servaddr, sizeof(servaddr));	
	servaddr.sin_family = AF_INET;	
	servaddr.sin_port = htons(DEFAULT_LOCAL_TCP_PORT);	
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 	

	do	{
		err = bind(sockfd, (struct sockaddr*)&servaddr, sizeof(struct sockaddr));
		if (err != 0) {
			INFO_PRINTF("TCP Server bind socket failed!\r\n");
			vTaskDelay(1500 / portTICK_RATE_MS);
		}
	}while(err != 0);
	INFO_PRINTF("TCP Server bind socket OK!\r\n");
	
	do	{
		err = listen(sockfd, MEMP_NUM_TCP_PCB_LISTEN);
		if (err != 0) 		{
			INFO_PRINTF("TCP Server listen failed!\r\n");
			vTaskDelay(1500 / portTICK_RATE_MS);
		}	
	}while(err != 0);
	INFO_PRINTF("TCP Server listen OK!\r\n");
}

static int Server_Accept(int *pSocket)
{
	struct sockaddr_in conn_addr;
	socklen_t addr_len;
	int sockfd;

	/* 等待客户端连接 */
	addr_len = sizeof(struct sockaddr_in);
	do{
		sockfd = accept(*pSocket, (struct sockaddr*)&conn_addr, &addr_len);
		if(sockfd < 0) {
			vTaskDelay(300 / portTICK_RATE_MS);
		}
		else
			break;
	}while(1);
	INFO_PRINTF("TCP Server connect OK (%s : %d)\r\n", inet_ntoa(conn_addr.sin_addr), ntohs(conn_addr.sin_port));

	return sockfd;
}

static void Server_Receive_Dispose(char *pBuf, int Len)
{

}

void vTCPServer_Task(void *pvParameters)
{
    volatile int ret = 0;
	char *ptr = NULL;
	int sockfd;

    // 分配接收缓存
    ptr = (char *)mem_malloc(SERVER_RXBUF_SIZE);
    if (ptr == NULL)
    {
        LOG_PRINTF(ERROR, "server failed to mem_malloc!\r\n");
    }

	Server_Build(&SocketServer);

	while(1) {
		sockfd = Server_Accept(&SocketServer);

        do {
#if ETH_SPEED_TEST
            ret = ETH_Speed_Detection(sockfd, ptr, SERVER_RXBUF_SIZE, lwip_read);
#else
        	ret = read(sockfd, ptr, SERVER_RXBUF_SIZE);
            if (ret > 0)
            {
                Server_Receive_Dispose(ptr, ret);
            }
            else if(ret < 0) {
                if(0 == TCPIP_Errno())
                    continue;
            }
#endif /* ETH_SPEED_TEST */
        }while(ret > 0);
        
        LOG_PRINTF(WARNING, "TCP Server socket disconnect! error status:%d\r\n", ret);
		close(sockfd);
	}
    // mem_free(ptr);

    // vTaskDelete(NULL);
}

