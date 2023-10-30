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


#define UDP_RXBUF_SIZE          (1024)
#define UDP_TXBUF_SIZE          (256)

#define MAX_RXBUF_SIZE          (4*UDP_RXBUF_SIZE)

#define NET_QUEUE_TX_LENGTH     5

char remote_server_ip[IF_NAMESIZE];
int remote_server_port;

sys_mbox_t xQueueUDP_send = NULL;

static int sockfd = -1;

static void UDP_Receive_Dispose(struct sockaddr_in *Addr, char *pBuf)
{

}

static void prvUDP_Send(void *pvParameters)
{
    int ret, cnt;
    struct sockaddr_in *rmt_addr = (struct sockaddr_in *)pvParameters;
    char *buf = NULL;

    xQueueUDP_send = xQueueCreate((unsigned portBASE_TYPE)NET_QUEUE_TX_LENGTH, sizeof(UDP_TXBUF_SIZE));
    if(xQueueUDP_send == NULL)
        LOG_PRINTF(ERROR, "Create xQueueUDP_send failed...\r\n");

    buf = (char *)pvPortMalloc(sizeof(UDP_TXBUF_SIZE));
    if (buf == NULL)
    {
        LOG_PRINTF(ERROR, "udp buf mem failed\r\n");
    }

    while (1)
    {
        xQueueReceive(xQueueUDP_send, (void *)&buf, portMAX_DELAY);

        cnt = 3;
        do
        {
            /* send packets to rmt_addr */
            ret = sendto(sockfd, buf, UDP_TXBUF_SIZE, 0, (struct sockaddr *)rmt_addr, sizeof(struct sockaddr_in));
        } while (-1 == ret && cnt--);
    }
}

void vUDP_Task(void *pvParameters)
{
    sys_thread_t thread_handle;
    struct sockaddr_in bod_addr, rmt_addr;
    volatile int ret = 0;
    char *ptr = NULL;
    socklen_t fromlen = sizeof(rmt_addr);
    ip_addr_t ipaddr;

    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);

    /* set up address to connect to */
    bzero(&rmt_addr, sizeof(struct sockaddr));
    rmt_addr.sin_family = AF_INET;
    rmt_addr.sin_port = htons(DEFAULT_LOCAL_UDP_PORT);
    rmt_addr.sin_addr.s_addr = htonl(INADDR_NONE);

    thread_handle = sys_thread_new("prvUDP_Send", prvUDP_Send, &rmt_addr, DEFAULT_THREAD_STACKSIZE, uxTaskPriorityGet(NULL) + 1);
    if(thread_handle == NULL) {
        INFO_PRINTF("Create prvUDP_Send fail!\n");
    }

    ptr = (char *)mem_malloc(MAX_RXBUF_SIZE);
    if (ptr == NULL)
        LOG_PRINTF(ERROR, "udp failed to mem_malloc!\r\n");

    bzero(&bod_addr, sizeof(struct sockaddr));
    bod_addr.sin_family = AF_INET;
    bod_addr.sin_port = htons(DEFAULT_LOCAL_UDP_PORT);
    bod_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    while(1){
        /* create the socket */
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sockfd < 0){
            INFO_PRINTF("UDP create socket failed!\n");
            continue;
        }
        INFO_PRINTF("UDP create socket %d OK!\n", sockfd);

        ret = bind(sockfd, (struct sockaddr *)&bod_addr, sizeof(bod_addr)); // 绑定侦听端口及地址

        if(ret < 0){
            close(sockfd);
            sockfd = -1;
            INFO_PRINTF("UDP socket bind failed!\n");
            continue;
        }
        INFO_PRINTF("UDP bind success.\n");

        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)); // 设置接收超时
        
        const int opt = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));    // 开启 UDP 广播


        while(-1 != sockfd){
            /* reveive packets from rmt_addr, and limit a reception to UDP_RXBUF_SIZE bytes */
            ret = recvfrom(sockfd, ptr, MAX_RXBUF_SIZE, 0, (struct sockaddr *)&rmt_addr, &fromlen);
            if(ret > 0){
                int i = 0;
                if(ret % UDP_RXBUF_SIZE != 0 && ret < UDP_RXBUF_SIZE)
                    INFO_PRINTF("udp receive error len = %d\n", ret);
                else
                    do{
                        UDP_Receive_Dispose(&rmt_addr, ptr + i*UDP_RXBUF_SIZE);
                        ret -= UDP_RXBUF_SIZE;
                        i++;
                    }while(ret >= UDP_RXBUF_SIZE);
            }
            else{
                if(0 == TCPIP_Errno())
                    continue;
                break;
            }
        }

        LOG_PRINTF(WARNING, "UDP socket disconnect! error status:%d\r\n", ret);
        close(sockfd);
        vTaskDelay(800 / portTICK_RATE_MS);
    }
    // mem_free(ptr);

    // vTaskDelete(NULL);
}
