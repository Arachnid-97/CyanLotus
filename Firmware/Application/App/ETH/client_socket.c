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
#include "cJSON.h"

#include "FreeRTOS.h"
#include "task.h"


#define CLIENT_RXBUF_SIZE       (4*1024)


int SocketClient;

sys_mbox_t xQueueTCP_Client = NULL;

static void test_send(void);


static int Client_Connet(int *pSockfd)
{
    struct sockaddr_in servaddr;

    if (!netif_is_link_up(&xnetif))
    { /* waiting for link up */
        return 0;
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
#if 0
    /* 远程 UDP 获取 */
    servaddr.sin_port = htons(remote_server_port);
    servaddr.sin_addr.s_addr = inet_addr(remote_server_ip);
#else
    /* 读取固定配置 */
    servaddr.sin_port = htons(DEFAULT_REMOTE_TCP_PORT);
    servaddr.sin_addr.s_addr = inet_addr(DEFAULT_REMOTE_TCP_IP);
#endif

    /* 创建 socket, 准备连接服务器 */
    *pSockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (*pSockfd < 0)
    {
        INFO_PRINTF("TCP Client create socket failed!\r\n");
        return -1;
    }
    else
        INFO_PRINTF("TCP Client create socket OK!\r\n");

    /* 发出连接请求 */
    if (connect(*pSockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0)
    {
        close(*pSockfd);
        INFO_PRINTF("TCP Client connect %s : %d failed!\r\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));
        return -2;
    }
    else
        INFO_PRINTF("TCP Client connect OK (%s : %d)\r\n", inet_ntoa(servaddr.sin_addr), ntohs(servaddr.sin_port));

    return 1;
}

static void TCP_Connect(void)
{
    const int opt = 1;
    // const int send_timeout = 800;
    // const int recv_timeout = 2000;
    const int nodelay_flag = 1;

    int error = 0;
    
    error = 0;
    do {
        vTaskDelay(4000 / portTICK_RATE_MS); // 连接服务器不用太频繁
        // sys_mutex_lock(&MuxSem_TCPconnect);
        error = Client_Connet(&SocketClient);
        // sys_mutex_unlock(&MuxSem_TCPconnect);
    } while(error <= 0);
    // 客户端已连接: 准备接收数据

    /* 应用保活特性 */
    setsockopt(SocketClient, SOL_SOCKET, SO_KEEPALIVE, (char *)&opt, sizeof(int));
    /* 收发超时 */
    // setsockopt(SocketClient, SOL_SOCKET, SO_SNDTIMEO, (char*)&send_timeout, sizeof(int));
    // setsockopt(SocketClient, SOL_SOCKET, SO_RCVTIMEO, (char*)&recv_timeout, sizeof(int));
    /* 无延时发送 */
    setsockopt(SocketClient, IPPROTO_TCP, TCP_NODELAY, (void *)&nodelay_flag, sizeof(nodelay_flag));

    vTaskDelay(1000 / portTICK_RATE_MS); // 等待服务器响应
}

static void Client_Receive_Dispose(char *pBuf, int Len)
{

}

static void prvClient_Send(void *pvParameters)
{
    struct TCP_SendBuf_TypeDef *buf = NULL;
    int ret = 0;

    /* lwip 数据发送邮箱 */
    xQueueTCP_Client = xQueueCreate((unsigned portBASE_TYPE)3, sizeof(struct TCP_SendBuf_TypeDef));
    if(xQueueTCP_Client == NULL)
        LOG_PRINTF(ERROR, "Create xQueueTCP_Client failed...\r\n");

    buf = (struct TCP_SendBuf_TypeDef *)pvPortMalloc(sizeof(struct TCP_SendBuf_TypeDef));
    if (buf == NULL)
    {
        LOG_PRINTF(ERROR, "client buf mem failed\r\n");
    }

    while(1)
    {
        {
            xQueueReceive(xQueueTCP_Client, (void *)buf, portMAX_DELAY);

            // ret = write(SocketClient, (void *)buf->Buffer, (size_t)buf->Counter);
            ret = send(SocketClient, (void *)buf->Buffer, (size_t)buf->Counter, MSG_DONTWAIT);		// 非阻塞
            if(ret != buf->Counter) {
                if(ret < 0) {
                    if(0 == TCPIP_Errno())
                        continue;
                }
                else if(ret > 0) {
                    LOG_PRINTF(WARNING, "TCP Client Incomplete data transmission! Data Remaining:%d\r\n", buf->Counter - ret);
                    continue;
                }
                
                LOG_PRINTF(WARNING, "TCP Client socket disconnect! error status:%d\r\n", ret);
                close(SocketClient);
                TCP_Connect();
            }
        }
    }
    // vPortFree(ptr);

    // vTaskDelete(NULL);
}

void vTCPClient_Task(void *pvParameters)
{
    sys_thread_t thread_handle;
    volatile int ret = 0;
    char *ptr = NULL;

    thread_handle = sys_thread_new("prvClient_Send", prvClient_Send, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
    if(thread_handle == NULL) {
        INFO_PRINTF("Create prvClient_Send fail!\n");
    }

    // 分配接收缓存
    ptr = (char *)mem_malloc(CLIENT_RXBUF_SIZE);
    if (ptr == NULL)
        LOG_PRINTF(ERROR, "client failed to mem_malloc!\r\n");

    while (1)
    {
        TCP_Connect();
        test_send();
        do {
#if ETH_SPEED_TEST
            ret = ETH_Speed_Detection(SocketClient, ptr, CLIENT_RXBUF_SIZE, lwip_write);
#else
            ret = read(SocketClient, ptr, CLIENT_RXBUF_SIZE);
            // ret = recv(SocketClient, ptr, CLIENT_RXBUF_SIZE, MSG_DONTWAIT);		// 非阻塞

            if (ret > 0)
            {
                // printf("%s\r\n", ptr);
                Client_Receive_Dispose(ptr, ret);
            }
            else if(ret < 0) {
                if(0 == TCPIP_Errno())
                    continue;
            }
#endif /* ETH_SPEED_TEST */
        }while(ret > 0);

        LOG_PRINTF(WARNING, "TCP Client socket disconnect! error status:%d\r\n", ret);
        close(SocketClient);
    }
    // mem_free(ptr);

    // vTaskDelete(NULL);
}

static void test_send(void)
{
    cJSON *jtest = NULL;
    cJSON *jfile = NULL;
    cJSON *jissue = NULL;
    cJSON *jdate = NULL;
    char *str = NULL;

    /* 创建一个 JSON 格式的主对象(主链表头结点) */
    jtest = cJSON_CreateObject();

    /* 追加字符串类型的 JSON数据到主对象中(添加一个链表节点) */
    cJSON_AddStringToObject(jtest, "project name", "CyanLotus");
    cJSON_AddStringToObject(jtest, "version", FIRMWARE_VERSIONS);

    /* 追加一个对象到主对象中(添加一个链表节点) */
    jfile = cJSON_AddObjectToObject(jtest, "development platform");
    /* 往追加的对象添加对应的值 */
    cJSON_AddStringToObject(jfile, "editor", "vscode");
    cJSON_AddStringToObject(jfile, "compiler", "gcc");
    cJSON_AddStringToObject(jfile, "debugger", "openocd");

    /* 创建一个 JSON 格式的数组(另一个链表头结点) */
    jissue = cJSON_CreateArray();
    /* 创建相应的值并把这些值添加到数组里 */
    jdate = cJSON_CreateString(__DATE__);
    cJSON_AddItemToArray(jissue, jdate);
    /* 把已经填好的数据的数组插入到主对象中 */
    cJSON_AddItemToObject(jtest, "released", jissue);

    /* 追加一个布尔类型的 JSON 数据到主对象中(添加一个链表节点) */
    if(USING_RTOS)
        cJSON_AddTrueToObject(jtest, "using rtos");
    else
        cJSON_AddFalseToObject(jtest, "using rtos");

    /* 打印 JSON对象(整条链表)的所有数据 */
    str = cJSON_Print(jtest);

    send(SocketClient, (void *)str, (size_t)strlen(str), MSG_DONTWAIT);

    cJSON_free(str);
    cJSON_Delete(jtest);
}