#include "eth_socket.h"
#include <string.h>
#include "bsp_uart.h"

#include "netconf.h"
#include "sys_guard.h"
#include "stm32f4x7_eth_bsp.h"

#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "FreeRTOS.h"
#include "task.h"


TaskHandle_t Server_Task_Handle = NULL;
TaskHandle_t Client_Task_Handle = NULL;


static void prvTCPIP_Task(void *pvParameters);


int TCPIP_Errno(void)
{
    if(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN){
     	// DEBUG_PRINTF("socket continue connect!\r\n");
        return 0;
    }
    else{
        if(errno == EBUSY){
            LOG_PRINTF(WARNING, "lwip during peak work!\r\n");
        }
        else if(errno == ENETDOWN || errno == ENETUNREACH 
                || errno == ENETRESET || errno == ECONNABORTED 
                    || errno == ECONNRESET){
            LOG_PRINTF(WARNING, "socket connect fault!\r\n");
        }
        LOG_PRINTF(ERROR, "socket errno status:%d\r\n", errno);
    }

    return -1;
}

void Ethernet_Init(void)
{
    /* configure ethernet (GPIOs, clocks, MAC, DMA) */ 
    ETH_BSP_Config();

    /* Initilaize the LwIP stack */
    LwIP_Init();

	sys_thread_new("prvTCPIP_Task", prvTCPIP_Task, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}

static void prvTCPIP_Task( void *pvParameters )
{
    // Server_Task_Handle = sys_thread_new("vTCPServer_Task", vTCPServer_Task, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

    Client_Task_Handle = sys_thread_new("vTCPClient_Task", vTCPClient_Task, NULL, 2 * DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

    // sys_thread_new("vTaskUDP", vUDP_Task, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	// while(1) {
	// 	vTaskDelay(1000 / portTICK_RATE_MS);
	// }
	
	vTaskDelete(NULL);
}

#if ETH_SPEED_TEST
int ETH_Speed_Detection(int sockfd, char *ptr, int len, int (*fun)(int, void *, unsigned int))
{
    static uint8_t init_flag = 1;
    static uint32_t tick1 = 0;
    uint32_t tick2;
    static uint64_t cnt = 0;
    int ret = 0;

    tick2 = sys_now();
    if (tick2 - tick1 >= configTICK_RATE_HZ * 5)
    {
        if(0 == init_flag){
            float f;
            f = (float)(cnt*configTICK_RATE_HZ/125/(tick2 - tick1));
            f /= 1000.0f;
            printf("test speed = %.4f Mbps!\n", f);
        }
        else
            init_flag = 0;

        tick1 = tick2;
        cnt = 0;
    }

    ret = fun(sockfd, ptr, len);
    if(ret){
        cnt += ret;
    }

    return ret;
}
#endif /* ETH_SPEED_TEST */

