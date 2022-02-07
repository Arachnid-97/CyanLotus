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
#include "event_groups.h"


TaskHandle_t Server_Task_Handle = NULL;
TaskHandle_t Client_Task_Handle = NULL;


static void prvTCPIP_Task(void *pvParameters);


static void SysGuard_TCP_Callback(void)
{
	/* 软复位前处理 */
	do{
		
		vTaskDelay(2000 / portTICK_RATE_MS);
	}while(0);
	
	
	NVIC_SystemReset();
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
	// Server_Task_Handle = sys_thread_new("vTCPServer_Task", vTCPServer_Task, NULL, 512, DEFAULT_THREAD_PRIO);

	// Client_Task_Handle = sys_thread_new("vTCPClient_Task", vTCPClient_Task, NULL, 1408, DEFAULT_THREAD_PRIO);

	// sys_thread_new("vTaskUDP", vTaskUDP, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	SysGuard_Reg(SG_TCP, "TCP Link Guard", 2, SysGuard_TCP_Callback);
	SysGuard_Start(SG_TCP);
	
	while(1){
		vTaskDelay(1000 / portTICK_RATE_MS);
        // TCPClient_CallBack();

		/* 线程喂狗 */
	    SysGuard_Online(SG_TCP);
	}
	
	// vTaskDelete(NULL);
}
