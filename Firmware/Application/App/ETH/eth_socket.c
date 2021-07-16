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

void vEthernet_Task( void *pvParameters )
{
    /* configure ethernet (GPIOs, clocks, MAC, DMA) */ 
    ETH_BSP_Config();

    /* Initilaize the LwIP stack */
    LwIP_Init();
	
	
	RAW_PRINTF(ENABLE, "\n");
	/* 上电处理 */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET){ /* 是否为 iwdg复位 */
		LOG_PRINTF(ERROR, "iwdg reset");
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET){
		LOG_PRINTF(WARNING, "NRST PIN reset");
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET){
		/* 软复位上电 */
		LOG_PRINTF(WARNING, "software reset");
	}
	else{
		LOG_PRINTF(WARNING, "MCU reset start");
	}
    RCC_ClearFlag();
	
	RAW_PRINTF(ENABLE, "\n");
	
	sys_thread_new("prvTCPIP_Task", prvTCPIP_Task, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

    vTaskDelete(NULL);
}

static void prvTCPIP_Task( void *pvParameters )
{
	//任务的优先级数值越小，那么此任务的优先级越低，空闲任务的优先级是 0
	// Server_Task_Handle = sys_thread_new("vTCPServer_Task", vTCPServer_Task, NULL, 512, DEFAULT_THREAD_PRIO);

	// Client_Task_Handle = sys_thread_new("vTCPClient_Task", vTCPClient_Task, NULL, 1408, DEFAULT_THREAD_PRIO);

	// sys_thread_new("vTaskUDP", vTaskUDP, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);

	SysGuard_Reg(SG_TCP, "TCP Link Guard", 2, SysGuard_TCP_Callback);
	// SysGuard_Start(SG_TCP);
	
	
	vTaskDelete(NULL);
}
