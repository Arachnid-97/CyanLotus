/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Standard includes. */
#include <stdio.h>
#include <string.h>

/* Library includes. */
#include "stm32f4xx.h"

/* Private app includes. */
#include "bsp.h"
#include "bsp_clock.h"
#include "bsp_uart.h"
#include "sys_guard.h"
#include "eth_socket.h"
#include "virtual_serial.h"
#include "user_sdcard.h"
#include "user_fatfs.h"
#include "canopen_app.h"
#include "mqtt_socket.h"
#include "bsp_sdram.h"
#include "./W25Qxx/w25qxx.h"
#include "./AT24Cxx/at24cxx.h"
#include "w5500_drv.h"
#include "MPU6050.h"


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"           // 任务
#include "queue.h"          // 队列
#include "semphr.h"         // 信号量
#include "event_groups.h"   // 事件组

/* Middware includes. */
#include "cJSON.h"
#include "mb.h"

/* Task priorities. */
#define mainCREATOR_TASK_PRIORITY           ( configMAX_PRIORITIES - 1 )

/*----------------------------- End -----------------------------*/

/* Public parameters */
#if configAPPLICATION_ALLOCATED_HEAP
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__ ((section (".ext_sdram")));
#endif /* configAPPLICATION_ALLOCATED_HEAP */

cJSON_Hooks cJSON_mem = {NULL, NULL};

/* Private parameters */

/*
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为 NULL。
 */
static TaskHandle_t UserTaskCreate_Handle = NULL;/* 创建任务句柄 */

/*
 * User Private Task.
 */
static void prvUser_Task( void *pvParameters );

/*
 * Configure the clocks, GPIO and other peripherals.
 */
static void prvSetupHardware( void );

/*----------------------------- End -----------------------------*/


/************************************************
函数名称 ： SoftwareDelay_ms
功    能 ： 软件 1毫秒延时(需根据实际频率调整)
参    数 ： Count
返 回 值 ： 无
*************************************************/
void SoftwareDelay_ms(uint32_t Cnt)
{
    uint8_t i, j;

    while (Cnt--)
    {
        for (i = 2; i > 0; i--)
            for (j = 43; j > 0; j--)
                continue;
    }
}

/************************************************
函数名称 ： main
功    能 ： 主函数入口
参    数 ： 无
返 回 值 ： 无
*************************************************/
int main( void )
{
#ifdef DEBUG
    debug();
#endif

    BaseType_t xReturn = pdPASS; /* 定义一个创建信息返回值，默认为 pdPASS */

    prvSetupHardware();

    /* Start the tasks defined within this file/specific to this demo. */
    xReturn = xTaskCreate( (TaskFunction_t)prvUser_Task,            /* 任务入口函数 */
                           (const char *)"prvUser_Task",            /* 任务名字 */
                           (uint16_t)configMINIMAL_STACK_SIZE,      /* 任务栈大小 */
                           (void *)NULL,                            /* 任务入口函数参数 */
                           (UBaseType_t)mainCREATOR_TASK_PRIORITY,  /* 任务的优先级 */
                           (TaskHandle_t *)UserTaskCreate_Handle ); /* 任务控制块指针 */

    if(pdPASS == xReturn) {
        /* Start the scheduler. */
        vTaskStartScheduler();
    }

    /* Will only get here if there was not enough heap space to create the
    idle task. */
    return 0;
}

/*----------------------------- End -----------------------------*/

/************************************************
函数名称 ： prvUser_Task
功    能 ： 用户任务
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void prvUser_Task( void *pvParameters )
{
    // cJSON_mem.malloc_fn = pvPortMalloc;
    // cJSON_mem.free_fn = vPortFree;
    // cJSON_InitHooks(&cJSON_mem);

    /* User-defined private tasks */
    // xTaskCreate( vSysGuard_Task, "vSysGuard_Task", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL );

    // Ethernet_Init();
    // MQTT_Init();

    // Virtual_Serial_Init();
    // eMBInit(MB_RTU, MB_DEVICE_ADDR, 0, 9600, MB_PAR_NONE);
    // eMBEnable();

    // CANOpen_App_Init();

    // SD_test();
    // FF_Test();

    // W25Qxx_Init();
    // AT24Cxx_Init();

    // W5500_Init();

    MPU6050_Init();
    MPU6050_Test_Start();

    // uint8_t temp[] = "hello world!";

    // VSPD_SendByte(COM1, 0xa5);
    // VSPD_SendString(COM1, temp, sizeof(temp));

    // SDRAM_Test();

    while (1)
    {
        // (void)eMBPoll();
        MPU6050_Test_Poll();

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    
    // printf("prvUser_Task delete.\r\n\n");
    vTaskDelete(UserTaskCreate_Handle); // 删除自己
}

/************************************************
函数名称 ： prvSetupHardware
功    能 ： 硬件接口初始化配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void prvSetupHardware( void )
{
    RCC_ClocksTypeDef RCC_clock;

    HSI_SetSysClock(16, 336, 2, 7);

    RCC_GetClocksFreq(&RCC_clock);    
    SystemCoreClock = RCC_clock.SYSCLK_Frequency;

    /* Set the Vector Table base location at 0x08000000 + XBDDD */
#ifndef VECT_TAB_SRAM
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, XBDDD);
#endif /* VECT_TAB_SRAM */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Other peripheral configuration */
    vSetupUSART();
    // vSetupTimer();
    // vSetupParPort();
    Basic_Peripheral_Config();


	printf("\r\n ****************************************************\r\n");
	printf("       ___                    __          _     __\r\n");
	printf("      /   |  _________ ______/ /_  ____  (_)___/ /\r\n");
	printf("     / /| | / ___/ __ `/ ___/ __ \\/ __ \\/ / __  / \r\n");
	printf("    / ___ |/ /  / /_/ / /__/ / / / / / / / /_/ /  \r\n");
	printf("   /_/  |_/_/   \\__,_/\\___/_/ /_/_/ /_/_/\\__,_/   \r\n\n");
	printf("   * Description      : Application entry\r\n");
	printf("   * Release Vertion  : %s\r\n",FIRMWARE_VERSIONS);
	printf("   * Release date     : %s\r\n",__DATE__);
	printf("   * Note             : COPYRIGHT(c) 2021 Arachnid\r\n");
	printf(" ****************************************************\r\n\r\n");
    fflush(stdout);


	RAW_PRINTF(ENABLE, "\n");
	/* 上电处理 */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET){ /* 是否为 iwdg复位 */
		LOG_PRINTF(ERROR, "IWDG reset");
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET){
		LOG_PRINTF(WARNING, "NRST PIN reset");
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST) != RESET){
		/* 软复位上电 */
		LOG_PRINTF(WARNING, "Software reset");
	}
	else{
		LOG_PRINTF(WARNING, "MCU reset start");
	}
    RCC_ClearFlag();
	
	RAW_PRINTF(ENABLE, "\n");

}

/*----------------------------- End -----------------------------*/

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
#ifdef  USE_FULL_ASSERT
/* Keep the linker happy. */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("[STM Info] Wrong parameters value: file %s on line %d of %s\r\n", file, (int)line, __FUNCTION__);

    /* Infinite loop */
    for( ;; )
    {
    }
}
#endif


/*---------------------------- END OF FILE ----------------------------*/


