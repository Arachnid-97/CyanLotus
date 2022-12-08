#ifndef __BSP_UART_H
#define __BSP_UART_H


#include "stm32f4xx.h"
#include <stdio.h>

#define DEBUG_PRINT_ON_UART
// #define DEBUG_PRINT_ON_SWO

#define DEBUG_USART                 USART1
#define DEBUG_USART_CLK             RCC_APB2Periph_USART1
#define DEBUG_USART_CLOCK_FUN(x, y) RCC_APB2PeriphClockCmd(x, y)

#define DEBUG_USART_GPIO_CLK        RCC_AHB1Periph_GPIOB
#define DEBUG_USART_GPIO_CLOCK_FUN(x, y) RCC_AHB1PeriphClockCmd(x, y)

#define DEBUG_USART_GPIO_AF_MAP     GPIO_AF_USART1
#define DEBUG_USART_TX_AF_PIN       GPIO_PinSource6
#define DEBUG_USART_RX_AF_PIN       GPIO_PinSource7

#define DEBUG_USART_TX_GPIO_PORT    GPIOB
#define DEBUG_USART_TX_GPIO_PIN     GPIO_Pin_6
#define DEBUG_USART_RX_GPIO_PORT    GPIOB
#define DEBUG_USART_RX_GPIO_PIN     GPIO_Pin_7

#define DEBUG_USART_IRQ             USART1_IRQn
#define DEBUG_USART_IRQHandler      USART1_IRQHandler


#define TxBUFFER_SIZE               100
#define RxBUFFER_SIZE               0xFF


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern SemaphoreHandle_t MuxSem_UartPrintf;
extern uint8_t g_DebugPrintf_flag;
#define RAW_PRINTF(type,fmt,arg...)     do{\
                                            if(type || g_DebugPrintf_flag){\
                                                xSemaphoreTake(MuxSem_UartPrintf, portMAX_DELAY);\
                                                printf(""fmt"",##arg);\
                                                xSemaphoreGive(MuxSem_UartPrintf);}\
                                        }while(0)

#ifndef USING_DEBUG
#define USING_DEBUG                0
#endif
#define DEBUG_PRINTF(fmt,arg...)        do{\
                                            if(USING_DEBUG || g_DebugPrintf_flag){\
                                                xSemaphoreTake(MuxSem_UartPrintf, portMAX_DELAY);\
                                                printf("<<-DEBUG->> %s > "fmt"",__FUNCTION__, ##arg);\
                                                xSemaphoreGive(MuxSem_UartPrintf);}\
                                        }while(0)

#ifndef USING_INFO
#define USING_INFO                1
#endif
#define INFO_PRINTF(fmt,arg...)         do{\
                                            if(USING_INFO || g_DebugPrintf_flag){\
                                                xSemaphoreTake(MuxSem_UartPrintf, portMAX_DELAY);\
                                                printf("<<-INFO->> "fmt"",##arg);\
                                                xSemaphoreGive(MuxSem_UartPrintf);}\
                                        }while(0)

#ifndef USING_LOG
#define USING_LOG                1
#endif
#define LOG_PRINTF(type,fmt,arg...)     do{\
                                            if(USING_LOG || g_DebugPrintf_flag){\
                                                xSemaphoreTake(MuxSem_UartPrintf, portMAX_DELAY);\
                                                printf("<<-%s->> %s > "fmt"",#type,__FUNCTION__, ##arg);\
                                                xSemaphoreGive(MuxSem_UartPrintf);}\
                                        }while(0)

typedef struct
{
    uint8_t Buffer[RxBUFFER_SIZE];  // 接收暂存缓冲区
    volatile uint16_t Counter;      // 接收数据个数
    volatile uint8_t Frame_flag;    // 一帧完成标志
}UartRx_Buff_TypeDef;
 
typedef struct
{
    uint8_t Buffer[TxBUFFER_SIZE];  // 发送暂存缓冲区
    volatile uint16_t Counter;      // 发送数据个数
    USART_TypeDef* COMx;            // 串口号
}UartTx_Buff_TypeDef;

extern UartRx_Buff_TypeDef DEBUG_Uart_Rx;

void vSetupUSART( void );
void DEBUG_USART_Init(void);
void USART_SendByte(USART_TypeDef *USARTx, uint8_t c);
void USART_SendString(USART_TypeDef *USARTx, const uint8_t *pData, uint16_t Length);
void USART_Printf(USART_TypeDef *USARTx, char *String);


#endif /* __BSP_UART_H */


/*---------------------------- END OF FILE ----------------------------*/


