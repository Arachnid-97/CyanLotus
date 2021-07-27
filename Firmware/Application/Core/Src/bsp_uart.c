#include "bsp_uart.h"
#include <string.h>
#include "bsp_time.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf calls __io_putchar()/ __io_getchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

#define BAUDRATE_1 115200; // 波特率设置	支持的波特率：115200,19200,9600,38400,57600,1200,2400,4800
#define BAUDRATE_2 115200; // 波特率设置	支持的波特率：115200,19200,9600,38400,57600,1200,2400,4800

#ifdef DEBUG_PRINT_ON_SWO
volatile int32_t ITM_RxBuffer;
#endif /* DEBUG_PRINT_ON_SWO */

UartRx_Buff_TypeDef DEBUG_Uart_Rx;

TaskHandle_t DebugUart_Handle = NULL;

SemaphoreHandle_t MuxSem_UartPrintf = NULL;

uint8_t g_DebugPrintf_flag = 0;

static void prvDEBUG_USART_Rx_Task(void *pvParameters)
{
	memset(DEBUG_Uart_Rx.Buffer, 0, RxBUFFER_SIZE);

	while (1)
	{
		if (0 == DEBUG_Uart_Rx.Frame_flag)
		{
			vTaskSuspend(DebugUart_Handle);
			continue;
		}
		else
		{
			/* code */
			
			DEBUG_Uart_Rx.Counter = 0;
			DEBUG_Uart_Rx.Frame_flag = 0;
		}
		
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

/************************************************
函数名称 ： vSetupUSART
功    能 ： UART初始化接口
参    数 ： 无
返 回 值 ： 无
*************************************************/
void vSetupUSART(void)
{
    BaseType_t xReturn = pdFAIL;

    DEBUG_USART_Init();

    MuxSem_UartPrintf = xSemaphoreCreateMutex();
	if(MuxSem_UartPrintf == NULL)
		printf("Create Mutex Semaphore UartPrintf error!");
	else
    	xSemaphoreGive(MuxSem_UartPrintf);
	
	xReturn = xTaskCreate( prvDEBUG_USART_Rx_Task, "prvDEBUG_USART_Rx_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &DebugUart_Handle );
	if(xReturn != pdPASS){
		printf("Create prvDEBUG_USART_Rx_Task fail!");
	}
}

/************************************************
函数名称 ： DEBUG_USART_GPIO_Config
功    能 ： UART1端口配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void DEBUG_USART_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* config DEBUG USART GPIO clock */
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_AF_PIN, DEBUG_USART_GPIO_AF_MAP);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_AF_PIN, DEBUG_USART_GPIO_AF_MAP);

    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
}

/************************************************
函数名称 ： DEBUG_USART_Init
功    能 ： 调试串口初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void DEBUG_USART_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* config DEBUG USART GPIO */
    DEBUG_USART_GPIO_Config();

    /* config DEBUG USART clock */
    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

    USART_DeInit(DEBUG_USART);

    /* Enable the DEBUG USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* DEBUG USART configured as follows:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
	*/
    USART_InitStructure.USART_BaudRate = BAUDRATE_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USART, &USART_InitStructure);

    USART_Cmd(DEBUG_USART, ENABLE);

    // USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);
    // USART_ITConfig(DEBUG_USART, USART_IT_IDLE, ENABLE);

    /* Uncomment the line below if usart send flag use USART_FLAG_TC */
    // USART_ClearFlag(DEBUG_USART, USART_FLAG_TC); // Fixed first byte sending failure
}

/************************************************
函数名称 ： USART_SendByte
功    能 ： 串口字符发送
参    数 ： c ---- 发送的数据
返 回 值 ： 无
*************************************************/
void USART_SendByte(USART_TypeDef *USARTx, uint8_t c)
{
    USART_SendData(USARTx, c);

    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
        ;
}

/************************************************
函数名称 ： USART_SendString
功    能 ： 串口字符串发送
参    数 ： USARTx ---- 串口
			pData ---- 字符串
			Length ---- 长度
返 回 值 ： 无
*************************************************/
void USART_SendString(USART_TypeDef *USARTx, const uint8_t *pData, uint16_t Length)
{
    while (Length--)
    {
        USART_SendByte(USARTx, *pData);
        pData++;
    }
}

/************************************************
函数名称 ： USART_Printf
功    能 ： 串口打印输出
参    数 ： USARTx ---- 串口
			String	---- 字符串
返 回 值 ： 无
*************************************************/
void USART_Printf(USART_TypeDef *USARTx, char *String)
{
    do
    {
        USART_SendByte(USARTx, *String);
        String++;
    } while ((*String) != '\0');
}

/**
  * 函数功能: 重定向 c库函数 printf到 DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
PUTCHAR_PROTOTYPE
{
#ifdef DEBUG_PRINT_ON_UART
    /* 发送一个字节数据到 DEBUG_USART */
    USART_SendData(DEBUG_USART, (uint8_t)ch);
    /* 等待发送完毕 */
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET)
        ;
#elif defined DEBUG_PRINT_ON_SWO
    ITM_SendChar(ch);
#elif defined DEBUG_PRINT_ON_SEGGER_RTT

#endif

    return ch;
}

/**
  * 函数功能: 重定向 c库函数 getchar,scanf到 DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
GETCHAR_PROTOTYPE
{
    int ch = 0;
#ifdef DEBUG_PRINT_ON_UART
    /* 等待 DEBUG_UART输入数据 */
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET)
        ;
    ch = (int)USART_ReceiveData(DEBUG_USART);
#elif defined DEBUG_PRINT_ON_SWO
    ch = ITM_ReceiveChar();
#elif defined DEBUG_PRINT_ON_SEGGER_RTT

#endif

    return ch;
}

/************************************************************************/
/*            STM32F10x USART Interrupt Handlers                        */
/************************************************************************/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void DEBUG_USART_IRQHandler(void)
{
    if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
#if 0
        Uart1_Rx.Buffer[Uart1_Rx.Counter++] = (USART_ReceiveData(COM1_UART) & 0x7F);  // 如果使能了奇偶校验，则最高位 bit 8需要省掉
#else
        DEBUG_Uart_Rx.Buffer[DEBUG_Uart_Rx.Counter++] = (USART_ReceiveData(DEBUG_USART) & 0xFF); // 获取数据
#endif

        if (DEBUG_Uart_Rx.Counter >= RxBUFFER_SIZE)
        {
            /* Disable the COM1_UART Receive interrupt */
            // USART_ITConfig(COM1_UART, USART_IT_RXNE, DISABLE);

            DEBUG_Uart_Rx.Counter = 0;
        }

        USART_ClearITPendingBit(DEBUG_USART, USART_IT_RXNE);
    }

    if (USART_GetITStatus(DEBUG_USART, USART_IT_IDLE))
    {
        DEBUG_Uart_Rx.Frame_flag = 1;
        USART_ReceiveData(DEBUG_USART); // Clear IDLE interrupt flag bit

        xTaskResumeFromISR(DebugUart_Handle);
    }

    // if (USART_GetITStatus(COM1_UART, USART_IT_TXE) != RESET)
    // {
    //     /* Write one byte to the transmit data register */
    //     USART_SendData(COM1_UART, TxBuffer[TxCounter++]);

    //     if (TxCounter == RxBUFFER_SIZE)
    //     {
    //         /* Disable the COM1_UART Transmit interrupt */
    //         USART_ITConfig(COM1_UART, USART_IT_TXE, DISABLE);
    //     }
    // }
}


/*---------------------------- END OF FILE ----------------------------*/


