#include "rs232.h"

#if USING_RS232

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#define DEFAULT_BAUDRATE        9600 // 波特率设置  支持的波特率：115200,19200,9600,38400,57600,1200,2400,4800

#define BUFFER_SIZE             64

RS_Buff_TypeDef RS232_Rx;
TaskHandle_t RS232_Rx_Handle = NULL;


static void RS232_USART_GPIO_Config(void);

static void prvRS232_USART_Rx_Task(void *pvParameters)
{
	memset(RS232_Rx.pBuffer, 0, BUFFER_SIZE);

	while (1)
	{
		if (0 == RS232_Rx.Frame_flag)
		{
			vTaskSuspend(RS232_Rx_Handle);
			continue;
		}
		else
		{
			/* code */
			
			RS232_Rx.Counter = 0;
			RS232_Rx.Frame_flag = 0;
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
void vSetupRS232(void)
{
    BaseType_t xReturn = pdFAIL;

    RS232_USART_Init(DEFAULT_BAUDRATE, RS_PAR_NONE);

    RS232_Rx.pBuffer = pvPortMalloc(BUFFER_SIZE);

#if USING_RS232_DMA
    RS232_DMA_Config(NULL, (void *)RS232_Rx.pBuffer, BUFFER_SIZE, BUFFER_SIZE);
#endif /* USING_RS232_DMA */

	xReturn = xTaskCreate( prvRS232_USART_Rx_Task, "prvRS232_USART_Rx_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &RS232_Rx_Handle );
	if(xReturn != pdPASS){
		printf("Create prvRS232_USART_Rx_Task fail!");
	}
}

/************************************************
函数名称 ： RS232_USART_Init
功    能 ： RS232串口初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void RS232_USART_Init(uint32_t BaudRate, RSParity Parity)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint16_t parity;

    /* config RS232 USART GPIO */
    RS232_USART_GPIO_Config();

    /* config RS232 USART clock */
    RS232_USART_CLOCK_FUN(RS232_USART_CLK, ENABLE);

    USART_DeInit(RS232_USART);

    /* Enable the RS232 USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RS232_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* RS232 USART configured as follows:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
	*/
    switch (Parity)
    {
    case RS_PAR_ODD:
        parity = USART_Parity_Odd;
        break;
    case RS_PAR_EVEN:
        parity = USART_Parity_Even;
        break;
    default:
        parity = USART_Parity_No;
        break;
    }
    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(RS232_USART, &USART_InitStructure);

    USART_Cmd(RS232_USART, ENABLE);

    USART_ITConfig(RS232_USART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(RS232_USART, USART_IT_IDLE, ENABLE);

    /* Uncomment the line below if usart send flag use USART_FLAG_TC */
    // USART_ClearFlag(RS232_USART, USART_FLAG_TC); // Fixed first byte sending failure
}

/************************************************
函数名称 ： RS232_DMA_Config
功    能 ： RS232 DMA配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
void RS232_DMA_Config(void *pTxBuf, void *pRxBuf, uint16_t TxSize, uint16_t RxSize)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 DMA时钟 */
    RCC_AHB1PeriphClockCmd(RS232_USART_DMA_CLK, ENABLE);

    /* DMA基本配置 */
    DMA_InitStructure.DMA_Channel             = RS232_USART_DMA_CHANNEL;        /* 配置 DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(RS232_USART->DR)); /* 源 */
    DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;      /* 外设地址是否自增 */
    DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;           /* 内存地址是否自增 */
    DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;    /* 目的数据带宽 */
    DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;        /* 源数据宽度 */
    DMA_InitStructure.DMA_Mode                = DMA_Mode_Normal;                /* 单次传输模式/循环传输模式 */
    DMA_InitStructure.DMA_Priority            = DMA_Priority_Medium;            /* DMA优先级 */
    DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;           /* FIFO模式/直接模式 */
    DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull;     /* FIFO大小 */
    DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;         /* 单次传输 */
    DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;

    if(NULL != pTxBuf)
    {
        DMA_DeInit(RS232_USARTTx_DMA_STREAM);

        /* 等待 DMA可配置 */
        while (DMA_GetCmdStatus(RS232_USARTTx_DMA_STREAM) != DISABLE);

        /* USARTx TX DMA2 Channel (triggered by USARTx Rx event) Config */
        DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)pTxBuf;               /* 目的 */
        DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;     /* 方向 */
        DMA_InitStructure.DMA_BufferSize          = TxSize;                         /* 长度 */                  

        /* 配置DMA */
        DMA_Init(RS232_USARTTx_DMA_STREAM, &DMA_InitStructure);

        /* 使能 DMA中断 */
        DMA_ITConfig(RS232_USARTTx_DMA_STREAM, DMA_IT_TC, ENABLE);

        /* 使能串口的 DMA发送 */
        USART_DMACmd(RS232_USART, USART_DMAReq_Tx, ENABLE);

        /* 配置 DMA中断优先级 */
        NVIC_InitStructure.NVIC_IRQChannel                   = RS232_USARTTx_DMA_IRQ ; // 发送中断通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;                  // 抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		           // 子优先级
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			   // IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);

        // DMA_Cmd(RS232_USARTTx_DMA_STREAM, ENABLE);
    }

    if(NULL != pRxBuf)
    {
        DMA_DeInit(RS232_USARTRx_DMA_STREAM);

        while (DMA_GetCmdStatus(RS232_USARTRx_DMA_STREAM) != DISABLE); // 等待 DMA可配置 

        /* USARTx RX DMA2 Channel (triggered by USARTx Rx event) Config */
        DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)pRxBuf;               /* 目的 */
        DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;     /* 方向 */
        DMA_InitStructure.DMA_BufferSize          = RxSize;                         /* 长度 */                  

        /* 配置DMA */
        DMA_Init(RS232_USARTRx_DMA_STREAM, &DMA_InitStructure);

        /* 由于接收不需要 DMA中断，故不设置 DMA中断 */
        DMA_ITConfig(RS232_USARTRx_DMA_STREAM, DMA_IT_TC, DISABLE);

        /* 使能串口的 DMA发送 */
        USART_DMACmd(RS232_USART, USART_DMAReq_Rx, ENABLE);

        /* 由于接收不需要 DMA中断，故不能配置 DMA中断优先级 */

        /* 使能 DMA */ 
        DMA_Cmd(RS232_USARTRx_DMA_STREAM, ENABLE);
    }
}

/************************************************
函数名称 ： RS232_USART_GPIO_Config
功    能 ： RS232端口配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void RS232_USART_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* config RS232 USART GPIO clock */
    RS232_USART_GPIO_CLOCK_FUN(RS232_USART_GPIO_CLK, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(RS232_USART_TX_GPIO_PORT, RS232_USART_TX_AF_PIN, RS232_USART_GPIO_AF_MAP);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(RS232_USART_RX_GPIO_PORT, RS232_USART_RX_AF_PIN, RS232_USART_GPIO_AF_MAP);

    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = RS232_USART_TX_GPIO_PIN;
    GPIO_Init(RS232_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = RS232_USART_RX_GPIO_PIN;
    GPIO_Init(RS232_USART_RX_GPIO_PORT, &GPIO_InitStructure);
}


/************************************************************************/
/*            STM32F4xx USART Interrupt Handlers                        */
/************************************************************************/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void RS232_USART_IRQHandler(void)
{
    if (USART_GetITStatus(RS232_USART, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
#if 0
        RS232_Rx.pBuffer[RS232_Rx.Counter++] = (USART_ReceiveData(RS232_USART) & 0x7F);  // 如果使能了奇偶校验，则最高位 bit 8需要省掉
#else
        RS232_Rx.pBuffer[RS232_Rx.Counter++] = (USART_ReceiveData(RS232_USART) & 0xFF); // 获取数据
#endif

        if (RS232_Rx.Counter >= BUFFER_SIZE)
        {
            /* Disable the COM1_UART Receive interrupt */
            // USART_ITConfig(COM1_UART, USART_IT_RXNE, DISABLE);

            RS232_Rx.Counter = 0;
        }

        // USART_ClearITPendingBit(RS232_USART, USART_IT_RXNE);
    }

    if (USART_GetITStatus(RS232_USART, USART_IT_IDLE))
    {
#if USING_RS232_DMA
		DMA_Cmd(RS232_USARTRx_DMA_STREAM, DISABLE);
		DMA_ClearFlag(RS232_USARTRx_DMA_STREAM, RS232_USARTRx_DMA_TC_FLAG);
		RS232_Rx.Counter = BUFFER_SIZE - DMA_GetCurrDataCounter(RS232_USARTRx_DMA_STREAM);    // 获取接收长度
		DMA_SetCurrDataCounter(RS232_USARTRx_DMA_STREAM, BUFFER_SIZE);
		DMA_Cmd(RS232_USARTRx_DMA_STREAM, ENABLE);
#endif /* USING_RS232_DMA */

        RS232_Rx.Frame_flag = 1;
        USART_ReceiveData(RS232_USART); // Clear IDLE interrupt flag bit

        xTaskResumeFromISR(RS232_Rx_Handle);
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

    if(USART_GetITStatus(RS232_USART, USART_IT_TC)) {
		USART_ClearITPendingBit(RS232_USART, USART_IT_TC);
	}
}

__attribute__((weak)) void RS232_USARTTx_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(RS232_USARTTx_DMA_STREAM, RS232_USARTTx_DMA_IT_TC_FLAG) != RESET)
	{
		DMA_ClearITPendingBit(RS232_USARTTx_DMA_STREAM, RS232_USARTTx_DMA_IT_TC_FLAG);
   		DMA_Cmd(RS232_USARTTx_DMA_STREAM, DISABLE);
    	USART_ITConfig(RS232_USART, USART_IT_TC, ENABLE);
	}
}

#endif /* USING_RS232 */


/*---------------------------- END OF FILE ----------------------------*/


