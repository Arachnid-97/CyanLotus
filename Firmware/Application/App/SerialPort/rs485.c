#include "rs485.h"

#if USING_RS485

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


#define DEFAULT_BAUDRATE        9600 // 波特率设置  支持的波特率：115200,19200,9600,38400,57600,1200,2400,4800

#define BUFFER_SIZE             64

RS_Buff_TypeDef RS485_Rx;


static void RS485_USART_GPIO_Config(void);


/************************************************
函数名称 ： vSetupRS485
功    能 ： UART初始化接口
参    数 ： 无
返 回 值 ： 无
*************************************************/
void vSetupRS485(void)
{
    RS485_USART_Init(DEFAULT_BAUDRATE, RS_PAR_NONE);

    RS485_Rx.pBuffer = pvPortMalloc(BUFFER_SIZE);

#if USING_RS485_DMA
    RS485_DMA_Config(NULL, (void *)RS485_Rx.pBuffer, BUFFER_SIZE, BUFFER_SIZE);
#endif /* USING_RS485_DMA */

}

/************************************************
函数名称 ： RS485_USART_Init
功    能 ： RS485串口初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void RS485_USART_Init(uint32_t BaudRate, RSParity Parity)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint16_t parity;

    /* config RS485 USART GPIO */
    RS485_USART_GPIO_Config();

    /* config RS485 USART clock */
    RS485_USART_CLOCK_FUN(RS485_USART_CLK, ENABLE);

    USART_DeInit(RS485_USART);

    /* Enable the RS485 USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* RS485 USART configured as follows:
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
    USART_Init(RS485_USART, &USART_InitStructure);

    USART_Cmd(RS485_USART, ENABLE);

    USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(RS485_USART, USART_IT_IDLE, ENABLE);

    // USART_ITConfig(RS485_USART, USART_IT_ERR, ENABLE);

    /* Uncomment the line below if usart send flag use USART_FLAG_TC */
    // USART_ClearFlag(RS485_USART, USART_FLAG_TC); // Fixed first byte sending failure
}

/************************************************
函数名称 ： RS485_DMA_Config
功    能 ： RS485 DMA配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
void RS485_DMA_Config(void *pTxBuf, void *pRxBuf, uint16_t TxSize, uint16_t RxSize)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 使能 DMA时钟 */
    RCC_AHB1PeriphClockCmd(RS485_USART_DMA_CLK, ENABLE);

    /* DMA基本配置 */
    DMA_InitStructure.DMA_Channel             = RS485_USART_DMA_CHANNEL;        /* 配置 DMA通道 */
    DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)(&(RS485_USART->DR)); /* 源 */
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
        DMA_DeInit(RS485_USARTTx_DMA_STREAM);

        /* 等待 DMA可配置 */
        while (DMA_GetCmdStatus(RS485_USARTTx_DMA_STREAM) != DISABLE);

        /* USARTx TX DMA2 Channel (triggered by USARTx Rx event) Config */
        DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)pTxBuf;               /* 目的 */
        DMA_InitStructure.DMA_DIR                 = DMA_DIR_MemoryToPeripheral;     /* 方向 */
        DMA_InitStructure.DMA_BufferSize          = TxSize;                         /* 长度 */                  

        /* 配置DMA */
        DMA_Init(RS485_USARTTx_DMA_STREAM, &DMA_InitStructure);

        /* 使能 DMA中断 */
        DMA_ITConfig(RS485_USARTTx_DMA_STREAM, DMA_IT_TC, ENABLE);

        /* 使能串口的 DMA发送 */
        USART_DMACmd(RS485_USART, USART_DMAReq_Tx, ENABLE);

        /* 配置 DMA中断优先级 */
        NVIC_InitStructure.NVIC_IRQChannel                   = RS485_USARTTx_DMA_IRQ ; // 发送中断通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;                  // 抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;		           // 子优先级
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;			   // IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);

        // DMA_Cmd(RS485_USARTTx_DMA_STREAM, ENABLE);
    }

    if(NULL != pRxBuf)
    {
        DMA_DeInit(RS485_USARTRx_DMA_STREAM);

        while (DMA_GetCmdStatus(RS485_USARTRx_DMA_STREAM) != DISABLE); // 等待 DMA可配置 

        /* USARTx RX DMA2 Channel (triggered by USARTx Rx event) Config */
        DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)pRxBuf;               /* 目的 */
        DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;     /* 方向 */
        DMA_InitStructure.DMA_BufferSize          = RxSize;                         /* 长度 */                  

        /* 配置DMA */
        DMA_Init(RS485_USARTRx_DMA_STREAM, &DMA_InitStructure);

        /* 由于接收不需要 DMA中断，故不设置 DMA中断 */
        DMA_ITConfig(RS485_USARTRx_DMA_STREAM, DMA_IT_TC, DISABLE);

        /* 使能串口的 DMA发送 */
        USART_DMACmd(RS485_USART, USART_DMAReq_Rx, ENABLE);

        /* 由于接收不需要 DMA中断，故不能配置 DMA中断优先级 */

        /* 使能 DMA */ 
        DMA_Cmd(RS485_USARTRx_DMA_STREAM, ENABLE);
    }
}

/************************************************
函数名称 ： RS485_USART_GPIO_Config
功    能 ： RS485端口配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void RS485_USART_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* config RS485 USART GPIO clock */
    RS485_USART_GPIO_CLOCK_FUN(RS485_USART_GPIO_CLK, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(RS485_USART_TX_GPIO_PORT, RS485_USART_TX_AF_PIN, RS485_USART_GPIO_AF_MAP);
    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(RS485_USART_RX_GPIO_PORT, RS485_USART_RX_AF_PIN, RS485_USART_GPIO_AF_MAP);

    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure USART Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_GPIO_PIN;
    GPIO_Init(RS485_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_GPIO_PIN;
    GPIO_Init(RS485_USART_RX_GPIO_PORT, &GPIO_InitStructure);
}


/************************************************************************/
/*            STM32F4xx USART Interrupt Handlers                        */
/************************************************************************/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
__attribute__((weak)) void RS485_USART_IRQHandler(void)
{
    if (USART_GetITStatus(RS485_USART, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
#if 0
        RS485_Rx.pBuffer[RS485_Rx.Counter++] = (USART_ReceiveData(RS485_USART) & 0x7F);  // 如果使能了奇偶校验，则最高位 bit 8需要省掉
#else
        RS485_Rx.pBuffer[RS485_Rx.Counter++] = (USART_ReceiveData(RS485_USART) & 0xFF); // 获取数据
#endif

        if (RS485_Rx.Counter >= BUFFER_SIZE)
        {
            /* Disable the COM1_UART Receive interrupt */
            // USART_ITConfig(COM1_UART, USART_IT_RXNE, DISABLE);

            RS485_Rx.Counter = 0;
        }

        // USART_ClearITPendingBit(RS485_USART, USART_IT_RXNE);
    }

    if (USART_GetITStatus(RS485_USART, USART_IT_IDLE))
    {
#if USING_RS485_DMA
		DMA_Cmd(RS485_USARTRx_DMA_STREAM, DISABLE);
		DMA_ClearFlag(RS485_USARTRx_DMA_STREAM, RS485_USARTRx_DMA_TC_FLAG);
		RS485_Rx.Counter = BUFFER_SIZE - DMA_GetCurrDataCounter(RS485_USARTRx_DMA_STREAM);    // 获取接收长度
		DMA_SetCurrDataCounter(RS485_USARTRx_DMA_STREAM, BUFFER_SIZE);
		DMA_Cmd(RS485_USARTRx_DMA_STREAM, ENABLE);
#endif /* USING_RS485_DMA */

        RS485_Rx.Frame_flag = 1;
        (void)USART_ReceiveData(RS485_USART); // Clear IDLE interrupt flag bit
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

    if(USART_GetITStatus(RS485_USART, USART_IT_TC)) {
		USART_ClearITPendingBit(RS485_USART, USART_IT_TC);
	}


	// if(USART_GetITStatus(RS485_USART, USART_IT_NE | USART_IT_ORE | USART_IT_FE)) {
    //     (void)USART_ReceiveData(RS485_USART); // Clear IDLE interrupt flag bit
	// }
}

__attribute__((weak)) void RS485_USARTTx_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(RS485_USARTTx_DMA_STREAM, RS485_USARTTx_DMA_IT_TC_FLAG) != RESET)
	{
		DMA_ClearITPendingBit(RS485_USARTTx_DMA_STREAM, RS485_USARTTx_DMA_IT_TC_FLAG);
   		DMA_Cmd(RS485_USARTTx_DMA_STREAM, DISABLE);
    	USART_ITConfig(RS485_USART, USART_IT_TC, ENABLE);
	}
}

#endif /* USING_RS485 */


/*---------------------------- END OF FILE ----------------------------*/


