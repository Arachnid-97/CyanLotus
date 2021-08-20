#ifndef __RS485_H
#define __RS485_H


#include "stm32f4xx.h"
#include "rs232_485_config.h"


#define RS485_USART                 USART6
#define RS485_USART_CLK             RCC_APB2Periph_USART6
#define RS485_USART_APBxClkCmd      RCC_APB2PeriphClockCmd

#define RS485_USART_GPIO_CLK        RCC_AHB1Periph_GPIOC
#define RS485_USART_GPIO_APBxClkCmd RCC_AHB1PeriphClockCmd

#define RS485_USART_GPIO_AF_MAP     GPIO_AF_USART6
#define RS485_USART_TX_AF_PIN       GPIO_PinSource6
#define RS485_USART_RX_AF_PIN       GPIO_PinSource7

#define RS485_USART_TX_GPIO_PORT    GPIOC
#define RS485_USART_TX_GPIO_PIN     GPIO_Pin_6
#define RS485_USART_RX_GPIO_PORT    GPIOC
#define RS485_USART_RX_GPIO_PIN     GPIO_Pin_7

#define RS485_USART_IRQ             USART6_IRQn
#define RS485_USART_IRQHandler      USART6_IRQHandler


#define RS485_USART_DMA_CLK         RCC_AHB1Periph_DMA2	
#define RS485_USART_DMA_CHANNEL     DMA_Channel_5
#define RS485_USARTTx_DMA_STREAM    DMA2_Stream6
#define RS485_USARTRx_DMA_STREAM    DMA2_Stream1

#define RS485_USARTTx_DMA_IRQ       DMA2_Stream6_IRQn
#define RS485_USARTTx_DMA_IRQHandler DMA2_Stream6_IRQHandler
#define RS485_USARTTx_DMA_TC_FLAG   DMA_FLAG_TCIF6
#define RS485_USARTTx_DMA_IT_TC_FLAG DMA_IT_TCIF6

#define RS485_USARTRx_DMA_TC_FLAG   DMA_FLAG_TCIF1


void vSetup485( void );
void RS485_USART_Init(uint32_t BaudRate, RSParity Parity);
void RS485_DMA_Config(void *pTxBuf, void *pRxBuf, uint16_t TxSize, uint16_t RxSize);


#endif /* __RS485_H */


/*---------------------------- END OF FILE ----------------------------*/


