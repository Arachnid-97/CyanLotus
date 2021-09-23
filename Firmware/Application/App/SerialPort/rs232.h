#ifndef __RS232_H
#define __RS232_H


#include "stm32f4xx.h"
#include "rs232_485_config.h"


#define RS232_USART                 USART1
#define RS232_USART_CLK             RCC_APB2Periph_USART1
#define RS232_USART_CLOCK_FUN(x, y) RCC_APB2PeriphClockCmd(x, y)

#define RS232_USART_GPIO_CLK        RCC_AHB1Periph_GPIOB
#define RS232_USART_GPIO_CLOCK_FUN(x, y) RCC_AHB1PeriphClockCmd(x, y)

#define RS232_USART_GPIO_AF_MAP     GPIO_AF_USART1
#define RS232_USART_TX_AF_PIN       GPIO_PinSource6
#define RS232_USART_RX_AF_PIN       GPIO_PinSource7

#define RS232_USART_TX_GPIO_PORT    GPIOB
#define RS232_USART_TX_GPIO_PIN     GPIO_Pin_6
#define RS232_USART_RX_GPIO_PORT    GPIOB
#define RS232_USART_RX_GPIO_PIN     GPIO_Pin_7

#define RS232_USART_IRQ             USART1_IRQn
#define RS232_USART_IRQHandler      USART1_IRQHandler


#define RS232_USART_DMA_CLK         RCC_AHB1Periph_DMA2	
#define RS232_USART_DMA_CHANNEL     DMA_Channel_4
#define RS232_USARTTx_DMA_STREAM    DMA2_Stream7
#define RS232_USARTRx_DMA_STREAM    DMA2_Stream5

#define RS232_USARTTx_DMA_IRQ       DMA2_Stream7_IRQn
#define RS232_USARTTx_DMA_IRQHandler DMA2_Stream7_IRQHandler
#define RS232_USARTTx_DMA_TC_FLAG   DMA_FLAG_TCIF7
#define RS232_USARTTx_DMA_IT_TC_FLAG DMA_IT_TCIF7

#define RS232_USARTRx_DMA_TC_FLAG   DMA_FLAG_TCIF5


void vSetupRS232( void );
void RS232_USART_Init(uint32_t BaudRate, RSParity Parity);
void RS232_DMA_Config(void *pTxBuf, void *pRxBuf, uint16_t TxSize, uint16_t RxSize);


#endif /* __RS232_H */


/*---------------------------- END OF FILE ----------------------------*/


