#ifndef __SDCARD_BASE_H
#define __SDCARD_BASE_H


#include "stm32f4xx.h"


/* 无硬件 IO检测则定义为 0 */
#define USING_SD_DETECT             0

#define SD_BLOCK_SIZE               512 /* Block Size in Bytes */


/**
  * @brief  SD FLASH SDIO Interface
  */
#define SD_Dx_CLOCK_FUN(x, y)       RCC_AHB1PeriphClockCmd(x, y)
#define SD_Dx_CLK                   RCC_AHB1Periph_GPIOC

#define SD_D0_PORT                  GPIOC
#define SD_D0_PINS                  GPIO_Pin_8
#define SD_D1_PORT                  GPIOC
#define SD_D1_PINS                  GPIO_Pin_9
#define SD_D2_PORT                  GPIOC
#define SD_D2_PINS                  GPIO_Pin_10
#define SD_D3_PORT                  GPIOC
#define SD_D3_PINS                  GPIO_Pin_11

#define SD_CLK_CLOCK_FUN(x, y)      RCC_AHB1PeriphClockCmd(x, y)
#define SD_CLK_CLK                  RCC_AHB1Periph_GPIOC
#define SD_CLK_PORT                 GPIOC
#define SD_CLK_PINS                 GPIO_Pin_12

#define SD_CMD_CLOCK_FUN(x, y)      RCC_AHB1PeriphClockCmd(x, y)
#define SD_CMD_CLK                  RCC_AHB1Periph_GPIOD
#define SD_CMD_PORT                 GPIOD
#define SD_CMD_PINS                 GPIO_Pin_2

#define SD_D0_SOURCE                GPIO_PinSource8
#define SD_D1_SOURCE                GPIO_PinSource9
#define SD_D2_SOURCE                GPIO_PinSource10
#define SD_D3_SOURCE                GPIO_PinSource11
#define SD_CLK_SOURCE               GPIO_PinSource12
#define SD_CMD_SOURCE               GPIO_PinSource2

#define SD_SDIO_GPIO_AF_MAP         GPIO_AF_SDIO

/* 如果 IO的桥接线一样就直接用下面的宏，否则分开配置 */
#define SD_SDIO_IO_CLOCK_FUN(x, y)  RCC_AHB1PeriphClockCmd(x, y)
#define SD_SDIO_IO_PORT             GPIOC


#define SD_SDIO_CLOCK_FUN(x, y)     RCC_APB2PeriphClockCmd(x, y)
#define SD_SDIO_CLK                 RCC_APB2Periph_SDIO

#define SD_SDIO_DMA_CLOCK_FUN(x, y) RCC_AHB1PeriphClockCmd(x, y)
#define SD_SDIO_DMA_CLK             RCC_AHB1Periph_DMA2

#define SD_SDIO_DMA_STREAM3         3
// #define SD_SDIO_DMA_STREAM6         6

#ifdef SD_SDIO_DMA_STREAM3
 #define SD_SDIO_DMA_STREAM         DMA2_Stream3
 #define SD_SDIO_DMA_CHANNEL        DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF      DMA_FLAG_FEIF3
 #define SD_SDIO_DMA_FLAG_DMEIF     DMA_FLAG_DMEIF3
 #define SD_SDIO_DMA_FLAG_TEIF      DMA_FLAG_TEIF3
 #define SD_SDIO_DMA_FLAG_HTIF      DMA_FLAG_HTIF3
 #define SD_SDIO_DMA_FLAG_TCIF      DMA_FLAG_TCIF3 
 #define SD_SDIO_DMA_IRQn           DMA2_Stream3_IRQn
 #define SD_SDIO_DMA_IRQHANDLER     DMA2_Stream3_IRQHandler 
#elif defined SD_SDIO_DMA_STREAM6
 #define SD_SDIO_DMA_STREAM         DMA2_Stream6
 #define SD_SDIO_DMA_CHANNEL        DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF      DMA_FLAG_FEIF6
 #define SD_SDIO_DMA_FLAG_DMEIF     DMA_FLAG_DMEIF6
 #define SD_SDIO_DMA_FLAG_TEIF      DMA_FLAG_TEIF6
 #define SD_SDIO_DMA_FLAG_HTIF      DMA_FLAG_HTIF6
 #define SD_SDIO_DMA_FLAG_TCIF      DMA_FLAG_TCIF6 
 #define SD_SDIO_DMA_IRQn           DMA2_Stream6_IRQn
 #define SD_SDIO_DMA_IRQHANDLER     DMA2_Stream6_IRQHandler
#endif /* SD_SDIO_DMA_STREAM3 */


/* 硬件检测 IO */
#if USING_SD_DETECT
    #define SD_DETECT_CLOCK_FUN(x, y)   RCC_APB2PeriphClockCmd(x, y)
    #define SD_DETECT_PIN           GPIO_Pin_0                 /* PE.00 */
    #define SD_DETECT_GPIO_PORT     GPIOE                      /* GPIOE */
    #define SD_DETECT_GPIO_CLK      RCC_APB2Periph_GPIOE
    
#endif /* USING_SD_DETECT */

#define SDIO_FIFO_ADDRESS           ((uint32_t)(SDIO_BASE + 0x80))
/** 
  * @brief  SDIO Intialization Frequency (400KHz max)
  */
#define SDIO_INIT_CLK_DIV           ((uint8_t)0x78)
/** 
  * @brief  SDIO Data Transfer Frequency (25MHz max) 
  */
#define SDIO_TRANSFER_CLK_DIV       ((uint8_t)0x00)


void SD_LowLevel_DMA_TxConfig( uint32_t *BufferSRC, uint32_t BufferSize );
void SD_LowLevel_DMA_RxConfig( uint32_t *BufferDST, uint32_t BufferSize );
void SD_LowLevel_Init(void);
void SD_LowLevel_DeInit(void);


#endif    /* __SDCARD_BASE_H */


/*---------------------------- END OF FILE ----------------------------*/

