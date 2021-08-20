#ifndef __RS232_485_H
#define __RS232_485_H


#include "stm32f4xx.h"

#include "bsp_uart.h"


#define USING_RS232                 1
#define USING_RS485                 1

#define USING_RS232_DMA             0
#define USING_RS485_DMA             0

typedef enum
{
    RS_PAR_NONE,                /*!< No parity. */
    RS_PAR_ODD,                 /*!< Odd parity. */
    RS_PAR_EVEN                 /*!< Even parity. */
} RSParity;

typedef struct
{
	volatile uint16_t Counter;      // 接收数据个数
	volatile uint8_t Frame_flag;    // 一帧完成标志
	uint8_t *pBuffer;               // 接收暂存缓冲区
} RS_Buff_TypeDef;


#endif /* __RS232_485_CONFIG_H */


/*---------------------------- END OF FILE ----------------------------*/


