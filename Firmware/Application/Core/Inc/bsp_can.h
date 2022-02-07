#ifndef __BSP_CAN_H
#define __BSP_CAN_H


#include "stm32f4xx.h"


#define CANx                CAN1
#define CAN_CLK             RCC_APB1Periph_CAN1
#define CAN_CLOCK_FUN(x, y) RCC_APB1PeriphClockCmd(x, y)

#define CAN_GPIO_CLK        RCC_AHB1Periph_GPIOB
#define CAN_GPIO_CLOCK_FUN(x, y) RCC_AHB1PeriphClockCmd(x, y)

#define CAN_GPIO_AF_MAP     GPIO_AF_CAN1
#define CAN_TX_AF_PIN       GPIO_PinSource9
#define CAN_RX_AF_PIN       GPIO_PinSource8

#define CAN_TX_GPIO_PORT    GPIOB
#define CAN_TX_GPIO_PIN     GPIO_Pin_9
#define CAN_RX_GPIO_PORT    GPIOB
#define CAN_RX_GPIO_PIN     GPIO_Pin_8

#define CAN_RX_IRQn         CAN1_RX0_IRQn
#define CAN_RX_IRQHandler   CAN1_RX0_IRQHandler


void CANx_Init(void);


#endif /* __BSP_CAN_H */


/*---------------------------- END OF FILE ----------------------------*/


