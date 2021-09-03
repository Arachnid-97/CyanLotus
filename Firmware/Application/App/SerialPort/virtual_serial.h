#ifndef __VIARTUAL_SERIAL_H
#define __VIARTUAL_SERIAL_H


#include "stm32f4xx.h"


#define VSPD_TIM                TIM12

#define VSPD_SEND_START()       TIM_ClearFlag(VSPD_TIM, TIM_FLAG_CC1);TIM_ITConfig(VSPD_TIM, TIM_IT_CC1, ENABLE)
#define VSPD_SEND_STOP()        TIM_ITConfig(VSPD_TIM, TIM_IT_CC1, DISABLE)

#define VSPD_RECEIVE_START()    TIM_ClearFlag(VSPD_TIM, TIM_FLAG_CC2);TIM_ITConfig(VSPD_TIM, TIM_IT_CC2, ENABLE)
#define VSPD_RECEIVE_STOP()     TIM_ITConfig(VSPD_TIM, TIM_IT_CC2, DISABLE)


#define COM_NUM             1

typedef enum
{
	COM1,
    COM2,
    COM3,
} VSP_ID_TypeDef;

/* x = COM1/COM2...COMx */
#define VIRUAL_SERIAL_PORT(x)   {x ## _RX_PORT, x ## _RX_PIN, x ## _TX_PORT, x ## _TX_PIN}

#define COM1_TX_PORT        GPIOB
#define COM1_TX_PIN         GPIO_Pin_6
#define COM1_RX_PORT        GPIOB
#define COM1_RX_PIN         GPIO_Pin_7

#define COM2_TX_PORT        GPIOC
#define COM2_TX_PIN         GPIO_Pin_6
#define COM2_RX_PORT        GPIOC
#define COM2_RX_PIN         GPIO_Pin_7

#define COM3_TX_PORT        GPIOE
#define COM3_TX_PIN         GPIO_Pin_0
#define COM3_RX_PORT        GPIOE
#define COM3_RX_PIN         GPIO_Pin_1


#ifndef HIGH
#define HIGH                1
#endif /* HIGH */

#ifndef LOW
#define LOW                 0
#endif /* LOW */

typedef struct
{
    VSP_ID_TypeDef id;
	volatile uint16_t len;
    uint16_t max_len;
    uint16_t frame;
    uint8_t busy;
    uint8_t *pbuf;
} VSPD_TypeDef;


void Virtual_Serial_Init(void);
char VSPD_SendByte(VSP_ID_TypeDef ID, uint8_t Data);
char VSPD_SendString(VSP_ID_TypeDef ID, uint8_t *pBuf, uint16_t Length);
char VSPD_RcvString(VSP_ID_TypeDef ID, uint8_t *pBuf, uint16_t Length);


#endif /* __VIARTUAL_SERIAL_H */


/*---------------------------- END OF FILE ----------------------------*/


