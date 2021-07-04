#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H


#include "stm32f4xx.h"


void IWDG_Config(uint8_t Prv ,uint16_t Rlv);
void IWDG_Feed(void);


#endif /* __BSP_IWDG_H */


/*---------------------------- END OF FILE ----------------------------*/


