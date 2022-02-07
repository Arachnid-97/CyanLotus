#ifndef __BSP_RNG_H
#define __BSP_RNG_H


#include "stm32f4xx.h"


uint8_t RNG_Init(void);
uint32_t RNG_Get_RandomNum(void);
uint32_t RNG_Get_RandomRange(uint32_t Min,uint32_t Max);


#endif /* __BSP_RNG_H */


/*---------------------------- END OF FILE ----------------------------*/


