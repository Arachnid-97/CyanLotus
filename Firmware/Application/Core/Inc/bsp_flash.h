#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H


#include "stm32f4xx.h"


// 写入的起始地址与结束地址
#define WRITE_START_ADDR  ((uint32_t)0x00000000)
#define WRITE_END_ADDR    ((uint32_t)0x0801FC00)

void Flash_Read_nWord( uint32_t Addr, uint8_t *Buff, uint16_t Len );
uint32_t Flash_Write_nWord( uint32_t Addr, uint8_t *Buff, uint16_t Len );
uint16_t ReadFlash_HalfWord( uint32_t Addr );


#endif /* __BSP_FLASH_H */


/*---------------------------- END OF FILE ----------------------------*/


