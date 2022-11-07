#ifndef __SYS_GUARD_H
#define __SYS_GUARD_H


#include "stm32f4xx.h"


typedef enum {
	SG_MAIN,
	SG_TCP,
	SG_NUM_TOTAL
}SGNum_TypeDef;

typedef struct
{
	const char* Name;
	uint8_t Enable;
	uint32_t Counter;
	uint32_t MaxValue;
	
	void (*errcallback)(void);
}SG_TypeDef;

void vSysGuard_Task(void *pvParameters);
void SysGuard_Reg(SGNum_TypeDef Num, const char* Name, uint32_t MaxValue, void (*errcb)(void));
void SysGuard_Online(SGNum_TypeDef Num);
void SysGuard_Start(SGNum_TypeDef Num);
void SysGuard_Stop(SGNum_TypeDef Num);


#endif /* __SYS_GUARD_H */
