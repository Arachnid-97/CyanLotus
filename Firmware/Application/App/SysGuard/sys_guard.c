#include "sys_guard.h"
#include "bsp_iwdg.h"
#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"


#ifndef USER_IWDG_ENABLE
#define USER_IWDG_ENABLE		0
#endif /* USER_IWDG_ENABLE */

#define SG_NAME_UNDEF			"Name Undefined"

static SG_TypeDef s_SG_Structure[SG_NUM_TOTAL];

static void SysGuard_Reset(void);
static void SysGuard_Scan(void);


void vSysGuard_Task(void *pvParameters)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    
	#if USER_IWDG_ENABLE
	IWDG_Config(IWDG_Prescaler_128 ,625);	// 4S
	#endif /* USER_IWDG_ENABLE */
	
	while(1)
	{
		vTaskDelay(1000 / portTICK_RATE_MS);
		#if USER_IWDG_ENABLE
		IWDG_Feed();
		#endif /* USER_IWDG_ENABLE */
		
		SysGuard_Scan();
	}
}

void SysGuard_Reg(SGNum_TypeDef Num, const char* Name, uint32_t MaxValue, void (*errcb)(void))
{
	if(NULL == Name)
		s_SG_Structure[Num].Name = SG_NAME_UNDEF;
	else
		s_SG_Structure[Num].Name = Name;
	
	s_SG_Structure[Num].Enable = 0;
	s_SG_Structure[Num].Counter = 0;
	s_SG_Structure[Num].MaxValue = MaxValue;
	
	if(NULL == errcb)
		s_SG_Structure[Num].errcallback = SysGuard_Reset;
	else
		s_SG_Structure[Num].errcallback = errcb;
}

void SysGuard_Online(SGNum_TypeDef Num)
{
	s_SG_Structure[Num].Counter = 0;
}

void SysGuard_Start(SGNum_TypeDef Num)
{
#if USER_IWDG_ENABLE
	if(!s_SG_Structure[Num].Enable){
		DEBUG_PRINTF("SG Start: %s\r\n", s_SG_Structure[Num].Name);
		s_SG_Structure[Num].Counter = 0;
		s_SG_Structure[Num].Enable = 1;
	}

#endif /* USER_IWDG_ENABLE */
}

void SysGuard_Stop(SGNum_TypeDef Num)
{
	if(s_SG_Structure[Num].Enable){
		DEBUG_PRINTF("SG Stop: %s\r\n", s_SG_Structure[Num].Name);
		s_SG_Structure[Num].Enable = 0;
		s_SG_Structure[Num].Counter = 0;
	}
}

static void SysGuard_Scan(void)
{
	uint8_t i;
	
	for(i = 0;i < SG_NUM_TOTAL;i++)
	{
		if((1 == s_SG_Structure[i].Enable) && (s_SG_Structure[i].MaxValue != 0))
		{
			if(s_SG_Structure[i].Counter <= s_SG_Structure[i].MaxValue){
				s_SG_Structure[i].Counter++;
			}
			else{
				DEBUG_PRINTF("SG_TimeOut Num = %d\r\n\n", i);
				s_SG_Structure[i].errcallback();
			}
		}
	}
}

static void SysGuard_Reset(void)
{
	#if USER_IWDG_ENABLE
	NVIC_SystemReset();
	#endif /* USER_IWDG_ENABLE */
}

