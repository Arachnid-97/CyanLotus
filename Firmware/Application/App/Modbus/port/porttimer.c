/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    /*
    T3.5个字符时间区分不同的帧，即接收到的两个字符之间时间间隔小于3.5个字符
    时间时认为是同一个帧的，如果间隔大于3.5个字符时间则认为是不同帧的
    在一般的串口通信中，发送 1个字符需要：1位起始位，8位数据位，1位校验位(可无),
    1位停止位,总共 1+8+1+1 = 11位，3.5个字符时间就是 3.5 * 11 = 38.5位，
    假如波特率是 9600,那么传输 1位的时间是 1/9600 = 0.10416667(ms) ,
    这样，3.5个字符时间就大约是 38.5 * 0.10416667 = 4.01 ms ,即定时器需要的中断时间
    */

    uint16_t PrescalerValue = 0;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM2);

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)(SystemCoreClock / 10000) - 1;       // 100us

    TIM_TimeBaseStructure.TIM_Period = 41 - 1;                      // 自动重装载寄存器的值
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;           // 时钟预分频数
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         // 时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    return TRUE;
}

inline void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */

#if USING_MODBUS_RxDMA
    static uint8_t init = 1; // 
    
    if(1 == init)
    {
#endif
        /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        TIM_SetCounter(TIM2, 0);
        TIM_Cmd(TIM2, ENABLE);

#if USING_MODBUS_RxDMA
        init = 0;
    }
#endif

}

inline void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    TIM_Cmd(TIM2, DISABLE);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

/**
  * @brief  Modbus Timers IRQHandler.
  * @param  None.
  * @return None.
  */
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
        prvvTIMERExpiredISR();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

