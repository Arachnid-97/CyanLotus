#include "canfestival.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"


extern xQueueHandle xQueueCAN_Send;

extern TIMEVAL last_counter_val;
extern TIMEVAL elapsed_time;

/**
 * @ingroup timer
 * @brief Initializes the timer
 *        turn on the interrupt and put the interrupt time to zero
 */
void initTimer(void)
{
    uint16_t PrescalerValue = 0;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
    TIM_DeInit(TIM10);

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t)(SystemCoreClock / 100000) - 1;      // 10us

    TIM_TimeBaseStructure.TIM_Period = TIMEVAL_MAX;                 // 自动重装载寄存器的值
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;           // 时钟预分频数
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;         // 时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM10, TIM_FLAG_Update);
    TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);

    TIM_SetCounter(TIM10, 0);
    TIM_Cmd(TIM10, ENABLE);
}

/**
 * @ingroup can
 * @brief Send a CAN message
 * @param port CanFestival file descriptor
 * @param *m pointer to message to send
 * @return 0 if succes
 */
UNS8 canSend(CAN_PORT notused, Message *m)
{
    uint8_t i;
    static CanTxMsg TxMsg;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    TxMsg.StdId = m->cob_id;

    if (m->rtr)
        TxMsg.RTR = CAN_RTR_REMOTE;
    else
        TxMsg.RTR = CAN_RTR_DATA;

    TxMsg.IDE = CAN_ID_STD;
    TxMsg.DLC = m->len;
    for (i = 0; i < m->len; i++)
        TxMsg.Data[i] = m->data[i];

    /* 判断是否在执行中断 */
    if (0 == __get_CONTROL())
    {
        if (xQueueSendFromISR(xQueueCAN_Send, &TxMsg, &xHigherPriorityTaskWoken) != pdPASS)
        {
            return 0xFF;
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        if (xQueueSend(xQueueCAN_Send, &TxMsg, 100) != pdPASS)
        {
            return 0xFF;
        }
    }

    return 0;
}

/**
 * @ingroup can
 * @brief Change the CANOpen device baudrate 
 * @param port CanFestival file descriptor 
 * @param *baud The new baudrate to assign
 * @return
 *       - 0 is returned upon success or if not supported by the CAN driver.
 *       - errorcode from the CAN driver is returned if an error occurs. (if implemented in the CAN driver)
 */
UNS8 canChangeBaudRate(CAN_PORT port, char* baud)
{
    (void)port;
    (void)baud;

    return 0;
}


/* The following function declaration comes from <timer_s.h> */

/**
 * @ingroup timer
 * @brief Set a timerfor a given time.
 * @param value The time value.
 */
void setTimer(TIMEVAL value)
{
    uint32_t timer = TIM_GetCounter(TIM10); // Copy the value of the running timer

    elapsed_time += timer - last_counter_val;
    last_counter_val = TIMEVAL_MAX - value;
    TIM_SetCounter(TIM10, TIMEVAL_MAX - value);
    TIM_Cmd(TIM10, ENABLE);
}

/**
 * @ingroup timer
 * @brief Get the time elapsed since latest timer occurence.
 * @return time elapsed since latest timer occurence
 */
TIMEVAL getElapsedTime(void)
{
    uint32_t timer = TIM_GetCounter(TIM10); // Copy the value of the running timer

    if (timer < last_counter_val)
        timer += TIMEVAL_MAX;

    TIMEVAL elapsed = timer - last_counter_val + elapsed_time;

    return elapsed;
}


/*---------------------------- END OF FILE ----------------------------*/


