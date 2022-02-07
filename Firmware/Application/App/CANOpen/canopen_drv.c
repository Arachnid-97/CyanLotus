#include "canopen_drv.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "TestMaster.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


xQueueHandle xQueueCAN_Send = NULL;
static xQueueHandle xQueueCAN_Receive = NULL;

TIMEVAL last_counter_val = 0;
TIMEVAL elapsed_time = 0;

static void prvCANSend_Task(void *pvParameters);
static void prvCANReceive_Task(void *pvParameters);

/**
 * @ingroup can
 * @brief Initialize the hardware to receive CAN messages 
 *        and start the timer for the CANopen stack
 * @param bitrate bitrate in kilobit
 * @return 0 if succes
 */
UNS8 canInit(UNS32 bitrate)
{
    BaseType_t xReturn;

    CANx_Init();
    initTimer();

    xQueueCAN_Send = xQueueCreate(CANTX_QUEUE_LEN, sizeof(CanTxMsg));
    if (xQueueCAN_Send == NULL)
        DEBUG_PRINTF("Create xQueueCAN_Send failed...\r\n");

    xQueueCAN_Receive = xQueueCreate(CANRX_QUEUE_LEN, sizeof(CanRxMsg));
    if (xQueueCAN_Receive == NULL)
        DEBUG_PRINTF("Create xQueueCAN_Receive failed...\r\n");

    /* 创建任务 */
    xReturn = xTaskCreate(prvCANSend_Task, "prvCANSend_Task", CANTX_STACK_SIZE, NULL, CANTX_TASK_PRIORITY, NULL);
    if (pdPASS != xReturn)
        DEBUG_PRINTF("prvCANSend_Task create failed...\r\n");

    xReturn = xTaskCreate(prvCANReceive_Task, "prvCANReceive_Task", CANRX_STACK_SIZE, NULL, CANRX_TASK_PRIORITY, NULL);
    if (pdPASS != xReturn)
        DEBUG_PRINTF("prvCANReceive_Task create failed...\r\n");

    (void)bitrate;

    return 0;
}

/**
 * @ingroup can
 * @brief received a CAN message
 * @param *m pointer to received CAN message
 * @return 0 if succes
 */
UNS8 canReceive(CanRxMsg *m)
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if (NULL != xQueueCAN_Receive)
    {
        xQueueSendFromISR(xQueueCAN_Receive, m, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }

    return 0;
}


/************************************************
函数名称 ： prvCANSend_Task
功    能 ： CAN发送应用任务程序
参    数 ： pvParameters
返 回 值 ： 无
*************************************************/
static void prvCANSend_Task(void *pvParameters)
{
    static CanTxMsg TxMsg;

    for (;;)
    {
        /* 等待接收有效数据包 */
        if (xQueueReceive(xQueueCAN_Send, &TxMsg, 100) == pdTRUE)
        {
            if (CAN_Transmit(CANx, &TxMsg) == CAN_NO_MB)
            {
                vTaskDelay(1); //第一次发送失败, 延时1个滴答, 再次发送
                CAN_Transmit(CANx, &TxMsg);
            }
        }
    }
}

/************************************************
函数名称 ： prvCANReceive_Task
功    能 ： CAN接收应用任务程序
参    数 ： pvParameters
返 回 值 ： 无
*************************************************/
static void prvCANReceive_Task(void *pvParameters)
{
    static CanRxMsg RxMsg;
    static Message msg;

    uint8_t i = 0;

    for (;;)
    {
        if (xQueueReceive(xQueueCAN_Receive, &RxMsg, 100) == pdTRUE)
        {
            msg.cob_id = RxMsg.StdId; // CAN-ID

            if (CAN_RTR_REMOTE == RxMsg.RTR)
                msg.rtr = 1; //远程帧
            else
                msg.rtr = 0; //数据帧

            msg.len = (UNS8)RxMsg.DLC; //长度

            for (i = 0; i < RxMsg.DLC; i++) //数据
                msg.data[i] = RxMsg.Data[i];

            TIM_ITConfig(TIM10, TIM_IT_Update, DISABLE);
            canDispatch(&TestMaster_Data, &msg); //调用协议相关接口
            TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);
        }
    }
}


/************************************************
函数名称 ： TIMx_DispatchFromISR
功    能 ： 定时调度(从定时器中断)
参    数 ： 无
返 回 值 ： 无
*************************************************/
void TIMx_DispatchFromISR(void)
{
    last_counter_val = 0;
    elapsed_time = 0;
    TimeDispatch();
}


/************************************************************************/
/*                 STM32F4xx CAN Interrupt Handlers                     */
/************************************************************************/

/**
  * @brief  CANx Timers IRQHandler.
  * @param  None.
  * @return None.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
        TIMx_DispatchFromISR();
    }
}

/**
 * @brief  This function handles CANx global interrupt request.
 * @param  None
 * @retval None
 */
void CAN_RX_IRQHandler(void)
{
    static CanRxMsg RxMessage;

    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);

    canReceive(&RxMessage);
}

