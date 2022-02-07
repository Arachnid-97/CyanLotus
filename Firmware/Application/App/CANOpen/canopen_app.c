#include "canopen_app.h"
#include "canopen_drv.h"
#include "TestMaster.h"


#define CAN_BAUDRATE            1000000

static void CANOpen_App_Task(void *pvParameters);

/************************************************
函数名称 ： CANOpen_App_Init
功    能 ： CANOpen应用初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void CANOpen_App_Init(void)
{
    BaseType_t xReturn;

    canInit(CAN_BAUDRATE);

    xReturn = xTaskCreate(CANOpen_App_Task, "CANOpen_App_Task", CANOPEN_STACK_SIZE, NULL, CANOPEN_TASK_PRIORITY, NULL);
    if (pdPASS != xReturn)
    {
        return; //创建接收任务失败
    }
}

/************************************************
函数名称 ： CANOpen_App_Task
功    能 ： CANOpen应用任务
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void CANOpen_App_Task(void *pvParameters)
{
    unsigned char nodeID = 0x00; //节点ID

    setNodeId(&TestMaster_Data, nodeID);
    setState(&TestMaster_Data, Initialisation);
    setState(&TestMaster_Data, Operational);

    for (;;)
    {
        vTaskDelay(500 / portTICK_RATE_MS);

    }
}

