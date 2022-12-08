#include "canopen_app.h"
#include "canopen_drv.h"
#include "TestMaster.h"
#include "bsp_uart.h"


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
    unsigned char ret_canInit;

    ret_canInit = canInit(CAN_BAUDRATE);
    if(0 == ret_canInit)
        DEBUG_PRINTF("CAN init finished...\r\n");

    xReturn = xTaskCreate(CANOpen_App_Task, "CANOpen_App_Task", CANOPEN_STACK_SIZE, NULL, CANOPEN_TASK_PRIORITY, NULL);
    if (pdPASS != xReturn)
    {
        return; //创建任务失败
    }
}

UNS32 OnAppCommandWordUpdate(CO_Data* d, const indextable * unsused_indextable, UNS8 unsused_bSubindex)
{
    DEBUG_PRINTF("OnMasterMap1Update:%d\n", MasterMap1);
    
    return 0;
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

    InitNodes(&TestMaster_Data, nodeID, &OnAppCommandWordUpdate);
    
    for (;;)
    {
        vTaskDelay(500 / portTICK_RATE_MS);

    }
}

