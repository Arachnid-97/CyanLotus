#ifndef _CANOPEN_APP_H
#define _CANOPEN_APP_H


#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


#define CANOPEN_STACK_SIZE        128                      //CANOPEN任务堆栈大小
#define CANOPEN_TASK_PRIORITY     2                        //CANOPEN任务优先级


void CANOpen_App_Init(void);


#endif /* _CANOPEN_APP_H */


/*---------------------------- END OF FILE ----------------------------*/


