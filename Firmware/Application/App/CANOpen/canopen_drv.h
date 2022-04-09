#ifndef _CANOPEN_DRV_H
#define _CANOPEN_DRV_H


#include "canfestival.h"


#define CANTX_QUEUE_LEN           10                       // CAN队列长度(队列的数量)(发送)
#define CANTX_QUEUE_SIZE          19                       // CAN队列大小(一个队列长度) - sizeof(CanTxMsg)

#define CANRX_QUEUE_LEN           10                       // CAN队列长度(队列的数量)(接收)
#define CANRX_QUEUE_SIZE          20                       // CAN队列大小(一个队列长度) - sizeof(CanRxMsg)

#define CANTX_STACK_SIZE          256                      // CAN通信任务堆栈大小(发送)
#define CANTX_TASK_PRIORITY       3                        // CAN通信任务优先级

#define CANRX_STACK_SIZE          256                      // CAN通信任务堆栈大小(接收)
#define CANRX_TASK_PRIORITY       3                        // CAN通信任务优先级


/* 函数申明 ------------------------------------------------------------------*/
UNS8 canInit(UNS32 bitrate);

void InitNodes(CO_Data* d, UNS32 id, ODCallback_t Callback);


#endif /* _CANOPEN_DRV_H */


/*---------------------------- END OF FILE ----------------------------*/


