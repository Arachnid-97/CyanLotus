#include "bsp_can.h"


/************************************************
函数名称 ： CAN_GPIO_Config
功    能 ： CAN端口配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* config CAN GPIO clock */
    CAN_GPIO_CLOCK_FUN(CAN_GPIO_CLK, ENABLE);

    /* Connect PXx to CANx_Tx*/
    GPIO_PinAFConfig(CAN_TX_GPIO_PORT, CAN_TX_AF_PIN, CAN_GPIO_AF_MAP);
    /* Connect PXx to CANx_Rx*/
    GPIO_PinAFConfig(CAN_RX_GPIO_PORT, CAN_RX_AF_PIN, CAN_GPIO_AF_MAP);

    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* Configure CAN Tx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = CAN_TX_GPIO_PIN;
    GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure CAN Rx as alternate function  */
    GPIO_InitStructure.GPIO_Pin = CAN_RX_GPIO_PIN;
    GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);
}

/************************************************
函数名称 ： CAN_Filter_Config
功    能 ： CAN筛选器配置
参    数 ： 无
返 回 值 ： 无
*************************************************/
static void CAN_Filter_Config(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;

    CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_ID_STD&0xFFFF0000) >> 16;
    CAN_FilterInitStructure.CAN_FilterIdLow = CAN_ID_STD&0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}

/************************************************
函数名称 ： CANx_Init
功    能 ： CANx初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void CANx_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;

    CAN_GPIO_Config();
    CAN_Filter_Config();

    /* 使能时钟 */
    CAN_CLOCK_FUN(CAN_CLK, ENABLE);

    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* CAN register init */
    CAN_DeInit(CANx);

    /* CAN cell init */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

    /* CAN Baudrate = 1 MBps (CAN clocked at 42 MHz -- PCLK1 clock) */
    // 42/(1+5+8)/3=1 Mbps
    CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_Init(CANx, &CAN_InitStructure);

    /* Enable FIFO 0 message pending Interrupt */
    CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}


/*---------------------------- END OF FILE ----------------------------*/


