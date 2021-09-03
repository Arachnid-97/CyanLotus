/* 
    Virtual serial configured as follows:
    - BaudRate = ? baudrate (19200,9600,2400,4800)
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
*/

#include "virtual_serial.h"
#include <stdio.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h" // 任务


#define BAUDRATE            9600    // 波特率设置  支持的波特率：19200,9600,2400,4800
#define BIT_RESOLUTION      8       // 8, 16, 4

struct COM_PORT
{
    GPIO_TypeDef *RXPort;
    uint16_t RXPin;
    GPIO_TypeDef *TXPort;
    uint16_t TXPin;
};

static struct COM_PORT COM_Port[COM_NUM] = {
    VIRUAL_SERIAL_PORT(COM1),
};

VSPD_TypeDef VSPD_Tx, VSPD_Rx;

static uint16_t s_Tx_interval = 0;
static uint16_t s_Rx_interval[5] = {0};
volatile static uint16_t s_Send_bit = 0;
volatile static uint16_t s_Rcv_bit = 0;
volatile static uint8_t s_Rcv_count = 0;
volatile static uint8_t s_Rcv_Sampledata = 0;


static void Virtual_Serial_GPIO_Config(uint8_t COMx);
static void Virtual_Serial_EXTILine_Config(uint8_t COMx);
static void Virtual_Serial_TIM_Config(void);
static char VSPD_SendFrame(uint8_t Data);


uint8_t temp[2] = {0};

void Virtual_Serial_Init(void)
{
    uint8_t port, i;
    uint16_t resolution = 0;

    // 9600波特率位宽 104.17us * 1Mz = 104
    s_Tx_interval = 1000000 / BAUDRATE + 0.5;
    
    resolution = s_Tx_interval / BIT_RESOLUTION;

    s_Rx_interval[0] = resolution * ((BIT_RESOLUTION - 2) >> 1);
    for (i = 1; i < 3; i++)
    {
        s_Rx_interval[i] = resolution;
    }
    s_Rx_interval[i] = 2 * s_Rx_interval[0];
    s_Rx_interval[++i] = s_Rx_interval[0] + 15 * s_Tx_interval;  //  1.5 byte overflow time

    for (port = 0; port < COM_NUM; port++)
    {
        Virtual_Serial_GPIO_Config(port);
        Virtual_Serial_EXTILine_Config(port);
    }

    Virtual_Serial_TIM_Config();

    VSPD_RcvString(COM1, temp, sizeof(temp));
}

/**
  * @brief  Configures EXTI Line6 (connected to PB6 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void Virtual_Serial_EXTILine_Config(uint8_t COMx)
{
    uint8_t i = 0;
    uint16_t pin = COM_Port[COMx].RXPin;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    while (pin >>= 1)
    {
        i++;
    }

    /* Connect EXTI Line_x to Rx pin */
    switch ((uint32_t)COM_Port[COMx].RXPort)
    {
    case GPIOA_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0 + i);
        break;
    case GPIOB_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0 + i);
        break;
    case GPIOC_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0 + i);
        break;
    case GPIOD_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0 + i);
        break;
    case GPIOE_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0 + i);
        break;
    case GPIOF_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource0 + i);
        break;
    case GPIOG_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG, EXTI_PinSource0 + i);
        break;
    case GPIOH_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOH, EXTI_PinSource0 + i);
        break;
    case GPIOI_BASE:
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOI, EXTI_PinSource0 + i);
        break;
    }

    /* Configure EXTI Line_x */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 << i;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line_x Interrupt */
    switch (i)
    {
    case 0:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
        break;
    case 1:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
        break;
    case 2:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
        break;
    case 3:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
        break;
    case 4:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
        break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
        break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
        break;
    default:
        break;
    }

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void Virtual_Serial_GPIO_Config(uint8_t COMx)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (COM_Port->TXPort != NULL)
    {
        switch ((uint32_t)COM_Port[COMx].TXPort)
        {
        case GPIOA_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
            break;
        case GPIOB_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
            break;
        case GPIOC_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
            break;
        case GPIOD_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
            break;
        case GPIOE_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
            break;
        case GPIOF_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
            break;
        case GPIOG_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
            break;
        case GPIOH_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
            break;
        case GPIOI_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
            break;
        }

        /* Configure GPIO pin as push-pull up output */
        GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Pin = COM_Port[COMx].TXPin;
        GPIO_Init(COM_Port[COMx].TXPort, &GPIO_InitStructure);
        GPIO_SetBits(COM_Port[COMx].TXPort, COM_Port[COMx].TXPin);
    }

    if (COM_Port->RXPort != NULL)
    {
        switch ((uint32_t)COM_Port[COMx].RXPort)
        {
        case GPIOA_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
            break;
        case GPIOB_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
            break;
        case GPIOC_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
            break;
        case GPIOD_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
            break;
        case GPIOE_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
            break;
        case GPIOF_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
            break;
        case GPIOG_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
            break;
        case GPIOH_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
            break;
        case GPIOI_BASE:
            RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
            break;
        }

        GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_Pin = COM_Port[COMx].RXPin;
        GPIO_Init(COM_Port[COMx].RXPort, &GPIO_InitStructure);
    }
}

static void Virtual_Serial_TIM_Config(void)
{
    uint16_t PrescalerValue = 0;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    PrescalerValue = (uint16_t)(SystemCoreClock / 2 / 1000000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue; // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(VSPD_TIM, &TIM_TimeBaseStructure);

    /* 配置定时器中断优先级并使能 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Output Compare Timing Mode configuration: Channelx */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_Pulse = 0xFFFF;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    /* Virtual Serial Send port */
    TIM_OC1Init(VSPD_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(VSPD_TIM, TIM_OCPreload_Disable); // 禁用通道重载缓冲器

    // TIM_ITConfig(VSPD_TIM, TIM_IT_CC1, ENABLE);

    /* Virtual Serial Receive port */
    TIM_OC2Init(VSPD_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(VSPD_TIM, TIM_OCPreload_Disable); // 禁用通道重载缓冲器

    // TIM_ITConfig(VSPD_TIM, TIM_IT_CC2, ENABLE);

    TIM_Cmd(VSPD_TIM, ENABLE);
}

static void VSPD_Tx_Comm(VSP_ID_TypeDef ID, bool Status)
{
    if (HIGH == Status)
        GPIO_SetBits(COM_Port[ID].TXPort, COM_Port[ID].TXPin);
    else
        GPIO_ResetBits(COM_Port[ID].TXPort, COM_Port[ID].TXPin);
}

char VSPD_SendByte(VSP_ID_TypeDef ID, uint8_t Data)
{
    char status;
    uint16_t timer_cnt;

    status = VSPD_SendFrame(Data);

    if (0 == status)
    {
        VSPD_Tx.id = ID;
        VSPD_Tx.len = 0;
        VSPD_Tx.max_len = 1;

        timer_cnt = TIM_GetCounter(VSPD_TIM);
        TIM_SetCompare1(VSPD_TIM, timer_cnt + s_Tx_interval);
        VSPD_SEND_START();
    }

    return status;
}

char VSPD_SendString(VSP_ID_TypeDef ID, uint8_t *pBuf, uint16_t Length)
{
    char status;
    uint16_t timer_cnt;

    status = VSPD_SendFrame(*pBuf);

    if ((0 == status) && (pBuf != NULL))
    {
        VSPD_Tx.id = ID;
        VSPD_Tx.len = 0;
        VSPD_Tx.max_len = Length;
        VSPD_Tx.pbuf = pBuf;

        timer_cnt = TIM_GetCounter(VSPD_TIM);
        TIM_SetCompare1(VSPD_TIM, timer_cnt + s_Tx_interval);
        VSPD_SEND_START();
    }

    return status;
}

char VSPD_RcvString(VSP_ID_TypeDef ID, uint8_t *pBuf, uint16_t Length)
{
    if (VSPD_Rx.busy)
        return -1;

    VSPD_Rx.id = ID;
    VSPD_Rx.max_len = Length;
    VSPD_Rx.pbuf = pBuf;
    VSPD_Rx.busy = 0;

    return 0;
}

static char VSPD_SendFrame(uint8_t Data)
{
    if (VSPD_Tx.busy)
        return -1;

    VSPD_Tx.frame = 0x200 | (Data << 1); // add stop bit and start bit
    VSPD_Tx.busy = 1;
    s_Send_bit = 1;

    return 0;
}

static void VSPD_TimSend_Deal(void)
{
    if (s_Send_bit >= 0x400)
    {
        VSPD_Tx.len++;
        VSPD_Tx.busy = 0;

        if (VSPD_Tx.len < VSPD_Tx.max_len)
        {
            VSPD_SendFrame(*(VSPD_Tx.pbuf + VSPD_Tx.len));
        }
        else
        {
            VSPD_SEND_STOP();
        }
    }
    else
    {
        VSPD_Tx_Comm(VSPD_Tx.id, (bool)(VSPD_Tx.frame & s_Send_bit));   // LSB
        s_Send_bit <<= 1;
    }
}

static void VSPD_TimRcv_Deal(void)
{
    s_Rcv_Sampledata += GPIO_ReadInputDataBit(COM_Port[VSPD_Rx.id].RXPort, COM_Port[VSPD_Rx.id].RXPin);

    if(2 == s_Rcv_count)
    {
        // s_Rcv_count = 1;
        VSPD_Rx.frame |= ((s_Rcv_Sampledata & 0x02)?1:0) << s_Rcv_bit; // Noise detection from sampled data
        if((9 == s_Rcv_bit) && (0x200 == (VSPD_Rx.frame & 0x201)))
        {
            *(VSPD_Rx.pbuf + VSPD_Rx.len) = (uint8_t)((VSPD_Rx.frame & 0x1FE) >> 1);    // Extract data, remove stop bit and start bit
            if(++VSPD_Rx.len >= VSPD_Rx.max_len)
                VSPD_Rx.len = 0;

            s_Rcv_count = 4;    // set overflow time

            EXTI->PR |= (EXTI_Line7);
            EXTI->IMR |= (EXTI_Line7);
            return;
        }
        s_Rcv_Sampledata = 0;
        s_Rcv_bit++;
    }
    else if(3 == s_Rcv_count)
        s_Rcv_count = 0;
    else if(s_Rcv_count >= 4)
    {
        VSPD_Rx.busy = 0;
        VSPD_RECEIVE_STOP();
    }

    s_Rcv_count++;
}

void TIM8_BRK_TIM12_IRQHandler(void)
{
    uint16_t timer_cnt = 0;

    timer_cnt = TIM_GetCounter(VSPD_TIM);

    if (TIM_GetITStatus(VSPD_TIM, TIM_IT_CC1) != RESET)
    {
        VSPD_TimSend_Deal();

        TIM_SetCompare1(VSPD_TIM, timer_cnt + s_Tx_interval);

        TIM_ClearITPendingBit(VSPD_TIM, TIM_IT_CC1);
    }

    if (TIM_GetITStatus(VSPD_TIM, TIM_IT_CC2) != RESET)
    {
        VSPD_TimRcv_Deal();

        TIM_SetCompare2(VSPD_TIM, timer_cnt + s_Rx_interval[s_Rcv_count]);

        TIM_ClearITPendingBit(VSPD_TIM, TIM_IT_CC2);
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint16_t timer_cnt = 0;

    if (EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        timer_cnt = TIM_GetCounter(VSPD_TIM);

        EXTI->IMR &= ~(EXTI_Line7);

        // VSPD_RECEIVE_STOP();

        VSPD_Rx.busy = 1;
        // VSPD_Rx.len = 0;
        VSPD_Rx.frame = 0;

        s_Rcv_bit = 0;
        s_Rcv_count = 0;
        s_Rcv_Sampledata = 0;
        TIM_SetCompare2(VSPD_TIM, timer_cnt + s_Rx_interval[s_Rcv_count]);

        VSPD_RECEIVE_START();

        /* Clear the EXTI line 7 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}
