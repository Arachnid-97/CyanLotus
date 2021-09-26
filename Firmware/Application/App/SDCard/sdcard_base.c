#include "./SDCard/sdcard_base.h"
#include "./SDCard/stm324x9i_eval_sdio_sd.h"


/************************************************
函数名称 ： SD_LowLevel_DMA_TxConfig
功    能 ： 配置 SDIO DMA发送请求
参    数 ： BufferDST ---- 指向目标缓冲区的指针
            BufferSize ---- 缓冲区大小
返 回 值 ： 无
*************************************************/
void SD_LowLevel_DMA_TxConfig( uint32_t *BufferSRC, uint32_t BufferSize )
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF \
        | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF \
                | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

    /* DMA2 Stream3  or Stream6 disable */
    DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

    /* DMA2 Stream3  or Stream6 Config */
    DMA_DeInit(SD_SDIO_DMA_STREAM);

    DMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = BufferSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    DMA_Init(SD_SDIO_DMA_STREAM, &DMA_InitStructure);

    DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
    DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

    /* DMA2 Stream3  or Stream6 enable */
    DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

/************************************************
函数名称 ： SD_LowLevel_DMA_RxConfig
功    能 ： 配置 SDIO DMA接收请求
参    数 ： BufferDST ---- 指向目标缓冲区的指针
            BufferSize ---- 缓冲区大小
返 回 值 ： 无
*************************************************/
void SD_LowLevel_DMA_RxConfig( uint32_t *BufferDST, uint32_t BufferSize )
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF \
        | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF \
                | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

    /* DMA2 Stream3  or Stream6 disable */
    DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

    /* DMA2 Stream3 or Stream6 Config */
    DMA_DeInit(SD_SDIO_DMA_STREAM);

    DMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = BufferSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    DMA_Init(SD_SDIO_DMA_STREAM, &DMA_InitStructure);

    DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
    DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

    /* DMA2 Stream3 or Stream6 enable */
    DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

/************************************************
函数名称 ： SD_LowLevel_Init
功    能 ： SD底层初始化
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SD_LowLevel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Dx /CLK and CMD clock enable */
    SD_SDIO_IO_CLOCK_FUN(SD_Dx_CLK | SD_CLK_CLK | SD_CMD_CLK, ENABLE);
    
#if USING_SD_DETECT
    /* sd detect clock enable */
    SD_DETECT_CLOCK_FUN(SD_DETECT_GPIO_CLK, ENABLE);

#endif /* USING_SD_DETECT */
    
    GPIO_PinAFConfig(SD_D0_PORT, SD_D0_SOURCE, SD_SDIO_GPIO_AF_MAP);
    GPIO_PinAFConfig(SD_D1_PORT, SD_D1_SOURCE, SD_SDIO_GPIO_AF_MAP);
    GPIO_PinAFConfig(SD_D2_PORT, SD_D2_SOURCE, SD_SDIO_GPIO_AF_MAP);
    GPIO_PinAFConfig(SD_D3_PORT, SD_D3_SOURCE, SD_SDIO_GPIO_AF_MAP);
    GPIO_PinAFConfig(SD_CLK_PORT, SD_CLK_SOURCE, SD_SDIO_GPIO_AF_MAP);
    GPIO_PinAFConfig(SD_CMD_PORT, SD_CMD_SOURCE, SD_SDIO_GPIO_AF_MAP);

    /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
    GPIO_InitStructure.GPIO_Pin = SD_D0_PINS | SD_D1_PINS \
                                    | SD_D2_PINS | SD_D3_PINS;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SD_SDIO_IO_PORT, &GPIO_InitStructure);

    /* Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = SD_CMD_PINS;
    GPIO_Init(SD_CMD_PORT, &GPIO_InitStructure);

    /* Configure PC.12 pin: CLK pin */
    GPIO_InitStructure.GPIO_Pin = SD_CLK_PINS;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SD_CLK_PORT, &GPIO_InitStructure);

#if USING_SD_DETECT
    /* Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
    GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

#endif /* USING_SD_DETECT */

    /* Enable the SDIO Clock */
    SD_SDIO_CLOCK_FUN(SD_SDIO_CLK, ENABLE);

    /* Enable the DMAx Clock */
    SD_SDIO_DMA_CLOCK_FUN(SD_SDIO_DMA_CLK, ENABLE);
    
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_Init(&NVIC_InitStructure);  
}

/************************************************
函数名称 ： SD_LowLevel_DeInit
功    能 ： SD底层复位
参    数 ： 无
返 回 值 ： 无
*************************************************/
void SD_LowLevel_DeInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Disable SDIO Clock */
    SDIO_ClockCmd(DISABLE);

    /* Set Power State to OFF */
    SDIO_SetPowerState(SDIO_PowerState_OFF);

    /* DeInitializes the SDIO peripheral */
    SDIO_DeInit();

    /* Disable the SDIO Clock */
    SD_SDIO_CLOCK_FUN(SD_SDIO_CLK, DISABLE);

    GPIO_PinAFConfig(SD_D0_PORT, SD_D0_SOURCE, GPIO_AF_MCO);
    GPIO_PinAFConfig(SD_D1_PORT, SD_D1_SOURCE, GPIO_AF_MCO);
    GPIO_PinAFConfig(SD_D2_PORT, SD_D2_SOURCE, GPIO_AF_MCO);
    GPIO_PinAFConfig(SD_D3_PORT, SD_D3_SOURCE, GPIO_AF_MCO);
    GPIO_PinAFConfig(SD_CLK_PORT, SD_CLK_SOURCE, GPIO_AF_MCO);
    GPIO_PinAFConfig(SD_CMD_PORT, SD_CMD_SOURCE, GPIO_AF_MCO);

    /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
    GPIO_InitStructure.GPIO_Pin = SD_D0_PINS | SD_D1_PINS \
                                    | SD_D2_PINS | SD_D3_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SD_SDIO_IO_PORT, &GPIO_InitStructure);

    /* Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = SD_CMD_PINS;
    GPIO_Init(SD_CMD_PORT, &GPIO_InitStructure);

    /* Configure PC.12 pin: CLK pin */
    GPIO_InitStructure.GPIO_Pin = SD_CLK_PINS;
    GPIO_Init(SD_CLK_PORT, &GPIO_InitStructure);
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/******************************************************************************/

/**
  * @brief  This function handles SDIO global interrupt request.
  * @param  None
  * @retval None
  */
void SDIO_IRQHandler(void)
{
    /* Process All SDIO Interrupt Sources */
    SD_ProcessIRQSrc();
}

/**
  * @brief  This function handles DMA2 Stream3 or DMA2 Stream6 global interrupts
  *         requests.
  * @param  None
  * @retval None
  */
void SD_SDIO_DMA_IRQHANDLER(void)
{
    /* Process DMA2 Stream3 or DMA2 Stream6 Interrupt Sources */
    SD_ProcessDMAIRQ();
}


/*---------------------------- END OF FILE ----------------------------*/


