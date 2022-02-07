#include "bsp_rng.h"


extern void SoftwareDelay_ms(uint32_t Cnt);

uint8_t RNG_Init(void)
{
    uint16_t retry = 0;

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);

    RNG_DeInit();

    RNG_Cmd(ENABLE);

    while(RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET && retry < 100) // 等待 RNG准备就绪
    {
        retry++;
        SoftwareDelay_ms(10);
    }

    if(retry >= 100)
        return 1; // 随机数产生器工作异常

    return 0;
}

inline uint32_t RNG_Get_RandomNum(void)
{
    return RNG_GetRandomNumber();
}

uint32_t RNG_Get_RandomRange(uint32_t Min,uint32_t Max)
{
    return RNG_GetRandomNumber() % (Max-Min+1) + Min;
}


/*---------------------------- END OF FILE ----------------------------*/


