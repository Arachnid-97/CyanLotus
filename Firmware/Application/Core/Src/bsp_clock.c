#include "bsp_clock.h"

/*
 * m: VCO输入时钟 分频因子, 取值 2~63
 * n: VCO输出时钟 倍频因子, 取值 192~432
 * p: PLLCLK时钟分频因子  , 取值 2,4,6,8
 * q: OTG_FS,SDIO,RNG时钟分频因子, 取值4~15
 * 函数调用举例,使用 HSI设置时钟
 * SYSCLK=HCLK=180M, PCLK2=HCLK/2=90M, PCLK1=HCLK/4=45M
 * PLL = HSI * n / (m * p) = 16MHz * 336 / (8 * 2) = 168MHz
 * HSI_SetSysClock(16,336,2,7);
 * HSE作为时钟来源,经过 PLL倍频作为系统时钟, 这是通常的做法

 * 系统时钟超频到 216M爽一下
 * HSI_SetSysClock(16, 432, 2, 9);
 */
void HSI_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
    __IO uint32_t HSIStartUpStatus = 0;

    // 把 RCC外设初始化成复位状态
    RCC_DeInit();

    // 使能 HSI, HSI=16M
    RCC_HSICmd(ENABLE);

    // 等待 HSI就绪
    HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;

    // 只有 HSI就绪之后则继续往下执行
    if (HSIStartUpStatus == RCC_CR_HSIRDY)
    {
        // 调压器电压输出级别配置为 1,以便在器件为最大频率
        // 工作时使性能和功耗实现平衡
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_VOS;

        // HCLK = SYSCLK / 1
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK / 2
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // PCLK1 = HCLK / 4
        RCC_PCLK1Config(RCC_HCLK_Div4);

        // 如果要超频就得在这里下手啦
        // 设置 PLL来源时钟, 设置 VCO分频因子 m, 设置 VCO倍频因子 n,
        // 设置系统时钟分频因子 p, 设置 OTG_FS,SDIO,RNG分频因子 q
        RCC_PLLConfig(RCC_PLLSource_HSI, m, n, p, q);

        // 使能 PLL
        RCC_PLLCmd(ENABLE);

        // 等待 PLL稳定
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /*-----------------------------------------------------*/
#ifdef STM32F429_439xx
        // 开启 OVER-RIDE模式, 以能达到更高频率
        PWR->CR |= PWR_CR_ODEN;
        while ((PWR->CSR & PWR_CSR_ODRDY) == 0)
        {
        }
        PWR->CR |= PWR_CR_ODSWEN;
        while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
        {
        }
#endif /* STM32F429_439xx */
        // 配置 FLASH预取指,指令缓存, 数据缓存和等待状态
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
        /*-----------------------------------------------------*/

        // 当 PLL稳定之后, 把 PLL时钟切换为系统时钟 SYSCLK
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // 读取时钟切换状态位, 确保 PLLCLK被选为系统时钟
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { // HSI启动出错处理
        while (1)
        {
        }
    }
}

/*
 * m: VCO输入时钟 分频因子, 取值 2~63
 * n: VCO输出时钟 倍频因子, 取值 192~432
 * p: SYSCLK时钟分频因子 , 取值 2, 4, 6, 8
 * q: OTG_FS,SDIO,RNG时钟分频因子, 取值 4~15
 * 函数调用举例, 使用 HSE 设置时钟
 * SYSCLK=HCLK=180M, PCLK2=HCLK/2=90M, PCLK1=HCLK/4=45M
 * HSE_SetSysClock(25, 360, 2, 7);
 * HSE 作为时钟来源, 经过 PLL倍频作为系统时钟, 这是通常的做法

 * 系统时钟超频到 216M爽一下
 * HSE_SetSysClock(25, 432, 2, 9);
 */
void HSE_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
    __IO uint32_t HSEStartUpStatus = 0;

    // 使能 HSE, 开启外部晶振
    RCC_HSEConfig(RCC_HSE_ON);

    // 等待 HSE启动稳定
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        // 调压器电压输出级别配置为 1, 以便在器件为最大频率
        // 工作时使性能和功耗实现平衡
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_VOS;

        // HCLK = SYSCLK / 1
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK / 2
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // PCLK1 = HCLK / 4
        RCC_PCLK1Config(RCC_HCLK_Div4);

        // 如果要超频就得在这里下手啦
        // 设置 PLL来源时钟, 设置 VCO分频因子 m, 设置 VCO倍频因子 n, 
        // 设置系统时钟分频因子 p, 设置 OTG_FS,SDIO,RNG分频因子 q
        RCC_PLLConfig(RCC_PLLSource_HSE, m, n, p, q);

        // 使能 PLL
        RCC_PLLCmd(ENABLE);

        // 等待 PLL 稳定
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
        /*-----------------------------------------------------*/
#ifdef STM32F429_439xx
        //开启 OVER-RIDE 模式, 以能达到更高频率
        PWR->CR |= PWR_CR_ODEN;
        while ((PWR->CSR & PWR_CSR_ODRDY) == 0)
        {
        }
        PWR->CR |= PWR_CR_ODSWEN;
        while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
        {
        }
#endif /* STM32F429_439xx */
        // 配置 FLASH 预取指,指令缓存,数据缓存和等待状态
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
        /*-----------------------------------------------------*/

        // 当 PLL稳定之后, 把 PLL时钟切换为系统时钟 SYSCLK
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // 读取时钟切换状态位, 确保 PLLCLK被选为系统时钟
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { // HSE 启动出错处理
        while (1)
        {
        }
    }
}

/*---------------------------- END OF FILE ----------------------------*/
