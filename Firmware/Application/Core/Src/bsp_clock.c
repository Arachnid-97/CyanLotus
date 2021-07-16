#include "bsp_clock.h"


/*
 * ʹ��HSIʱ������ϵͳʱ�ӵĲ���
 * 1������HSI �����ȴ� HSI �ȶ�
 * 2������ AHB��APB2��APB1��Ԥ��Ƶ����
 * 3������PLL��ʱ����Դ
 *    ����VCO����ʱ�� ��Ƶ����        m
 *    ����VCO���ʱ�� ��Ƶ����        n
 *    ����SYSCLKʱ�ӷ�Ƶ����          p
 *    ����OTG FS,SDIO,RNGʱ�ӷ�Ƶ���� q
 * 4������PLL�����ȴ�PLL�ȶ�
 * 5����PLLCK�л�Ϊϵͳʱ��SYSCLK
 * 6����ȡʱ���л�״̬λ��ȷ��PLLCLK��ѡΪϵͳʱ��
 */

/*
 * m: VCO����ʱ�� ��Ƶ���ӣ�ȡֵ2~63
 * n: VCO���ʱ�� ��Ƶ���ӣ�ȡֵ192~432
 * p: PLLCLKʱ�ӷ�Ƶ����  ��ȡֵ2��4��6��8
 * q: OTG FS,SDIO,RNGʱ�ӷ�Ƶ���ӣ�ȡֵ4~15
 * �������þ�����ʹ��HSI����ʱ��
 * SYSCLK = HCLK = 180M,PCLK2 = HCLK/2 = 90M,PCLK1 = HCLK/4 = 45M
 * PLL = HSI * n / (m * p) = 16MHz * 336 / (8 * 2) = 168MHz
 * HSI_SetSysClock(16, 360, 2, 7);
 * HSE��Ϊʱ����Դ������PLL��Ƶ��Ϊϵͳʱ�ӣ�����ͨ��������

 * ϵͳʱ�ӳ�Ƶ��216Mˬһ��
 * HSI_SetSysClock(16, 432, 2, 9);
 */

void HSI_SetSysClock(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
    __IO uint32_t HSIStartUpStatus = 0;

    // ��RCC�����ʼ���ɸ�λ״̬
    RCC_DeInit();

    //ʹ��HSI, HSI=16M
    RCC_HSICmd(ENABLE);

    // �ȴ� HSI ����
    HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;

    // ֻ�� HSI����֮�����������ִ��
    if (HSIStartUpStatus == RCC_CR_HSIRDY)
    {
        // ��ѹ����ѹ�����������Ϊ1���Ա�������Ϊ���Ƶ��
        // ����ʱʹ���ܺ͹���ʵ��ƽ��
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_VOS;

        // HCLK = SYSCLK / 1
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK / 2
        RCC_PCLK2Config(RCC_HCLK_Div2);

        // PCLK1 = HCLK / 4
        RCC_PCLK1Config(RCC_HCLK_Div4);

        // ���Ҫ��Ƶ�͵�������������
        // ����PLL��Դʱ�ӣ�����VCO��Ƶ����m������VCO��Ƶ����n��
        //  ����ϵͳʱ�ӷ�Ƶ����p������OTG FS,SDIO,RNG��Ƶ����q
        RCC_PLLConfig(RCC_PLLSource_HSI, m, n, p, q);

        // ʹ��PLL
        RCC_PLLCmd(ENABLE);

        // �ȴ� PLL�ȶ�
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /*-----------------------------------------------------*/
#ifdef STM32F429_439xx
        //���� OVER-RIDEģʽ�����ܴﵽ����Ƶ��
        PWR->CR |= PWR_CR_ODEN;
        while ((PWR->CSR & PWR_CSR_ODRDY) == 0)
        {
        }
        PWR->CR |= PWR_CR_ODSWEN;
        while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
        {
        }
#endif /* STM32F429_439xx */
        // ����FLASHԤȡָ,ָ���,���ݻ���͵ȴ�״̬
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;
        /*-----------------------------------------------------*/

        // ��PLL�ȶ�֮�󣬰�PLLʱ���л�Ϊϵͳʱ��SYSCLK
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // ��ȡʱ���л�״̬λ��ȷ��PLLCLK��ѡΪϵͳʱ��
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { // HSI����������
        while (1)
        {
        }
    }
}


/*---------------------------- END OF FILE ----------------------------*/


