#include "bsp_flash.h"


/************************************************
函数名称 ： Flash_ReadHalfWord
功    能 ： 从 Flash连续读 n个字（16 bit）
参    数 ： address ---- 地址位
			Buff ---- 读取的数据
			Len ---- 长度
返 回 值 ： 无
*************************************************/
void Flash_Read_nWord( uint32_t Addr, uint8_t *Buff, uint16_t Len )
{
    uint32_t i, cnt = Len >> 2;
    uint32_t *pData = (uint32_t*)Buff;
    uint32_t *pAddr = (uint32_t*)Addr;

    if(Len & 0x03) cnt++;

    for(i = 0; i < cnt; i++) {
        *pData++ = *pAddr;
        pAddr++;
    }
}

/************************************************
函数名称 ： Flash_WriteWord
功    能 ： 向 Flash写 n个字（32 bit）
参    数 ： address ---- 地址位
			Buff ---- 存储的数据
			Len ---- 长度
返 回 值 ： 0 / 1
*************************************************/
uint32_t Flash_Write_nWord( uint32_t Addr, uint8_t *Buff, uint16_t Len )
{
    uint32_t i, cnt = Len >> 2;
    uint32_t *pData = (uint32_t*)Buff;

    if(Len & 0x03) cnt++;

    FLASH_Unlock();

    /* 清空所有标志位 */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

    for(i = 0; i < cnt; i++) {
        if(FLASH_ProgramWord(Addr, *pData) == FLASH_COMPLETE) {
            pData++;
            Addr += 4;
        }
        else{
            break;
        }
    }

    FLASH_Lock();

    return i*4;
}

/************************************************
函数名称 ： ReadFlash_HalfWord
功    能 ： 从 Flash读半个字（16 bit）
参    数 ： address ---- 地址位
			Buff ---- 读取的数据
			Len ---- 长度
返 回 值 ： 无
*************************************************/
uint16_t ReadFlash_HalfWord( uint32_t Addr )
{
    uint32_t Address;

    Address = WRITE_START_ADDR + Addr;

    return (*(vu16*) Address);				//读指定地址的半个字的数据
}

/**************************************************************** 
 * Function:    Flash_EnableReadProtection 
 * Description: Enable the read protection of user flash area. 
 * Input: 
 * Output: 
 * Return:      1: Read Protection successfully enable 
 *              2: Error: Flash read unprotection failed 
*****************************************************************/
int Flash_EnableReadProtection(void)
{
    /* Returns the FLASH Read Protection level. */
    if (FLASH_OB_GetRDP() == RESET)
    {
        /* Unlock the Option Bytes */
        FLASH_OB_Unlock();

        /* Sets the read protection level. */
        FLASH_OB_RDPConfig(OB_RDP_Level_1);

        /* Start the Option Bytes programming process. */
        if (FLASH_OB_Launch() != FLASH_COMPLETE)
        {
            /* Disable the Flash option control register access (recommended to protect
            the option Bytes against possible unwanted operations) */
            FLASH_OB_Lock();

            /* Error: Flash read unprotection failed */
            return (-1);
        }

        /* Disable the Flash option control register access (recommended to protect
        the option Bytes against possible unwanted operations) */
        FLASH_OB_Lock();

        /* Read Protection successfully enable */
        return (1);
    }

    /* Read Protection successfully enable */
    return (0);
}

/**************************************************************** 
 * Function:    Flash_DisableReadProtection 
 * Description: Disable the read protection of user flash area. 
 * Input: 
 * Output: 
 * Return:      1: Read Protection successfully disable 
 *              2: Error: Flash read unprotection failed 
*****************************************************************/
uint32_t Flash_DisableReadProtection(void)
{
    /* Returns the FLASH Read Protection level. */
    if (FLASH_OB_GetRDP() != RESET)
    {
        /* Unlock the Option Bytes */
        FLASH_OB_Unlock();

        /* Sets the read protection level. */
        FLASH_OB_RDPConfig(OB_RDP_Level_0);

        /* Start the Option Bytes programming process. */
        if (FLASH_OB_Launch() != FLASH_COMPLETE)
        {
            /* Disable the Flash option control register access (recommended to protect
            the option Bytes against possible unwanted operations) */
            FLASH_OB_Lock();

            /* Error: Flash read unprotection failed */
            return (-1);
        }

        /* Disable the Flash option control register access (recommended to protect
        the option Bytes against possible unwanted operations) */
        FLASH_OB_Lock();

        /* Read Protection successfully disable */
        return (1);
    }

    /* Read Protection successfully disable */
    return (0);
}


/*---------------------------- END OF FILE ----------------------------*/


