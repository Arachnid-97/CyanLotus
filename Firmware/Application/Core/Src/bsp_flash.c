#include "bsp_flash.h"


/************************************************
�������� �� Flash_ReadHalfWord
��    �� �� �� Flash������ n���֣�16 bit��
��    �� �� address ---- ��ַλ
			Buff ---- ��ȡ������
			Len ---- ����
�� �� ֵ �� ��
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
�������� �� Flash_WriteWord
��    �� �� �� Flashд n���֣�32 bit��
��    �� �� address ---- ��ַλ
			Buff ---- �洢������
			Len ---- ����
�� �� ֵ �� 0 / 1
*************************************************/
uint32_t Flash_Write_nWord( uint32_t Addr, uint8_t *Buff, uint16_t Len )
{
    uint32_t i, cnt = Len >> 2;
    uint32_t *pData = (uint32_t*)Buff;

    if(Len & 0x03) cnt++;

    FLASH_Unlock();

    /* ������б�־λ */
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
�������� �� ReadFlash_HalfWord
��    �� �� �� Flash������֣�16 bit��
��    �� �� address ---- ��ַλ
			Buff ---- ��ȡ������
			Len ---- ����
�� �� ֵ �� ��
*************************************************/
uint16_t ReadFlash_HalfWord( uint32_t Addr )
{
    uint32_t Address;

    Address = WRITE_START_ADDR + Addr;

    return (*(vu16*) Address);				//��ָ����ַ�İ���ֵ�����
}


/*---------------------------- END OF FILE ----------------------------*/


