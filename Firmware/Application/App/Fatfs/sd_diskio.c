#include "sd_diskio.h"
#include "diskio.h"
#include <string.h>
#include "./SDCard/sdcard_base.h"
#include "./SDCard/stm324x9i_eval_sdio_sd.h"


extern SD_CardInfo SDCardInfo;

uint8_t SD_ioctl(BYTE cmd, void *buff)
{
    int result = 0;

    switch (cmd) {
    case CTRL_SYNC :        /* Wait for end of internal write process of the drive */
        result = 1;
        break;

    case GET_SECTOR_COUNT : /* Get drive capacity in unit of sector (DWORD) */
        *(DWORD * )buff = SDCardInfo.CardCapacity /SDCardInfo.CardBlockSize;
        result = 1;
        break;

    case GET_SECTOR_SIZE :  // Get R/W sector size (WORD) 
        *(WORD * )buff = SD_BLOCK_SIZE;
        result = 1;
        break;

    case GET_BLOCK_SIZE :   /* Get erase block size in unit of sector (DWORD) */
        *(DWORD * )buff = 1;
        result = 1;
        break;

    case CTRL_TRIM :        /* Erase a block of sectors (used when _USE_ERASE == 1) */
        result = 1;
        break;

    default:
        break;
    }

    return result;
}



/* User defined Fatfs functions ----------------------------------------------*/
/*-----------------------------------------------------------------------*/
/* Get SD disk status                                                    */
/*-----------------------------------------------------------------------*/
uint8_t SD_disk_status(void)
{
    return 1;
}

/*-----------------------------------------------------------------------*/
/* Initialize SD disk drive                                              */
/*-----------------------------------------------------------------------*/
uint8_t SD_disk_initialize(void)
{
    if(SD_Init() == SD_OK)
        return 1;
    else
        return 0;
}

/*-----------------------------------------------------------------------*/
/* Read SD sector(s)                                                     */
/*-----------------------------------------------------------------------*/
uint8_t SD_disk_read( uint8_t *Buff, uint32_t Sector, uint32_t Count )
{
    SD_Error SD_state = SD_OK;
	uint8_t temp = 0;

	if((uint32_t)Buff & 3)
	{
		_Bool flag = 1;
		uint32_t scratch[SD_BLOCK_SIZE / 4];
		while(Count--)
		{
			flag = SD_disk_read((void *)scratch, Sector++, 1);

			if(flag != 1)
			{
				break;
			}
			memcpy(Buff, scratch, SD_BLOCK_SIZE);
			Buff += SD_BLOCK_SIZE;
		}
	}
	else
	{
		SD_state = SD_ReadMultiBlocks(Buff,(uint64_t)Sector*SD_BLOCK_SIZE,	\
										SD_BLOCK_SIZE,Count);
	}
	
    if(SD_state == SD_OK)
    {
        /* Check if the Transfer is finished */
        SD_state=SD_WaitReadOperation();
        while(SD_GetStatus() != SD_TRANSFER_OK);
    }
	if(SD_state != SD_OK)
		temp = 0;
	else
		temp = 1;
	
	return temp;
}

/*-----------------------------------------------------------------------*/
/* Write SD sector(s)                                                    */
/*-----------------------------------------------------------------------*/
uint8_t SD_disk_write( const uint8_t *Buff, uint32_t Sector, uint32_t Count )
{
    SD_Error SD_state = SD_OK;
	uint8_t temp = 0;

	if((uint32_t)Buff & 3)
	{
		_Bool flag = 1;
		uint32_t scratch[SD_BLOCK_SIZE / 4];
		while(Count--)
		{
			memcpy(scratch, Buff, SD_BLOCK_SIZE);
			flag = SD_disk_write((void *)scratch, Sector++, 1);

			if(flag != 1)
			{
				break;
			}
			Buff += SD_BLOCK_SIZE;
		}
	}
	else
	{
		SD_state = SD_WriteMultiBlocks((uint8_t *)Buff,(uint64_t)Sector*SD_BLOCK_SIZE,	\
										SD_BLOCK_SIZE,Count);
	}
	
    if(SD_state == SD_OK)
    {
        /* Check if the Transfer is finished */
        SD_state=SD_WaitWriteOperation();
        while(SD_GetStatus() != SD_TRANSFER_OK);
    }
	if(SD_state != SD_OK)
		temp = 0;
	else
		temp = 1;
	
	return temp;
}


/*---------------------------- END OF FILE ----------------------------*/


