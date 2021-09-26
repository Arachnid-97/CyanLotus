#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H


#include "ff.h"


uint8_t SD_ioctl(BYTE cmd, void *buff);

uint8_t SD_disk_status(void);
uint8_t SD_disk_initialize(void);
uint8_t SD_disk_read( uint8_t *Buff, uint32_t Sector, uint32_t Count );
uint8_t SD_disk_write( const uint8_t *Buff, uint32_t Sector, uint32_t Count );


#endif /* __SD_DISKIO_H */


/*---------------------------- END OF FILE ----------------------------*/


