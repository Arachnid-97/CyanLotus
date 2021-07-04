#ifndef __USER_FATFS_H
#define __USER_FATFS_H


#include "stm32f4xx.h"
#include "ff.h"


#define FF_MAX_SS			4096

extern BYTE FF_Buff[FF_MAX_SS];

void FF_Test(void);
void FF_System_Creates( char *Drive, uint8_t Opt );
uint8_t FF_OpenWrite( char *pFile, void *pStr, uint16_t Len );
uint8_t FF_OpenRead( char *pFile, void *pStr, uint16_t Len );
void FF_ViewRootDir( char *pDrive );


#endif	/* __USER_FATFS_H */


/*---------------------------- END OF FILE ----------------------------*/


