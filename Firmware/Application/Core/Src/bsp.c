#include "bsp.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "md5.h"
#include "bsp_uart.h"


char Sole_ID[19] = {'A', 'C'};

typedef struct
{
    unsigned int id[3];
} ChipID;

ChipID Get_ChipID(void)
{
    ChipID chipid = {0};

    chipid.id[0] = *(__I uint32_t *)(0x1FFF7A10 + 0x00);
    chipid.id[1] = *(__I uint32_t *)(0x1FFF7A10 + 0x04);
    chipid.id[2] = *(__I uint32_t *)(0x1FFF7A10 + 0x08);

    printf("ID Code:0x%08X\r\n", chipid.id[0]);
    printf("ID Code:0x%08X\r\n", chipid.id[1]);
    printf("ID Code:0x%08X\r\n", chipid.id[2]);

    return chipid;
}

void Hash_ID(void)
{
    ChipID unique_ID;
    MD5_CTX md5;
    MD5Init(&md5);
    int i;
    // unsigned char encrypt[] ="admin";//21232f297a57a5a743894a0e4a801fc3
    unsigned char decrypt[16];
    char temp[25] = {0};

    unique_ID = Get_ChipID();

    // snprintf(temp, 9, "%08X", unique_ID.id[0]);
    // snprintf(&temp[8], 9, "%08X", unique_ID.id[1]);
    // snprintf(&temp[16], 9, "%08X", unique_ID.id[2]);

    snprintf(temp, sizeof(temp), "%08X%08X%08X", unique_ID.id[0], unique_ID.id[1], unique_ID.id[2]);

    // printf("\n>>>%s\n", temp);

    MD5Update(&md5, (uint8_t *)temp, strlen((char *)temp));
    MD5Final(&md5, decrypt);

    for (i = 0; i < 8; i++)
    {
        sprintf(Sole_ID + 2 + i * 2, "%02X", decrypt[i + 4]);
    }
    printf("machine ID:%s\n", Sole_ID);

#if 0
    printf("加密前:%s\n加密后16位:",temp);  
    for(i=4;i<12;i++)  
    {  
        printf("%02X",decrypt[i]);  //02x前需要加上 %  
    }  

    printf("\n加密前:%s\n加密后32位:",temp);  
    for(i=0;i<16;i++)  
    {  
        printf("%02X",decrypt[i]);  //02x前需要加上 %  
    }

#endif
}


/*---------------------------- END OF FILE ----------------------------*/


