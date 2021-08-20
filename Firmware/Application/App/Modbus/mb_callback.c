/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"

/* ----------------------- User RTU Function Selection ----------------------*/
#define RTU_REG_INPUT			1
#define RTU_REG_HOLDING			1
#define RTU_REG_COILS			1
#define RTU_REG_DISCRETE		1

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START			0x0001		// 输入寄存器启动地址
#define REG_INPUT_NREGS 		8			// 输入寄存器大小(HalfWord)
#define REG_HOLDING_START 		0x0001		// 保持寄存器启动地址
#define REG_HOLDING_NREGS 		8			// 保持寄存器大小(HalfWord)
#define REG_COIL_START 			0x0001		// 线圈寄存器启动地址
#define REG_COIL_NREGS			64			// 线圈大小(1bit)
#define REG_DISCRETE_START 		0x0001		// 离散输入寄存器启动地址
#define REG_DISCRETE_NREGS		64			// 离散输入大小(1bit)

/* ----------------------- Static variables ---------------------------------*/
#if RTU_REG_INPUT
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS] = {0x0000,0x1111,0x2222,0x3333};
#endif /* RTU_REG_INPUT */

#if RTU_REG_HOLDING
static USHORT   usRegHoldingStart = REG_HOLDING_START;
static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS] = {0x0000,0x5678,0x1234,0x0596};
#endif /* RTU_REG_HOLDING */

#if RTU_REG_COILS
static USHORT   usRegCoilStart = REG_COIL_START;
#if REG_COIL_NREGS % 8
static UCHAR    usRegCoilBuf[REG_COIL_NREGS/8+1];
#else
static UCHAR    usRegCoilBuf[REG_COIL_NREGS / 8];
#endif
#endif /* RTU_REG_COILS */

#if RTU_REG_DISCRETE
static USHORT   usRegDiscreteStart = REG_DISCRETE_START;
#if REG_DISCRETE_NREGS % 8
static UCHAR    usRegDiscreteBuf[REG_DISCRETE_NREGS/8+1];
#else
static UCHAR    usRegDiscreteBuf[REG_DISCRETE_NREGS / 8];
#endif
#endif /* RTU_REG_DISCRETE */


#include "mbutils.h"
// 线圈跟离散是 8bit
//extern void xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits,
//							UCHAR ucValue );
//extern UCHAR xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits );

/**
  *****************************************************************************
  * @Name   : 读输入寄存器
  * @Brief  : 对应功能码
  * 			0x04 -> 读单个或多个输入寄存器eMBFuncReadInputRegister
  * @Input  : pucRegBuffer   数据缓冲区
  *           usAddress:     寄存器地址
  *           usNRegs:       寄存器数量
  * @Output : none
  * @Return : Modbus状态信息
  * @Notice : 3 区
  * 		  对象类型：16位
  *****************************************************************************
**/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer,
               USHORT usAddress,
               USHORT usNRegs )
{
	
#if RTU_REG_INPUT
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;

    /* it already plus one in modbus function method. */
    // usAddress--;

    /* 请求地址大于起始地址 && 地址长度小于设定长度 */
    if ((usAddress >= REG_INPUT_START)
		&& (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (USHORT)(usAddress - usRegInputStart);
        while ( usNRegs > 0 )
        {
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

#else
	eMBErrorCode eStatus = MB_ENOREG;
	
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNRegs;

#endif

    return eStatus;
}

/**
  *****************************************************************************
  * @Name   : 保持寄存器
  * @Brief  : 对应功能码
  * 			0x03 -> 读单个或多个寄存器eMBFuncReadHoldingRegister
  * 			0x06 -> 写单个保持寄存器eMBFuncWriteHoldingRegister
  * 			0x10 -> 写多个保持寄存器eMBFuncWriteMultipleHoldingRegister
  * @Input  : pucRegBuffer:  数据缓冲区
  *           usAddress:     寄存器地址
  *           usNRegs:       寄存器数量
  * 		  eMode: 		 功能码
  * @Output : none
  * @Return : Modbus状态信息
  * @Notice : 4 区
  * 		  对象类型：16位
  *****************************************************************************
**/
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer,
                 USHORT usAddress,
                 USHORT usNRegs,
                 eMBRegisterMode eMode )
{
	
#if RTU_REG_HOLDING
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;

    /* it already plus one in modbus function method. */
    // usAddress--;

    if((usAddress >= REG_HOLDING_START) 
		&& (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = (USHORT)(usAddress - usRegHoldingStart);
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else {
        eStatus = MB_ENOREG;
    }

#else
	eMBErrorCode eStatus = MB_ENOREG;
	
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNRegs;
	( void )eMode;

#endif

    return eStatus;
}

/**
  *****************************************************************************
  * @Name   : 线圈寄存器
  * @Brief  : 对应功能码
  * 			0x01 -> 读单个线圈eMBFuncReadCoils
  * 			0x05 -> 写单个线圈eMBFuncWriteCoil
  * 			0x0F -> 写多个线圈eMBFuncWriteMultipleCoils
  * @Input  : pucRegBuffer:  数据缓冲区
  *           usAddress:     寄存器地址
  *           usNRegs:       寄存器数量
  * 		  eMode: 		 功能码
  * @Output : none
  * @Return : Modbus状态信息
  * @Notice : 0 区
  * 		  对象类型：单个位
  *****************************************************************************
**/
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer,
               USHORT usAddress,
               USHORT usNCoils,
               eMBRegisterMode eMode )
{

#if RTU_REG_COILS
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex, iRegBitIndex, iNReg;
    iNReg = usNCoils / 8 + 1;

    /* it already plus one in modbus function method. */
    // usAddress--;

    if ((usAddress >= REG_COIL_START) 
		&& (usAddress + usNCoils <= REG_COIL_START + REG_COIL_NREGS))
    {
        iRegIndex = (USHORT)(usAddress - usRegCoilStart) / 8;
        iRegBitIndex = (USHORT)(usAddress - usRegCoilStart) % 8;
        switch (eMode)
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&usRegCoilBuf[iRegIndex++], iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
                xMBUtilSetBits(&usRegCoilBuf[iRegIndex++], iRegBitIndex, 8, *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&usRegCoilBuf[iRegIndex++], iRegBitIndex, usNCoils, *pucRegBuffer++);
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

#else
	eMBErrorCode eStatus = MB_ENOREG;
	
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNCoils;
	( void )eMode;

#endif

    return eStatus;
}

/**
  *****************************************************************************
  * @Name   : 读离散输入寄存器
  * @Brief  : 对应功能码
  * 			0x02 -> 读单个或多个离散输入寄存器eMBFuncReadDiscreteInputs
  * @Input  : pucRegBuffer:  数据缓冲区
  *           usAddress:     寄存器地址
  *           usNRegs:       寄存器数量
  * @Output : none
  * @Return : Modbus状态信息
  * @Notice : 1 区
  * 		  对象类型：单个位
  *****************************************************************************
**/
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer,
                  USHORT usAddress,
                  USHORT usNDiscrete )
{

#if RTU_REG_DISCRETE
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex, iRegBitIndex, iNReg;
    iNReg = usNDiscrete / 8 + 1;

#if 1
	uint16_t i;
	const char buf[] = {0x01,0x02,0x03,0x04,0x05,0x07,0x08,0x09,0x0A};
	
	for(i = 0;i < REG_DISCRETE_NREGS / 8;i++){
		usRegDiscreteBuf[i] = *(buf + i);
	}

#endif
    /* it already plus one in modbus function method. */
    // usAddress--;

    if ((usAddress >= REG_DISCRETE_START) 
		&& (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_NREGS))
    {
        iRegIndex = (USHORT)(usAddress - usRegDiscreteStart) / 8;
        iRegBitIndex = (USHORT)(usAddress - usRegDiscreteStart) % 8;

        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&usRegDiscreteBuf[iRegIndex++],
                                             iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }
	
#else
	eMBErrorCode eStatus = MB_ENOREG;
	
    ( void )pucRegBuffer;
    ( void )usAddress;
    ( void )usNDiscrete;

#endif
	
    return eStatus;
}

