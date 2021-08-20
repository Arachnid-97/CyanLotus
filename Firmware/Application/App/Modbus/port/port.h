/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: port.h,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#ifndef _PORT_H
#define _PORT_H

#include <assert.h>
#include <inttypes.h>

#include "rs485.h"


#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define MB_DEVICE_ADDR				0x01

/* Enable modbus dma mode */
#define USING_MODBUS_TxDMA			1
#define USING_MODBUS_RxDMA			1


#define MBPortSerial                RS485_USART
#define MBPortSerial_IRQHandler     RS485_USART_IRQHandler
#define MBPortSerial_TxDMA          RS485_USARTTx_DMA_STREAM
#define MBPortSerial_RxDMA          RS485_USARTRx_DMA_STREAM
#define MBPortSerial_DMA_IRQHandler RS485_USARTTx_DMA_IRQHandler

#define MBPortSerial_TxDMA_TC_FLAG  RS485_USARTTx_DMA_TC_FLAG
#define MBPortSerial_TxDMA_IT_TC_FLAG  RS485_USARTTx_DMA_IT_TC_FLAG
#define MBPortSerial_RxDMA_TC_FLAG  RS485_USARTRx_DMA_TC_FLAG


#define ENTER_CRITICAL_SECTION( )   __set_PRIMASK(1)	// 关总中断
#define EXIT_CRITICAL_SECTION( )    __set_PRIMASK(0)	// 开总中断

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#if (USING_MODBUS_TxDMA || USING_MODBUS_RxDMA)
// The value corresponding to the 'MB_SER_PDU_SIZE_MAX'
#define RTU_BUF_SIZE                256

static inline void SendBuffer_DMA_Size(USHORT SER_PDU_SIZE_MAX)
{
	while (DMA_GetCmdStatus(MBPortSerial_TxDMA) != DISABLE);
	DMA_SetCurrDataCounter(MBPortSerial_TxDMA, SER_PDU_SIZE_MAX);
}

static inline USHORT RcvBuffer_DMA_Size(USHORT SER_PDU_SIZE_MAX)
{
	return SER_PDU_SIZE_MAX - DMA_GetCurrDataCounter(MBPortSerial_RxDMA);
}
#endif /* USING_MODBUS_TxDMA || USING_MODBUS_RxDMA */

#endif
