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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


#if (USING_MODBUS_TxDMA || USING_MODBUS_RxDMA)
    // #warning "Please build external file './App/Modbus/eif_mbrtu.c' instead of the default file './middleware/freemodbus/modbus/rtu/mbrtu.c'!"
#endif /* (USING_MODBUS_TxDMA || USING_MODBUS_RxDMA) */

extern volatile UCHAR ucRTUBuf[];

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    if(TRUE == xRxEnable)
    {
#if (!USING_MODBUS_RxDMA)
		USART_ITConfig(MBPortSerial, USART_IT_RXNE, ENABLE);
#else
        USART_ITConfig(MBPortSerial, USART_IT_IDLE, ENABLE);
#endif
    }
    else
    {
#if (!USING_MODBUS_RxDMA)
		USART_ITConfig(MBPortSerial, USART_IT_RXNE, DISABLE);
#else
        USART_ITConfig(MBPortSerial, USART_IT_IDLE, DISABLE);
#endif
    }

    if(TRUE == xTxEnable)
    {
#if (!USING_MODBUS_TxDMA)
        USART_ITConfig(MBPortSerial, USART_IT_TXE, ENABLE);
#else
        DMA_Cmd(MBPortSerial_TxDMA, ENABLE);
#endif
    }
    else
    {
#if (!USING_MODBUS_TxDMA)
        USART_ITConfig(MBPortSerial, USART_IT_TXE, DISABLE);
#else
        DMA_Cmd(MBPortSerial_TxDMA, DISABLE);
#endif
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    RSParity parity = RS_PAR_NONE;

    ( void )ucPORT;
    if(8 == ucDataBits) // RTU == 8DataBits
    {
        switch(eParity) {
        case MB_PAR_NONE: {
            parity = RS_PAR_NONE;
            break;
        }
        case MB_PAR_ODD: {
            parity = RS_PAR_ODD;
            break;
        }
        case MB_PAR_EVEN: {
            parity = RS_PAR_EVEN;
            break;
        }
        }
        RS485_USART_Init(ulBaudRate, parity);

#if (USING_MODBUS_TxDMA)
        RS485_DMA_Config((void *)ucRTUBuf, NULL, RTU_BUF_SIZE, 0);
#endif /* USING_MODBUS_TxDMA */

#if (!USING_MODBUS_RxDMA)
        USART_ITConfig(MBPortSerial, USART_IT_IDLE, DISABLE);
#else
        USART_ITConfig(MBPortSerial, USART_IT_RXNE, DISABLE);
        RS485_DMA_Config(NULL, (void *)ucRTUBuf, 0, RTU_BUF_SIZE);
#endif /* USING_MODBUS_RxDMA */

        return TRUE;
    }
    else
        return FALSE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    USART_SendByte(MBPortSerial, ucByte);

    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
#if (!USING_MODBUS_RxDMA)
    *pucByte = USART_ReceiveData(MBPortSerial);
#else
    *pucByte = '\0';
#endif

    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}

/**
  * @brief  Modbus Serial IRQHandler.
  * @param  None.
  * @return None.
  */
void MBPortSerial_IRQHandler(void)
{
	if(USART_GetITStatus(MBPortSerial, USART_IT_RXNE)) {
		prvvUARTRxISR();
	}
	
	if(USART_GetITStatus(MBPortSerial, USART_IT_IDLE)) {
		(void)USART_ReceiveData(MBPortSerial);
        
#if USING_MODBUS_RxDMA
        /* 关闭接收 DMA */
        DMA_Cmd(MBPortSerial_RxDMA, DISABLE);
        /* 清除标志位 */
        DMA_ClearFlag(MBPortSerial_RxDMA, MBPortSerial_RxDMA_TC_FLAG);  
#endif

        prvvUARTRxISR();

#if USING_MODBUS_RxDMA
        pxMBPortCBTimerExpired();

        DMA_SetCurrDataCounter(MBPortSerial_RxDMA, RTU_BUF_SIZE);  
        /* 打开 DMA */
        DMA_Cmd(MBPortSerial_RxDMA, ENABLE);
#endif

	}

	if(USART_GetITStatus(MBPortSerial, USART_IT_TXE | USART_IT_TC)) {
		prvvUARTTxReadyISR();
	}
    

	if(USART_GetITStatus(MBPortSerial, USART_IT_NE | USART_IT_ORE | USART_IT_FE)) {
		(void)USART_ReceiveData(MBPortSerial);
	}
}

/**
  * @brief  Modbus Serial DMA IRQHandler.
  * @param  None.
  * @return None.
  */
#if USING_MODBUS_TxDMA
void MBPortSerial_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(MBPortSerial_TxDMA, MBPortSerial_TxDMA_IT_TC_FLAG) != RESET)
    {
        DMA_ClearITPendingBit(MBPortSerial_TxDMA, MBPortSerial_TxDMA_IT_TC_FLAG);
        prvvUARTTxReadyISR();
    }
}
#endif
