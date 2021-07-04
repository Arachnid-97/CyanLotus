/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "lwip/err.h"
#include "ethernetif.h"

#include "stm32f4x7_eth.h"
#include <string.h>
#include "data_info.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"


#define netifMTU                                (1500)
#define netifINTERFACE_TASK_STACK_SIZE		( 350 )
#define netifINTERFACE_TASK_PRIORITY		( configMAX_PRIORITIES - 2 )
#define netifGUARD_BLOCK_TIME			( 250 )
/* The time to block waiting for input. */
#define emacBLOCK_TIME_WAITING_FOR_INPUT	( ( portTickType ) 100 )

/* Define those to better describe your network interface. */
#define IFNAME0 's'
#define IFNAME1 't'


static struct netif *s_pxNetIf = NULL;
xSemaphoreHandle s_xSemaphore = NULL;


/* Ethernet Rx & Tx DMA Descriptors */
extern ETH_DMADESCTypeDef *DMARxDscrTab, *DMATxDscrTab; // 以太网 DMA收发描述符数据结构体指针

/* Ethernet Receive buffers  */
extern uint8_t *Rx_Buff; // 以太网底层驱动接收 buffer指针

/* Ethernet Transmit buffers */
extern uint8_t *Tx_Buff; // 以太网底层驱动发送 buffer指针

/* Global pointers to track current transmit and receive descriptors */
extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

/* Global pointer for last received frame infos */
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;


static void ethernetif_input( void * pvParameters );
static void arp_timer(void *arg);

//static FrameTypeDef ETH_RxPkt_ChainMode(void);
//static u32 ETH_GetCurrentTxBuffer(void);
//static u32 ETH_TxPkt_ChainMode(u16 FrameLength);


/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
    uint32_t i;
    uint8_t *macaddress = SysParam->mac;
//    struct ethernetif *ethernetif = netif->state;


    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* set MAC hardware address */
    netif->hwaddr[0] =  macaddress[0];
    netif->hwaddr[1] =  macaddress[1];
    netif->hwaddr[2] =  macaddress[2];
    netif->hwaddr[3] =  macaddress[3];
    netif->hwaddr[4] =  macaddress[4];
    netif->hwaddr[5] =  macaddress[5];

    /* set netif maximum transfer unit */
    netif->mtu = 1500;

    /* Accept broadcast address and ARP traffic */
//    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    s_pxNetIf =netif;

    /* create binary semaphore used for informing ethernetif of frame reception */
    if (s_xSemaphore == NULL)
    {
//        s_xSemaphore= xSemaphoreCreateCounting(20,0);
        vSemaphoreCreateBinary(s_xSemaphore);
        xSemaphoreTake( s_xSemaphore, 0);
    }

    /* initialize MAC address in ethernet MAC */
    ETH_MACAddressConfig(ETH_MAC_Address0, netif->hwaddr);

    /* Initialize Tx Descriptors list: Chain Mode */
    ETH_DMATxDescChainInit(DMATxDscrTab, Tx_Buff, ETH_TXBUFNB);
    /* Initialize Rx Descriptors list: Chain Mode  */
    ETH_DMARxDescChainInit(DMARxDscrTab, Rx_Buff, ETH_RXBUFNB);

    /* Enable Ethernet Rx interrrupt */
    {
        for(i=0; i<ETH_RXBUFNB; i++)
        {
            ETH_DMARxDescReceiveITConfig(&DMARxDscrTab[i], ENABLE);
        }
    }

#ifdef CHECKSUM_BY_HARDWARE
    /* Enable the checksum insertion for the Tx frames */
    {
        for(i=0; i<ETH_TXBUFNB; i++)
        {
            ETH_DMATxDescChecksumInsertionConfig(&DMATxDscrTab[i], ETH_DMATxDesc_ChecksumTCPUDPICMPFull);
        }
    }
#endif

    /* create the task that handles the ETH_MAC */
    xTaskCreate(ethernetif_input, (const char*) "Eth_if", netifINTERFACE_TASK_STACK_SIZE, NULL,
                netifINTERFACE_TASK_PRIORITY,NULL);

    /* Enable MAC and DMA transmission and reception */
    ETH_Start();
}


/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{

#if 0
    static xSemaphoreHandle xTxSemaphore = NULL;
    struct pbuf *q;
    uint32_t l = 0;
    u8 *buffer ;

    if (xTxSemaphore == NULL)
    {
        vSemaphoreCreateBinary (xTxSemaphore);
    }

    if (xSemaphoreTake(xTxSemaphore, netifGUARD_BLOCK_TIME))
    {
        buffer =  (u8 *)(ETH_GetCurrentTxBuffer());
        for(q = p; q != NULL; q = q->next)
        {
            memcpy((u8_t*)&buffer[l], q->payload, q->len);
            l = l + q->len;
        }
        ETH_TxPkt_ChainMode(l);
        xSemaphoreGive(xTxSemaphore);
    }
#else
    static xSemaphoreHandle xTxSemaphore = NULL;
    struct pbuf *q;
    u8 *buffer ;
    __IO ETH_DMADESCTypeDef *DmaTxDesc;
    uint16_t framelength = 0;
    uint32_t bufferoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t payloadoffset = 0;

    if (xTxSemaphore == NULL)
    {
        vSemaphoreCreateBinary (xTxSemaphore);
    }

    if (xSemaphoreTake(xTxSemaphore, netifGUARD_BLOCK_TIME))
    {
        DmaTxDesc = DMATxDescToSet;
        buffer = (u8 *)(DmaTxDesc->Buffer1Addr);
        bufferoffset = 0;

        for(q = p; q != NULL; q = q->next)
        {
            if((DmaTxDesc->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
            {
                goto error;
            }

            /* Get bytes in current lwIP buffer  */
            byteslefttocopy = q->len;
            payloadoffset = 0;

            /* Check if the length of data to copy is bigger than Tx buffer size*/
            while( (byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE )
            {
                /* Copy data to Tx buffer*/
                memcpy( (u8_t*)((u8_t*)buffer + bufferoffset), (u8_t*)((u8_t*)q->payload + payloadoffset), (ETH_TX_BUF_SIZE - bufferoffset) );

                /* Point to next descriptor */
                DmaTxDesc = (ETH_DMADESCTypeDef *)(DmaTxDesc->Buffer2NextDescAddr);

                /* Check if the buffer is available */
                if((DmaTxDesc->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
                {
                    goto error;
                }

                buffer = (u8 *)(DmaTxDesc->Buffer1Addr);

                byteslefttocopy = byteslefttocopy - (ETH_TX_BUF_SIZE - bufferoffset);
                payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
                framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
                bufferoffset = 0;
            }

            /* Copy the remaining bytes */
            memcpy( (u8_t*)((u8_t*)buffer + bufferoffset), (u8_t*)((u8_t*)q->payload + payloadoffset), byteslefttocopy );
            bufferoffset = bufferoffset + byteslefttocopy;
            framelength = framelength + byteslefttocopy;
        }

        /* Prepare transmit descriptors to give to DMA*/
        ETH_Prepare_Transmit_Descriptors(framelength);

        /* Give semaphore and exit */
error:

        xSemaphoreGive(xTxSemaphore);
    }
#endif

    return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf * low_level_input(struct netif *netif)
{

#if 0
    struct pbuf *p, *q;
    u16_t len;
    int l =0;
//	int i;
    FrameTypeDef frame;
    u8 *buffer;
//	__IO ETH_DMADESCTypeDef *DMARxNextDesc;

    p = NULL;

    /* get received frame */
    frame = ETH_RxPkt_ChainMode();

    /* Check if the descriptor is exist */
    if(!frame.descriptor)
        return p;

    /* Obtain the size of the packet and put it into the "len" variable. */
    len = frame.length;
    buffer = (u8 *)frame.buffer;



    /* We allocate a pbuf chain of pbufs from the pool. */
    p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (p != NULL)
    {
        for (q = p; q != NULL; q = q->next)
        {
            memcpy((u8_t*)q->payload, (u8_t*)&buffer[l], q->len);
            l = l + q->len;
        }
    }


    /* Set Own bit of the Rx descriptor Status: gives the buffer back to ETHERNET DMA */
    frame.descriptor->Status = ETH_DMARxDesc_OWN;

    /* Release descriptors to DMA */
    /* Check if frame with multiple DMA buffer segments */
//	if (DMA_RX_FRAME_infos->Seg_Count > 1)
//	{
//		DMARxNextDesc = DMA_RX_FRAME_infos->FS_Rx_Desc;
//	}
//	else
//	{
//		DMARxNextDesc = frame.descriptor;
//	}
//
//	/* Set Own bit in Rx descriptors: gives the buffers back to DMA */
//	for (i=0; i<DMA_RX_FRAME_infos->Seg_Count; i++)
//	{
//		DMARxNextDesc->Status = ETH_DMARxDesc_OWN;
//		DMARxNextDesc = (ETH_DMADESCTypeDef *)(DMARxNextDesc->Buffer2NextDescAddr);
//	}
//
//	/* Clear Segment_Count */
//	DMA_RX_FRAME_infos->Seg_Count =0;

    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
    {
        /* Clear RBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        ETH->DMARPDR = 0;
    }

    return p;

#else
    struct pbuf *p= NULL, *q;
    u32_t len;
    FrameTypeDef frame;
    u8 *buffer;
    __IO ETH_DMADESCTypeDef *DMARxDesc;
    uint32_t bufferoffset = 0;
    uint32_t payloadoffset = 0;
    uint32_t byteslefttocopy = 0;
    uint32_t i=0;

    /* get received frame */
    frame = ETH_Get_Received_Frame_interrupt();

    /* Obtain the size of the packet and put it into the "len" variable. */
    len = frame.length;
    buffer = (u8 *)frame.buffer;

    if (len > 0)
    {
        /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
        p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    }

    if (p != NULL)
    {
        DMARxDesc = frame.descriptor;
        bufferoffset = 0;
        for(q = p; q != NULL; q = q->next)
        {
            byteslefttocopy = q->len;
            payloadoffset = 0;

            /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size*/
            while( (byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE )
            {
                /* Copy data to pbuf*/
                memcpy( (u8_t*)((u8_t*)q->payload + payloadoffset), (u8_t*)((u8_t*)buffer + bufferoffset), (ETH_RX_BUF_SIZE - bufferoffset));

                /* Point to next descriptor */
                DMARxDesc = (ETH_DMADESCTypeDef *)(DMARxDesc->Buffer2NextDescAddr);
                buffer = (unsigned char *)(DMARxDesc->Buffer1Addr);

                byteslefttocopy = byteslefttocopy - (ETH_RX_BUF_SIZE - bufferoffset);
                payloadoffset = payloadoffset + (ETH_RX_BUF_SIZE - bufferoffset);
                bufferoffset = 0;
            }

            /* Copy remaining data in pbuf */
            memcpy( (u8_t*)((u8_t*)q->payload + payloadoffset), (u8_t*)((u8_t*)buffer + bufferoffset), byteslefttocopy);
            bufferoffset = bufferoffset + byteslefttocopy;
        }
    }

    /* Release descriptors to DMA */
    DMARxDesc =frame.descriptor;

    /* Set Own bit in Rx descriptors: gives the buffers back to DMA */
    for (i=0; i<DMA_RX_FRAME_infos->Seg_Count; i++)
    {
        DMARxDesc->Status = ETH_DMARxDesc_OWN;
        DMARxDesc = (ETH_DMADESCTypeDef *)(DMARxDesc->Buffer2NextDescAddr);
    }

    /* Clear Segment_Count */
    DMA_RX_FRAME_infos->Seg_Count =0;

    /* When Rx Buffer unavailable flag is set: clear it and resume reception */
    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
    {
        /* Clear RBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_RBUS;
        /* Resume DMA reception */
        ETH->DMARPDR = 0;
    }
    return p;

#endif
}


/**
 * This function is the ethernetif_input task, it is processed when a packet
 * is ready to be read from the interface. It uses the function low_level_input()
 * that should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input( void * pvParameters )
{
    struct netif *netif;
    struct pbuf *p;
    netif = (struct netif*) s_pxNetIf;

#if 0
    for( ;; ) {
        if (xSemaphoreTake( s_xSemaphore, emacBLOCK_TIME_WAITING_FOR_INPUT) == pdTRUE) {
            while(1)
            {
                p = low_level_input( s_pxNetIf );
                if(p != NULL) {
                    if (ERR_OK != s_pxNetIf->input( p, s_pxNetIf)) {
                        pbuf_free(p);
                        p = NULL;
                    }
                }
                else
                    break;
            }
        }
    }
#else

    for( ;; ) {
        if (xSemaphoreTake( s_xSemaphore, emacBLOCK_TIME_WAITING_FOR_INPUT ) == pdTRUE)
        {
TRY_GET_NEXT_FRAME:
            /* move received packet into a new pbuf */
            // taskENTER_CRITICAL();
            p = low_level_input(netif);
            // taskEXIT_CRITICAL();
            /* points to packet payload, which starts with an Ethernet header */
            if (p != NULL)
            {
                // taskENTER_CRITICAL();
                /* full packet send to tcpip_thread to process */
                if (netif->input(p, netif) != ERR_OK)
                {
                    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                    pbuf_free(p);
                    p = NULL;
                }
                else
                {
                    goto TRY_GET_NEXT_FRAME;
                }
                // taskEXIT_CRITICAL();
            }
        }
    }
#endif
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* initialize the hardware */
    low_level_init(netif);

    etharp_init();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);

    return ERR_OK;
}


static void arp_timer(void *arg)
{
    etharp_tmr();
    sys_timeout(ARP_TMR_INTERVAL, arp_timer, NULL);
}


#if 0
/*******************************************************************************
* Function Name  : ETH_RxPkt_ChainMode
* Description    : Receives a packet.
* Input          : None
* Output         : None
* Return         : frame: farme size and location
*******************************************************************************/
static FrameTypeDef ETH_RxPkt_ChainMode(void)
{
    u32 framelength = 0;
    FrameTypeDef frame = {0,0};

    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
    {
        frame.length = ETH_ERROR;

        if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
        {
            /* Clear RBUS ETHERNET DMA flag */
            ETH->DMASR = ETH_DMASR_RBUS;
            /* Resume DMA reception */
            ETH->DMARPDR = 0;
        }

        /* Return error: OWN bit set */
        return frame;
    }

    if(((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) &&
            ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) &&
            ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))
    {
        /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
        framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;

        /* Get the addrees of the actual buffer */
        frame.buffer = DMARxDescToGet->Buffer1Addr;
    }
    else
    {
        /* Return ERROR */
        framelength = ETH_ERROR;
    }

    frame.length = framelength;


    frame.descriptor = DMARxDescToGet;

    /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */
    /* Chained Mode */
    /* Selects the next DMA Rx descriptor list for next buffer to read */
    DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);

    /* Return Frame */
    return (frame);
}

/*******************************************************************************
* Function Name  : ETH_TxPkt_ChainMode
* Description    : Transmits a packet, from application buffer, pointed by ppkt.
* Input          : - FrameLength: Tx Packet size.
* Output         : None
* Return         : ETH_ERROR: in case of Tx desc owned by DMA
*                  ETH_SUCCESS: for correct transmission
*******************************************************************************/
static u32 ETH_TxPkt_ChainMode(u16 FrameLength)
{
    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
    {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }

    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);

    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;

    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
    {
        /* Clear TBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_TBUS;
        /* Resume DMA transmission*/
        ETH->DMATPDR = 0;
    }

    /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);


    /* Return SUCCESS */
    return ETH_SUCCESS;
}


/*******************************************************************************
* Function Name  : ETH_GetCurrentTxBuffer
* Description    : Return the address of the buffer pointed by the current descritor.
* Input          : None
* Output         : None
* Return         : Buffer address
*******************************************************************************/
static u32 ETH_GetCurrentTxBuffer(void)
{
    /* Return Buffer address */
    return (DMATxDescToSet->Buffer1Addr);
}

#endif

