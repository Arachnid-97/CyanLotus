/**
  ******************************************************************************
  * @file    netconf.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   Network connection configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/dhcp.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"
#include "netconf.h"
#include <stdio.h>


#include "lwip/tcp.h"
// #include "lwip/udp.h"
// #include "lwip/etharp.h"
// #include "lwip/inet.h"
// #include "stm32f4x7_eth.h"
#include "stm32f4x7_eth_bsp.h"
// #include "string.h"
// #include "bsp_uart.h"
// #include "ethernet.h"

#include "FreeRTOS.h"
// #include "task.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_DHCP_TRIES 4

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif xnetif; /* network interface structure */
extern __IO uint32_t  EthStatus;
#ifdef USE_DHCP
__IO DHCP_State_ TypeDef DHCP_state;
#endif /* USE_DHCP */

// u8_t *ram_heap = NULL;

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
void LwIP_Init(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

#ifndef USE_DHCP
  // uint8_t iptab[4] = {0};
  // uint8_t iptxt[20];
#endif

  // lwip_comm_mem_malloc();

// /** All allocated blocks will be MIN_SIZE bytes big, at least!
//  * MIN_SIZE can be overridden to suit your needs. Smaller values save space,
//  * larger values could prevent too small blocks to fragment the RAM too much. */
// #ifndef MIN_SIZE
// #define MIN_SIZE             12
// #endif /* MIN_SIZE */
// /* some alignment macros: we define them here for better source code layout */
// #define MIN_SIZE_ALIGNED     LWIP_MEM_ALIGN_SIZE(MIN_SIZE)
// #define SIZEOF_STRUCT_MEM    LWIP_MEM_ALIGN_SIZE(6)
// #define MEM_SIZE_ALIGNED     LWIP_MEM_ALIGN_SIZE(MEM_SIZE)
//   ram_heap = pvPortMalloc(MEM_SIZE_ALIGNED + (2*SIZEOF_STRUCT_MEM) + MEM_ALIGNMENT);


  /* Create tcp_ip stack thread */
  tcpip_init( NULL, NULL );

  /* IP address setting */
#ifdef USE_DHCP
    ipaddr.addr = 0;
    netmask.addr = 0;
    gw.addr = 0;
#else
    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

	// ipaddr.addr = inet_addr(DEFAULT_NETIP);//SysParam->ipaddr.addr;
	// netmask.addr = inet_addr(DEFAULT_NETMASK);//SysParam->netmask.addr;
	// gw.addr = inet_addr(DEFAULT_NETGW);//SysParam->gw.addr;
#endif
  
  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))

  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  netif_add(&xnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /*  Registers the default network interface.*/
  netif_set_default(&xnetif);

  if (EthStatus == (ETH_INIT_FLAG | ETH_LINK_FLAG))
  { 
    /* Set Ethernet link flag */
    xnetif.flags |= NETIF_FLAG_LINK_UP;

    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&xnetif);
#ifdef USE_DHCP
    DHCP_state = DHCP_START;
#else
    /* Display static IP address */
    printf("Static IP address: %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
#endif /* USE_DHCP */
  }
  else
  {
    /*  When the netif link is down this function must be called.*/
    netif_set_down(&xnetif);
#ifdef USE_DHCP
    DHCP_state = DHCP_LINK_DOWN;
#endif /* USE_DHCP */
    printf("Network Cable is not connected\r\n");
  }

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&xnetif, ETH_link_callback);
}

#ifdef USE_DHCP
/**
  * @brief  LwIP_DHCP_Process_Handle
  * @param  None
  * @retval None
  */
void LwIP_DHCP_Process_Handle(void)
{
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;
    uint32_t IPaddress;
    uint8_t iptab[4] = {0};
    uint8_t iptxt[20];

    switch (DHCP_state)
    {
        case DHCP_START:
        {
            dhcp_start(&xnetif);
            /* IP address should be set to 0
               every time we want to assign a new DHCP address */
            IPaddress = 0;
            DHCP_state = DHCP_WAIT_ADDRESS;

            printf("Looking for DHCP server, please wait...\r\n");
        }
        break;

        case DHCP_WAIT_ADDRESS:
        {
            /* Read the new IP address */
            IPaddress = xnetif.ip_addr.addr;

            if (IPaddress!=0)
            {
                DHCP_state = DHCP_ADDRESS_ASSIGNED;

                /* Stop DHCP */
                dhcp_stop(&xnetif);
				
                /* Display the IP address */
                printf("IP address assigned \r\n");
                printf("  by a DHCP server  \r\n");
                printf("IP address: %d.%d.%d.%d", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
            }
            else
            {
                /* DHCP timeout */
                if (xnetif.dhcp->tries > MAX_DHCP_TRIES)
                {
                    DHCP_state = DHCP_TIMEOUT;

                    /* Stop DHCP */
                    dhcp_stop(&xnetif);

                    /* Static address used */
                    IP4_ADDR(&ipaddr, IP_ADDR0,IP_ADDR1, IP_ADDR2, IP_ADDR3 );
                    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
                    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
                    netif_set_addr(&xnetif, &ipaddr, &netmask, &gw);
                    printf("DHCP timeout\r\n");
                    printf("Static IP address: %d.%d.%d.%d\r\n", IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
                }
            }
        }
        break;
        
        default:
            break;
    }
}
#endif  /* USE_DHCP */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
