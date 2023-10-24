/**
  ******************************************************************************
  * @file    netconf.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013 
  * @brief   This file contains all the functions prototypes for the netconf.c 
  *          file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETCONF_H
#define __NETCONF_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum 
{ 
    DHCP_START = 1,
    DHCP_WAIT_ADDRESS,
    DHCP_ADDRESS_ASSIGNED,
    DHCP_TIMEOUT,
    DHCP_LINK_DOWN
} 
DHCP_State_TypeDef;
/* Exported constants --------------------------------------------------------*/
extern struct netif xnetif;
/* Exported macro ------------------------------------------------------------*/
/* MAC ADDRESS*/
#define MAC_ADDR0   02
#define MAC_ADDR1   00
#define MAC_ADDR2   00
#define MAC_ADDR3   00
#define MAC_ADDR4   00
#define MAC_ADDR5   00
 
/*Static IP ADDRESS*/
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   10
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   1

#define __MERGE(a,b) a##b
#define MERGE(a,b) __MERGE(a,b)
#define __STR(x) #x
#define STR(x) __STR(x)

#define DEFAULT_LOCAL_UDP_PORT 		1080             // 本地 UDP 端口
#define DEFAULT_LOCAL_TCP_PORT 		6200             // 本地 TCP 服务端口
#define DEFAULT_REMOTE_TCP_PORT 	5001             // 远程服务器端口

#define DEFAULT_LOCAL_IP   STR(MERGE(MERGE(IP_ADDR0.,IP_ADDR1)., \
                                  MERGE(IP_ADDR2.,IP_ADDR3)))    // 本地 IP

#define DEFAULT_REMOTE_TCP_IP   "192.168.1.100"    // 远程服务器连接


/* Exported functions ------------------------------------------------------- */
void LwIP_Init(void);
void LwIP_DHCP_Process_Handle(void);

#ifdef __cplusplus
}
#endif

#endif /* __NETCONF_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
