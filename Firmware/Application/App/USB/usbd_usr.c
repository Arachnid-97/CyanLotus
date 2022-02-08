/**
  ******************************************************************************
  * @file    usbd_usr.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    17-March-2018
  * @brief   This file includes the user application layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usb_bsp.h"

/** @addtogroup USBD_USER
  * @{
  */

/** @addtogroup USBD_MSC_DEMO_USER_CALLBACKS
  * @{
  */

/** @defgroup USBD_USR 
  * @brief    This file includes the user application layer
  * @{
  */

/** @defgroup USBD_USR_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_USR_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_USR_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_USR_Private_Variables
  * @{
  */
/* Points to the DEVICE_PROP structure of current device */
/* The purpose of this register is to speed up the execution */

USBD_Usr_cb_TypeDef USBD_USR_cb = {
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,

  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};

/**
  * @}
  */

/** @defgroup USBD_USR_Private_Constants
  * @{
  */

#define USER_INFORMATION1  (uint8_t*)"INFO : Single Lun configuration"
#define USER_INFORMATION2  (uint8_t*)"INFO : microSD is used"
/**
  * @}
  */



/** @defgroup USBD_USR_Private_FunctionPrototypes
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_USR_Private_Functions
  * @{
  */

/**
* @brief  Displays the message on LCD on device lib initialization
* @param  None
* @retval None
*/
void USBD_USR_Init(void)
{
#ifdef USE_USB_OTG_HS 
  #ifdef USE_EMBEDDED_PHY
  USB_DEBUG("> USB OTG HS_IN_FS MSC Device");
  #else
  USB_DEBUG("> USB OTG HS MSC Device");
  #endif
#else
  USB_DEBUG("> USB OTG FS MSC Device");
#endif
  USB_DEBUG("> USB device library started.\n");
  USB_DEBUG("     USB Device Library V1.2.1");
  
  USB_DEBUG(USER_INFORMATION1);
  USB_DEBUG(USER_INFORMATION2);

}

/**
* @brief  Displays the message on LCD on device reset event
* @param  speed : device speed
* @retval None
*/
void USBD_USR_DeviceReset(uint8_t speed)
{
  switch (speed)
  {
    case USB_OTG_SPEED_HIGH: 
      USB_DEBUG ("     USB Device Library V1.2.1  [HS]" );
      break;

    case USB_OTG_SPEED_FULL: 
      USB_DEBUG ("     USB Device Library V1.2.1  [FS]" );
      break;
	  
    default:
      USB_DEBUG ("     USB Device Library V1.2.1  [??]" );     
  }
}


/**
* @brief  Displays the message on LCD on device config event
* @param  None
* @retval Status
*/
void USBD_USR_DeviceConfigured(void)
{
  USB_DEBUG("> MSC Interface started.\n");
}

/**
* @brief  Displays the message on LCD on device suspend event 
* @param  None
* @retval None
*/
void USBD_USR_DeviceSuspended(void)
{
  USB_DEBUG("> Device In suspend mode.\n");
}


/**
* @brief  Displays the message on LCD on device resume event
* @param  None
* @retval None
*/
void USBD_USR_DeviceResumed(void)
{

}


/**
* @brief  USBD_USR_DeviceConnected
*         Displays the message on LCD on device connection Event
* @param  None
* @retval Status
*/
void USBD_USR_DeviceConnected(void)
{
  USB_DEBUG("> USB Device Connected.\n");
}


/**
* @brief  USBD_USR_DeviceDisonnected
*         Displays the message on LCD on device disconnection Event
* @param  None
* @retval Status
*/
void USBD_USR_DeviceDisconnected(void)
{
  USB_DEBUG("> USB Device Disconnected.\n");
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
