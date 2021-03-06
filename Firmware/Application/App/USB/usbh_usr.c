/**
  ******************************************************************************
  * @file    usbh_usr.c
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file includes the usb host library user callbacks
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
#include "usbh_usr.h"
#include "usb_conf.h"
#include "usb_bsp.h"

/** @addtogroup USBH_USER
* @{
*/

/** @addtogroup USBH_MSC_DEMO_USER_CALLBACKS
* @{
*/

/** @defgroup USBH_USR 
* @brief    This file includes the usb host stack user callbacks
* @{
*/

/** @defgroup USBH_USR_Private_TypesDefinitions
* @{
*/
/**
* @}
*/


/** @defgroup USBH_USR_Private_Defines
* @{
*/
#define IMAGE_BUFFER_SIZE    512
/**
* @}
*/


/** @defgroup USBH_USR_Private_Macros
* @{
*/
/**
* @}
*/


/** @defgroup USBH_USR_Private_Variables
* @{
*/
uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;


/* Points to the DEVICE_PROP structure of current device */
/* The purpose of this register is to speed up the execution */

USBH_Usr_cb_TypeDef USBH_USR_cb = {
  USBH_USR_Init,
  USBH_USR_DeInit,
  USBH_USR_DeviceAttached,
  USBH_USR_ResetDevice,
  USBH_USR_DeviceDisconnected,
  USBH_USR_OverCurrentDetected,
  USBH_USR_DeviceSpeedDetected,
  USBH_USR_Device_DescAvailable,
  USBH_USR_DeviceAddressAssigned,
  USBH_USR_Configuration_DescAvailable,
  USBH_USR_Manufacturer_String,
  USBH_USR_Product_String,
  USBH_USR_SerialNum_String,
  USBH_USR_EnumerationDone,
  USBH_USR_UserInput,
  USBH_USR_MSC_Application,
  USBH_USR_DeviceNotSupported,
  USBH_USR_UnrecoveredError
};

/**
* @}
*/

/** @defgroup USBH_USR_Private_Constants
* @{
*/
/*--------------- LCD Messages ---------------*/
const uint8_t MSG_HOST_INIT[] = "> Host Library Initialized\n";
const uint8_t MSG_DEV_ATTACHED[] = "> Device Attached \n";
const uint8_t MSG_DEV_DISCONNECTED[] = "> Device Disconnected\n";
const uint8_t MSG_DEV_ENUMERATED[] = "> Enumeration completed \n";
const uint8_t MSG_DEV_HIGHSPEED[] = "> High speed device detected\n";
const uint8_t MSG_DEV_FULLSPEED[] = "> Full speed device detected\n";
const uint8_t MSG_DEV_LOWSPEED[] = "> Low speed device detected\n";
const uint8_t MSG_DEV_ERROR[] = "> Device fault \n";

const uint8_t MSG_MSC_CLASS[] = "> Mass storage device connected\n";
const uint8_t MSG_HID_CLASS[] = "> HID device connected\n";
const uint8_t MSG_DISK_SIZE[] = "> Size of the disk in MBytes: \n";
const uint8_t MSG_LUN[] = "> LUN Available in the device:\n";
const uint8_t MSG_ROOT_CONT[] = "> Exploring disk flash ...\n";
const uint8_t MSG_WR_PROTECT[] = "> The disk is write protected\n";
const uint8_t MSG_UNREC_ERROR[] = "> UNRECOVERED ERROR STATE\n";

/**
* @}
*/


/** @defgroup USBH_USR_Private_FunctionPrototypes
* @{
*/

/**
* @}
*/


/** @defgroup USBH_USR_Private_Functions
* @{
*/


/**
* @brief  USBH_USR_Init
*         Displays the message on LCD for host lib initialization
* @param  None
* @retval None
*/
void USBH_USR_Init(void)
{
  static uint8_t startup = 0;

  if (startup == 0)
  {
    startup = 1;

#ifdef USE_USB_OTG_HS
  #ifdef USE_EMBEDDED_PHY
    USB_DEBUG("> USB OTG HS_IN_FS MSC Host");
  #else
    USB_DEBUG("> USB OTG HS MSC Host");
  #endif
#else
    USB_DEBUG("> USB OTG FS MSC Host");
#endif
    USB_DEBUG("> USB Host library started.\n");
    USB_DEBUG("     USB Host Library v2.2.1");
  }
}

/**
* @brief  USBH_USR_DeviceAttached
*         Displays the message on LCD on device attached
* @param  None
* @retval None
*/
void USBH_USR_DeviceAttached(void)
{
  USB_DEBUG(MSG_DEV_ATTACHED);
}


/**
* @brief  USBH_USR_UnrecoveredError
* @param  None
* @retval None
*/
void USBH_USR_UnrecoveredError(void)
{

  /* Set default screen color */
  USB_DEBUG(MSG_UNREC_ERROR);
}


/**
* @brief  USBH_DisconnectEvent
*         Device disconnect event
* @param  None
* @retval Status
*/
void USBH_USR_DeviceDisconnected(void)
{
  /* Set default screen color */
  USB_DEBUG(MSG_DEV_DISCONNECTED);
}

/**
* @brief  USBH_USR_ResetUSBDevice
* @param  None
* @retval None
*/
void USBH_USR_ResetDevice(void)
{
  /* callback for USB-Reset */
  USB_DEBUG("> USBH_USR_ResetDevice \r\n");
}


/**
* @brief  USBH_USR_DeviceSpeedDetected
*         Displays the message on LCD for device speed
* @param  Device speed
* @retval None
*/
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
  if (DeviceSpeed == HPRT0_PRTSPD_HIGH_SPEED)
  {
    USB_DEBUG(MSG_DEV_HIGHSPEED);
  }
  else if (DeviceSpeed == HPRT0_PRTSPD_FULL_SPEED)
  {
    USB_DEBUG(MSG_DEV_FULLSPEED);
  }
  else if (DeviceSpeed == HPRT0_PRTSPD_LOW_SPEED)
  {
    USB_DEBUG(MSG_DEV_LOWSPEED);
  }
  else
  {
    USB_DEBUG(MSG_DEV_ERROR);
  }
}

/**
* @brief  USBH_USR_Device_DescAvailable
*         Displays the message on LCD for device descriptor
* @param  device descriptor
* @retval None
*/
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
  USBH_DevDesc_TypeDef *hs;
  hs = DeviceDesc;

  USB_DEBUG("> VID : %04luh\n", (uint32_t) (*hs).idVendor);
  USB_DEBUG("> PID : %04luh\n", (uint32_t) (*hs).idProduct);
}

/**
* @brief  USBH_USR_DeviceAddressAssigned
*         USB device is successfully assigned the Address
* @param  None
* @retval None
*/
void USBH_USR_DeviceAddressAssigned(void)
{

}


/**
* @brief  USBH_USR_Conf_Desc
*         Displays the message on LCD for configuration descriptor
* @param  Configuration descriptor
* @retval None
*/
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef *cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
  USBH_InterfaceDesc_TypeDef *id;

  id = itfDesc;

  if ((*id).bInterfaceClass == 0x08)
  {
    USB_DEBUG(MSG_MSC_CLASS);
  }
  else if ((*id).bInterfaceClass == 0x03)
  {
    USB_DEBUG(MSG_HID_CLASS);
  }
}

/**
* @brief  USBH_USR_Manufacturer_String
*         Displays the message on LCD for Manufacturer String
* @param  Manufacturer String
* @retval None
*/
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
  USB_DEBUG("> Manufacturer : %s\n", (char *)ManufacturerString);
}

/**
* @brief  USBH_USR_Product_String
*         Displays the message on LCD for Product String
* @param  Product String
* @retval None
*/
void USBH_USR_Product_String(void *ProductString)
{
  USB_DEBUG("> Product : %s\n", (char *)ProductString);
}

/**
* @brief  USBH_USR_SerialNum_String
*         Displays the message on LCD for SerialNum_String
* @param  SerialNum_String
* @retval None
*/
void USBH_USR_SerialNum_String(void *SerialNumString)
{
  USB_DEBUG("> Serial Number : %s\n", (char *)SerialNumString);
}

/**
* @brief  EnumerationDone
*         User response request is displayed to ask application jump to class
* @param  None
* @retval None
*/
void USBH_USR_EnumerationDone(void)
{
  /* Enumeration complete */
  USB_DEBUG(MSG_DEV_ENUMERATED);
}

/**
* @brief  USBH_USR_DeviceNotSupported
*         Device is not supported
* @param  None
* @retval None
*/
void USBH_USR_DeviceNotSupported(void)
{
  USB_DEBUG("> No registered class for this device. \n\r");
}

/**
* @brief  USBH_USR_UserInput
*         User Action for application state entry
* @param  None
* @retval USBH_USR_Status : User response for key button
*/
USBH_USR_Status USBH_USR_UserInput(void)
{
#if 1
  /* HOST_ENUMERATION ??? HOST_CLASS_REQUEST
     ??????????????????????????????????????????????????????
     ??????????????????OK??????????????????
  */
  return USBH_USR_RESP_OK;
#else
  USBH_USR_Status usbh_usr_status;

  usbh_usr_status = USBH_USR_NO_RESP;

  /* Key button is in polling mode to detect user action */
  if (STM_EVAL_PBGetState(Button_KEY) == RESET)
  {

    usbh_usr_status = USBH_USR_RESP_OK;

  }
  return usbh_usr_status;
#endif
}

/**
* @brief  USBH_USR_OverCurrentDetected
*         Over Current Detected on VBUS
* @param  None
* @retval Status
*/
void USBH_USR_OverCurrentDetected(void)
{
  USB_DEBUG("> Overcurrent detected.");
}

/**
* @brief  USBH_USR_MSC_Application
*         Demo application for mass storage
* @param  None
* @retval Status
*/
int USBH_USR_MSC_Application(void)
{
  return 0;
}

/**
* @brief  USBH_USR_DeInit
*         Deinit User state and associated variables
* @param  None
* @retval None
*/
void USBH_USR_DeInit(void)
{
  USBH_USR_ApplicationState = USH_USR_FS_INIT;
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
