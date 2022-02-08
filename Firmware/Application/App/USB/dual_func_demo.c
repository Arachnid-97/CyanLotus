/**
  ******************************************************************************
  * @file    dual_func_demo.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    17-March-2018
  * @brief   This file contain the demo implementation
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
#include "dual_func_demo.h"
#include "usbh_usr.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbh_msc_core.h"
#include "usbd_msc_core.h"

#include "usb_conf.h"


/** @addtogroup USBH_USER
* @{
*/

/** @addtogroup USBH_DUAL_FUNCT_DEMO
* @{
*/

/** @defgroup USBH_DUAL_FUNCT_DEMO 
* @brief    This file includes the usb host stack user callbacks
* @{
*/

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_TypesDefinitions
* @{
*/
/**
* @}
*/


/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Defines
* @{
*/
/**
* @}
*/


/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Macros
* @{
*/
/**
* @}
*/


/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Variables
* @{
*/


uint8_t USBH_USR_ApplicationState = USH_USR_FS_INIT;

uint8_t filenameString[15] = { 0 };
DEMO_StateMachine demo;
uint8_t line_idx = 0;
__IO uint8_t wait_user_input = 0;
uint8_t Enum_Done = 0;
uint8_t *DEMO_main_menu[] = {
  (uint8_t *)
    "      1 - Host Demo                                                          ",
  (uint8_t *)
    "      2 - Device Demo                                                        ",
  (uint8_t *)
    "      3 - Credits                                                            ",
};

uint8_t *DEMO_HOST_menu[] = {
  (uint8_t *)
    "      1 - Explore Flash content                                              ",
  (uint8_t *)
    "      2 - Write File to disk                                                 ",
  (uint8_t *)
    "      3 - Show BMP file                                                      ",
  (uint8_t *)
    "      4 - Return                                                             ",
};

uint8_t *DEMO_DEVICE_menu[] = {
  (uint8_t *)
    "      1 - Return                                                             ",
  (uint8_t *)
    "                                                                             ",
  (uint8_t *)
    "                                                                             ",
};

uint8_t writeTextBuff[] =
  "STM32 Connectivity line Host Demo application using FAT_FS   ";

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* !< IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* !< IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* !< IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_FS_Core __ALIGN_END;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ )      /* !< IAR Compiler */
#pragma data_alignment=4
#endif
#endif                          /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HOST USB_FS_Host __ALIGN_END;

/**
* @}
*/

/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Constants
* @{
*/
static void Demo_Application(void);
static void Demo_SelectItem(uint8_t ** menu, uint8_t item);

/**
* @}
*/



/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_FunctionPrototypes
* @{
*/
/**
* @}
*/


/** @defgroup USBH_DUAL_FUNCT_DEMO_Private_Functions
* @{
*/

/**
* @brief  Demo_Init 
*         Demo initialization
* @param  None
* @retval None
*/

void Demo_Init(void)
{
  LCD_LOG_Init();
  LCD_LOG_SetHeader((uint8_t *) " USB Manual DRD demo");
  LCD_UsrLog("> Initializing demo....\n");



  Demo_SelectItem(DEMO_main_menu, 0);

  USBH_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
            USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#endif
            &USB_Host, &USBH_MSC_cb, &USBH_USR_cb);

  USB_OTG_BSP_mDelay(500);
  DEMO_UNLOCK();
}

/**
* @brief  Demo_Application 
*         Demo background task
* @param  None
* @retval None
*/

void Demo_Process(void)
{
  if (demo.state == DEMO_HOST)
  {
    if (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
      USBH_Process(&USB_OTG_Core, &USB_Host);
    }
  }

  Demo_Application();
}

/**
* @brief  Demo_ProbeKey 
*         Probe the joystick state
* @param  state : joystick state
* @retval None
*/

void Demo_ProbeKey(JOYState_TypeDef state)
{
  /* Explorer running */
  if ((wait_user_input == 1) && (state != JOY_NONE))
  {
    wait_user_input = 2;
    return;
  }

  /* Critical process ongoing : do not interrupt */
  if (DEMO_IS_LOCKED())
  {
    return;
  }

  if ((state == JOY_UP) && (demo.select > 0))
  {
    demo.select--;
  }
  else if (((state == JOY_DOWN) && (demo.select < 2)) ||
           ((demo.state == DEMO_HOST) && (state == JOY_DOWN) &&
            (demo.select < 3)))
  {

    if (!((demo.state == DEMO_DEVICE) && demo.select == 0))
    {
      demo.select++;
    }
  }
  else if (state == JOY_SEL)
  {
    demo.select |= 0x80;
  }
}

/**
* @brief  Demo_SelectItem 
*         manage the menu on the screen
* @param  menu : menu table
*         item : selected item to be highlighted
* @retval None
*/

static void Demo_SelectItem(uint8_t ** menu, uint8_t item)
{

}

/**
* @brief  Demo_Application 
*         Demo state machine
* @param  None
* @retval None
*/

static void Demo_Application(void)
{
  static uint8_t prev_select = 0;
  uint16_t bytesWritten, bytesToWrite;

  switch (demo.state)
  {
  case DEMO_IDLE:
    __disable_irq();
    Demo_SelectItem(DEMO_main_menu, 0);
    __enable_irq();
    demo.state = DEMO_WAIT;
    demo.select = 0;
    break;
  case DEMO_WAIT:


    if (demo.select != prev_select)
    {
      prev_select = demo.select;
      Demo_SelectItem(DEMO_main_menu, demo.select & 0x7F);

      /* Handle select item */
      if (demo.select & 0x80)
      {
        demo.select &= 0x7F;

        switch (demo.select)
        {
        case 0:
          LCD_LOG_ClearTextZone();
          demo.state = DEMO_HOST;
          demo.Host_state = DEMO_HOST_IDLE;
          break;

        case 1:
          LCD_LOG_ClearTextZone();
          demo.state = DEMO_DEVICE;
          demo.Device_state = DEMO_DEVICE_IDLE;
          break;

        case 2:
          LCD_LOG_ClearTextZone();
          LCD_DbgLog("\nSystem Information :\n");
          LCD_DbgLog("_________________________\n\n");

          LCD_UsrLog("Board : STM324x9I_EVAL-Eval.\n");
          LCD_UsrLog("Device: STM32F4xx.\n");
          LCD_UsrLog("USB Host Library v2.2.0.\n");
          LCD_UsrLog("USB Device Library V1.2.1.\n");
          LCD_UsrLog("USB OTG Driver v2.2.0\n");
          LCD_UsrLog("STM32 Std Library v1.5.0.\n");

          demo.state = DEMO_IDLE;
          break;
        default:
          break;
        }
      }
    }
    break;
  case DEMO_HOST:
    switch (demo.Host_state)
    {
    case DEMO_HOST_IDLE:
      DEMO_LOCK();

      /* Init HS Core : Demo start in host mode */
#ifdef USE_USB_OTG_HS
      LCD_DbgLog("> Initializing USB Host High speed...\n");
#else
      LCD_DbgLog("> Initializing USB Host Full speed...\n");
#endif
      USBH_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
                USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
                USB_OTG_HS_CORE_ID,
#endif
                &USB_Host, &USBH_MSC_cb, &USBH_USR_cb);

      demo.Host_state = DEMO_HOST_WAIT;
      DEMO_UNLOCK();
      break;

    case DEMO_HOST_WAIT:

      if (!HCD_IsDeviceConnected(&USB_OTG_Core))
      {
        Demo_HandleDisconnect();
        LCD_ErrLog("Please, connect a device and try again.\n");
      }

      if (Enum_Done == 1)
      {
#ifdef USE_USB_OTG_HS
        LCD_DbgLog("> USB Host High speed initialized.\n");
#else
        LCD_DbgLog("> USB Host Full speed initialized.\n");
#endif

        LCD_UsrLog("> File System initialized.\n");
        LCD_UsrLog("> Disk capacity : %lu Bytes\n",
                   USBH_MSC_Param.MSCapacity * USBH_MSC_Param.MSPageLength);
        Demo_SelectItem(DEMO_HOST_menu, 0);
        demo.select = 0;
        Enum_Done = 2;
      }

      if (Enum_Done == 2)
      {
        if (demo.select != prev_select)
        {
          prev_select = demo.select;
          USB_OTG_DisableGlobalInt(&USB_OTG_Core);
          Demo_SelectItem(DEMO_HOST_menu, demo.select & 0x7F);
          USB_OTG_EnableGlobalInt(&USB_OTG_Core);

          /* Handle select item */
          if (demo.select & 0x80)
          {
            demo.select &= 0x7F;
            switch (demo.select)
            {
            case 0:
              DEMO_LOCK();
              Explore_Disk("0:/", 1);
              line_idx = 0;
              DEMO_UNLOCK();
              break;
            case 1:
              /* Writes a text file, STM32.TXT in the disk */
              LCD_UsrLog("> Writing File to disk flash ...\n");
              if (USBH_MSC_Param.MSWriteProtect == DISK_WRITE_PROTECTED)
              {

                LCD_ErrLog("> Disk flash is write protected \n");
                USBH_USR_ApplicationState = USH_USR_FS_DRAW;
                break;
              }
              DEMO_LOCK();
              /* Register work area for logical drives */
              f_mount(&fatfs, "", 0);

              if (f_open(&file, "0:STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) ==
                  FR_OK)
              {
                /* Write buffer to file */
                bytesToWrite = sizeof(writeTextBuff);
                res =
                  f_write(&file, writeTextBuff, bytesToWrite,
                          (void *)&bytesWritten);

                if ((bytesWritten == 0) || (res != FR_OK))  /* EOF or Error */
                {
                  LCD_ErrLog("> STM32.TXT CANNOT be writen.\n");
                }
                else
                {
                  LCD_UsrLog("> 'STM32.TXT' file created\n");
                }

                /* close file and filesystem */
                f_close(&file);
              }
              DEMO_UNLOCK();
              break;
            case 2:
              if (f_mount(&fatfs, "", 0) != FR_OK)
              {
                /* fat_fs initialisation fails */
                break;
              }
#if !defined(USE_STM324x9I_EVAL)
              Image_Browser("0:/");
#endif
              /* Clear windows */
              LCD_LOG_ClearTextZone();
              LCD_LOG_SetHeader((uint8_t *) " USB Manual DRD demo");
              LCD_UsrLog("> Slide show application closed.\n");
              break;

            case 3:
              Demo_HandleDisconnect();
              f_mount(NULL, "", 0);
              USB_OTG_StopHost(&USB_OTG_Core);

              /* Manage User disconnect operations */
              USB_Host.usr_cb->DeviceDisconnected();

              /* Re-Initilaize Host for new Enumeration */
              USBH_DeInit(&USB_OTG_Core, &USB_Host);
              USB_Host.usr_cb->DeInit();
              USB_Host.class_cb->DeInit(&USB_OTG_Core, &USB_Host.device_prop);
              LCD_UsrLog("> Use Joystick to Select demo.\n");
              break;

            default:
              break;
            }
          }
        }
      }
      break;
    default:
      break;
    }
    break;
  case DEMO_DEVICE:


    switch (demo.Device_state)
    {
    case DEMO_DEVICE_IDLE:
      DEMO_LOCK();
      USBD_Init(&USB_OTG_Core,
#ifdef USE_USB_OTG_FS
                USB_OTG_FS_CORE_ID,
#elif defined USE_USB_OTG_HS
                USB_OTG_HS_CORE_ID,
#endif
                &USR_desc, &USBD_MSC_cb, &USBD_USR_cb);


      demo.Device_state = DEMO_DEVICE_WAIT;
      demo.select = 0;
      DEMO_UNLOCK();
      break;

    case DEMO_DEVICE_WAIT:

      if (demo.select != prev_select)
      {
        prev_select = demo.select;
        __disable_irq();
        Demo_SelectItem(DEMO_DEVICE_menu, demo.select & 0x7F);
        __enable_irq();

        /* Handle select item */
        if (demo.select & 0x80)
        {
          demo.select &= 0x7F;
          switch (demo.select)
          {
          case 0:

            __disable_irq();
            demo.state = DEMO_IDLE;
            demo.select = 0;
            LCD_LOG_ClearTextZone();
            LCD_UsrLog("> Device application closed.\n");
            __enable_irq();
            DCD_DevDisconnect(&USB_OTG_Core);
            USB_OTG_StopDevice(&USB_OTG_Core);
            break;
          default:
            break;
          }
        }
      }
      break;

    default:
      break;
    }
    break;
  default:
    break;
  }

}


/**
* @brief  Explore_Disk 
*         Displays disk content
* @param  path: pointer to root path
* @retval None
*/
static uint8_t Explore_Disk(char *path, uint8_t recu_level)
{

  FRESULT res;
  FILINFO fno;
  DIR dir;
  char *fn;
  char tmp[14];


  res = f_opendir(&dir, path);
  if (res == FR_OK)
  {
    while (HCD_IsDeviceConnected(&USB_OTG_Core))
    {
      res = f_readdir(&dir, &fno);
      if (res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      if (fno.fname[0] == '.')
      {
        continue;
      }
      fn = fno.fname;
      strcpy(tmp, fn);
      line_idx++;
      if (line_idx > 12)
      {
        line_idx = 0;
        wait_user_input = 1;
        LCD_SetTextColor(Green);
        LCD_DisplayStringLine(LCD_PIXEL_HEIGHT - 48,
                              (uint8_t *) "Press any key to continue...");
        LCD_SetTextColor(LCD_LOG_DEFAULT_COLOR);

        while ((HCD_IsDeviceConnected(&USB_OTG_Core)) && (wait_user_input != 2))
        {
          Toggle_Leds();
        }
        STM_EVAL_LEDOff(LED1);
        STM_EVAL_LEDOff(LED2);
        STM_EVAL_LEDOff(LED3);
        STM_EVAL_LEDOff(LED4);
        LCD_ClearLine(LCD_PIXEL_HEIGHT - 48);
      }
      wait_user_input = 0;

      if (recu_level == 1)
      {
        LCD_DbgLog("   |__");
      }
      else if (recu_level == 2)
      {
        LCD_DbgLog("   |   |__");
      }
      if ((fno.fattrib & AM_MASK) == AM_DIR)
      {
        strcat(tmp, "\n");
        LCD_UsrLog((void *)tmp);
      }
      else
      {
        strcat(tmp, "\n");
        LCD_DbgLog((void *)tmp);
      }

      if (((fno.fattrib & AM_MASK) == AM_DIR) && (recu_level == 1))
      {
        Explore_Disk(fn, 2);
      }
    }
  }
  return res;
}

/**
* @brief  Demo_HandleDisconnect
*         deinit demo and astart again the enumeration
* @param  None
* @retval None
*/
void Demo_HandleDisconnect(void)
{
  LCD_LOG_ClearTextZone();
  demo.state = DEMO_IDLE;
  USBH_DeInit(&USB_OTG_Core, &USB_Host);
  Enum_Done = 0;
  DEMO_UNLOCK();
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
