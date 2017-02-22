/**
  ******************************************************************************
  * @file    allmems1_config.h
  * @version V2.2.0
  * @date    30-November-2016
  * @brief   allmems1 configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ALLMEMS1_CONFIG_H
#define __ALLMEMS1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* Define the ALLMEMS1 MAC address, otherwise it will create a MAC related to STM32 UID */
//#define MAC_ALLMEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

#ifndef MAC_ALLMEMS
  /* For creating one MAC related to STM32 UID, Otherwise the BLE will use it's random MAC */
  #define MAC_STM32UID_ALLMEMS
#endif /* MAC_ALLMEMS */



/*************** Debug Defines ******************/
/* For enabling the printf on UART */
#ifdef STM32_SENSORTILE
  /* Enabling this define for SensorTile..
   * it will introduce a delay of 10Seconds before starting the application
   * for having time to open the Terminal
   * for looking the ALLMEMS1 Initialization phase */
  //#define ALLMEMS1_ENABLE_PRINTF
#else /* STM32_SENSORTILE */
  /* For Nucleo F401RE/L476RG it's enable by default */
  #define ALLMEMS1_ENABLE_PRINTF
#endif /* STM32_SENSORTILE */

/* For enabling connection and notification subscriptions debug */
#define ALLMEMS1_DEBUG_CONNECTION

/* For enabling trasmission for notified services */
#define ALLMEMS1_DEBUG_NOTIFY_TRAMISSION

/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define MICS_DB_UPDATE_MUL_10MS 5


/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define ALLMEMS1_VERSION_MAJOR '2'
#define ALLMEMS1_VERSION_MINOR '2'
#define ALLMEMS1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 char long */
#define NAME_ALLMEMS 'A','M','1','V',ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH

/* Package Name */
#define ALLMEMS1_PACKAGENAME "FP-SNS-ALLMEMS1"

#ifdef STM32_NUCLEO
#define AUDIO_VOLUME_VALUE        64
#define AUDIO_CHANNELS            2
#define AUDIO_SAMPLING_FREQUENCY  16000
#endif /* STM32_NUCLEO */

#ifdef STM32_SENSORTILE
#define AUDIO_VOLUME_VALUE        4
#define AUDIO_CHANNELS            1
#define AUDIO_SAMPLING_FREQUENCY  16000
#endif /* STM32_SENSORTILE */

#ifdef ALLMEMS1_ENABLE_PRINTF
  #ifdef STM32_NUCLEO
    #define ALLMEMS1_PRINTF(...) printf(__VA_ARGS__)
  #elif STM32_SENSORTILE
    #include "usbd_cdc_interface.h"
    #define ALLMEMS1_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }
  #endif /* STM32_NUCLEO */
#else /* ALLMEMS1_ENABLE_PRINTF */
#define ALLMEMS1_PRINTF(...) 
#endif /* ALLMEMS1_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#endif /* __ALLMEMS1_CONFIG_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
