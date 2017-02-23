/**
  ******************************************************************************
  * @file    main.h 
  * @author  Central LAB
  * @version V2.2.0
  * @date    23-December-2016
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "console.h" 

#include "osal.h"
#include "debug.h"
#include "osx_bms_config.h"

/* Code for MotionFX integration - Start Section */
#include "MotionFX_Manager.h"
#include "osx_motion_fx.h"
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
#include "MotionAR_Manager.h"
#include "osx_motion_ar.h"
/* Code for MotionAR integration - End Section */
    
/* Code for MotionCP integration - Start Section */
#include "MotionCP_Manager.h"
#include "osx_motion_cp.h"
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
#include "MotionGR_Manager.h"
#include "osx_motion_gr.h"
/* Code for MotionGR integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
#include "AcousticSL_Manager.h"
#include "osx_acoustic_sl.h"
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

/* Exported defines and variables  ------------------------------------------------------- */

/* Code for MotionFX and MotionGR integration - Start Section */
/* 10kHz/100 For MotionFX@100Hz or MotionGR@100Hz as defaul value */
#define DEFAULT_uhCCR1_Val  100
/* Code for MotionFX and MotionGR integration - End Section */

/* Code for MotionCP integration - Start Section */
/* 10kHz/50 For MotionCP@50Hz as defaul value */
#define DEFAULT_uhCCR2_Val  200
/* Code for MotionCP integration - End Section */

/* Code for MotionAR integration - Start Section */
/* 10kHz/16  For MotionAR@16Hz as defaul value */
#define DEFAULT_uhCCR3_Val  625
/* Code for MotionAR integration - End Section */

//10kHz/20  For Acc/Gyro/Mag@20Hz
#define DEFAULT_uhCCR4_Val  500

#ifndef STM32_NUCLEO
  #define BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER ((uint32_t)0x12345678)
#endif /* STM32_NUCLEO */

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
