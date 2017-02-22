/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BABYLOVE_H
#define __BABYLOVE_H

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
