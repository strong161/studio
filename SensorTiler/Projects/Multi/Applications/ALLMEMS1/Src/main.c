/**
  ******************************************************************************
  * @file    main.c
  * @version V2.2.0
  * @date    30-November-2016
  * @brief   Main program body
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

/**
 * @mainpage FP-SNS-ALLMEMS1 Bluetooth Low Energy and Sensors Software
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This firmware package includes Components Device Drivers, Board Support Package
 * and example application for the following STMicroelectronics elements:
 * - X-NUCLEO-IDB04A1/X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards
 * - X-NUCLEO-IKS01A1 Expansion board for four MEMS sensor devices:
 *       HTS221, LPS25H, LSM6DS0, LSM6DS3, LIS3MDL
 * - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
 *       HTS221, LPS22HB, LSM6DSL, LSM303AGR
 * - X-NUCLEO-CCAM02M1 Digital MEMS microphones expansion board
 * - NUCLEO-F401RE NUCLEO-L476RG Nucleo boards
 * - STEVAL-STLKT01V1 (SensorTile) evaluation board that contains the following MEMS sensor devices:
 *      - HTS221, LPS22HB, LSM303, LSM6DSM
 *      - digital microphone: MP34DT04
 *      - Gas Gauge IC with alarm output: STC3115
 * - USB device library (for only STEVAL-STLCS01V1) provides support of multi packet transfer to allow sending big amount of data without split them into max packet size transfers.
 *
 * @attention
 * <b>Important Hardware Additional Information</b><br>
 * <br>\image html X-NUCLEO-IKS01A1_HW_Info.jpg "Figure 1: X-NUCLEO-IKS01A1 expansion board"
 * <br>Before to connect X-NUCLEO-IKS01A1 with X-NUCLEO-IDB04A1 (or X-NUCLEO-IDB05A1) expansion board through the Arduino UNO R3 extension connector,
 * remove the 0-ohm resistors SB25, SB26 and SB27 onto X-NUCLEO-IKS01A1 board, as the above figure 1 shows.<br>
 * <br>\image html X-NUCLEO-IKS01A2_HW_Info.jpg "Figure 2: X-NUCLEO-IKS01A2 expansion board"
 * <br>Before to connect X-NUCLEO-IKS01A2 with X-NUCLEO-CCAM02M1 expansion board through the Arduino UNO R3 extension connector,
 * on to X-NUCLEO-IKS01A2 board remove these 0-ohm resistor:
 * - For F4 STM32 Nucleo motherboard remove SB25, SB26 and SB27
 * - For L4 STM32 Nucleo motherboard remove SB25 if additional microphones are plugged on to X-NUCLEO-CCA02M1 board.<br>
 * .
 * <br>\image html X-NUCLEO-CCA02M1_HW_Info.jpg "Figure 3: X-NUCLEO-CCA02M1 expansion board"
 * <br>For only L4 STM32 Nucleo motherboard, before to connect the board X-NUCLEO-CCA02M1 with the STM32 L4 Nucleo motherboard through the Morpho connector layout,
 * on to X-NUCLEO-CCA02M1 board:
 * - close the solder bridges SB12, SB16 and open the solder bridges SB7, SB15 and SB17
 * - if additional microphones are plugged, close the solder bridge SB17.<br>
 *
 * <b>Example Application</b>
 *
 * The Example application initizializes all the Components and Library creating 3 Custom Bluetooth services:
 * - The first service exposes all the HW characteristics:
 *      - related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelleromenter,
 *        LED status and Microphones Signal Noise dB level.
 *      - battery Gas Gauge (for only SensorTile) 
 * - The second service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * For NUCLEO boards the example application allows the user to control the initialization phase via UART.
 * Launch a terminal application and set the UART port to 460800 bps, 8 bit, No Parity, 1 stop bit.
 * <br>For having the same UART functionality on SensorTile board, is necessary to recompile the code uncomment the line 66
 * <br><b>       //#define ALLMEMS1_ENABLE_PRINTF</b>
 * <br>on file <b>Projects\Multi\Applications\ALLMEMS1\Inc\allmems1_config.h</b>
 * <br>This enables the UART that starts with a delay of 10 Seconds for allowing the time to open the UART for looking
 * the initialization phase.
 *
 * This example must be used with the related BlueMS Android/iOS application available on Play/itune store,
 * in order to read the sent information by Bluetooth Low Energy protocol
 *
 *                              -----------------------
 *                              | VERY IMPORTANT (1): |
 *                              -----------------------
 * 1) For only X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards, this example support the Firmware-Over-The-Air (FOTA)
 * update using the BlueMS Android/iOS application (Version 3.0.0 Or higher)
 *
 * 2) For each IDE (IAR/µVision/System Workbench), and for each platform (NUCLEO-F401RE/NUCLEO-L476RG/SensorTile), there are some scripts *.bat or *.sh that makes the following operations:
 * - Full Flash Erase
 * - Load the BootLoader on the rigth flash region
 * - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
 * - Dump back one single binary that contain BootLoader+Program that could be flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
 * - Reset the board

 * 3) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
 * is saved at the beginning of the FLASH (address 0x08000000)
 *
 *                              -----------------------
 *                              | VERY IMPORTANT (2): |
 *                              -----------------------
 * It's necessary to choose the right Target configuration during the compilation.
 * If the code is compiled for IKS01A1 could not run if it's attached the X-NUCLEO-IKS01A2 and vice versa
 *
 *                               --------------------
 *                               | KNOWN LIMITATION |
 *                               --------------------
 * - For NUCLEO-F401RE board, there is an hardware conflict between the boards X-NUCLEO-IKS01A2 and the X-NUCLEO-CCA02M1.
 *   The hardware features of the LSM6DSL are disabled.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
    extern TIM_HandleTypeDef  TimHandle;
    extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

uint32_t ConnectionBleStatus  =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef    TimCCHandle;

uint8_t bdaddr[6];

uint32_t uhCCR1_Val = DEFAULT_uhCCR1_Val;
uint32_t uhCCR2_Val = DEFAULT_uhCCR2_Val;
uint32_t uhCCR3_Val = DEFAULT_uhCCR3_Val;


extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];

#ifdef USE_STM32F4XX_NUCLEO
extern uint16_t PDM_Buffer[];
#endif /* USE_STM32F4XX_NUCLEO */

/* Private variables ---------------------------------------------------------*/
static volatile int MEMSInterrupt        =0;
static volatile uint32_t HCI_ProcessEvent=0;
static volatile uint32_t SendEnv         =0;
static volatile uint32_t SendAudioLevel  =0;
static volatile uint32_t SendAccGyroMag  =0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);

static void InitTimers(void);
static void SendEnvironmentalData(void);
static void MEMSCallback(void);

static void SendMotionData(void);
static void SendAudioLevelData(void);

void AudioProcess(void);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

#ifdef STM32_NUCLEO
  InitTargetPlatform(TARGET_NUCLEO);
#elif STM32_SENSORTILE
  InitTargetPlatform(TARGET_SENSORTILE);
#endif /* STM32_NUCLEO */

  ALLMEMS1_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
         "\tSend Every %4ldmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4ldmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4ldmS dB noise\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         uhCCR1_Val/10,
         uhCCR2_Val/10,
         uhCCR3_Val/10);

#ifdef ALLMEMS1_DEBUG_CONNECTION
  ALLMEMS1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* ALLMEMS1_DEBUG_CONNECTION */

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  ALLMEMS1_PRINTF("Debug Notify Trasmission Enabled\r\n");
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();  

  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }

#ifdef STM32_NUCLEO
  if( !TargetBoardFeatures.SensorEmulation) {
#endif /* STM32_NUCLEO */
    /* Set Accelerometer Full Scale to 2G */
    Set2GAccelerometerFullScale();

    /* Read the Acc Sensitivity */
    BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
    sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
#ifdef STM32_NUCLEO
  }
#endif /* STM32_NUCLEO */
  
  /* initialize timers */
  InitTimers();

  /* Infinite loop */
  while (1){
    /* Led Blinking when there is not a client connected */
    if(!connected) {
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
    }
    
    if(set_connectable)
    {
      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }

    /* Handle Interrupt from MEMS */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt=0;
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      HCI_Process();
    }

    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
    }
    
    /* Mic Data */
    if (SendAudioLevel) {
      SendAudioLevel = 0;
      SendAudioLevelData();
    }

    /* Motion Data */
    if(SendAccGyroMag) {
      SendAccGyroMag=0;
      SendMotionData();
    }

    /* Wait for Event */
    __WFI();
  }
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,2.0f);
  
  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set4GAccelerometerFullScale(void)
{
  
  /* Set Full Scale to +/-4g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,4.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
 
  /* Environmental */
  /* TIM1_CH1 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    SendEnv=1;
  }
  
  /* Acc/Gyro/Mag */
  /* TIM1_CH2 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
    SendAccGyroMag=1;
  }

  /* Mic Data */
  /* TIM1_CH3 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
    SendAudioLevel=1;
  }
}


#ifdef STM32_SENSORTILE
#ifdef ALLMEMS1_ENABLE_PRINTF
/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimHandle)) {
    CDC_TIM_PeriodElapsedCallback(htim);
  }
}
#endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */


/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  uint8_t stat = 0;

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    /* Check if the interrupt is due to Free Fall */
    BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_FREE_FALL);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    /* Check if the interrupt is due to Double Tap */
    BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_DOUBLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    /* Check if the interrupt is due to Single Tap */
    BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_SINGLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_WAKE_UP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    /* Check if the interrupt is due to Tilt */
    BSP_ACCELERO_Get_Tilt_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_TILT);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    /* Check if the interrupt is due to 6D Orientation */
    BSP_ACCELERO_Get_6D_Orientation_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEventType Orientation = GetHWOrientation6D();
      AccEvent_Notify(Orientation);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    /* Check if the interrupt is due to Pedometer */
    BSP_ACCELERO_Get_Pedometer_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      uint16_t StepCount = GetStepHWPedometer();
      AccEvent_Notify(StepCount);
    }
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  SensorAxes_t ACC_Value;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

#ifdef STM32_NUCLEO
  if( !TargetBoardFeatures.SensorEmulation) {
#endif /* STM32_NUCLEO */
    /* Read the Acc values */
    BSP_ACCELERO_Get_Axes(TargetBoardFeatures.HandleAccSensor,&ACC_Value);

    /* Read the Magneto values */
    BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

    /* Read the Gyro values */
    BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);
#ifdef STM32_NUCLEO
  } else {
    /* sensor emulation */
    ACC_Value.AXIS_X = 100 + (((int32_t)rand())&0x5FF);
    ACC_Value.AXIS_Y = 300 - (((int32_t)rand())&0x5FF);
    ACC_Value.AXIS_Z = -200 + (((int32_t)rand())&0x5FF);

    MAG_Value.AXIS_X = -200 + (((int32_t)rand())&0x5FF);
    MAG_Value.AXIS_Y = 300 - (((int32_t)rand())&0x5FF);
    MAG_Value.AXIS_Z = 200 + (((int32_t)rand())&0x5FF);

    GYR_Value.AXIS_X =  30000 + (((int32_t)rand())&0x4FFFF);
    GYR_Value.AXIS_Y =  30000 - (((int32_t)rand())&0x4FFFF);
    GYR_Value.AXIS_Z = -30000 + (((int32_t)rand())&0x4FFFF);
  }
#endif /* STM32_NUCLEO */

  AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value);
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess(void)
{
  int32_t i;
  int32_t NumberMic;

  for(i = 0; i < 16; i++){
    for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
      RMS_Ch[NumberMic] += ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]));
    }
  }
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
  
  for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= (16.0f*MICS_DB_UPDATE_MUL_10MS*10);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_VALUE /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  AudioLevel_Update(DBNOISE_Value_Ch);
}

#ifdef STM32_SENSORTILE
/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  
#ifdef USE_STM32F4XX_NUCLEO
  BSP_AUDIO_IN_PDMToPCM(PDM_Buffer, PCM_Buffer); 
#endif /* USE_STM32F4XX_NUCLEO */
  
  AudioProcess();
}
#else
/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  
#ifdef USE_STM32F4XX_NUCLEO
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
  {
    BSP_AUDIO_IN_PDMToPCM(PDM_Buffer, PCM_Buffer);
  }
#endif /* USE_STM32F4XX_NUCLEO */
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
    AudioProcess();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  
#ifdef USE_STM32F4XX_NUCLEO
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
  {
    BSP_AUDIO_IN_PDMToPCM(PDM_Buffer, PCM_Buffer);
  }
#endif /* USE_STM32F4XX_NUCLEO */
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
    AudioProcess();
}
#endif /* STM32_SENSORTILE */

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  uint8_t Status;

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("Sending: ");
  }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */

#ifdef STM32_SENSORTILE
  /* Battery Informations */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT)) {
    GG_Update();
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Battery Report ");
       Term_Update(BufferToWrite,BytesToWrite);
     } else {
      ALLMEMS1_PRINTF("Battery Report ");
     }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
  }
#endif /* STM32_SENSORTILE */
  
  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

#ifdef STM32_NUCLEO
    if(!TargetBoardFeatures.SensorEmulation) {
#endif /* STM32_NUCLEO */
      if(TargetBoardFeatures.HandlePressSensor) {
        if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
          BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
          MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
          PressToSend=intPart*100+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Press=%ld ",PressToSend);
          }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
        }
      }

      if(TargetBoardFeatures.HandleHumSensor) {
        if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
          BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          HumToSend = intPart*10+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Hum=%d ",HumToSend);
          }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
        }
      }

      if(TargetBoardFeatures.NumTempSensors==2) {
        if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
          BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          Temp1ToSend = intPart*10+decPart; 
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Temp1=%d ",Temp1ToSend);
          }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
        }
        
        if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
          BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          Temp2ToSend = intPart*10+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Temp2=%d ",Temp2ToSend);
          }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
        }
      } else if(TargetBoardFeatures.NumTempSensors==1) {
        if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
          BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          Temp1ToSend = intPart*10+decPart;
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Temp=%d ",Temp1ToSend);
          }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
        }
      }
#ifdef STM32_NUCLEO
    } else {
      /* Sensor Emulation */
      Temp1ToSend  = 260    + (((int16_t)rand())&0x33);
      Temp2ToSend  = 290    + (((int16_t)rand())&0x33);
      HumToSend    = 500    + (((int16_t)rand())&0x1F);
      PressToSend  = 100000 + (((int32_t)rand())&0x1FF);
      
#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
            Term_Update(BufferToWrite,BytesToWrite);
            BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
            Term_Update(BufferToWrite,BytesToWrite);
            BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
            Term_Update(BufferToWrite,BytesToWrite);
            BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            ALLMEMS1_PRINTF("Press=%ld ",PressToSend);
            ALLMEMS1_PRINTF("Hum=%d ",HumToSend);
            ALLMEMS1_PRINTF("Temp1=%d ",Temp1ToSend);
            ALLMEMS1_PRINTF("Temp2=%d ",Temp2ToSend);
          }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
    }
#endif /* STM32_NUCLEO */  
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }

#ifdef ALLMEMS1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    ALLMEMS1_PRINTF("\r\n");
  }
#endif /* ALLMEMS1_DEBUG_NOTIFY_TRAMISSION */
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance (Motion)*/
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }  
  
  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_ALLMEMS,0};

  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;  
  
#ifdef MAC_ALLMEMS
  {
    uint8_t tmp_bdaddr[6]= {MAC_ALLMEMS};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_ALLMEMS */
  
#ifndef STM32_NUCLEO
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
#endif /* STM32_NUCLEO */
  
  /* Initialize the BlueNRG HCI */
  HCI_Init();
    
  /* Reset BlueNRG hardware */
  BlueNRG_RST();
  
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  if (hwVersion > 0x30) {
    /* X-NUCLEO-IDB05A1 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB05A1;
  } else {
    /* X-NUCLEO-IDB0041 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB04A1;
  }
  
  /* 
   * Reset BlueNRG again otherwise it will fail.
   */
  BlueNRG_RST();

#ifndef MAC_ALLMEMS
  #ifdef MAC_STM32UID_ALLMEMS
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
#ifdef STM32_NUCLEO
    /* if IDB05A1 = Number between 100->199
     * if IDB04A1 = Number between 0->99
     * where Y == (OSX_BMS_VERSION_MAJOR + OSX_BMS_VERSION_MINOR)&0xF */
    bdaddr[4] = (hwVersion > 0x30) ?
         ((((ALLMEMS1_VERSION_MAJOR-48)*10) + (ALLMEMS1_VERSION_MINOR-48)+100)&0xFF) :
         ((((ALLMEMS1_VERSION_MAJOR-48)*10) + (ALLMEMS1_VERSION_MINOR-48)    )&0xFF) ;
#else /* STM32_NUCLEO */
    bdaddr[4] = (((ALLMEMS1_VERSION_MAJOR-48)*10) + (ALLMEMS1_VERSION_MINOR-48)+100)&0xFF;
#endif  /* STM32_NUCLEO */
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_ALLMEMS */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

    if(ret){
      ALLMEMS_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_ALLMEMS */
#else /* MAC_ALLMEMS */

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    //ALLMEMS1_PRINTF("\r\nSetting BD_ADDR failed\r\n");
    ALLMEMS1_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
    goto fail;
  }
  
#endif /* MAC_ALLMEMS */
  
  ret = aci_gatt_init();    
  if(ret){
     ALLMEMS1_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
     ALLMEMS1_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }
  
#ifndef  MAC_ALLMEMS
  #ifdef MAC_STM32UID_ALLMEMS
  ret = hci_le_set_random_address(bdaddr);

  if(ret){
    ALLMEMS1_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
    goto fail;
  }
  #endif /* MAC_STM32UID_ALLMEMS */
#endif /* MAC_ALLMEMS */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     ALLMEMS1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     ALLMEMS1_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  ALLMEMS1_PRINTF("SERVER: BLE Stack Initialized \r\n"
         "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
         (TargetBoardFeatures.bnrg_expansion_board==IDB05A1) ? "IDB05A1" : "IDB04A1",
         hwVersion,
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}
  
/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  
  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     ALLMEMS1_PRINTF("HW      Service W2ST added successfully\r\n");
  }
  else
  {
     ALLMEMS1_PRINTF("\r\nError while adding HW Service W2ST\r\n");
  }

/*  ret = Add_SWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
     ALLMEMS1_PRINTF("SW      Service W2ST added successfully\r\n");
  else
     ALLMEMS1_PRINTF("\r\nError while adding SW Service W2ST\r\n");*/

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     ALLMEMS1_PRINTF("Console Service W2ST added successfully\r\n");
  }
  else
  {
     ALLMEMS1_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     ALLMEMS1_PRINTF("Config  Service W2ST added successfully\r\n");
  }
  else
  {
     ALLMEMS1_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

#ifdef USE_STM32F4XX_NUCLEO
#ifdef STM32_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow:
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
    Error_Handler();
  }
}
#endif /* STM32_NUCLEO */
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#ifdef STM32_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;  /* Previous value = RCC_MSIRANGE_6; */
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6; /* Previous value = 1 */
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 4; /* Previous value = 2 */
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
}
#elif STM32_SENSORTILE
/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();
  
  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}
#endif /* STM32_NUCLEO */
#endif /* USE_STM32L4XX_NUCLEO */

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
#ifdef STM32_NUCLEO
    case SPI1_CMN_DEFAULT_IRQ_PIN:
#else
    case BNRG_SPI_EXTI_PIN:
#endif /* STM32_NUCLEO */   
      HCI_Isr();
      HCI_ProcessEvent=1;
    break;
#ifdef STM32_NUCLEO
  case KEY_BUTTON_PIN:
    break;
#endif /* STM32_NUCLEO */

#ifdef STM32_NUCLEO
  #ifdef IKS01A1
    case M_INT1_PIN:
  #elif IKS01A2
    case LSM6DSL_INT1_O_PIN:
  #endif /* IKS01A1 */
#elif STM32_SENSORTILE
  case LSM6DSM_INT2_PIN:
#endif /* STM32_NUCLEO */
    MEMSInterrupt=1;
    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: ALLMEMS1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
