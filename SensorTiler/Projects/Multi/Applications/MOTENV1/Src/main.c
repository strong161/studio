/**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V2.2.0
  * @date    24-November-2016
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
 * @mainpage MOTENV1 Bluetooth Low Energy and Sensors Software
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
 *      HTS221, LPS22HB, LSM6DSL, LSM303AGR
 * - NUCLEO-F401RE NUCLEO-L476RG Nucleo boards
 * - STEVAL-STLCS01V1 (SensorTile) evaluation board that contains the following MEMS sensor devices:
 *      HTS221, LPS22HB, LSM303, LSM6DSM
 *
 * <b>Example Application</b>
 *
 * The Example application initizializes all the Components and Library creating 3 Custom Bluetooth services:
 * - The first service exposes all the HW characteristics related to MEMS sensor devices: Temperature, Humidity,
 * Pressure, Magnetometer, Gyroscope and Accelleromenter, LED status
 * - The Second Service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * For NUCLEO (STM32F401RE/STM32L476RG) boards the example application allows the user to control the initialization phase via UART.
 * Launch a terminal application and set the UART port to 460800 bps, 8 bit, No Parity,
 * 1 stop bit.For having the same UART functionality on SensorTile board, is necessary to recompile the code uncomment the line
 *      //#define MOTENV_ENABLE_PRINTF
 * on file:
 *      Projects\Multi\Applications\MOTENV1\Inc\motenv1_config.h file
 * This enables the UART that starts with a delay of 10 Seconds for allowing the time to open the UART for looking the initialization phase
 *
 * This example must be used with the related BlueMS Android/iOS application available on Play/itune store,
 * in order to read the sent information by Bluetooth Low Energy protocol
 *
 *                              -----------------------
 *                              | VERY IMPORTANT (1): |
 *                              -----------------------
 * The implementation for Nucleo-F401RE/Nucleo-L476RG and STEVAL-STLCS01V1 allow the Firmware-Over-The-Air (FOTA).
 * The FOTA is not available for Nucleo-L053R8 for FLASH size that is too little.
 *
 * ONLY for the implementations where FOTA is supported (Nucleo-F401RE/Nucleo-L476RG and STEVAL-STLCS01V1):
 * 
 * 1) The Firmware-Over-The-Air (FOTA) is done using the BlueMS Android/iOS application (Version 3.0.0 and above)
 * The FOTA does not work when using X-NUCLEO-IDB04A1
 *
 * 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
 * is saved at the beginning of the FLASH (address 0x08000000)
 *
 * The implementation of NUCLEO-L053R8 must run at the usual FLASH address (address 0x08000000) and the BootLoader must be avoided.
 *
 * For each IDE (IAR/µVision/System Workbench), and for each platform (NUCLEO-F401RE/NUCLEO-L476RG/SensorTile),
 *  there are some scripts CleanMotEnv1.bat/CleanMotEnv1.sh that makes the following operations:
 * - Full Flash Erase
 * - Load the BootLoader on the right flash region
 * - Load the Program (after the compilation) on the right flash region (This could be used for a FOTA)
 * - Dump back one single binary that contain BootLoader+Program that could be
 *  flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
 * - Reset the board
 *
 *                              -----------------------
 *                              | VERY IMPORTANT (2): |
 *                              -----------------------
 * The code for STM32L476RG-Nucleo and for TM32F401RE-Nucleo makes the auto discovery 
 * of the Sensors Board attached (IKS01A1/IKS01A2).
 * The code for STM32L053R8-Nucleo has not this functionality.
 * So it's necessary to choose the right Target configuration during the compilation.
 * If the code is compiled for IKS01A1 could not run if it's attached the X-NUCLEO-IKS01A2 and vice versa.
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
  #ifdef MOTENV_ENABLE_PRINTF
    extern TIM_HandleTypeDef  TimHandle;
    extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  #endif /* MOTENV_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
/* Exported Variables -------------------------------------------------------------*/
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

uint32_t ConnectionBleStatus  =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef    TimCCHandle;

uint8_t bdaddr[6];

uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;
uint32_t uhCCR1_Val = DEFAULT_uhCCR1_Val;

/* Private variables ---------------------------------------------------------*/
static volatile int ButtonPressed        =0;
static volatile int MEMSInterrupt        =0;
static volatile uint32_t HCI_ProcessEvent=0;
static volatile uint32_t SendEnv         =0;
static volatile uint32_t SendAccGyroMag  =0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);
static void InitTimers(void);
static void SendEnvironmentalData(void);
static void MEMSCallback(void);
static void ButtonCallback(void);
static void SendMotionData(void);

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

  MOTENV_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
         "\tSend Every %4ldmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4ldmS Acc/Gyro/Magneto\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         uhCCR1_Val/10,
         uhCCR4_Val/10);

#ifdef MOTENV_DEBUG_CONNECTION
  MOTENV_PRINTF("Debug Connection         Enabled\r\n");
#endif /* MOTENV_DEBUG_CONNECTION */

#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
  MOTENV_PRINTF("Debug Notify Trasmission Enabled\r\n");
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();  

  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }

  /* Set Full Scale to +/-2g */
  (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Set_FS_Value_IKS01A2 : BSP_ACCELERO_Set_FS_Value)(TargetBoardFeatures.HandleAccSensor,2.0f);

  /* Read the Acc Sensitivity */
  (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Sensitivity_IKS01A2 : BSP_ACCELERO_Get_Sensitivity)(TargetBoardFeatures.HandleAccSensor,&sensitivity_Mul);
  sensitivity_Mul *= ((float) FROM_MG_TO_G);

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

    /* Handle Interrupt from MEMS */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt=0;
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;       
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      HCI_Process();
    }

    if(set_connectable){
      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }

    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
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
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  /* TIM1_CH1 toggling with frequency = 2Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    SendEnv=1;
  }

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}

#ifdef STM32_SENSORTILE
#ifdef MOTENV_ENABLE_PRINTF
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
#endif /* MOTENV_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  MOTENV_PRINTF("UserButton Pressed\r\n");
}

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
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_FREE_FALL);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    /* Check if the interrupt is due to Double Tap */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_DOUBLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    /* Check if the interrupt is due to Single Tap */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_SINGLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_WAKE_UP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    /* Check if the interrupt is due to Tilt */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Tilt_Detection_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Tilt_Detection_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_TILT);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    /* Check if the interrupt is due to 6D Orientation */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_6D_Orientation_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_6D_Orientation_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEventType Orientation = GetHWOrientation6D();
      AccEvent_Notify(Orientation);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    /* Check if the interrupt is due to Pedometer */
    (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Pedometer_Status_Ext_IKS01A2 : BSP_ACCELERO_Get_Pedometer_Status_Ext)(TargetBoardFeatures.HandleAccSensor,&stat);
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

  /* Read the Acc values */
  (TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Get_Axes_IKS01A2 : BSP_ACCELERO_Get_Axes)(TargetBoardFeatures.HandleAccSensor,&ACC_Value);

  /* Read the Magneto values */
  (TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Get_Axes_IKS01A2 : BSP_MAGNETO_Get_Axes)(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  (TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Get_Axes_IKS01A2 : BSP_GYRO_Get_Axes)(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);

  AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value);
}


/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  uint8_t Status;

#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    MOTENV_PRINTF("Sending: ");
  }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
  
#ifdef STM32_SENSORTILE
  /* Battery Informations */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT)) {
    GG_Update();
#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Battery Report ");
       Term_Update(BufferToWrite,BytesToWrite);
     } else {
      MOTENV_PRINTF("Battery Report ");
     }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
  }
#endif /* STM32_SENSORTILE */

  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.HandlePressSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_IsInitialized_IKS01A2 : BSP_PRESSURE_IsInitialized)(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
        (TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Get_Press_IKS01A2 : BSP_PRESSURE_Get_Press)(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
        PressToSend=intPart*100+decPart;
#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV_PRINTF("Press=%ld ",PressToSend);
        }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_IsInitialized_IKS01A2 : BSP_HUMIDITY_IsInitialized)(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        (TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Get_Hum_IKS01A2 : BSP_HUMIDITY_Get_Hum)(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        HumToSend = intPart*10+decPart;
#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV_PRINTF("Hum=%d ",HumToSend);
        }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
      }
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_IsInitialized_IKS01A2 : BSP_TEMPERATURE_IsInitialized)(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        (TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Get_Temp_IKS01A2 : BSP_TEMPERATURE_Get_Temp)(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV_PRINTF("Temp=%d ",Temp1ToSend);
        }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
      }

      if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_IsInitialized_IKS01A2 : BSP_TEMPERATURE_IsInitialized)(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
        (TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Get_Temp_IKS01A2 : BSP_TEMPERATURE_Get_Temp)(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart;
#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV_PRINTF("Temp2=%d ",Temp2ToSend);
        }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
      }
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_IsInitialized_IKS01A2 : BSP_TEMPERATURE_IsInitialized)(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        (TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Get_Temp_IKS01A2 : BSP_TEMPERATURE_Get_Temp)(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV_PRINTF("Temp1=%d ",Temp1ToSend);
        }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
      }
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }

#ifdef MOTENV_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    MOTENV_PRINTF("\r\n");
  }
#endif /* MOTENV_DEBUG_NOTIFY_TRAMISSION */
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

  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance (Motion)*/
  /* Set TIM1 instance */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  TimCCHandle.Instance = TIM1;
#elif (defined (USE_STM32L0XX_NUCLEO))
  TimCCHandle.Instance = TIM2;
#endif
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
  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
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
  const char BoardName[8] = {NAME_MOTENV,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;

#ifdef MAC_MOTENV
  {
    uint8_t tmp_bdaddr[6]= {MAC_MOTENV};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_MOTENV */
  
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
  
#ifndef MAC_MOTENV
  #ifdef MAC_STM32UID_MOTENV
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
#ifdef STM32_NUCLEO
    /* if IDB05A1 = Number between 100->199
     * if IDB04A1 = Number between 0->99
     * where Y == (MOTENV_VERSION_MAJOR + OSX_BMS_VERSION_MINOR)&0xF */
    bdaddr[4] = (hwVersion > 0x30) ?
         ((((MOTENV_VERSION_MAJOR-48)*10) + (MOTENV_VERSION_MINOR-48)+100)&0xFF) :
         ((((MOTENV_VERSION_MAJOR-48)*10) + (MOTENV_VERSION_MINOR-48)    )&0xFF) ;
#else /* STM32_NUCLEO */
    bdaddr[4] = (((MOTENV_VERSION_MAJOR-48)*10) + (MOTENV_VERSION_MINOR-48)+100)&0xFF;
#endif  /* STM32_NUCLEO */
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_MOTENV */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

    if(ret){
      MOTENV_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_MOTENV */
#else /* MAC_MOTENV */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret){
     MOTENV_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
     goto fail;
  }
#endif /* MAC_MOTENV */
  
  ret = aci_gatt_init();    
  if(ret){
     MOTENV_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
     MOTENV_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

#ifndef  MAC_MOTENV
  #ifdef MAC_STM32UID_MOTENV
  ret = hci_le_set_random_address(bdaddr);

  if(ret){
     MOTENV_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
     goto fail;
  }
  #endif /* MAC_STM32UID_MOTENV */
#endif /* MAC_MOTENV */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     MOTENV_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     MOTENV_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  MOTENV_PRINTF("SERVER: BLE Stack Initialized \r\n"
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
  if(ret == BLE_STATUS_SUCCESS) {
     MOTENV_PRINTF("HW      Service W2ST added successfully\r\n");
  } else {
     MOTENV_PRINTF("\r\nError while adding HW Service W2ST\r\n");
  }

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     MOTENV_PRINTF("Console Service W2ST added successfully\r\n");
  } else {
     MOTENV_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     MOTENV_PRINTF("Config  Service W2ST added successfully\r\n");
  } else {
     MOTENV_PRINTF("\r\nError while adding Config Service W2ST\r\n");
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

#ifdef STM32_NUCLEO
#ifdef USE_STM32L0XX_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock configuration
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  __PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __SYSCFG_CLK_ENABLE();
}
#endif /* USE_STM32L0XX_NUCLEO */
#endif /* STM32_NUCLEO */

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
    ButtonPressed = 1;
    break;
#endif /* STM32_NUCLEO */

#ifdef STM32_NUCLEO
#ifdef USE_STM32L0XX_NUCLEO
  #ifdef IKS01A1
    case M_INT1_PIN:
  #elif IKS01A2
    case LSM6DSL_INT1_O_PIN:
  #endif /* IKS01A1 */
#else /* USE_STM32L0XX_NUCLEO */
   case M_INT1_PIN:
   case LSM6DSL_INT1_O_PIN:
#endif /* USE_STM32L0XX_NUCLEO */
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
     ex: MOTENV_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
