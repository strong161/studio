/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  Central LAB
  * @version V2.2.0
  * @date    30-November-2016
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
    #include "usbd_core.h"
    #include "usbd_cdc.h"
    #include "usbd_cdc_interface.h"
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Imported variables ---------------------------------------------------------*/

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
     extern USBD_DescriptorsTypeDef VCP_Desc;
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

#ifdef STM32_SENSORTILE
  #ifdef ALLMEMS1_ENABLE_PRINTF
    USBD_HandleTypeDef  USBD_Device;
  #endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

volatile float RMS_Ch[AUDIO_CHANNELS];
float DBNOISE_Value_Old_Ch[AUDIO_CHANNELS];
uint16_t PCM_Buffer[AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY/1000];

#ifdef USE_STM32F4XX_NUCLEO
uint16_t PDM_Buffer[AUDIO_CHANNELS*AUDIO_SAMPLING_FREQUENCY/1000*64/8];
#endif /* USE_STM32F4XX_NUCLEO */

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);
static void Init_MEMS_Mics(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  TargetType_t BoardType Nucleo/BlueCoin/SensorTile
  * @retval None
  */
void InitTargetPlatform(TargetType_t BoardType)
{
  TargetBoardFeatures.BoardType = BoardType;
#ifdef STM32_NUCLEO
#ifdef ALLMEMS1_ENABLE_PRINTF
  /* UART Initialization */
  if(UART_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    ALLMEMS1_PRINTF("UART Initialized\r\n");
  }
#endif /* ALLMEMS1_ENABLE_PRINTF */

  /* I2C Initialization */
  if(I2C_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    ALLMEMS1_PRINTF("I2C  Initialized\r\n");
  }
  
  /* Initialize the BlueNRG SPI driver */
  if(SPI_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    ALLMEMS1_PRINTF("SPI  Initialized\r\n");
  }
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#elif STM32_SENSORTILE

  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();  
  

#ifdef ALLMEMS1_ENABLE_PRINTF
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Configure the CDC */
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  /* 10 seconds ... for having time to open the Terminal
   * for looking the BlueMicrosystem Initialization phase */
  HAL_Delay(10000);
#endif /* ALLMEMS1_ENABLE_PRINTF */
#endif /* STM32_NUCLEO */
  
  /* Initialize LED */
#ifdef STM32_NUCLEO  
  BSP_LED_Init(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Init( LED1 );
#endif /* STM32_NUCLEO */

  ALLMEMS1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
#ifdef USE_STM32F4XX_NUCLEO
  #ifdef STM32_NUCLEO
        "\tSTM32F401RE-Nucleo board"
  #endif /* STM32_NUCLEO */
#elif USE_STM32L4XX_NUCLEO
  #ifdef STM32_SENSORTILE
        "\tSTM32476RG-SensorTile board"
  #elif STM32_NUCLEO
        "\tSTM32L476RG-Nucleo board"
  #endif /* STM32_SENSORTILE */


#endif /* USE_STM32L4XX_NUCLEO */
          "\r\n",
          ALLMEMS1_PACKAGENAME,
          ALLMEMS1_VERSION_MAJOR,ALLMEMS1_VERSION_MINOR,ALLMEMS1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();

#ifdef STM32_SENSORTILE
  /* Inialize the Gas Gouge if the battery is present */
  if(BSP_GG_Init(&TargetBoardFeatures.HandleGGComponent) == COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Gas Gouge Component\n\r");
  } else {
    ALLMEMS1_PRINTF("Battery not present\n\r");
  }
#endif /* STM32_SENSORTILE */
  
  /* Initialize Mic */
  Init_MEMS_Mics();
  
  TargetBoardFeatures.bnrg_expansion_board = IDB04A1; /* IDB04A1 by default */
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
#ifdef STM32_NUCLEO
  int32_t AccOK     =0;
  int32_t GyroOK    =0;
  int32_t MagOK     =0;
  int32_t TempOK[2] = {0,0};
  int32_t PressOK   =0;
  int32_t HumOK     =0;
#endif /* STM32_NUCLEO */

#ifdef IKS01A1
  ALLMEMS1_PRINTF("Code compiled for X-NUCLEO-IKS01A1\r\n");
#elif IKS01A2
  ALLMEMS1_PRINTF("Code compiled for X-NUCLEO-IKS01A2\r\n");
#endif /* IKS01A1 */

  /* Accelero */
  if (BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Accelero Sensor\n\r");
#ifdef STM32_NUCLEO
    AccOK=1;
#endif /* STM32_NUCLEO */
  }
  
  /* DS3/DSM or DS0 */
#ifdef STM32_NUCLEO

  #ifdef IKS01A1
  {
    uint8_t WhoAmI;
    BSP_ACCELERO_Get_WhoAmI(TargetBoardFeatures.HandleAccSensor,&WhoAmI);
    if(LSM6DS3_ACC_GYRO_WHO_AM_I==WhoAmI) {
      ALLMEMS1_PRINTF("\tDS3 DIL24 Present\n\r");
      TargetBoardFeatures.HWAdvanceFeatures = 1;
    } else {
      TargetBoardFeatures.HWAdvanceFeatures = 0;
    }
  }
  #elif IKS01A2
    #ifndef DISABLE_LSM6DSL_IT
    TargetBoardFeatures.HWAdvanceFeatures = 1;
    #else
    TargetBoardFeatures.HWAdvanceFeatures = 0;
    #endif
  #endif /* IKS01A1 */
#else /* STM32_NUCLEO */
  TargetBoardFeatures.HWAdvanceFeatures = 1;
#endif /* STM32_NUCLEO */

  /* Gyro */
  if(BSP_GYRO_Init( GYRO_SENSORS_AUTO, &TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Gyroscope Sensor\n\r");
#ifdef STM32_NUCLEO
    GyroOK=1;
#endif /* STM32_NUCLEO */
  }

  if(BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Magneto Sensor\n\r");
#ifdef STM32_NUCLEO
    MagOK=1;
#endif /* STM32_NUCLEO */


  } 

  /* Humidity */  
  if(BSP_HUMIDITY_Init( HUMIDITY_SENSORS_AUTO, &TargetBoardFeatures.HandleHumSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Humidity Sensor\n\r");
#ifdef STM32_NUCLEO
    HumOK=1;
#endif /* STM32_NUCLEO */
  }

  /* Temperature1 */
  if(BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     ALLMEMS1_PRINTF("OK Temperature Sensor1\n\r");
#ifdef STM32_NUCLEO
     TempOK[0]=1;
#endif /* STM32_NUCLEO */
     TargetBoardFeatures.NumTempSensors++;

  } 


  /* Temperature2 */
#ifdef STM32_NUCLEO

  #ifdef IKS01A1
  if(BSP_TEMPERATURE_Init( LPS25HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
  #elif IKS01A2
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
  #endif /* IKS01A1 */


     ALLMEMS1_PRINTF("OK Temperature Sensor2\n\r");
	 TempOK[1]=1;
     TargetBoardFeatures.NumTempSensors++;

  } 

#elif STM32_SENSORTILE
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     ALLMEMS1_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    ALLMEMS1_PRINTF("Error Temperature Sensor2\n\r");
  }
#endif /* STM32_NUCLEO */
  
  /* Pressure */
  if(BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &TargetBoardFeatures.HandlePressSensor )==COMPONENT_OK){
    ALLMEMS1_PRINTF("OK Pressure Sensor\n\r");
#ifdef STM32_NUCLEO
    PressOK=1;
#endif /* STM32_NUCLEO */
  }

  
#ifdef STM32_NUCLEO
  if(AccOK+GyroOK+MagOK+HumOK+PressOK+TempOK[0]+TempOK[1]) {
    /* There is at least one sensor present */
    if(!AccOK) {
       ALLMEMS1_PRINTF("Error Accelero Sensor\n\r");
    }
    if(!MagOK) {
       ALLMEMS1_PRINTF("Error Magneto Sensor\n\r");
    }
    if(!GyroOK) {
       ALLMEMS1_PRINTF("Error Gyroscope Sensor\n\r");
    }
    if(!TempOK[0]) {
       ALLMEMS1_PRINTF("Error Temperature Sensor1\n\r");
    }
    if(!TempOK[1]) {
       ALLMEMS1_PRINTF("Error Temperature Sensor2\n\r");
    }
    if(!HumOK) {
       ALLMEMS1_PRINTF("Error Humidity Sensor\n\r");
    }
    if(!PressOK) {
       ALLMEMS1_PRINTF("Error Pressure Sensor\n\r");
    }
    TargetBoardFeatures.SensorEmulation=0;
  } else {
    ALLMEMS1_PRINTF("Sensors Board not present -> Emulated\r\n");
    TargetBoardFeatures.SensorEmulation=1;
  }
#endif /* STM32_NUCLEO */

  /*  Enable all the sensors */
#ifdef STM32_NUCLEO
  if(!TargetBoardFeatures.SensorEmulation) {
#endif /* STM32_NUCLEO */
    if(TargetBoardFeatures.HandleAccSensor) {
      if(BSP_ACCELERO_Sensor_Enable( TargetBoardFeatures.HandleAccSensor )==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Accelero Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleGyroSensor) {
      if(BSP_GYRO_Sensor_Enable( TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Gyroscope Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleMagSensor) {
      if(BSP_MAGNETO_Sensor_Enable( TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Magneto Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if(BSP_HUMIDITY_Sensor_Enable( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Humidity Sensor\n\r");
      }
    }

    if(TargetBoardFeatures.HandleTempSensors[0]){
      if(BSP_TEMPERATURE_Sensor_Enable( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Temperature Sensor1\n\r");
      }
    }
  
    if(TargetBoardFeatures.HandleTempSensors[1]){
      if(BSP_TEMPERATURE_Sensor_Enable( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Temperature Sensor2\n\r");
      }
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      if(BSP_PRESSURE_Sensor_Enable( TargetBoardFeatures.HandlePressSensor)==COMPONENT_OK) {
        ALLMEMS1_PRINTF("Enabled Pressure Sensor\n\r");
        }
      }

#ifdef STM32_NUCLEO
  }
#endif /* STM32_NUCLEO */
}

/** @brief Initialize all the MEMS's Microphones
 * @param None
 * @retval None
 */
static void Init_MEMS_Mics(void) {
  uint8_t ret;
  ret = BSP_AUDIO_IN_Init(AUDIO_SAMPLING_FREQUENCY, 16, AUDIO_CHANNELS);
  
  if(ret!=AUDIO_OK)
  {
    ALLMEMS1_PRINTF("Error Audio Init\r\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("OK Audio Init\r\n");
  }
  
  /* Set the volume level to 64 */
  ret = BSP_AUDIO_IN_SetVolume(AUDIO_VOLUME_VALUE);
  if(ret!=AUDIO_OK)
  {
    ALLMEMS1_PRINTF("Error Audio Volume\r\n\n");
    
    while(1) {
      ;
    }
  }
  else
  {
    ALLMEMS1_PRINTF("OK Audio Volume\r\n\n");
  }

  /* Number of Microphones */
  TargetBoardFeatures.NumMicSensors=AUDIO_CHANNELS;
}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_On(LED2);
#elif STM32_SENSORTILE
  BSP_LED_On( LED1 );  
#endif /* STM32_NUCLEO */
  
  TargetBoardFeatures.LedStatus = 1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
#ifdef STM32_NUCLEO
  BSP_LED_Off(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Off( LED1 );  
#endif /* STM32_NUCLEO */
  
  TargetBoardFeatures.LedStatus = 0;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
