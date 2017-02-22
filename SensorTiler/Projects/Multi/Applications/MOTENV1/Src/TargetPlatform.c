/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  Central LAB
  * @version V2.2.0
  * @date    24-November-2016
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
  #ifdef MOTENV_ENABLE_PRINTF
    #include "usbd_core.h"
    #include "usbd_cdc.h"
    #include "usbd_cdc_interface.h"
  #endif /* MOTENV_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Imported variables ---------------------------------------------------------*/
#ifdef STM32_SENSORTILE
  #ifdef MOTENV_ENABLE_PRINTF
    extern USBD_DescriptorsTypeDef VCP_Desc;
  #endif /* MOTENV_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

#ifdef STM32_SENSORTILE
  #ifdef MOTENV_ENABLE_PRINTF
    USBD_HandleTypeDef  USBD_Device;
  #endif /* MOTENV_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  TargetType_t BoardType Nucleo/SensorTile
  * @retval None
  */
void InitTargetPlatform(TargetType_t BoardType)
{
  TargetBoardFeatures.BoardType = BoardType;
#if ((defined STM32_NUCLEO) && (defined MOTENV_ENABLE_PRINTF))
  /* UART Initialization */
  if(UART_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    MOTENV_PRINTF("UART Initialized\r\n");
  }
#endif /* ((defined STM32_NUCLEO) && (defined MOTENV_ENABLE_PRINTF)) */
#ifdef STM32_NUCLEO
  /* I2C Initialization */
  if(I2C_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    MOTENV_PRINTF("I2C  Initialized\r\n");
  }
  
  /* Initialize the BlueNRG SPI driver */
  if(SPI_Global_Init()!=HAL_OK) {
    Error_Handler();
  } else {
    MOTENV_PRINTF("SPI  Initialized\r\n");
  }
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#elif STM32_SENSORTILE

  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();

#ifdef MOTENV_ENABLE_PRINTF
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
   * for looking the MOTENV1 Initialization phase */
  HAL_Delay(10000);
#endif /* MOTENV_ENABLE_PRINTF */
#endif /* STM32_NUCLEO */
  
  /* Initialize LED */
#ifdef STM32_NUCLEO  
  BSP_LED_Init(LED2);
#elif STM32_SENSORTILE
  BSP_LED_Init( LED1 );  
#endif /* STM32_NUCLEO */

  MOTENV_PRINTF("\r\nSTMicroelectronics %s:\r\n"
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
#elif USE_STM32L0XX_NUCLEO
        "\tSTM32L053R8-Nucleo board"
#endif /* USE_STM32F4XX_NUCLEO */
          "\r\n",
          MOTENV_PACKAGENAME,
          MOTENV_VERSION_MAJOR,MOTENV_VERSION_MINOR,MOTENV_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the Target's Features */
  Init_MEM1_Sensors();

#ifdef STM32_SENSORTILE
  /* Inialize the Gas Gouge if the battery is present */
  if(BSP_GG_Init(&TargetBoardFeatures.HandleGGComponent) == COMPONENT_OK){
    MOTENV_PRINTF("OK Gas Gouge Component\n\r");
  } else {
    MOTENV_PRINTF("Battery not present\n\r");
  }
#endif /* STM32_SENSORTILE */

  TargetBoardFeatures.bnrg_expansion_board = IDB04A1; /* IDB04A1 by default */
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
   /* Accelero */
#ifdef STM32_NUCLEO
  #ifdef USE_STM32L0XX_NUCLEO
  if (BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK) {
      MOTENV_PRINTF("OK Accelero Sensor\n\r");
    #ifdef IKS01A1
      TargetBoardFeatures.SnsAltFunc = 0;
    #else /* IKS01A1 */
      TargetBoardFeatures.SnsAltFunc = 1;
    #endif /* IKS01A1 */
   }
  #else /* USE_STM32L0XX_NUCLEO */
    /* Test if the board is IK01A1 */
    if (BSP_ACCELERO_Init_IKS01A1( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK) {
      MOTENV_PRINTF("IKS01A1 board\n\r");
      MOTENV_PRINTF("OK Accelero Sensor\n\r");
      TargetBoardFeatures.SnsAltFunc = 0;
    } else {
      TargetBoardFeatures.SnsAltFunc = 1;
      MOTENV_PRINTF("IKS01A2 board\n\r");
      if (BSP_ACCELERO_Init_IKS01A2( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK){
        MOTENV_PRINTF("OK Accelero Sensor\n\r");
      }
    }
  #endif /* USE_STM32L0XX_NUCLEO */
#else /* STM32_NUCLEO */
  if (BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &TargetBoardFeatures.HandleAccSensor ) == COMPONENT_OK) {
    MOTENV_PRINTF("OK Accelero Sensor\n\r");
    TargetBoardFeatures.SnsAltFunc = 1;
  }
#endif /* STM32_NUCLEO */
  
  /* DS3/DSM or DS0 */
#ifdef STM32_NUCLEO
  #ifndef IKS01A2
  /* This section works with IKS01A1 or with IKS01A1/A2 Autodiscovery */
  if(!TargetBoardFeatures.SnsAltFunc){
    uint8_t WhoAmI;
    BSP_ACCELERO_Get_WhoAmI(TargetBoardFeatures.HandleAccSensor,&WhoAmI);
    if(LSM6DS3_ACC_GYRO_WHO_AM_I==WhoAmI) {
      MOTENV_PRINTF("\tDS3 DIL24 Present\n\r");
      TargetBoardFeatures.HWAdvanceFeatures = 1;
    } else {
      TargetBoardFeatures.HWAdvanceFeatures = 0;
    }
  } else {
    TargetBoardFeatures.HWAdvanceFeatures = 1;
  }
  #else /* IKS01A2 */
    TargetBoardFeatures.HWAdvanceFeatures = 1;
  #endif /* IKS01A2 */
#else /* STM32_NUCLEO */
  TargetBoardFeatures.HWAdvanceFeatures = 1;
#endif /* STM32_NUCLEO */

  /* Gyro */
  if((TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Init_IKS01A2 : BSP_GYRO_Init)( GYRO_SENSORS_AUTO, &TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK){
    MOTENV_PRINTF("OK Gyroscope Sensor\n\r");
  } else {
    MOTENV_PRINTF("Error Gyroscope Sensor\n\r");
    while(1);
  }

  if((TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Init_IKS01A2 : BSP_MAGNETO_Init)( MAGNETO_SENSORS_AUTO, &TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK){
    MOTENV_PRINTF("OK Magneto Sensor\n\r");
  } else {
    MOTENV_PRINTF("Error Magneto Sensor\n\r");
    while(1);
  }

  /* Humidity */  
  if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Init_IKS01A2 : BSP_HUMIDITY_Init)( HUMIDITY_SENSORS_AUTO, &TargetBoardFeatures.HandleHumSensor )==COMPONENT_OK){
    MOTENV_PRINTF("OK Humidity Sensor\n\r");
  } else {
    MOTENV_PRINTF("Error Humidity Sensor\n\r");
  }

  /* Temperature1 */
  if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Init_IKS01A2 : BSP_TEMPERATURE_Init)( TEMPERATURE_SENSORS_AUTO, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     MOTENV_PRINTF("OK Temperature Sensor1\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    MOTENV_PRINTF("Error Temperature Sensor1\n\r");
  }

  /* Temperature2 */
#ifdef STM32_NUCLEO
  #ifdef USE_STM32L0XX_NUCLEO
    #ifdef IKS01A1
      if(BSP_TEMPERATURE_Init(LPS25HB_T_0,&TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
    #else /* IKS01A1 */
       if(BSP_TEMPERATURE_Init(LPS22HB_T_0,&TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
    #endif /* IKS01A1 */
  #else /* USE_STM32L0XX_NUCLEO */
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Init_IKS01A2 : BSP_TEMPERATURE_Init) (TargetBoardFeatures.SnsAltFunc ? LPS22HB_T_0: LPS25HB_T_0,&TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
  #endif /* USE_STM32L0XX_NUCLEO */    
     MOTENV_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    MOTENV_PRINTF("Error Temperature Sensor2\n\r");
  }
#elif STM32_SENSORTILE
  if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &TargetBoardFeatures.HandleTempSensors[TargetBoardFeatures.NumTempSensors] )==COMPONENT_OK){
     MOTENV_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
  } else {
    MOTENV_PRINTF("Error Temperature Sensor2\n\r");
  }
#endif /* STM32_NUCLEO */
  
  /* Pressure */
  if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Init_IKS01A2 : BSP_PRESSURE_Init)( PRESSURE_SENSORS_AUTO, &TargetBoardFeatures.HandlePressSensor )==COMPONENT_OK){
    MOTENV_PRINTF("OK Pressure Sensor\n\r");
  } else {
    MOTENV_PRINTF("Error Pressure Sensor\n\r");
  }

  /*  Enable all the sensors */
  if(TargetBoardFeatures.HandleAccSensor) {
    if((TargetBoardFeatures.SnsAltFunc ? BSP_ACCELERO_Sensor_Enable_IKS01A2 : BSP_ACCELERO_Sensor_Enable)( TargetBoardFeatures.HandleAccSensor )==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Accelero Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleGyroSensor) {
    if((TargetBoardFeatures.SnsAltFunc ? BSP_GYRO_Sensor_Enable_IKS01A2 : BSP_GYRO_Sensor_Enable)( TargetBoardFeatures.HandleGyroSensor )==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Gyroscope Sensor\n\r");
    }
  } 

  if(TargetBoardFeatures.HandleMagSensor) {
    if((TargetBoardFeatures.SnsAltFunc ? BSP_MAGNETO_Sensor_Enable_IKS01A2 : BSP_MAGNETO_Sensor_Enable)( TargetBoardFeatures.HandleMagSensor )==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Magneto Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    if((TargetBoardFeatures.SnsAltFunc ? BSP_HUMIDITY_Sensor_Enable_IKS01A2 : BSP_HUMIDITY_Sensor_Enable) ( TargetBoardFeatures.HandleHumSensor)==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Humidity Sensor\n\r");
    }
  }

  if(TargetBoardFeatures.HandleTempSensors[0]){
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Sensor_Enable_IKS01A2 : BSP_TEMPERATURE_Sensor_Enable)( TargetBoardFeatures.HandleTempSensors[0])==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Temperature Sensor1\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[1]){
    if((TargetBoardFeatures.SnsAltFunc ? BSP_TEMPERATURE_Sensor_Enable_IKS01A2 : BSP_TEMPERATURE_Sensor_Enable)( TargetBoardFeatures.HandleTempSensors[1])==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Temperature Sensor2\n\r");
    }
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    if((TargetBoardFeatures.SnsAltFunc ? BSP_PRESSURE_Sensor_Enable_IKS01A2 : BSP_PRESSURE_Sensor_Enable)( TargetBoardFeatures.HandlePressSensor)==COMPONENT_OK) {
      MOTENV_PRINTF("Enabled Pressure Sensor\n\r");
    }
  }
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
  TargetBoardFeatures.LedStatus=1;
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
  TargetBoardFeatures.LedStatus=0;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
