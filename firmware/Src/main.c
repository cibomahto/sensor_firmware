/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "hp203b.h"
#include "crc16_modbus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ADC internal channels related definitions */
/* Internal voltage reference VrefInt */
#define VREFINT_CAL_ADDR ((uint16_t*) (0x1FFFF7BAU)) /* Internal voltage reference, address of parameter VREFINT_CAL: VrefInt ADC raw data acquired at temperature 30 DegC (tolerance: +-5 DegC), Vref+ = 3.3 V (tolerance: +-10 mV). */
#define VREFINT_CAL_VREF ( 3300U)                    /* Analog voltage reference (Vref+) value with which temperature sensor has been calibrated in production (tolerance: +-10 mV) (unit: mV). */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adc_raw[2];
volatile uint8_t adc_ch_index = 0;
uint16_t vdda;
uint16_t vfb;

typedef struct {
    uint16_t SOF; // 0x5555
    uint8_t version; // 0
    uint8_t type; // Packet type: 0
    uint16_t vfb; // ADC value
    uint32_t airflow;
    uint32_t pressure;
    int32_t temperature;
    uint16_t crc; // TODO
} __attribute__((__packed__)) message_packet_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if 0
void init() {
    ssd1306_TestAll();
}

void loop() {
	HAL_Delay(100);
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//  HAL_ADCEx_Calibration_Start(&hadc);
  LL_ADC_StartCalibration(ADC1);
  while(LL_ADC_IsCalibrationOnGoing(ADC1)) {}

  LL_ADC_Enable(ADC1);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Start an hp203b reset, then just wait for a while
  hp203b_init();

  while (1)
  {
    // Start a (single? ADC conversion)
//    HAL_ADC_Start_IT(&hadc);

    if(hp203b_start_read_temp_pressure() != HP203B_ERROR_OK) {
      // TODO error handling
    }

    // Sample the first two ADC channels:
    // channel 0: Feedback voltage
    // channel 1: Reference voltage
    LL_ADC_REG_StartConversion(ADC1);

    while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) {}
    const uint16_t vdda_raw = LL_ADC_REG_ReadConversionData12(ADC1);

    while (LL_ADC_IsActiveFlag_EOC(ADC1) == 0) {}
    const uint16_t vref_raw = LL_ADC_REG_ReadConversionData12(ADC1);

    adc_ch_index = 0;
    //HAL_ADC_Stop(hdl);
    vdda = 3300 * (*VREFINT_CAL_ADDR) / vref_raw;
    vfb = vdda * vdda_raw / 4095 * 2; // TODO: Add calibration for resistor divider

    int32_t temperature = 0;
    uint32_t pressure = 0;

    if(hp203b_read_temp_pressure(&temperature, &pressure) != HP203B_ERROR_OK) {
      // TODO: Errror handling
    }

    // Calculate the wind speed given a linear fit
    // pressure(l/m) = e^(a*vfb_v+b)
    const double p0 = 2.467;
    const double p1 = -6.830;

    const double vfb_v = vfb/1000.0;

    const double af_lm = exp(p0*vfb_v + p1);


    const uint16_t airflow = round(af_lm*100.0);	// Convert to l/minute * 100

    message_packet_t packet = {
        .SOF = 0x5555,
        .version = 0,
        .type = 0,
        .vfb = vfb,
	.airflow = airflow,
        .pressure = pressure,
        .temperature = temperature,
        .crc = 0,
    };
    packet.crc = crc16_modbus((uint8_t *)&packet, sizeof(packet));
    HAL_UART_Transmit(&huart1, (uint8_t *)&packet, sizeof(packet), 0xFFFF);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

// CH0 (adc_raw[0]) : ADC_IN5, this is VFB
// CH1 (adc_raw[1]) : Vrefint
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hdl)
{
  if (__HAL_ADC_GET_FLAG(hdl, ADC_FLAG_EOC))
  {
    adc_raw[adc_ch_index++] = HAL_ADC_GetValue(hdl);
  }

  if (__HAL_ADC_GET_FLAG(hdl, ADC_FLAG_EOS))
  {
    adc_ch_index = 0;
    //HAL_ADC_Stop(hdl);
    vdda = 3300 * (*VREFINT_CAL_ADDR) / adc_raw[1];
    vfb = vdda * adc_raw[0] / 4095 * 2; // TODO: Add calibration for resistor divider
  }
}
*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
