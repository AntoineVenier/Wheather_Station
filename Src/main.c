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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
signed long temperature_raw, pressure_raw, humidity_raw;
unsigned short dig_T1, dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2, dig_H4, dig_H5;
unsigned char dig_H1, dig_H3;
signed char dig_H6;
float temperature, pressure, humidity;

char resultat[100];
uint8_t rx_buffer[8];
double var1, var2;
double t_fine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void init_BME280(void);
void callibration_BME280(void);
void calc_Temp(void);
void calc_Press(void);
void calc_Hum(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	HAL_TIM_Base_Start_IT(&htim2);
	
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_Led_Pin|Orange_Led_Pin|Red_Led_Pin|Blue_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_Led_Pin Orange_Led_Pin Red_Led_Pin Blue_Led_Pin */
  GPIO_InitStruct.Pin = Green_Led_Pin|Orange_Led_Pin|Red_Led_Pin|Blue_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void init_BME280(void)
{
		uint8_t register_init[2];
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	
		/*Init*/
		/*Activate measure for temp and press*/
		register_init[0]=0xF4;//register ctrl_meas
		register_init[1]=0x4A;
		HAL_I2C_Master_Transmit(&hi2c1,0xEC,register_init,2,10000);
		
		/*Activate measure for humidity*/
		/*The “ctrl_hum”register sets the humidity data acquisition options 
		of the device.Changes to this register only become effective after 
		a write operation to “ctrl_meas”.*/
		register_init[0]=0xF2;//register ctrl_hum
		register_init[1]=0x01;
		HAL_I2C_Master_Transmit(&hi2c1,0xEC,register_init,2,10000);
	
}

void callibration_BME280(void)
{
		uint8_t rx_buff_temp_press[24];
		uint8_t rx_buff_hum_1[1];
		uint8_t rx_buff_hum_2[7];
		uint8_t starting_address_temp_press=0x88;
		uint8_t starting_address_hum_1=0xA1;
		uint8_t starting_address_hum_2=0xE1;
		
	
		/*calibration*/		
		HAL_I2C_Master_Transmit(&hi2c1,0xEC,&starting_address_temp_press,1,10000);
		HAL_I2C_Master_Receive(&hi2c1,0xED,&rx_buff_temp_press[0],24,10000);


		dig_T1=(rx_buff_temp_press[0])+(rx_buff_temp_press[1]<<8);
		dig_T2=(rx_buff_temp_press[2])+(rx_buff_temp_press[3]<<8);
		dig_T3=(rx_buff_temp_press[4])+(rx_buff_temp_press[5]<<8);
		dig_P1=(rx_buff_temp_press[6])+(rx_buff_temp_press[7]<<8);
		dig_P2=(rx_buff_temp_press[8])+(rx_buff_temp_press[9]<<8);
		dig_P3=(rx_buff_temp_press[10])+(rx_buff_temp_press[11]<<8);
		dig_P4=(rx_buff_temp_press[12])+(rx_buff_temp_press[13]<<8);
		dig_P5=(rx_buff_temp_press[14])+(rx_buff_temp_press[15]<<8);
		dig_P6=(rx_buff_temp_press[16])+(rx_buff_temp_press[17]<<8);
		dig_P7=(rx_buff_temp_press[18])+(rx_buff_temp_press[19]<<8);
		dig_P8=(rx_buff_temp_press[20])+(rx_buff_temp_press[21]<<8);
		dig_P9=(rx_buff_temp_press[22])+(rx_buff_temp_press[23]<<8);
		
		HAL_I2C_Master_Transmit(&hi2c1,0xEC,&starting_address_hum_1,1,10000);
		HAL_I2C_Master_Receive(&hi2c1,0xED,&rx_buff_hum_1[0],1,10000);
		
		dig_H1=rx_buff_hum_1[0];
		
		HAL_I2C_Master_Transmit(&hi2c1,0xEC,&starting_address_hum_2,1,10000);
		HAL_I2C_Master_Receive(&hi2c1,0xED,&rx_buff_hum_2[0],7,10000);
		
		dig_H2=(rx_buff_hum_2[1]<<8)|(rx_buff_hum_2[0]);
		dig_H3=rx_buff_hum_2[2];
		dig_H4=(rx_buff_hum_2[3]<<4)|(0x0F & rx_buff_hum_2[4]);
		dig_H5=((rx_buff_hum_2[5]<<4))| ((rx_buff_hum_2[4]>>4) & 0x0F);
		dig_H6=rx_buff_hum_2[6];
	
}

void calc_Temp(void)
{ 
		uint8_t starting=0xF7;
	
		/*Calc value*/
		HAL_I2C_Master_Transmit(&hi2c1,0xEC,&starting,1,10000);
		HAL_I2C_Master_Receive(&hi2c1,0xED,&rx_buffer[0],8,10000);

		volatile uint32_t temp[3];
		temp[2]=rx_buffer[3];
		temp[1]=rx_buffer[4];
		temp[0]=rx_buffer[5];
		temperature_raw=(temp[2]<<12)+(temp[1]<<4)+(temp[0]>>4);

		temp[2]=rx_buffer[0];
		temp[1]=rx_buffer[1];
		temp[0]=rx_buffer[2];
		pressure_raw=(temp[2]<<12)+(temp[1]<<4)+(temp[0]>>4);
		
		temp[2]=rx_buffer[6];
		temp[1]=rx_buffer[7];
		humidity_raw=(temp[2]<<8)+(temp[1]);

		/*Temperature*/
		
		var1=(((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
		var2=((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
		t_fine = (int32_t)(var1+var2);
		volatile	float T = (var1+var2)/5120.0;
		
		temperature=T;
	
}

void calc_Press(void)
{
		/*press*/
		var1=((double)t_fine/2.0)-64000.0;
		var2=var1*var1*((double)dig_P6)/32768.0;
		var2=var2+var1*((double)dig_P5)*2.0;
		var2=(var2/4.0)+(((double)dig_P4)*65536.0);
		var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
		var1=(1.0+var1/32768.0)*((double)dig_P1);
		volatile	double p=1048576.0-(double)pressure_raw;
		p=(p-(var2/4096.0))*6250.0/var1;
		var1=((double)dig_P9)*p*p/2147483648.0;
		var2=p*((double)dig_P8)/32768.0;
		p=p+(var1+var2+((double)dig_P7))/16.0;
	
		pressure=p;
}

void calc_Hum(void)
{
		/*humidity*/
  	var1 = (t_fine - ((int32_t)76800));
		var1 = (((((humidity_raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
		((int32_t)16384))/32768) * (((((((var1 * ((int32_t)dig_H6))/1024) * (((var1 *
		((int32_t)dig_H3))/2048) + ((int32_t)32768)))/1024) + ((int32_t)2097152)) *
		((int32_t)dig_H2) + 8192)/16384));
		var1 = (var1 - (((((var1/32768) * (var1/32768))/128) * ((int32_t)dig_H1))/16));
		var1 = (var1 < 0 ? 0 : var1);
		var1 = (var1 > 419430400 ? 419430400 : var1);
		volatile	double h=((uint32_t)(var1/4096))/1024.0;
	
	  humidity=h;
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
		init_BME280();
		callibration_BME280();
		calc_Temp();
		calc_Press();
		calc_Hum();
		sprintf(resultat,"%f\n%f\n%f\n",temperature,pressure,humidity);
		
		HAL_UART_Transmit(&huart2,(uint8_t *)resultat,sizeof(resultat),10);
}

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
