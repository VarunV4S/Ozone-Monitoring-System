/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t ADC_Read(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TxData1[15];
uint8_t TxData2[15];
uint8_t TxData3[15];
uint8_t TxData4[15];
uint8_t TxData5[15];
uint8_t TxData6[15];
uint8_t TxData7[15];
uint8_t TxData8[15];
uint8_t rxData[3] , TxozoneBuffer[4];
uint16_t channel_0_value, channel_1_value;
float vol1 , vol2;
uint16_t ozone1 , ozone2;
uint16_t Setpoint1 , Setpoint2;
uint16_t crc_value, C;
uint8_t txData[4] = {0xAA, 0xBB, 0xCC, 0xDD}; // Data to be transmitted

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  TxData1[0] = 0xFF;  // slave address
    TxData1[1] = 0x05;  // Force single coil
    TxData1[2] = 0;  // coil address high
    TxData1[3] = 0;  // coil address low
    //The coil address will be 00000000 00000000 = 0 + 1 = 1
    TxData1[4] = 0xFF;  // force data high
    TxData1[5] = 0;  // force data low
    TxData1[6] = 0x99;//crc
    TxData1[7] = 0xE4;

    TxData2[0] = 0xFF;  // slave address
    TxData2[1] = 0x05;  // Force single coil
    TxData2[2] = 0;  // coil address high
    TxData2[3] = 0;  // coil address low
    //The coil address will be 00000000 00000000 = 0 + 1 = 1
    TxData2[4] = 0;  // force data high
    TxData2[5] = 0;  // force data low
    TxData2[6] = 0xD8;//crc
    TxData2[7] = 0x14;

    TxData3[0] = 0xFF;  // slave address
      TxData3[1] = 0x05;  // Force single coil
      TxData3[2] = 0;  // coil address high
      TxData3[3] = 0X01;  // coil address low
      //The coil address will be 00000000 00000000 = 0 + 1 = 1
      TxData3[4] = 0xFF;  // force data high
      TxData3[5] = 0;  // force data low
      TxData3[6] = 0xC8;//crc
      TxData3[7] = 0x24;

      TxData4[0] = 0xFF;  // slave address
      TxData4[1] = 0x05;  // Force single coil
      TxData4[2] = 0;  // coil address high
      TxData4[3] = 0X01;  // coil address low
      //The coil address will be 00000000 00000000 = 0 + 1 = 1
      TxData4[4] = 0;  // force data high
      TxData4[5] = 0;  // force data low
      TxData4[6] = 0x89;//crc
      TxData4[7] = 0xD4;

      TxData5[0] = 0xFF;  // slave address
        TxData5[1] = 0x05;  // Force single coil
        TxData5[2] = 0;  // coil address high
        TxData5[3] = 0X02;  // coil address low
        //The coil address will be 00000000 00000000 = 0 + 1 = 1
        TxData5[4] = 0xFF;  // force data high
        TxData5[5] = 0;  // force data low
        TxData5[6] = 0x38;//crc
        TxData5[7] = 0x24;

        TxData6[0] = 0xFF;  // slave address
        TxData6[1] = 0x05;  // Force single coil
        TxData6[2] = 0;  // coil address high
        TxData6[3] = 0X02;  // coil address low
        //The coil address will be 00000000 00000000 = 0 + 1 = 1
        TxData6[4] = 0;  // force data high
        TxData6[5] = 0;  // force data low
        TxData6[6] = 0x79;//crc
        TxData6[7] = 0xD4;

        TxData7[0] = 0xFF;  // slave address
          TxData7[1] = 0x05;  // Force single coil
          TxData7[2] = 0;  // coil address highagain
          TxData7[3] = 0X03;  // coil address low
          //The coil address will be 00000000 00000000 = 0 + 1 = 1
          TxData7[4] = 0xFF;  // force data high
          TxData7[5] = 0;  // force data low
          TxData7[6] = 0x69;//crc
          TxData7[7] = 0xE4;
          TxData8[0] = 0xFF;  // slave address
          TxData8[1] = 0x05;  // Force single coil
          TxData8[2] = 0;  // coil address high
          TxData8[3] = 0X03;  // c oil address low
          //The coil address will be 00000000 00000000 = 0 + 1 = 1
          TxData8[4] = 0;  // force data high
          TxData8[5] = 0;  // force data low
          TxData8[6] = 0x28;//crc
          TxData8[7] = 0x14;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			channel_0_value = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 1000);
			// Read channel 0 value
			// Wait for ADC conversion to complete
			channel_1_value = HAL_ADC_GetValue(&hadc2);
			HAL_ADC_Stop(&hadc2);

	  	         // Read channel 1 value
	  	  	 	   vol1 = channel_0_value * 0.00080;
	  	  	 	   vol2 = channel_1_value * 0.00080;
	  	  	 	   ozone1 = vol1 * 3030;
	  	  	 	   ozone2 = vol2 * 3030;
	  	  	 	if(ozone1<=10)
	  	  	 				{
	  	  	 		 	 		ozone1=10;
	  	  	 		 	 	}
	  	  	 	if(ozone2<=10)
	  	  	 		  	 				{
	  	  	 		  	 		 	 		ozone2=10;
	  	  	 		  	 		 	 	}
	  	  	 	   TxozoneBuffer[0] = ozone1/100;//(ozone1 & 0xff00) >> 8;
	  	  	 	   TxozoneBuffer[1] = ozone1%100;//(ozone1 & 0x00ff);
	  	  	 	   TxozoneBuffer[2] = ozone2/100;//(ozone2 & 0xff00) >> 8;
	  	  	 	   TxozoneBuffer[3] = ozone2%100;//(ozone2 & 0x00ff);


	  	  	  HAL_UART_Transmit(&huart1,(uint8_t*)&TxozoneBuffer,4, 500);


//	  	  	  HAL_UART_Transmit(&huart4, (uint8_t*)&crc_value, sizeof(crc_value), HAL_MAX_DELAY); // Transmit the CRC


	  	  	  //Break
	  	  	  HAL_UART_Receive_IT (&huart1, rxData, 3);
	  	  	 	if(rxData[0]==1)
	  	  	 	{
	  	  	 		Setpoint1 =	((rxData[1] & 0x00FF) << 8) + (rxData[2] & 0x00FF);
	  	  	 	HAL_Delay(10);
	  	  	 	}
	  	  	 	else if(rxData[0]==2)
	  	  	 	{
	  	  	 		Setpoint2 = ((rxData[1] & 0x00FF) << 8) + (rxData[2] & 0x00FF);
	  	  	 	HAL_Delay(20);
	  	  	 	}
	  	 	 	  if(Setpoint1 > ozone1)
	  			  {
	  	  	 		HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
	  	  	 			   	HAL_UART_Transmit(&huart2, TxData1, 8, 1000);
	  	  	 			   	HAL_Delay(1000);
	  			  }
	  	  	 	  else
	  	  	 	  {
	  	  	 		HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
	  	  	 		HAL_UART_Transmit(&huart2, TxData2, 8, 1000);
	  	  	 		  	 			   	HAL_Delay(1000);
	  	  	 	  }
	  	 	 	if(Setpoint2 > ozone2)
	  	 	 				  {
	  	 	 		  	 		HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
	  	 	 		  	 			   	HAL_UART_Transmit(&huart2, TxData3, 8, 1000);
	  	 	 		  	 			   	HAL_Delay(1000);
	  	 	 				  }
	  	 	 		  	 	  else
	  	 	 		  	 	  {
	  	 	 		  	 		HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
	  	 	 		  	 		HAL_UART_Transmit(&huart2, TxData4, 8, 1000);
	  	 	 		  	 		  	 			   	HAL_Delay(1000);
	  	 	 		  	 	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TX_EN_Pin */
  GPIO_InitStruct.Pin = TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
