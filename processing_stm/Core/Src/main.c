/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : This subsystem receives the raw samples from the Sampling microcontroller,
  * 				  processes the audio data, and transmits it to a PC for analysis and storage.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdint.h>

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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to flash LED for debugging (adds delay!)
void flash_LED() {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	HAL_Delay(50);
}

/**
 * @brief Measures distance using HC-SR04 ultrasonic sensor.
 * @return Distance in cm (8-bit integer)
 */
uint8_t read_distance_cm() {
	float distance = 100;
	// Start timer
	HAL_TIM_Base_Start(&htim16);

	// Set counter to 0
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, 1);

	// Set Trigger pin high for 10us
	while (__HAL_TIM_GET_COUNTER(&htim16) < 10);

	// Reset to low
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, 0);

	// Wait for Echo pin to go high
	while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 0){
		// stop waiting and assume no object as time is too long
		if (__HAL_TIM_GET_COUNTER(&htim16)>1000){
			return (uint8_t)distance;
		}
	}

	// Reset counter
	__HAL_TIM_SET_COUNTER(&htim16, 0);

	// Wait for Echo to go low
	while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == 1){
		if (__HAL_TIM_GET_COUNTER(&htim16)>1000){
			return (uint8_t)distance;
		}
	}


	// Measure this interval by getting counter value
	uint16_t echoTimeHigh = __HAL_TIM_GET_COUNTER(&htim16);

	// Calculate distance in cm using formula
	distance = echoTimeHigh * 0.01715f;

	return (uint8_t)distance;
}

uint8_t receivedADCValue;  	// To store received 10-bit ADC value
// uint8_t receivedADCValue8bit;	// To store rescaled 8-bit value
uint8_t uart1DataReady = 0;		// Check if new data received over UART1

uint8_t receivedCommand;		// To store command received over UART2
uint8_t uart2DataReady = 0;		// Check if new command received over UART2

uint8_t objectWithinRange = 0;	// Check if object is within range of ultrasonic
uint8_t mode = 0;				// 1 = manual mode, 2 = distance trigger mode

// For moving average filter:
uint8_t lastValue = 0;
uint8_t sendValue = 0;
uint8_t isFirstValue = 1; // Initialise to true for first value

// For ultrasonic sensor:
uint8_t distance = 0;

// Callback function for every time UART value is received
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) { // Data received from sampling STM via UART1
		uart1DataReady = 1;

		// Restart reception for next value
		HAL_UART_Receive_IT(&huart1, &receivedADCValue, sizeof(receivedADCValue));
	}

	else if (huart->Instance == USART2) { // Command received from Python via UART2
		uart2DataReady = 1;

		// Restart reception for next command
		HAL_UART_Receive_IT(&huart2, &receivedCommand, sizeof(receivedCommand));
	}
}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // Start receiving first ADC value and trigger interrupt once 1 byte received
  HAL_UART_Receive_IT(&huart1, &receivedADCValue, sizeof(receivedADCValue));
  // Start receiving first 1-byte command over UART2 and trigger interrupt once received
  HAL_UART_Receive_IT(&huart2, &receivedCommand, sizeof(receivedCommand));

  // Ensure LED is off
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (uart2DataReady == 1) { // If new command has been received over UART2
	  		  if (receivedCommand == 'm') {
	  			  mode = 1;										// Set mode to manual
	  			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);	// Turn on LED
	  			  objectWithinRange = 0;						// Reset object detection
	  		  }
	  		  else if (receivedCommand == 'd') {
	  			  mode = 2;										// Set mode to distance trigger
	  			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);	// Turn off LED
	  			  objectWithinRange = 0;						// Reset object detection
	  		  }

	  		  // Reset flag
	  		  uart2DataReady = 0;
	  	  }

	  if (uart1DataReady == 1) {	// If new data has been received over UART1
		  if ((mode == 1) || objectWithinRange) {
			  // Transmit data to PC only if in manual mode, or if in distance mode with object in range

			  // Rescale from 10 bits to 8 bits
			  // receivedADCValue8bit = (receivedADCValue * 255 - 511) / 1023;

			  // Moving average filter:
			  if (isFirstValue) {
				  // Send first value as is and reset flag
				  sendValue = receivedADCValue;
				  isFirstValue = 0;
			  } else {
				  // Get the average of the previous and current value (casting to uint16_t to avoid overflow)
				  sendValue = ((uint16_t)receivedADCValue + (uint16_t)lastValue) / 2;
			  }

			  // Transmit value to PC
			  HAL_UART_Transmit(&huart2, &sendValue, sizeof(sendValue), HAL_MAX_DELAY);

			  // Store the last value
			  lastValue = receivedADCValue;
		  }

		  // Reset flag
		  uart1DataReady = 0;
	  }


	  if (mode == 2) {// Get data from ultrasonic if in distance trigger mode
		  // Read distance
		  distance = read_distance_cm();

		  if (distance < 10) { // object detected within 10cm
			  objectWithinRange = 1;
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
		  }
		  else {
			  // reset object detection
			  objectWithinRange = 0;
			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
		  }

		  // 60ms delay between ultrasonic readings
		  HAL_Delay(60);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Trigger_Pin */
	GPIO_InitStruct.Pin = Trigger_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Echo_Pin */
	GPIO_InitStruct.Pin = Echo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
