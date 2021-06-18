/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t sysTicked = 0;

uint8_t buttonState[5] = {0,0,0,0,0}; 	// For tracking of physical button state
uint8_t inputState[5] = {0,0,0,0,0}; 	// For tracking how button press should be handled
uint16_t debounceTim[5] = {0,0,0,0,0}; 	// Debounce timers

uint32_t ref =10;

uint8_t startGame = 0;
uint8_t playerPos[2] = {0,0};
uint32_t playerTimer = 0;
uint8_t batTim = 0;
uint32_t playerFlashTim = 0;
uint8_t goalFlashTim = 0;
uint16_t mazeUARTTim = 0;

uint8_t ballPos[2] = {4,4};
uint16_t ballTim = 0;
uint16_t ballSpeedTim = 0; // How long before decreasing ball speed
uint32_t ballSpeed = 700; // How long ball timer runs
uint8_t ballDir = 1; // 4 1 2 5 0 3, clockwise starting at 1 hours. Ball never travels horisontally
uint8_t ballHits = 0;
uint8_t ballSpeedCnt = 1;
uint8_t tennisUARTTim = 0;
uint8_t inputMode = 0;		// Allows using slider and buttons by detecting which the user is trying to use

uint8_t gameMode = 4;  // 0 to 3 M1 to M4, 4 menu, 5 tennis, 6 mp tennis
uint8_t modeState = 1; // State of mode, 0 - new, 1 - ... - game sub-states

uint8_t curCol = 1;
uint8_t dispDat[8][8] = {
    {0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0}
};

uint8_t clearDisp[8][8] = {
    {0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0}
};

uint8_t menuDisp[8][8] = {
    {1,0,0,0,0,0,0,1},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0},
	{1,0,0,0,0,0,0,1}
};

uint8_t number1[8][8] = {
    {0,0,0,0,0,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,0,0,0,0,0,0}
};

uint8_t number2[8][8] = {
    {0,0,0,0,0,0,0,0},
	{0,0,0,1,1,0,0,0},
	{0,0,1,0,0,1,0,0},
	{0,0,0,0,0,1,0,0},
	{0,0,0,0,1,0,0,0},
	{0,0,0,1,0,0,0,0},
	{0,0,1,1,1,1,0,0},
	{0,0,0,0,0,0,0,0}
};

uint8_t number3[8][8] = {
    {0,0,0,0,0,0,0,0},
	{0,0,1,1,0,0,0,0},
	{0,0,0,0,1,0,0,0},
	{0,0,1,1,0,0,0,0},
	{0,0,0,0,1,0,0,0},
	{0,0,0,0,1,0,0,0},
	{0,0,1,1,0,0,0,0},
	{0,0,0,0,0,0,0,0}
};

uint8_t number4[8][8] = {
    {0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,1,0},
	{0,0,0,0,0,1,0,0},
	{0,0,0,0,1,0,0,0},
	{0,0,0,1,0,1,0,0},
	{0,0,1,1,1,1,1,0},
	{0,0,0,0,0,1,0,0},
	{0,0,0,0,0,0,0,0}
};

uint8_t maze1[8][8] = {
    {0,0,0,0,0,0,1,0},
	{1,1,0,1,1,0,0,0},
	{0,0,0,0,0,0,1,0},
	{0,1,1,1,1,1,0,1},
	{0,0,0,0,1,0,0,0},
	{1,1,0,1,1,0,1,0},
	{0,0,0,1,0,0,1,0},
	{0,1,0,0,0,1,1,2}
};

uint8_t goalPos[4][2] = {
		{7,7},
		{3,3},
		{7,7},
		{0,2},

};

uint8_t goalIndex = 1;

uint8_t maze2[8][8] = {
	    {0,0,0,0,0,0,0,0},
		{0,1,0,1,0,1,1,0},
		{0,0,1,0,1,0,0,0},
		{0,1,0,2,0,0,1,1},
		{0,0,1,1,1,1,0,0},
		{1,0,1,0,0,1,1,0},
		{0,0,1,0,0,0,1,0},
		{0,0,0,0,1,0,0,0}
	};

uint8_t maze3[8][8] = {
    {0,0,0,1,1,0,0,0},
	{0,1,0,0,1,0,1,0},
	{0,0,1,0,1,0,1,0},
	{1,0,1,0,0,0,1,0},
	{0,0,1,0,1,1,1,0},
	{0,1,0,0,1,0,0,0},
	{0,1,0,1,1,0,1,1},
	{0,0,0,0,1,0,0,2}
};

uint8_t maze4[8][8] = {
    {0,1,2,1,0,0,0,0},
	{0,1,0,0,0,1,1,0},
	{0,1,1,1,1,0,0,0},
	{0,1,0,0,1,0,1,0},
	{0,1,0,1,0,0,1,0},
	{0,0,0,0,0,1,0,0},
	{1,1,1,1,1,1,0,1},
	{0,0,0,0,0,0,0,0}
};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */

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
	void displayTest();
	void inputHandler();
	void updateDisp();
	void timeKeeper();
	void updateMovement();
	void updateMovement1();
	void copyMatrix(uint8_t arr[8][8]);
	void renderPaddle();
	void updateBall();
	void mazeSelect();
	void modeTennis();
	void modeMaze();

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t data[] = {'$', '2', '1', '7' ,'8' ,'5', '0', '7', '4', '\n'};

  HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);


  displayTest();

  copyMatrix(menuDisp);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  /* ToDo

	   */
	  /* NOTES FOR REPORT
	   *
	   */



	  if (sysTicked == 1) timeKeeper();
	  updateDisp();
	  inputHandler();



	  if (gameMode == 0) modeMaze();
	  if (gameMode == 1) mazeSelect();
	  if (gameMode == 5) modeTennis();
	  else if (gameMode == 4)
	  {
		  if (inputState[3] == 1)
		  	  {
			  	  inputState[3] = 3;
		  		  gameMode = 1;
		  		  modeState = 0;
		  	  }

		  if (inputState[0] == 1)
			{
			  	  inputState[0] = 3;
				  gameMode = 5;
				  modeState = 0;
			}


		  copyMatrix(menuDisp);
	  }


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_11;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|R3_Pin|R4_Pin|C5_Pin 
                          |C6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, R1_Pin|R2_Pin|C1_Pin|C2_Pin 
                          |C3_Pin|C4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C7_Pin|C8_Pin|R5_Pin|R6_Pin 
                          |R7_Pin|R8_Pin|LED1_Pin|LED2_Pin 
                          |LED3_Pin|LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 R3_Pin R4_Pin C5_Pin 
                           C6_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|R3_Pin|R4_Pin|C5_Pin 
                          |C6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin R2_Pin C1_Pin C2_Pin 
                           C3_Pin C4_Pin */
  GPIO_InitStruct.Pin = R1_Pin|R2_Pin|C1_Pin|C2_Pin 
                          |C3_Pin|C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C7_Pin C8_Pin R5_Pin R6_Pin 
                           R7_Pin R8_Pin LED1_Pin LED2_Pin 
                           LED3_Pin LED5_Pin */
  GPIO_InitStruct.Pin = C7_Pin|C8_Pin|R5_Pin|R6_Pin 
                          |R7_Pin|R8_Pin|LED1_Pin|LED2_Pin 
                          |LED3_Pin|LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_Pin Cener_Pin */
  GPIO_InitStruct.Pin = Right_Pin|Cener_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Up_Pin Down_Pin Left_Pin */
  GPIO_InitStruct.Pin = Up_Pin|Down_Pin|Left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM3_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void updateDisp()
{

	if (ref <= 0)
	{
		ref = 3;

		if (curCol == 1) 		// FIRST SELECT COLUMN
		{
			HAL_GPIO_WritePin(GPIOB, C8_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, C1_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 2)
		{
			HAL_GPIO_WritePin(GPIOA, C1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, C2_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 3)
		{
			HAL_GPIO_WritePin(GPIOA, C2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, C3_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 4)
		{
			HAL_GPIO_WritePin(GPIOA, C3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, C4_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 5) 		// FIRST SELECT COLUMN
		{
			HAL_GPIO_WritePin(GPIOA, C4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, C5_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 6)
		{
			HAL_GPIO_WritePin(GPIOC, C5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, C6_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 7)
		{
			HAL_GPIO_WritePin(GPIOC, C6_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, C7_Pin, GPIO_PIN_SET);
		}
		else if (curCol == 8)
		{
			HAL_GPIO_WritePin(GPIOB, C7_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, C8_Pin, GPIO_PIN_SET);
		}



		//					// TOGGLE LEDS IN COL
		if (dispDat[0][curCol - 1] == 1) {HAL_GPIO_WritePin(GPIOA, R1_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOA, R1_Pin, GPIO_PIN_RESET);}

		if (dispDat[1][curCol - 1]== 1) {HAL_GPIO_WritePin(GPIOA, R2_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOA, R2_Pin, GPIO_PIN_RESET);}

		if (dispDat[2][curCol - 1]== 1) {HAL_GPIO_WritePin(GPIOC, R3_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOC, R3_Pin, GPIO_PIN_RESET);}

		if (dispDat[3][curCol - 1]== 1) {HAL_GPIO_WritePin(GPIOC, R4_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOC, R4_Pin, GPIO_PIN_RESET);}

		if (dispDat[4][curCol - 1] == 1) {HAL_GPIO_WritePin(GPIOB, R5_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOB, R5_Pin, GPIO_PIN_RESET);}

		if (dispDat[5][curCol - 1]== 1) {HAL_GPIO_WritePin(GPIOB, R6_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOB, R6_Pin, GPIO_PIN_RESET);}

		if (dispDat[6][curCol - 1]== 1) {HAL_GPIO_WritePin(GPIOB, R7_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOB, R7_Pin, GPIO_PIN_RESET);}

		if (dispDat[7][curCol - 1]== 1) {HAL_GPIO_WritePin(GPIOB, R8_Pin, GPIO_PIN_SET);} else {HAL_GPIO_WritePin(GPIOB, R8_Pin, GPIO_PIN_RESET);}


		if (curCol < 8) curCol++; 	else curCol = 1;
	}
}

void timeKeeper()
{

	sysTicked = 0;
	if (playerFlashTim > 0) playerFlashTim--;
	for (uint8_t i = 0; i<5; i++)
	{
		if (debounceTim[i] > 0) debounceTim[i]--;
	}

	if (playerTimer > 0) playerTimer--;

	if (ref > 0) ref--;

	if (ballTim > 0) ballTim --;
	if (batTim > 0) batTim --;
	if (ballSpeedTim > 0) ballSpeedTim--;
	if (goalFlashTim > 0) goalFlashTim--;
	if (mazeUARTTim > 0) mazeUARTTim--;
	if (tennisUARTTim > 0) tennisUARTTim--;
	/*
	 * uint8_t playerFlashTim = 0;
		uint8_t goalFlashTim = 0;
	 *
	 */

}

void renderPaddle()
{
	dispDat[playerPos[0]][playerPos[1]+1] = 1;
}

void updateMovement()
{
	if (playerTimer == 0)
	{
		uint8_t oldPlayerPos[2] = {playerPos[0], playerPos[1]};

		if (inputState[2] == 1)				// Allows button holds
			{
				if (playerPos[0] < 7) playerPos[0] ++;
				playerTimer = 300;
			}

		else if (inputState[1] == 1)
			{
				if (playerPos[0] > 0) playerPos[0] --;
				playerTimer = 300;
			}

		else if (inputState[3] == 1)
			{
				if (playerPos[1] > 0) playerPos[1] --;
				playerTimer = 300;
			}

		else if (inputState[4] == 1)
			{
				if (playerPos[1] < 7) playerPos[1] ++;
				playerTimer = 300;
			}

		if ((dispDat[playerPos[0]][playerPos[1]] == 1) && !((playerPos[0] == goalPos[goalIndex][0])&&(playerPos[1] == goalPos[goalIndex][1])))  { playerPos[0] = oldPlayerPos[0]; playerPos[1] = oldPlayerPos[1];}
		else {
			if (dispDat[oldPlayerPos[0]][oldPlayerPos[1]] == 1 ) {
				dispDat[oldPlayerPos[0]][oldPlayerPos[1]] = 0;
				dispDat[playerPos[0]][playerPos[1]] = 1;
			}
		}


	}
}

void updateMovement1()
{		//			NEEDS UPDATING - Only sample ADC when ADC so that buttons can be used as well.
		//			THOUGH ADC NOT USED IN FINAL ITERATION...
	if (batTim == 0)
	{
		dispDat[playerPos[0]][playerPos[1]] = 0;
		dispDat[playerPos[0]+1][playerPos[1]] = 0;



		if (inputState[2] == 1)				// Allows button holds
					{
						if (playerPos[0] < 7) playerPos[0] ++;
						batTim = 100;
					}

				else if (inputState[1] == 1)
					{
						if (playerPos[0] > 0) playerPos[0] --;
						batTim = 100;
					}

				else if (inputState[3] == 1)
					{
						if (playerPos[1] > 0) playerPos[1] --;
						batTim = 100;
					}

				else if (inputState[4] == 1)
					{
						if (playerPos[1] < 7) playerPos[1] ++;
						batTim = 100;
					}
		/*

			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			uint16_t raw = HAL_ADC_GetValue(&hadc1);

			if (raw < 800){
				if (playerPos[0] > 0) playerPos[0]--;
				else if (playerPos[0] < 0) playerPos[0]++;
			}
			else if (raw < 1400)
				{
					if (playerPos[0] > 1) playerPos[0]--;
					else if (playerPos[0] < 1) playerPos[0]++;
				}
			else if (raw < 2000)
			{
				if (playerPos[0] > 2) playerPos[0]--;
				else if (playerPos[0] < 2) playerPos[0]++;
			}
			else if (raw < 2600)
			{
				if (playerPos[0] > 3) playerPos[0]--;
				else if (playerPos[0] < 3) playerPos[0]++;
			}
			else if (raw < 3200)
			{
				if (playerPos[0] > 4) playerPos[0]--;
				else if (playerPos[0] < 4) playerPos[0]++;
			}
			else if (raw < 3800)
			{
				if (playerPos[0] > 5) playerPos[0]--;
				else if (playerPos[0] < 5) playerPos[0]++;
			}
			else  			{
				if (playerPos[0] > 5) playerPos[0]--;
				else if (playerPos[0] < 5) playerPos[0]++;
			}
		 */

		dispDat[playerPos[0]][playerPos[1]] = 1;
		dispDat[playerPos[0]+1][playerPos[1]] = 1;

	}
}


void updateBall()
{

	if (ballTim <= 0)
	{
		dispDat[ballPos[0]][ballPos[1]] = 0;
		ballTim = ballSpeed;
		uint8_t recalc = 1;
		uint8_t ballHit = 0;

		while (recalc == 1)
		{

			if (ballDir == 4)
			{
				if (ballPos[0] == 0)
					{
						ballDir = 2;
					}
				else if (ballPos[1] == 7)
					{
						ballDir = 3;
					}
				else
					{
						recalc = 0;
						ballPos[0]--;
						ballPos[1]++;
					}
			}
			else if (ballDir == 1)
			{
				if (ballPos[1] == 7)
					{
						ballDir = 0;
					}
				else
				{
					recalc = 0;
					ballPos[1]++;
				}
			}

			else if (ballDir == 0)
			{
				if (ballPos[1] == 0){
					recalc = 0;// End game
					modeState = 2;
				}

				else if (ballPos[1] == playerPos[1]+1)
				{
					if (ballPos[0] == playerPos[0]){
						ballDir = 4;
						ballHit = 1;
					}
					else if (ballPos[0] == (playerPos[0]+1)){
						ballDir = 2;
						ballHit = 1;
				} else {
					recalc = 0;
					ballPos[1]--;
				}

				}
				else {
					recalc = 0;
					ballPos[1]--;
				}
			}

			else if (ballDir == 2)
			{
				if (ballPos[1] == 7)
					{
						ballDir = 5;
					}
				else if (ballPos[0] == 7)
					{
						ballDir = 4;
					}
				else
				{
					recalc = 0;
					ballPos[0]++;
					ballPos[1]++;
				}
			}

			else if (ballDir == 3)
			{
				if (ballPos[1] == 0){
					recalc = 0;// End game

					modeState = 2;
				}
				else if (ballPos[0] == 0){
					ballDir = 5;
				}

				else if (ballPos[1] == playerPos[1]+1)
				{
					if (ballPos[0] == playerPos[0]){
						ballDir = 4;
						ballHit = 1;
					}
					else if (ballPos[0] == (playerPos[0]+1)){
						ballDir = 4;
						ballHit = 1;
					}
					else if (ballPos[0] == (playerPos[0]+2)){
						ballDir = 2;
						ballHit = 1;
					} else {
						recalc = 0;

						ballPos[0]--;
						ballPos[1]--;
					}

				}
				else {
					recalc = 0;
					ballPos[0]--;
					ballPos[1]--;
				}
			}

			else if (ballDir == 5)
			{
				if (ballPos[1] == 0){
					recalc = 0;// End game

					modeState = 2;

				}
				else if (ballPos[0] == 7){
					ballDir = 3;
				}

				else if (ballPos[1] == playerPos[1]+1)
				{
					if (ballPos[0] == playerPos[0]){
						ballHit = 1;
						ballDir = 2;
					}
					else if (ballPos[0] == (playerPos[0]+1)){
						ballDir = 2;
						ballHit = 1;
					}
					else if (ballPos[0] == (playerPos[0]-1)){
						ballDir = 4;
						ballHit = 1;
					}else {
						recalc = 0;
						ballPos[0]++;
						ballPos[1]--;
					}

				}
				else {
					recalc = 0;
					ballPos[0]++;
					ballPos[1]--;
				}
			}

		}

		if (ballHit == 1) ballHits++;

			dispDat[ballPos[0]][ballPos[1]] = 1;

	}



}



void inputHandler()
{
	// Track physical states
	buttonState[0] = !HAL_GPIO_ReadPin(GPIOB,Cener_Pin);
	buttonState[1] = !HAL_GPIO_ReadPin(GPIOA,Up_Pin);
	buttonState[2] = !HAL_GPIO_ReadPin(GPIOA,Down_Pin);
	buttonState[3] = !HAL_GPIO_ReadPin(GPIOA,Left_Pin);
	buttonState[4] = !HAL_GPIO_ReadPin(GPIOB,Right_Pin);

	for (uint8_t i = 0; i<5; i++)
	{

		if (inputState[i] == 0)				// Allow input to be set if they are in dormant state
		{
			inputState[i]  = buttonState[i];
		}
		else if ((buttonState[i] == 0) && (debounceTim[i] == 0) && (inputState[i] != 2))		// debounce if not already debounced, button released and input not dormant
		{
			inputState[i] = 2;
			debounceTim[i] = 10;
		}
		else if ((inputState[i] == 2) && (debounceTim[i] == 0))									// reset after debouncing period
		{
			inputState[i] = 0;
			debounceTim[i] = 0;
		}

		// WHAT HAVE I DONE?
		/*
		 * Buttons can be set
		 * Input state may be modified to a value not used here by external functions (usefull for button holds)
		 * Debouncing independent of external manipulation
		 */

	}





}

void copyMatrix(uint8_t arr[8][8])
{
	for (uint8_t i = 0; i < 8; i++)
	{
		for (uint8_t j = 0; j < 8; j++)
			{
				dispDat[i][j] = arr[i][j];
			}
	}

}

void mazeSelect()
{
	HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);

		if (inputState[1] == 1)
			{
				if (modeState < 3) modeState++;
				else modeState = 0;
				inputState[1] = 3;
			}

			if (inputState[2] == 1)
			{
				if (modeState > 0) modeState--;
				else modeState = 3;
				inputState[2] = 3;
			}

		if (modeState == 0) {
			copyMatrix(number1);
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
		}
		else if (modeState == 1)
		{
			copyMatrix(number2);
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);

		}
		else if (modeState == 2)
		{
			copyMatrix(number3);
			HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
		}
		else if (modeState == 3)
		{
			copyMatrix(number4);
			HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_SET);
		}


		if (inputState[0] == 1)
					{
			if (modeState == 0) {
						copyMatrix(maze1);
						goalIndex = 0;

					}
					else if (modeState == 1)
					{
						copyMatrix(maze2);
						goalIndex = 1;
					}
					else if (modeState == 2)
					{
						copyMatrix(maze3);
						goalIndex = 2;
					}
					else if (modeState == 3)
					{
						copyMatrix(maze4);
						goalIndex = 3;
					}

						gameMode = 0;
						modeState = 0;
						inputState[0] = 3;
					}




}

void modeMaze()
{
	if (modeState == 0)
	{

		playerPos[0] = 0; playerPos[1] = 0;
		modeState = 1;
	}

	if (modeState == 1)
	{

		char c3 = '0';
		char c4 = '0';
		updateMovement();
		if (playerFlashTim == 0)
		{
			playerFlashTim = 300;
			if (dispDat[playerPos[0]][playerPos[1]] != 0){
					dispDat[playerPos[0]][playerPos[1]] = 0;
					c3 = '0';

				}
			else if (dispDat[playerPos[0]][playerPos[1]] != 1) {
				dispDat[playerPos[0]][playerPos[1]] = 1;
				c3 = '1';
			}
		}

		if (goalFlashTim == 0)
				{
					goalFlashTim = 100;
					if (dispDat[goalPos[goalIndex][0]][goalPos[goalIndex][1]] != 0)
						{
						dispDat[goalPos[goalIndex][0]][goalPos[goalIndex][1]] = 0;
						c4 = '0';
						}
					else if (dispDat[goalPos[goalIndex][0]][goalPos[goalIndex][1]] != 1) {
						dispDat[goalPos[goalIndex][0]][goalPos[goalIndex][1]] = 1;
						c4 = '1';
					}
				}

		if (mazeUARTTim == 0)
		{
			mazeUARTTim = 300;
			char c1 = playerPos[1]+'0';
			char c2 = playerPos[0]+'0';

			char c5 = '1';				//would be used for IMU

			uint8_t data[] = {'$' , '3',c1,c2,c3,c4,c5,'_','_','\n'};

			HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);
		}

		if ((playerPos[0] == goalPos[goalIndex][0])&&(playerPos[1] == goalPos[goalIndex][1]))
		{
			modeState = 1;
			// AN UNFORSEEN PROBLEM - IF ENDS ON LEFT PRESS, MAKE SURE IT IS NOT HOLDABLE
			inputState[3] = 3;
			gameMode = 4;
			HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
			    HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
		}

		if (inputState[0] == 1)
		{
				inputState[0] = 3;

				gameMode = 4;
				HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
				    HAL_GPIO_WritePin(GPIOB, LED5_Pin, GPIO_PIN_RESET);
		}
	}




}

void modeTennis()
	{
		/*
		 * Timers
		 * display
		 * user input
		 * playermovement
		 * ballmovement
		 */

		if (modeState == 0)
		{
			ballDir = 0;
			ballSpeed = 700;
			ballHits = 0;
			copyMatrix(clearDisp);
			playerPos[0] = 4; playerPos[1] = 0;
			ballPos[0] = 4; ballPos[1] = 7;
			modeState = 1;
			ballSpeedCnt = 1;
		}


		if (modeState == 1)
		{


			if ((ballHits == 3) && ( ballSpeedCnt < 9)) {
				ballHits = 0;
				ballSpeed -= 50;
				ballSpeedCnt ++;
				ballTim = ballSpeed;
			}

			updateBall();

			updateMovement1();

			if (tennisUARTTim == 0)
					{
						tennisUARTTim = 100;
						char c1 = ballPos[1]+'0';
						char c2 = ballPos[0]+'0';
						char c3 = playerPos[1]+'0';
						char c4 = playerPos[0]+'0';
						char c5 = ballSpeedCnt + '0';
						char c6 = ballDir + '0';
						char c7 = '1'; // for IMU

						uint8_t data[] = {'$' , '2',c1,c2,c5,c6,c3,c4,c7,'\n'};

						HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);
					}

		}

		if (modeState == 2)
		{
			modeState = 1;
			gameMode = 4;
		}

		if (inputState[0] == 1)
				{
					inputState[0] = 3;
					modeState = 1;
					gameMode = 4;

				}




	}


void displayTest()
{
	uint32_t timerA = 0;

	while (timerA < 8002)

	{
		if (sysTicked == 1)
			{
				timerA++;
				sysTicked = 0;
				timeKeeper();
				updateDisp();

				if (timerA == 1)
				{
					uint8_t data[] = {'$' , '1','0','_','_','_','_','_','_','\n'};

					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);

					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][0] = 1;
					}


				}

				else if (timerA == 1001)
				{
					uint8_t data[] = {'$' , '1','1','_','_','_','_','_','_','\n'};
					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);
					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][1] = 1;
					}


				}
				else if (timerA == 2001)
				{
					uint8_t data[] = {'$' , '1','2','_','_','_','_','_','_','\n'};
					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);


					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][2] = 1;
					}

				}
				else if (timerA == 3001)
				{
					uint8_t data[] = {'$' , '1','3','_','_','_','_','_','_','\n'};
					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);

					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][3] = 1;
					}

				}
				else if (timerA == 4001)
				{
					uint8_t data[] = {'$' , '1','4','_','_','_','_','_','_','\n'};
					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);

					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][4] = 1;
					}

				}
				else if (timerA == 5001)
				{
					uint8_t data[] = {'$' , '1','5','_','_','_','_','_','_','\n'};
										HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);


										copyMatrix(clearDisp);
										for (uint8_t i = 0; i <= 7; i++)
										{
											dispDat[i][5] = 1;
										}
				}
				else if (timerA == 6001)
				{
					uint8_t data[] = {'$' , '1','6','_','_','_','_','_','_','\n'};
					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);

					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][6]= 1;
					}

				}
				else if (timerA == 7001)
				{
					uint8_t data[] = {'$' , '1','7','_','_','_','_','_','_','\n'};
					HAL_UART_Transmit(&huart2, data, sizeof(data) , 100);

					copyMatrix(clearDisp);
					for (uint8_t i = 0; i <= 7; i++)
					{
						dispDat[i][7] = 1;
					}

				}
				else if (timerA == 8001)
				{
					copyMatrix(clearDisp);
					dispDat[0][0] = 1;dispDat[0][7] = 1;dispDat[7][0] = 1;dispDat[7][7] = 1;

				}

			}

	}







}

  /* @brief  This function is executed in case of error occurrence.
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
