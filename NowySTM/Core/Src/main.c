/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t counter = 60;
uint8_t state = 0;
uint8_t pumpPower = 50;
uint8_t interruptFlag = 0;
uint8_t program = 0;
uint8_t progLength [3]= {10, 20, 50};
uint8_t progPWM[3][2] = {
		{50, 0},  // program 1
		{0, 50},  // program 2
		{80, 0},  // program 3
};
uint8_t temp1, temp2, temp3, temp4;
uint8_t segmentNumber[10][7] = {
		{1, 0, 0, 0, 0, 0, 0},  // 0
		{1, 1, 1, 1, 0, 0, 1},  // 1
		{0, 1, 0, 0, 1, 0, 0},  // 2
		{0, 1, 0, 1, 0, 0, 0},  // 3
		{0, 0, 1, 1, 0, 0, 1},  // 4
		{0, 0, 0, 1, 0, 1, 0},  // 5
		{0, 0, 0, 0, 0, 1, 0},  // 6
		{1, 1, 1, 1, 0, 0, 0},  // 7
		{0, 0, 0, 0, 0, 0, 0},  // 8
		{0, 0, 0, 1, 0, 0, 0}   // 9
};

void SevenSegment_Update(uint8_t number){
	HAL_GPIO_WritePin(SevSegA_GPIO_Port, SevSegA_Pin, segmentNumber[number][6]);
	HAL_GPIO_WritePin(SevSegB_GPIO_Port, SevSegB_Pin, segmentNumber[number][5]);
	HAL_GPIO_WritePin(SevSegC_GPIO_Port, SevSegC_Pin, segmentNumber[number][4]);
	HAL_GPIO_WritePin(SevSegD_GPIO_Port, SevSegD_Pin, segmentNumber[number][3]);
	HAL_GPIO_WritePin(SevSegE_GPIO_Port, SevSegE_Pin, segmentNumber[number][2]);
	HAL_GPIO_WritePin(SevSegF_GPIO_Port, SevSegF_Pin, segmentNumber[number][1]);
	HAL_GPIO_WritePin(SevSegG_GPIO_Port, SevSegG_Pin, segmentNumber[number][0]);
}

void ProgramEnd(){
	HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 1);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
    HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 0);
    HAL_Delay(50);
    state = 0;
    program = 0;
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
}


void First_program(){
	if ((interruptFlag == 1) || (counter <= 0)){
	    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
	    HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	    ProgramEnd();
	}
}

void Second_program(){
    if ((interruptFlag == 1) || (counter <= 0)){
	    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	    HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	    ProgramEnd();
    }
    else {
    	temp1 = counter/1000;
    	temp2 = ((counter/100)%10);
    	temp3 = ((counter/10)%10);
    	temp4 = (counter%10);
    	SevenSegment_Update(temp2);
    	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
    	HAL_Delay(5);
    	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);

    	SevenSegment_Update(temp3);
    	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
    	HAL_Delay(5);
    	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);

    	SevenSegment_Update(temp4);
    	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
    	HAL_Delay(5);
    	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);}
}

void Third_program(){
	if ((interruptFlag == 1) || (counter <= 0)){
	    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	    HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	    ProgramEnd();
	}
	else if (counter == 30)
	{
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	}
	else if (counter == 20)
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3){
			counter--;}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   	  // przycisk do programu 1
   	  if ((HAL_GPIO_ReadPin(USER_BUTTON_1_GPIO_Port, USER_BUTTON_1_Pin) == GPIO_PIN_RESET)&&(state == 0)) {
   		  counter = progLength[0];
   		  program = 1;
   		  state = 1;
   		  interruptFlag = 0;
   		  HAL_TIM_Base_Start_IT(&htim3);
   		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
   		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
   		  HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 0);
   		  TIM2->CCR1 = progPWM[0][0];
   		  TIM2->CCR2 = progPWM[0][1];
   		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   	  }
   	  // przycisk do programu 2
   	   if ((HAL_GPIO_ReadPin(USER_BUTTON_2_GPIO_Port, USER_BUTTON_2_Pin) == GPIO_PIN_RESET)&&(state == 0)) {
   		  counter = progLength[1];
   		  program = 2;
   		  state = 1;
   		  interruptFlag = 0;
   		  HAL_TIM_Base_Start_IT(&htim3);
   		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
   		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
   		  HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 0);
   		  TIM2->CCR1 = progPWM[1][0];
   		  TIM2->CCR2 = progPWM[1][1];
   		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   	  }
   	  // przycisk do programu 3
   	  if ((HAL_GPIO_ReadPin(USER_BUTTON_3_GPIO_Port, USER_BUTTON_3_Pin) == GPIO_PIN_RESET)&&(state == 0)) {
   		  counter = progLength[2];
   		  program = 3;
   		  state = 1;
   		  interruptFlag = 0;
   		  HAL_TIM_Base_Start_IT(&htim3);
   		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
   		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
   		  HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, 0);
   		  TIM2->CCR1 = progPWM[2][0];
   		  TIM2->CCR2 = progPWM[2][1];
   		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   	  }

   	  	/*const char message[] = "Hello world!\r\n";
   	    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);*/
   	  switch (program){
   	  case 1: First_program(); 		//egzekucja pierwszego programu
   	  break;
   	  case 2: Second_program();		//egzekucja drugiego programu
   	  break;
   	  case 3: Third_program();		//egzekucja trzeciego programu
   	  break;
   	  default:
   		  HAL_Delay(50);
   	    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|SevSegD_Pin|SevSegE_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD10_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SevSegF_Pin|SevSegA_Pin|SevSegC_Pin|D1_Pin
                          |SevSegB_Pin|D3_Pin|D2_Pin|SevSegG_Pin
                          |D4_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STM_BUTTON_Pin USER_BUTTON_3_Pin USER_BUTTON_2_Pin USER_BUTTON_1_Pin */
  GPIO_InitStruct.Pin = STM_BUTTON_Pin|USER_BUTTON_3_Pin|USER_BUTTON_2_Pin|USER_BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 SevSegD_Pin SevSegE_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|SevSegD_Pin|SevSegE_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD10_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD10_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SevSegF_Pin SevSegA_Pin SevSegC_Pin D1_Pin
                           SevSegB_Pin D3_Pin D2_Pin SevSegG_Pin
                           D4_Pin LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = SevSegF_Pin|SevSegA_Pin|SevSegC_Pin|D1_Pin
                          |SevSegB_Pin|D3_Pin|D2_Pin|SevSegG_Pin
                          |D4_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_BUTTON_Pin */
  GPIO_InitStruct.Pin = RESET_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RESET_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_12) {
		interruptFlag = 1;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  }
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
