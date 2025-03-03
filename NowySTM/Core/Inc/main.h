/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STM_BUTTON_Pin GPIO_PIN_13
#define STM_BUTTON_GPIO_Port GPIOC
#define RGB1_B_Pin GPIO_PIN_2
#define RGB1_B_GPIO_Port GPIOC
#define LD10_Pin GPIO_PIN_4
#define LD10_GPIO_Port GPIOC
#define USER_BUTTON_3_Pin GPIO_PIN_5
#define USER_BUTTON_3_GPIO_Port GPIOC
#define SevSegF_Pin GPIO_PIN_1
#define SevSegF_GPIO_Port GPIOB
#define SevSegA_Pin GPIO_PIN_2
#define SevSegA_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_12
#define D1_GPIO_Port GPIOB
#define SevSegB_Pin GPIO_PIN_13
#define SevSegB_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_14
#define D3_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_15
#define D2_GPIO_Port GPIOB
#define USER_BUTTON_2_Pin GPIO_PIN_6
#define USER_BUTTON_2_GPIO_Port GPIOC
#define USER_BUTTON_1_Pin GPIO_PIN_8
#define USER_BUTTON_1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_9
#define LD1_GPIO_Port GPIOC
#define SevSegD_Pin GPIO_PIN_8
#define SevSegD_GPIO_Port GPIOA
#define SevSegE_Pin GPIO_PIN_9
#define SevSegE_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_10
#define D4_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOA
#define RESET_BUTTON_Pin GPIO_PIN_12
#define RESET_BUTTON_GPIO_Port GPIOA
#define RESET_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define RGB2_G_Pin GPIO_PIN_13
#define RGB2_G_GPIO_Port GPIOA
#define RGB2_B_Pin GPIO_PIN_14
#define RGB2_B_GPIO_Port GPIOA
#define RGB1_R_Pin GPIO_PIN_15
#define RGB1_R_GPIO_Port GPIOA
#define USER_BUTTON_CONFIRM_Pin GPIO_PIN_10
#define USER_BUTTON_CONFIRM_GPIO_Port GPIOC
#define RGB2_R_Pin GPIO_PIN_12
#define RGB2_R_GPIO_Port GPIOC
#define SevSegC_Pin GPIO_PIN_4
#define SevSegC_GPIO_Port GPIOB
#define SevSegG_Pin GPIO_PIN_5
#define SevSegG_GPIO_Port GPIOB
#define RGB1_G_Pin GPIO_PIN_7
#define RGB1_G_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
