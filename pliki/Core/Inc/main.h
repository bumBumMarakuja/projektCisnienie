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
#include "stm32g0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOB
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define USER_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define SevSegA_Pin GPIO_PIN_4
#define SevSegA_GPIO_Port GPIOB
#define SevSegF_Pin GPIO_PIN_5
#define SevSegF_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_10
#define D3_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_9
#define LD1_GPIO_Port GPIOD
#define USER_BUTTON_4_Pin GPIO_PIN_0
#define USER_BUTTON_4_GPIO_Port GPIOC
#define SevSegD_Pin GPIO_PIN_7
#define SevSegD_GPIO_Port GPIOA
#define SevSegC_Pin GPIO_PIN_7
#define SevSegC_GPIO_Port GPIOC
#define SevSegG_Pin GPIO_PIN_9
#define SevSegG_GPIO_Port GPIOA
#define USER_BUTTON_1_Pin GPIO_PIN_8
#define USER_BUTTON_1_GPIO_Port GPIOD
#define SevSegE_Pin GPIO_PIN_6
#define SevSegE_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_14
#define D1_GPIO_Port GPIOB
#define USER_BUTTON_2_Pin GPIO_PIN_15
#define USER_BUTTON_2_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_8
#define D4_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_10
#define LD2_GPIO_Port GPIOB
#define LD10_Pin GPIO_PIN_13
#define LD10_GPIO_Port GPIOB
#define SevSegB_Pin GPIO_PIN_4
#define SevSegB_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOC
#define USER_BUTTON_3_Pin GPIO_PIN_2
#define USER_BUTTON_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
