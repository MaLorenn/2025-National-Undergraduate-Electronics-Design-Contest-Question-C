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
#include "stm32g4xx_hal.h"

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
#define key1_Pin GPIO_PIN_12
#define key1_GPIO_Port GPIOB
#define key2_Pin GPIO_PIN_13
#define key2_GPIO_Port GPIOB
#define key3_Pin GPIO_PIN_14
#define key3_GPIO_Port GPIOB
#define key4_Pin GPIO_PIN_15
#define key4_GPIO_Port GPIOB
#define en1_Pin GPIO_PIN_9
#define en1_GPIO_Port GPIOD
#define dir1_Pin GPIO_PIN_10
#define dir1_GPIO_Port GPIOD
#define dir2_Pin GPIO_PIN_12
#define dir2_GPIO_Port GPIOD
#define en2_Pin GPIO_PIN_14
#define en2_GPIO_Port GPIOD
#define TIM8_PUL1_Pin GPIO_PIN_6
#define TIM8_PUL1_GPIO_Port GPIOC
#define TIM3_PUL2_Pin GPIO_PIN_7
#define TIM3_PUL2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
