/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define dir_step3_Pin GPIO_PIN_2
#define dir_step3_GPIO_Port GPIOA
#define pulse_step3_Pin GPIO_PIN_3
#define pulse_step3_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_4
#define SS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_6
#define MOSI_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_7
#define MISO_GPIO_Port GPIOA
#define XL5_Pin GPIO_PIN_0
#define XL5_GPIO_Port GPIOB
#define XL6_Pin GPIO_PIN_1
#define XL6_GPIO_Port GPIOB
#define XL7_Pin GPIO_PIN_10
#define XL7_GPIO_Port GPIOB
#define XL8_Pin GPIO_PIN_11
#define XL8_GPIO_Port GPIOB
#define pulse_dc1_Pin GPIO_PIN_12
#define pulse_dc1_GPIO_Port GPIOB
#define dir_step1_Pin GPIO_PIN_13
#define dir_step1_GPIO_Port GPIOB
#define dir_step2_Pin GPIO_PIN_14
#define dir_step2_GPIO_Port GPIOB
#define pulse_dc2_Pin GPIO_PIN_15
#define pulse_dc2_GPIO_Port GPIOB
#define XL1_Pin GPIO_PIN_8
#define XL1_GPIO_Port GPIOA
#define XL2_Pin GPIO_PIN_9
#define XL2_GPIO_Port GPIOA
#define XL3_Pin GPIO_PIN_10
#define XL3_GPIO_Port GPIOA
#define XL4_Pin GPIO_PIN_11
#define XL4_GPIO_Port GPIOA
#define dir_dc1_Pin GPIO_PIN_6
#define dir_dc1_GPIO_Port GPIOB
#define pulse_step1_Pin GPIO_PIN_7
#define pulse_step1_GPIO_Port GPIOB
#define pulse_step2_Pin GPIO_PIN_8
#define pulse_step2_GPIO_Port GPIOB
#define dir_dc2_Pin GPIO_PIN_9
#define dir_dc2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
