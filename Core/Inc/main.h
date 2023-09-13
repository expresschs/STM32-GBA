/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define KEY_SELECT_Pin GPIO_PIN_5
#define KEY_SELECT_GPIO_Port GPIOI
#define KEY_START_Pin GPIO_PIN_11
#define KEY_START_GPIO_Port GPIOG
#define KEY_B_Pin GPIO_PIN_13
#define KEY_B_GPIO_Port GPIOH
#define LCD_DC_Pin GPIO_PIN_11
#define LCD_DC_GPIO_Port GPIOJ
#define SPI5_CS_Pin GPIO_PIN_5
#define SPI5_CS_GPIO_Port GPIOH
#define KEY_DOWN_Pin GPIO_PIN_10
#define KEY_DOWN_GPIO_Port GPIOH
#define KEY_LEFT_Pin GPIO_PIN_11
#define KEY_LEFT_GPIO_Port GPIOH
#define KEY_RIGHT_Pin GPIO_PIN_9
#define KEY_RIGHT_GPIO_Port GPIOH
#define KEY_UP_Pin GPIO_PIN_12
#define KEY_UP_GPIO_Port GPIOH
#define LCD_BACKLIGHT_Pin GPIO_PIN_6
#define LCD_BACKLIGHT_GPIO_Port GPIOH
#define KEY_A_Pin GPIO_PIN_8
#define KEY_A_GPIO_Port GPIOH
#define SD_SELECT_Pin GPIO_PIN_0
#define SD_SELECT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
