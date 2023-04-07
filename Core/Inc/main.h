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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAN_LED_Pin GPIO_PIN_6
#define CAN_LED_GPIO_Port GPIOF
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOF
#define LEDF8_Pin GPIO_PIN_8
#define LEDF8_GPIO_Port GPIOF
#define LEDF9_Pin GPIO_PIN_9
#define LEDF9_GPIO_Port GPIOF
#define DIO1_Pin GPIO_PIN_6
#define DIO1_GPIO_Port GPIOD
#define CLK1_Pin GPIO_PIN_7
#define CLK1_GPIO_Port GPIOD
#define DIO2_Pin GPIO_PIN_9
#define DIO2_GPIO_Port GPIOG
#define CLK2_Pin GPIO_PIN_10
#define CLK2_GPIO_Port GPIOG
#define TC_Button_Pin GPIO_PIN_11
#define TC_Button_GPIO_Port GPIOG
#define TC_LED_Pin GPIO_PIN_12
#define TC_LED_GPIO_Port GPIOG
#define TV_Button_Pin GPIO_PIN_13
#define TV_Button_GPIO_Port GPIOG
#define TV_LED_Pin GPIO_PIN_14
#define TV_LED_GPIO_Port GPIOG
#define INV_RST_Pin GPIO_PIN_3
#define INV_RST_GPIO_Port GPIOB
#define INV_RST_LED_Pin GPIO_PIN_4
#define INV_RST_LED_GPIO_Port GPIOB
#define RTDS_Pin GPIO_PIN_5
#define RTDS_GPIO_Port GPIOB
#define BUZZ_OUT_Pin GPIO_PIN_6
#define BUZZ_OUT_GPIO_Port GPIOB
#define TSMS_3V3_Pin GPIO_PIN_7
#define TSMS_3V3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
