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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C_PD_SDA_Pin GPIO_PIN_9
#define I2C_PD_SDA_GPIO_Port GPIOB
#define GATE_PWM_Pin GPIO_PIN_7
#define GATE_PWM_GPIO_Port GPIOA
#define GPIO_BTN_OK_Pin GPIO_PIN_2
#define GPIO_BTN_OK_GPIO_Port GPIOB
#define GPIO_BTN_OK_EXTI_IRQn EXTI2_3_IRQn
#define GPIO_BTN_UP_Pin GPIO_PIN_8
#define GPIO_BTN_UP_GPIO_Port GPIOA
#define GPIO_BTN_UP_EXTI_IRQn EXTI4_15_IRQn
#define GPIO_BTN_DOWN_Pin GPIO_PIN_9
#define GPIO_BTN_DOWN_GPIO_Port GPIOA
#define GPIO_BTN_DOWN_EXTI_IRQn EXTI4_15_IRQn
#define I2C_OLED_SCL_Pin GPIO_PIN_11
#define I2C_OLED_SCL_GPIO_Port GPIOA
#define I2C_OLED_SDA_Pin GPIO_PIN_12
#define I2C_OLED_SDA_GPIO_Port GPIOA
#define GPIO_LED_Pin GPIO_PIN_15
#define GPIO_LED_GPIO_Port GPIOA
#define I2C_PD_SCL_Pin GPIO_PIN_8
#define I2C_PD_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
