/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_STATUS_Pin GPIO_PIN_13
#define LED_STATUS_GPIO_Port GPIOC
#define LED_ERROR_Pin GPIO_PIN_14
#define LED_ERROR_GPIO_Port GPIOC
#define LED_ACTIVITY_Pin GPIO_PIN_15
#define LED_ACTIVITY_GPIO_Port GPIOC
#define V_SENSE_Pin GPIO_PIN_0
#define V_SENSE_GPIO_Port GPIOA
#define PWM_LED1_Pin GPIO_PIN_1
#define PWM_LED1_GPIO_Port GPIOA
#define CUR_SENSE_Pin GPIO_PIN_4
#define CUR_SENSE_GPIO_Port GPIOA
#define PWM_LED2_Pin GPIO_PIN_5
#define PWM_LED2_GPIO_Port GPIOA
#define MOTOR_CH1_Pin GPIO_PIN_6
#define MOTOR_CH1_GPIO_Port GPIOA
#define MOTOR_CH2_Pin GPIO_PIN_7
#define MOTOR_CH2_GPIO_Port GPIOA
#define MOTOR_CH3_Pin GPIO_PIN_0
#define MOTOR_CH3_GPIO_Port GPIOB
#define MOTOR_CH4_Pin GPIO_PIN_1
#define MOTOR_CH4_GPIO_Port GPIOB
#define SPI1_INT_Pin GPIO_PIN_2
#define SPI1_INT_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_INT_Pin GPIO_PIN_8
#define SPI2_INT_GPIO_Port GPIOA
#define FSYNC_Pin GPIO_PIN_9
#define FSYNC_GPIO_Port GPIOA
#define XSHUT_Pin GPIO_PIN_10
#define XSHUT_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
