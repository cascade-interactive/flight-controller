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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_2
#define LED_1_GPIO_Port GPIOE
#define LED_2_Pin GPIO_PIN_3
#define LED_2_GPIO_Port GPIOE
#define LED_3_Pin GPIO_PIN_4
#define LED_3_GPIO_Port GPIOE
#define EXPOSED_E5_Pin GPIO_PIN_5
#define EXPOSED_E5_GPIO_Port GPIOE
#define EXPOSED_E6_Pin GPIO_PIN_6
#define EXPOSED_E6_GPIO_Port GPIOE
#define ESC_C_Pin GPIO_PIN_0
#define ESC_C_GPIO_Port GPIOC
#define M_1_Pin GPIO_PIN_0
#define M_1_GPIO_Port GPIOA
#define M_2_Pin GPIO_PIN_1
#define M_2_GPIO_Port GPIOA
#define M_3_Pin GPIO_PIN_2
#define M_3_GPIO_Port GPIOA
#define M_4_Pin GPIO_PIN_3
#define M_4_GPIO_Port GPIOA
#define BARO_INT_Pin GPIO_PIN_4
#define BARO_INT_GPIO_Port GPIOA
#define ESC_V_Pin GPIO_PIN_4
#define ESC_V_GPIO_Port GPIOC
#define BARO_CS_Pin GPIO_PIN_5
#define BARO_CS_GPIO_Port GPIOC
#define AUX_L3_Pin GPIO_PIN_0
#define AUX_L3_GPIO_Port GPIOB
#define AUX_L4_Pin GPIO_PIN_1
#define AUX_L4_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_2
#define FLASH_CS_GPIO_Port GPIOB
#define ESC_TELEM_Pin GPIO_PIN_7
#define ESC_TELEM_GPIO_Port GPIOE
#define EXPOSED_UART7_TX_Pin GPIO_PIN_8
#define EXPOSED_UART7_TX_GPIO_Port GPIOE
#define M_5_Pin GPIO_PIN_9
#define M_5_GPIO_Port GPIOE
#define M_6_Pin GPIO_PIN_11
#define M_6_GPIO_Port GPIOE
#define M_7_Pin GPIO_PIN_13
#define M_7_GPIO_Port GPIOE
#define M_8_Pin GPIO_PIN_14
#define M_8_GPIO_Port GPIOE
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_8
#define IMU_INT_GPIO_Port GPIOD
#define IMU_FSYNC_Pin GPIO_PIN_9
#define IMU_FSYNC_GPIO_Port GPIOD
#define AUX_L1_Pin GPIO_PIN_12
#define AUX_L1_GPIO_Port GPIOD
#define AUX_L2_Pin GPIO_PIN_13
#define AUX_L2_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOC
#define SDMMC1_DETECT_Pin GPIO_PIN_8
#define SDMMC1_DETECT_GPIO_Port GPIOA
#define COMPANION_CTS_Pin GPIO_PIN_3
#define COMPANION_CTS_GPIO_Port GPIOD
#define COMPANION_RTS_Pin GPIO_PIN_4
#define COMPANION_RTS_GPIO_Port GPIOD
#define COMPANION_TX_Pin GPIO_PIN_5
#define COMPANION_TX_GPIO_Port GPIOD
#define COMPANION_RX_Pin GPIO_PIN_6
#define COMPANION_RX_GPIO_Port GPIOD
#define SYNC_Pin GPIO_PIN_7
#define SYNC_GPIO_Port GPIOD
#define AUX_S1_Pin GPIO_PIN_4
#define AUX_S1_GPIO_Port GPIOB
#define AUX_S2_Pin GPIO_PIN_5
#define AUX_S2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
