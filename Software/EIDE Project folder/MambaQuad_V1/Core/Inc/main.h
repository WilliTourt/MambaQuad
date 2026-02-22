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
#define S1_Pin GPIO_PIN_13
#define S1_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_14
#define S2_GPIO_Port GPIOC
#define LED_SENS_Pin GPIO_PIN_15
#define LED_SENS_GPIO_Port GPIOC
#define LED_ERR_Pin GPIO_PIN_0
#define LED_ERR_GPIO_Port GPIOC
#define VL53L8CX_LPn_Pin GPIO_PIN_1
#define VL53L8CX_LPn_GPIO_Port GPIOC
#define GPS_1PPS_Pin GPIO_PIN_2
#define GPS_1PPS_GPIO_Port GPIOC
#define LED_LR_Pin GPIO_PIN_3
#define LED_LR_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_0
#define GPS_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_1
#define GPS_TX_GPIO_Port GPIOA
#define LoRa_RX_Pin GPIO_PIN_2
#define LoRa_RX_GPIO_Port GPIOA
#define LoRa_TX_Pin GPIO_PIN_3
#define LoRa_TX_GPIO_Port GPIOA
#define LoRa_RST_Pin GPIO_PIN_4
#define LoRa_RST_GPIO_Port GPIOA
#define ICM42688P_SCK_Pin GPIO_PIN_5
#define ICM42688P_SCK_GPIO_Port GPIOA
#define ICM42688P_MISO_Pin GPIO_PIN_6
#define ICM42688P_MISO_GPIO_Port GPIOA
#define ICM42688P_MOSI_Pin GPIO_PIN_7
#define ICM42688P_MOSI_GPIO_Port GPIOA
#define ICM42688P_CS_Pin GPIO_PIN_4
#define ICM42688P_CS_GPIO_Port GPIOC
#define ICM42688P_INT_Pin GPIO_PIN_5
#define ICM42688P_INT_GPIO_Port GPIOC
#define ICM42688P_INT_EXTI_IRQn EXTI9_5_IRQn
#define CURRENT_Pin GPIO_PIN_0
#define CURRENT_GPIO_Port GPIOB
#define BUZ_Pin GPIO_PIN_1
#define BUZ_GPIO_Port GPIOB
#define ICP_VL53_SCL_Pin GPIO_PIN_10
#define ICP_VL53_SCL_GPIO_Port GPIOB
#define ICP_VL53_SDA_Pin GPIO_PIN_11
#define ICP_VL53_SDA_GPIO_Port GPIOB
#define LED_RR_Pin GPIO_PIN_12
#define LED_RR_GPIO_Port GPIOB
#define DShot_M1_Pin GPIO_PIN_6
#define DShot_M1_GPIO_Port GPIOC
#define DShot_M2_Pin GPIO_PIN_7
#define DShot_M2_GPIO_Port GPIOC
#define DShot_M3_Pin GPIO_PIN_8
#define DShot_M3_GPIO_Port GPIOC
#define DShot_M4_Pin GPIO_PIN_9
#define DShot_M4_GPIO_Port GPIOC
#define LoRa_STATUS_Pin GPIO_PIN_8
#define LoRa_STATUS_GPIO_Port GPIOA
#define LoRa_STATUS_EXTI_IRQn EXTI9_5_IRQn
#define USB_DEBUG_DM_Pin GPIO_PIN_11
#define USB_DEBUG_DM_GPIO_Port GPIOA
#define USB_DEBUG_DP_Pin GPIO_PIN_12
#define USB_DEBUG_DP_GPIO_Port GPIOA
#define LED_RF_Pin GPIO_PIN_15
#define LED_RF_GPIO_Port GPIOA
#define ESC_TX_Pin GPIO_PIN_11
#define ESC_TX_GPIO_Port GPIOC
#define QMC5883P_SCL_Pin GPIO_PIN_6
#define QMC5883P_SCL_GPIO_Port GPIOB
#define QMC5883P_SDA_Pin GPIO_PIN_7
#define QMC5883P_SDA_GPIO_Port GPIOB
#define LED_LF_Pin GPIO_PIN_9
#define LED_LF_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
