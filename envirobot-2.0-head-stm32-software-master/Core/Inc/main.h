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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_CM4_Pin GPIO_PIN_2
#define EN_CM4_GPIO_Port GPIOE
#define PG_CM4_Pin GPIO_PIN_3
#define PG_CM4_GPIO_Port GPIOE
#define nBT_CM4_Pin GPIO_PIN_4
#define nBT_CM4_GPIO_Port GPIOE
#define nWL_CM4_Pin GPIO_PIN_5
#define nWL_CM4_GPIO_Port GPIOE
#define STOP_CM4_Pin GPIO_PIN_6
#define STOP_CM4_GPIO_Port GPIOE
#define DRDY_IMU_Pin GPIO_PIN_0
#define DRDY_IMU_GPIO_Port GPIOA
#define nRST_IMU_Pin GPIO_PIN_1
#define nRST_IMU_GPIO_Port GPIOA
#define TX2_RADIO_Pin GPIO_PIN_2
#define TX2_RADIO_GPIO_Port GPIOA
#define RX2_RADIO_Pin GPIO_PIN_3
#define RX2_RADIO_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOA
#define WAKE_4G_Pin GPIO_PIN_4
#define WAKE_4G_GPIO_Port GPIOC
#define WDISABLE_4G_Pin GPIO_PIN_5
#define WDISABLE_4G_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOB
#define PERST_4G_Pin GPIO_PIN_1
#define PERST_4G_GPIO_Port GPIOB
#define DTR_4G_Pin GPIO_PIN_2
#define DTR_4G_GPIO_Port GPIOB
#define RX7_4G_Pin GPIO_PIN_7
#define RX7_4G_GPIO_Port GPIOE
#define TX7_4G_Pin GPIO_PIN_8
#define TX7_4G_GPIO_Port GPIOE
#define RTS7_4G_Pin GPIO_PIN_9
#define RTS7_4G_GPIO_Port GPIOE
#define CTS7_4G_Pin GPIO_PIN_10
#define CTS7_4G_GPIO_Port GPIOE
#define LED_STATUS_Pin GPIO_PIN_11
#define LED_STATUS_GPIO_Port GPIOE
#define LED_ACTIVITY_Pin GPIO_PIN_12
#define LED_ACTIVITY_GPIO_Port GPIOE
#define LED_FAULT_Pin GPIO_PIN_13
#define LED_FAULT_GPIO_Port GPIOE
#define LED_USER1_Pin GPIO_PIN_14
#define LED_USER1_GPIO_Port GPIOE
#define LED_USER2_Pin GPIO_PIN_15
#define LED_USER2_GPIO_Port GPIOE
#define LED_USER3_Pin GPIO_PIN_10
#define LED_USER3_GPIO_Port GPIOB
#define LED_UART_CM4_Pin GPIO_PIN_11
#define LED_UART_CM4_GPIO_Port GPIOB
#define DE3_RS485_Pin GPIO_PIN_14
#define DE3_RS485_GPIO_Port GPIOB
#define TX3_RS485_Pin GPIO_PIN_8
#define TX3_RS485_GPIO_Port GPIOD
#define RX3_RS485_Pin GPIO_PIN_9
#define RX3_RS485_GPIO_Port GPIOD
#define SW_OUTPUT_Pin GPIO_PIN_10
#define SW_OUTPUT_GPIO_Port GPIOD
#define SW_OVERWRITE_Pin GPIO_PIN_11
#define SW_OVERWRITE_GPIO_Port GPIOD
#define SW_INPUT_Pin GPIO_PIN_12
#define SW_INPUT_GPIO_Port GPIOD
#define TX6_BACK_Pin GPIO_PIN_6
#define TX6_BACK_GPIO_Port GPIOC
#define RX6_BACK_Pin GPIO_PIN_7
#define RX6_BACK_GPIO_Port GPIOC
#define TX1_FRONT_Pin GPIO_PIN_9
#define TX1_FRONT_GPIO_Port GPIOA
#define RX1_FRONT_Pin GPIO_PIN_10
#define RX1_FRONT_GPIO_Port GPIOA
#define SD_PRESENCE_Pin GPIO_PIN_15
#define SD_PRESENCE_GPIO_Port GPIOA
#define RX4_GNSS_Pin GPIO_PIN_0
#define RX4_GNSS_GPIO_Port GPIOD
#define TX4_GNSS_Pin GPIO_PIN_1
#define TX4_GNSS_GPIO_Port GPIOD
#define LED_RS485_Pin GPIO_PIN_3
#define LED_RS485_GPIO_Port GPIOD
#define LED_UART_BACK_Pin GPIO_PIN_4
#define LED_UART_BACK_GPIO_Port GPIOD
#define LED_FDCAN2_Pin GPIO_PIN_5
#define LED_FDCAN2_GPIO_Port GPIOD
#define LED_FDCAN1_Pin GPIO_PIN_6
#define LED_FDCAN1_GPIO_Port GPIOD
#define LED_GNSS_Pin GPIO_PIN_7
#define LED_GNSS_GPIO_Port GPIOD
#define WATER_DETECTION_Pin GPIO_PIN_5
#define WATER_DETECTION_GPIO_Port GPIOB
#define RX8_EXT_Pin GPIO_PIN_0
#define RX8_EXT_GPIO_Port GPIOE
#define TX8_EXT_Pin GPIO_PIN_1
#define TX8_EXT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
