/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define LED_STATUS_Pin GPIO_PIN_2
#define LED_STATUS_GPIO_Port GPIOE
#define LED_CONTROLLER_Pin GPIO_PIN_3
#define LED_CONTROLLER_GPIO_Port GPIOE
#define LED_FAULT_Pin GPIO_PIN_4
#define LED_FAULT_GPIO_Port GPIOE
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define TX2_PIC_Pin GPIO_PIN_2
#define TX2_PIC_GPIO_Port GPIOA
#define RX2_PIC_Pin GPIO_PIN_3
#define RX2_PIC_GPIO_Port GPIOA
#define LED_USER1_Pin GPIO_PIN_4
#define LED_USER1_GPIO_Port GPIOA
#define LED_USER2_Pin GPIO_PIN_5
#define LED_USER2_GPIO_Port GPIOA
#define LED_USER3_Pin GPIO_PIN_7
#define LED_USER3_GPIO_Port GPIOA
#define SHIFTER_EN_Pin GPIO_PIN_0
#define SHIFTER_EN_GPIO_Port GPIOB
#define MOTOR_IN1_Pin GPIO_PIN_9
#define MOTOR_IN1_GPIO_Port GPIOE
#define MOTOR_IN2_Pin GPIO_PIN_11
#define MOTOR_IN2_GPIO_Port GPIOE
#define MOTOR_PWM_Pin GPIO_PIN_13
#define MOTOR_PWM_GPIO_Port GPIOE
#define MOTOR_EN_Pin GPIO_PIN_14
#define MOTOR_EN_GPIO_Port GPIOE
#define LED_FDCAN2_Pin GPIO_PIN_15
#define LED_FDCAN2_GPIO_Port GPIOE
#define DE3_RS485_Pin GPIO_PIN_14
#define DE3_RS485_GPIO_Port GPIOB
#define TX3_RS485_Pin GPIO_PIN_8
#define TX3_RS485_GPIO_Port GPIOD
#define RX3_RS485_Pin GPIO_PIN_9
#define RX3_RS485_GPIO_Port GPIOD
#define TX6_BACK_Pin GPIO_PIN_6
#define TX6_BACK_GPIO_Port GPIOC
#define RX6_BACK_Pin GPIO_PIN_7
#define RX6_BACK_GPIO_Port GPIOC
#define TX1_FRONT_Pin GPIO_PIN_9
#define TX1_FRONT_GPIO_Port GPIOA
#define RX1_FRONT_Pin GPIO_PIN_10
#define RX1_FRONT_GPIO_Port GPIOA
#define LED_UART_FRONT_Pin GPIO_PIN_15
#define LED_UART_FRONT_GPIO_Port GPIOA
#define LED_RS485_Pin GPIO_PIN_10
#define LED_RS485_GPIO_Port GPIOC
#define LED_UART_BACK_Pin GPIO_PIN_11
#define LED_UART_BACK_GPIO_Port GPIOC
#define TX5_EXT_Pin GPIO_PIN_12
#define TX5_EXT_GPIO_Port GPIOC
#define LED_FDCAN1_Pin GPIO_PIN_0
#define LED_FDCAN1_GPIO_Port GPIOD
#define RX5_EXT_Pin GPIO_PIN_2
#define RX5_EXT_GPIO_Port GPIOD
#define WATER_DETECTION_Pin GPIO_PIN_3
#define WATER_DETECTION_GPIO_Port GPIOD
#define WATER_DETECTION_EXTI_IRQn EXTI3_IRQn
#define SERVO1_Pin GPIO_PIN_4
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_5
#define SERVO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
