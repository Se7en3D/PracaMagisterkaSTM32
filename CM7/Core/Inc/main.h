/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define L298NOUT1A_Pin GPIO_PIN_2
#define L298NOUT1A_GPIO_Port GPIOE
#define L298NOUT2A_Pin GPIO_PIN_3
#define L298NOUT2A_GPIO_Port GPIOE
#define L298NOUT1B_Pin GPIO_PIN_4
#define L298NOUT1B_GPIO_Port GPIOE
#define L298NOUT1BE5_Pin GPIO_PIN_5
#define L298NOUT1BE5_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define IR_NR_1_Pin GPIO_PIN_1
#define IR_NR_1_GPIO_Port GPIOB
#define IR_NR_2_Pin GPIO_PIN_2
#define IR_NR_2_GPIO_Port GPIOB
#define IR_NR_3_Pin GPIO_PIN_11
#define IR_NR_3_GPIO_Port GPIOF
#define IR_NR_4_Pin GPIO_PIN_14
#define IR_NR_4_GPIO_Port GPIOF
#define IR_NR_5_Pin GPIO_PIN_15
#define IR_NR_5_GPIO_Port GPIOF
#define IR_NR_6_Pin GPIO_PIN_7
#define IR_NR_6_GPIO_Port GPIOE
#define IR_NR_7_Pin GPIO_PIN_8
#define IR_NR_7_GPIO_Port GPIOE
#define IR_NR_8_Pin GPIO_PIN_9
#define IR_NR_8_GPIO_Port GPIOE
#define OPTOCOUPLER_POWER_ON_Pin GPIO_PIN_10
#define OPTOCOUPLER_POWER_ON_GPIO_Port GPIOE
#define LIMIT_SWITCH_SERVO_Pin GPIO_PIN_11
#define LIMIT_SWITCH_SERVO_GPIO_Port GPIOE
#define VL53L0X_XSHUT_Pin GPIO_PIN_12
#define VL53L0X_XSHUT_GPIO_Port GPIOE
#define VL53L0X_GPIO1_Pin GPIO_PIN_13
#define VL53L0X_GPIO1_GPIO_Port GPIOE
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
