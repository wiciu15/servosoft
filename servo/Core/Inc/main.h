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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define enc_alarm_Pin GPIO_PIN_13
#define enc_alarm_GPIO_Port GPIOC
#define softstart_Pin GPIO_PIN_15
#define softstart_GPIO_Port GPIOC
#define ENC_B_Pin GPIO_PIN_0
#define ENC_B_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_1
#define ENC_A_GPIO_Port GPIOA
#define SSI_ENC_TX_Pin GPIO_PIN_2
#define SSI_ENC_TX_GPIO_Port GPIOA
#define SSI_ENC_RX_Pin GPIO_PIN_3
#define SSI_ENC_RX_GPIO_Port GPIOA
#define I_U_Pin GPIO_PIN_4
#define I_U_GPIO_Port GPIOA
#define I_V_Pin GPIO_PIN_5
#define I_V_GPIO_Port GPIOA
#define DC_LINK_VOLTAGE_Pin GPIO_PIN_6
#define DC_LINK_VOLTAGE_GPIO_Port GPIOA
#define HEATSINK_Pin GPIO_PIN_7
#define HEATSINK_GPIO_Port GPIOA
#define INPUTS_LOAD_Pin GPIO_PIN_0
#define INPUTS_LOAD_GPIO_Port GPIOB
#define INPUTS_CS_Pin GPIO_PIN_1
#define INPUTS_CS_GPIO_Port GPIOB
#define DISP_LATCH_Pin GPIO_PIN_2
#define DISP_LATCH_GPIO_Port GPIOB
#define INV_ENABLE_Pin GPIO_PIN_12
#define INV_ENABLE_GPIO_Port GPIOB
#define INVERTER_DISABLE_Pin GPIO_PIN_13
#define INVERTER_DISABLE_GPIO_Port GPIOB
#define U_PWM_Pin GPIO_PIN_8
#define U_PWM_GPIO_Port GPIOA
#define V_PWM_Pin GPIO_PIN_9
#define V_PWM_GPIO_Port GPIOA
#define W_PWM_Pin GPIO_PIN_10
#define W_PWM_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_11
#define STEP_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_12
#define DIR_GPIO_Port GPIOA
#define ENC_Z_Pin GPIO_PIN_15
#define ENC_Z_GPIO_Port GPIOA
#define DISP_EN_Pin GPIO_PIN_4
#define DISP_EN_GPIO_Port GPIOB
#define OVERCURRENT_Pin GPIO_PIN_5
#define OVERCURRENT_GPIO_Port GPIOB
#define MODBUS_TX_Pin GPIO_PIN_6
#define MODBUS_TX_GPIO_Port GPIOB
#define MODBUS_RX_Pin GPIO_PIN_7
#define MODBUS_RX_GPIO_Port GPIOB
#define MODBUS_DE_Pin GPIO_PIN_8
#define MODBUS_DE_GPIO_Port GPIOB
#define ADC_CS_Pin GPIO_PIN_9
#define ADC_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
