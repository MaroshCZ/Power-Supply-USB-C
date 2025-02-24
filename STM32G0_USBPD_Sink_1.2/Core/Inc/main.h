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

#include "stm32g0xx_ll_lpuart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_ucpd.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

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
#define VDDA_APPLI 3300
#define FLT_IN_TCPP_Pin GPIO_PIN_1
#define FLT_IN_TCPP_GPIO_Port GPIOA
#define OCP_DAC_LIMIT_Pin GPIO_PIN_4
#define OCP_DAC_LIMIT_GPIO_Port GPIOA
#define VSENSE_Pin GPIO_PIN_5
#define VSENSE_GPIO_Port GPIOA
#define OCP_ADC_I_Pin GPIO_PIN_6
#define OCP_ADC_I_GPIO_Port GPIOA
#define ISENSE_Pin GPIO_PIN_7
#define ISENSE_GPIO_Port GPIOA
#define SW2_DEBUG_BTN_Pin GPIO_PIN_4
#define SW2_DEBUG_BTN_GPIO_Port GPIOC
#define SW2_DEBUG_BTN_EXTI_IRQn EXTI4_15_IRQn
#define CC1_G4_Pin GPIO_PIN_0
#define CC1_G4_GPIO_Port GPIOB
#define SW3_OFF_ON_Pin GPIO_PIN_1
#define SW3_OFF_ON_GPIO_Port GPIOB
#define SW3_OFF_ON_EXTI_IRQn EXTI0_1_IRQn
#define SW1_TOGGLE_I_V_Pin GPIO_PIN_2
#define SW1_TOGGLE_I_V_GPIO_Port GPIOB
#define SW1_TOGGLE_I_V_EXTI_IRQn EXTI2_3_IRQn
#define CS_MAX7219_Pin GPIO_PIN_12
#define CS_MAX7219_GPIO_Port GPIOB
#define ENC_TIM3_CH1_Pin GPIO_PIN_6
#define ENC_TIM3_CH1_GPIO_Port GPIOC
#define ENC_TIM3_CH2_Pin GPIO_PIN_7
#define ENC_TIM3_CH2_GPIO_Port GPIOC
#define ENC_TOGGLE_UNITS_Pin GPIO_PIN_8
#define ENC_TOGGLE_UNITS_GPIO_Port GPIOD
#define ENC_TOGGLE_UNITS_EXTI_IRQn EXTI4_15_IRQn
#define CC2_G4_Pin GPIO_PIN_4
#define CC2_G4_GPIO_Port GPIOB
#define OCP_ALERT_Pin GPIO_PIN_6
#define OCP_ALERT_GPIO_Port GPIOB
#define OCP_RESET_Pin GPIO_PIN_8
#define OCP_RESET_GPIO_Port GPIOB
#define RELAY_ON_OFF_Pin GPIO_PIN_10
#define RELAY_ON_OFF_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
