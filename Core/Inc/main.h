/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32wb0x_hal.h"
#include "app_entry.h"
#include "app_common.h"
#include "app_debug.h"
#include "compiler.h"

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
#define PA9_GPIO_OUTPUT_to_MIA_M10Q_RESET_Pin GPIO_PIN_9
#define PA9_GPIO_OUTPUT_to_MIA_M10Q_RESET_GPIO_Port GPIOA
#define SPI2_NSS_to_Slave_CS_Pin GPIO_PIN_4
#define SPI2_NSS_to_Slave_CS_GPIO_Port GPIOA
#define PA11_GPIO_EXTI11_to_MVH4003D_ALERT_Pin GPIO_PIN_11
#define PA11_GPIO_EXTI11_to_MVH4003D_ALERT_GPIO_Port GPIOA
#define PA11_GPIO_EXTI11_to_MVH4003D_ALERT_EXTI_IRQn GPIOA_IRQn
#define PA12_GPIO_EXTI12_to_BMI270_INT1_Pin GPIO_PIN_12
#define PA12_GPIO_EXTI12_to_BMI270_INT1_GPIO_Port GPIOA
#define PA12_GPIO_EXTI12_to_BMI270_INT1_EXTI_IRQn GPIOA_IRQn
#define PB0_GPIO_EXTI0_to_BMI270_INT2_Pin GPIO_PIN_0
#define PB0_GPIO_EXTI0_to_BMI270_INT2_GPIO_Port GPIOB
#define PB0_GPIO_EXTI0_to_BMI270_INT2_EXTI_IRQn GPIOA_IRQn
#define PB4_GPIO_EXTI4_to_TMAG5273C1QDBVR_INT_Pin GPIO_PIN_4
#define PB4_GPIO_EXTI4_to_TMAG5273C1QDBVR_INT_GPIO_Port GPIOB
#define PB4_GPIO_EXTI4_to_TMAG5273C1QDBVR_INT_EXTI_IRQn GPIOA_IRQn
#define PB5_GPIO_EXTI5_to_LPS22HHTR_DRDY_Pin GPIO_PIN_5
#define PB5_GPIO_EXTI5_to_LPS22HHTR_DRDY_GPIO_Port GPIOB
#define PB5_GPIO_EXTI5_to_LPS22HHTR_DRDY_EXTI_IRQn GPIOA_IRQn
#define PB6_GPIO_EXTI6_to_MIA_M10Q_TIME_PULSE_SAFEBOOT_Pin GPIO_PIN_6
#define PB6_GPIO_EXTI6_to_MIA_M10Q_TIME_PULSE_SAFEBOOT_GPIO_Port GPIOB
#define PB6_GPIO_EXTI6_to_MIA_M10Q_TIME_PULSE_SAFEBOOT_EXTI_IRQn GPIOA_IRQn
#define PB7_GPIO_EXTI7_to_MIA_M10Q_INT_Pin GPIO_PIN_7
#define PB7_GPIO_EXTI7_to_MIA_M10Q_INT_GPIO_Port GPIOB
#define PB7_GPIO_EXTI7_to_MIA_M10Q_INT_EXTI_IRQn GPIOA_IRQn
#define PB8_GPIO_EXTI8_to_ZMOD4510AI4R_INT_Pin GPIO_PIN_8
#define PB8_GPIO_EXTI8_to_ZMOD4510AI4R_INT_GPIO_Port GPIOB
#define PB8_GPIO_EXTI8_to_ZMOD4510AI4R_INT_EXTI_IRQn GPIOA_IRQn
#define PB9_GPIO_OUTPUT_to_MIA_M10Q_VOL_SEL_Pin GPIO_PIN_9
#define PB9_GPIO_OUTPUT_to_MIA_M10Q_VOL_SEL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
