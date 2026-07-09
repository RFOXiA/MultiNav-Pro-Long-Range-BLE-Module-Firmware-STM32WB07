/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "stm32wb07.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PA2   ------> DEBUG_SWDIO
     OSCOUT   ------> RCC_OSC_OUT
     OSCIN   ------> RCC_OSC_IN
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level
   * RESET_N is active-low: keep it HIGH (released) at boot so the M10Q can
   * start up. GNSS_Init performs a clean reset pulse later. */
  HAL_GPIO_WritePin(GPIOA, PA9_GPIO_OUTPUT_to_MIA_M10Q_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI2_NSS_to_Slave_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level
   * VOL_SEL must be HIGH (or left open) for 3.3V I/O - same as the rest of
   * the system. Driving it LOW would select 1.8V mode; the M10Q samples this
   * pin at power-up, so it must be HIGH from the very start. */
  HAL_GPIO_WritePin(GPIOB, PB9_GPIO_OUTPUT_to_MIA_M10Q_VOL_SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PA9_GPIO_OUTPUT_to_MIA_M10Q_RESET_Pin */
  GPIO_InitStruct.Pin = PA9_GPIO_OUTPUT_to_MIA_M10Q_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PA9_GPIO_OUTPUT_to_MIA_M10Q_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS_to_Slave_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_to_Slave_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_NSS_to_Slave_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9_GPIO_OUTPUT_to_MIA_M10Q_VOL_SEL_Pin */
  GPIO_InitStruct.Pin = PB9_GPIO_OUTPUT_to_MIA_M10Q_VOL_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PB9_GPIO_OUTPUT_to_MIA_M10Q_VOL_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11_GPIO_EXTI11_to_MVH4003D_ALERT_Pin */
  GPIO_InitStruct.Pin = PA11_GPIO_EXTI11_to_MVH4003D_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PA11_GPIO_EXTI11_to_MVH4003D_ALERT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12_GPIO_EXTI12_to_BMI270_INT1_Pin */
  GPIO_InitStruct.Pin = PA12_GPIO_EXTI12_to_BMI270_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PA12_GPIO_EXTI12_to_BMI270_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0_GPIO_EXTI0_to_BMI270_INT2_Pin */
  GPIO_InitStruct.Pin = PB0_GPIO_EXTI0_to_BMI270_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB0_GPIO_EXTI0_to_BMI270_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4_GPIO_EXTI4_to_TMAG5273C1QDBVR_INT_Pin */
  GPIO_InitStruct.Pin = PB4_GPIO_EXTI4_to_TMAG5273C1QDBVR_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB4_GPIO_EXTI4_to_TMAG5273C1QDBVR_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5_GPIO_EXTI5_to_LPS22HHTR_DRDY_Pin */
  GPIO_InitStruct.Pin = PB5_GPIO_EXTI5_to_LPS22HHTR_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB5_GPIO_EXTI5_to_LPS22HHTR_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6_GPIO_EXTI6_to_MIA_M10Q_TIME_PULSE_SAFEBOOT_Pin */
  GPIO_InitStruct.Pin = PB6_GPIO_EXTI6_to_MIA_M10Q_TIME_PULSE_SAFEBOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB6_GPIO_EXTI6_to_MIA_M10Q_TIME_PULSE_SAFEBOOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7_GPIO_EXTI7_to_MIA_M10Q_INT_Pin */
  GPIO_InitStruct.Pin = PB7_GPIO_EXTI7_to_MIA_M10Q_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB7_GPIO_EXTI7_to_MIA_M10Q_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8_GPIO_EXTI8_to_ZMOD4510AI4R_INT_Pin */
  GPIO_InitStruct.Pin = PB8_GPIO_EXTI8_to_ZMOD4510AI4R_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB8_GPIO_EXTI8_to_ZMOD4510AI4R_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SWDIO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_4|PWR_GPIO_BIT_9);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_A, PWR_GPIO_BIT_4|PWR_GPIO_BIT_9);

  /**/
  HAL_PWREx_DisableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_9);

  /**/
  HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_9);

  /**/
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_2);

  /*RT DEBUG GPIO_Init */
  RT_DEBUG_GPIO_Init();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(GPIOA_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(GPIOA_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
