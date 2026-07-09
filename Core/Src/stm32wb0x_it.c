/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wb0x_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32wb0x_it.h"
#include "hw_pka.h"
#include "ble_stack.h"
#include "miscutil.h"
#include "stm32wb0x_ll_usart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern PKA_HandleTypeDef hpka;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;  /* TIM1 for 10Hz IMU updates */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WB0x Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wb0x.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 Interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA global interrupt.
  */
void DMA_IRQHandler(void)
{
  /* USER CODE BEGIN DMA_IRQn 0 */

  /* USER CODE END DMA_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA_IRQn 1 */

  /* USER CODE END DMA_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}

/**
  * @brief This function handles PKA global interrupt.
  */
void PKA_IRQHandler(void)
{
  /* USER CODE BEGIN PKA_IRQn 0 */

  /* USER CODE END PKA_IRQn 0 */
  HAL_PKA_IRQHandler(&hpka);
  /* USER CODE BEGIN PKA_IRQn 1 */

  /* USER CODE END PKA_IRQn 1 */
}

/**
  * @brief This function handles GPIOA interrupt.
  */
void GPIOA_IRQHandler(void)
{
  /* USER CODE BEGIN GPIOA_IRQn 0 */

  /* USER CODE END GPIOA_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIOA, PA11_GPIO_EXTI11_to_MVH4003D_ALERT_Pin);
  HAL_GPIO_EXTI_IRQHandler(GPIOA, PA12_GPIO_EXTI12_to_BMI270_INT1_Pin);
  // scan_Pin removed - PA0 is now I2C1_SCL, not GPIO
  /* USER CODE BEGIN GPIOA_IRQn 1 */

  /* USER CODE END GPIOA_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TIMER_CPU_WKUP global interrupt.
  */
void RADIO_TIMER_CPU_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_CPU_WKUP_IRQn 0 */

  /* USER CODE END RADIO_TIMER_CPU_WKUP_IRQn 0 */
  HAL_RADIO_TIMER_CPU_WKUP_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_CPU_WKUP_IRQn 1 */

  /* USER CODE END RADIO_TIMER_CPU_WKUP_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TIMER_TXRX_WKUP global interrupt.
  */
void RADIO_TIMER_TXRX_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_TXRX_WKUP_IRQn 0 */

  /* USER CODE END RADIO_TIMER_TXRX_WKUP_IRQn 0 */
  HAL_RADIO_TIMER_TXRX_WKUP_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_TXRX_WKUP_IRQn 1 */

  /* USER CODE END RADIO_TIMER_TXRX_WKUP_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TIMER_ERROR global interrupt.
  */
void RADIO_TIMER_ERROR_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TIMER_ERROR_IRQn 0 */

  /* USER CODE END RADIO_TIMER_ERROR_IRQn 0 */
  HAL_RADIO_TIMER_ERROR_IRQHandler();
  /* USER CODE BEGIN RADIO_TIMER_ERROR_IRQn 1 */

  /* USER CODE END RADIO_TIMER_ERROR_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TXRX global interrupt.
  */
void RADIO_TXRX_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TXRX_IRQn 0 */

  /* USER CODE END RADIO_TXRX_IRQn 0 */
  HAL_RADIO_TXRX_IRQHandler();
  /* USER CODE BEGIN RADIO_TXRX_IRQn 1 */

  /* USER CODE END RADIO_TXRX_IRQn 1 */
}

/**
  * @brief This function handles RADIO_TXRX_SEQ global interrupt.
  */
void RADIO_TXRX_SEQ_IRQHandler(void)
{
  /* USER CODE BEGIN RADIO_TXRX_SEQ_IRQn 0 */

  /* USER CODE END RADIO_TXRX_SEQ_IRQn 0 */
  HAL_RADIO_TXRX_SEQ_IRQHandler();
  /* USER CODE BEGIN RADIO_TXRX_SEQ_IRQn 1 */

  /* USER CODE END RADIO_TXRX_SEQ_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles TIM1 global interrupt (10Hz IMU updates).
  */
void TIM1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_IRQn 0 */

  /* USER CODE END TIM1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_IRQn 1 */

  /* USER CODE END TIM1_IRQn 1 */
}

/* USER CODE END 1 */
