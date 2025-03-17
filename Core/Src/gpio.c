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

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE(); // 使能 GPIOF 时钟
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 配置 PE13, PE14, PE15 为推挽输出模式 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL; // 无上拉下拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 低速输出
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* 设置初始电平：PF13 = 3.3V，PE14 = 3.3V，PE15 = 0V */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET); // 设置高电平
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET); // 设置高电平
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET); // 设置低电平

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
