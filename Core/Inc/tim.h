/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
// 定时器句柄
extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */
// 定时器中断函数宏定义
#define TIM8_ARRUpdate_IRQn                 TIM8_UP_TIM13_IRQn
#define TIM8_ARRUpdate_IRQHandler           TIM8_UP_TIM13_IRQHandler
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
// 定时器配置函数声明
void TIM9_Config(void);
void TIM1_Config(void);  
void TIM2_Config(void);
void TIM3_Config(void);
void TIM4_Config(void);
void TIM5_Config(void);

void MX_TIM8_Init(void);  // ADC采样定时器 TIM8 配置
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_npwm_set(uint32_t npwm);

// 错误处理函数声明
void Error_Handler(void);
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* timHandle);
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
//void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

