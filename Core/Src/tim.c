/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* 独立CLK定时器 TIM9 句柄 */
TIM_HandleTypeDef htim9;  
/* 主定时器 TIM1 句柄 */
TIM_HandleTypeDef htim1;  
/* 从定时器 TIM2/3/4/5 句柄 */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;  
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
/* ADC采样用定时器 TIM8 句柄 */
TIM_HandleTypeDef htim8; 

/* CLK生成定时器 TIM9 配置 -- 1MHz CLK 输出*/
void TIM9_Config(void)
{
	__HAL_RCC_TIM9_CLK_ENABLE();  // 挂载APB2总线时钟(PCLK2)168MHz,使能 TIM9 时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();   // TIM9ch1->GPIOE5

	// TIM9的基础初始化
	htim9.Instance = TIM9; 
	htim9.Init.Prescaler = 84 - 1;     // 取值范围0~65536,设置预分频值83，2MHz 时钟
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 2 - 1;        // ARR=2, PWM周期1us
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim9.Init.RepetitionCounter = 0;   // 寄存器RCR
	HAL_TIM_Base_Init(&htim9);

	// CLK信号的输出比较通道初始化
	TIM_OC_InitTypeDef sConfigOC_CLK ;
	/* 在PWM1的模式下，CCR = 有效电平，且此处设置有效电平为高       *
	 * 在PWM2的模式下，ARR - CCR + 1 = 有效电平，且此处设置有效电平为高 */
	// PWM1即向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平
	sConfigOC_CLK.OCMode = TIM_OCMODE_PWM1; 
	sConfigOC_CLK.Pulse = 2 - 1;  // 50% 占空比，0.5us高电平
	sConfigOC_CLK.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_CLK.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC_CLK, TIM_CHANNEL_2); 

	// 启动ch2的PWM波
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	
	/* GPIO配置,PA3 */
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.Pin = GPIO_PIN_3;                 /* 通道y的GPIO口 */
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;               /* 复用推完输出 */
	gpio_init_struct.Pull = GPIO_NOPULL;                   /* 无上拉下拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
	gpio_init_struct.Alternate = GPIO_AF3_TIM9;            /* 端口复用 */		
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);

}


/* 主定时器 TIM1 配置 -- RESET */
void TIM1_Config(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();  // 挂载APB2总线时钟(PCLK2)168MHz,使能 TIM1 时钟
	  __HAL_RCC_GPIOE_CLK_ENABLE();

	  // TIM1的基础初始化
    htim1.Instance = TIM1; 
    htim1.Init.Prescaler = 168 - 1;     // 取值范围0~65536,设置预分频值168-1，1MHz 时钟 
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim1.Init.Period = 2000 - 1;        // PWM周期2ms
	  htim1.Init.Period = 20000 - 1;        // PWM周期200ms
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  htim1.Init.RepetitionCounter = 0;   // 寄存器RCR
	  HAL_TIM_Base_Init(&htim1);
	  
	  // 主模式配置
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;// TIM_TRGO_RESET | TIM_TRGO_ENABLE | TIM_TRGO_OC1REF不行
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //主模式输出TRG0设置为更新事件
	  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    // RESET信号的输出比较通道初始化
    TIM_OC_InitTypeDef sConfigOC_RESET ;
    sConfigOC_RESET.OCMode = TIM_OCMODE_PWM1;  // PWM1即向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平
    sConfigOC_RESET.Pulse = 1000 - 1;  // 25% 占空比，500us高电平
    sConfigOC_RESET.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC_RESET.OCFastMode = TIM_OCFAST_DISABLE;
	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_RESET, TIM_CHANNEL_3); 
		HAL_TIM_PWM_Init(&htim1);
	
		// 启动ch1的PWM波
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		
		
		/* GPIO配置,PE13 */
		GPIO_InitTypeDef gpio_init_struct;
		gpio_init_struct.Pin = GPIO_PIN_13;                 /* PE13 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* 无上拉下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                      /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;            /* 端口复用 */		
		HAL_GPIO_Init(GPIOE, &gpio_init_struct);

}


		
/* 从定时器 TIM2 配置 -- SH3信号 */
void TIM2_Config(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();  // 使能 TIM2 时钟，挂载APB1, 84MHz
		__HAL_RCC_GPIOA_CLK_ENABLE();   // PA2-TIM2ch3
	  
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1;     // psc,设置预分频值84，1MHz 时钟,1us
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 58- 1;     // arr,40us，ARR-CCR=Delay
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1个时钟周期记一次数，也可以1个时钟周期记2次数（TIM_CLOCKDIVISION_DIV2）
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  htim2.Init.RepetitionCounter = 0;   // 寄存器RCR
//	  HAL_TIM_Base_Init(&htim2);

	  
	  // TIM2主模式配置
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger =  TIM_TRGO_UPDATE;   //主模式输出TRG0设置为更新事件
	  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
	
	  // TIM2从模式配置
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	  sSlaveConfig.InputTrigger = TIM_TS_ITR0;    // 从定时器TIM2的ITRO是TIM1
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig);
		
		/* IC配置,获取ITR0(TRA) */
		TIM_IC_InitTypeDef sConfigIC_SH3;
		sConfigIC_SH3.ICFilter = 0;
		sConfigIC_SH3.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_SH3.ICSelection = TIM_ICSELECTION_TRC;  // TRA包含ITR0
		//TIM2->CCER &= ~TIM_CCER_CC1E_Msk;  // 先将CC1E位置0，即不使能
		//TIM2->CCMR1 |= TIM_CCMR1_CC1S; // CC1S：CC1 通道配置为输入， IC1 映射到 TRC 上。此模式仅在通过 TS 位（ TIMx_SMCR 寄存器）选择内部触发输入时有效
		HAL_TIM_IC_Init(&htim2);
		HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC_SH3, TIM_CHANNEL_3);
		HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_3);
		
		/* OC配置，SH3 单脉冲-->ch3引脚 = GPIOA2 */
		TIM_OC_InitTypeDef sConfigOC_SH3;
		sConfigOC_SH3.OCMode = TIM_OCMODE_PWM2;   // PWM2即向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为无效电平，否则为有效电平
		sConfigOC_SH3.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC_SH3.OCFastMode = TIM_OCFAST_ENABLE;
		/* CCR2设置为49-1，即t<48为无效电平(Tdelay)，t>=48为有效电平(高电平),宽度为 58 - 27 + 1 = 32us */
		sConfigOC_SH3.Pulse = 27 - 1;  
		HAL_TIM_OC_Init(&htim2);
		HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC_SH3, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3);
		
		/* 必要的，产生一个脉冲就停 */
	  HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE); 
		
		/* GPIO配置,PA2 */
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_2 ;                 /* GPIOA2 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* 无上拉下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;                /* 端口复用 */		
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
		__HAL_TIM_ENABLE(&htim2);
}
	

/* 从定时器 TIM3 配置 -- ST信号 */
void TIM3_Config(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();  // 使能 TIM2 时钟，挂载APB1， 84MHz
	  __HAL_RCC_GPIOA_CLK_ENABLE();   // PB4-TIM1ch1
	
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 21 - 1;     // psc,设置预分频值84，1MHz 时钟,1us1个时钟周期记2次数（TIM_CLOCKDIVISION_DIV2）
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 8 - 1;     // ARR-CCR + 1 = Delay
//    htim3.Init.Period = 640 - 1;     // 结束于60us，ARR-CCR=Delay
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1个时钟周期记一次数，也可以
	  htim3.Init.RepetitionCounter = 0;   // 寄存器RCR
	  
	  /* TIM3主模式配置 */
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2的MMS位 主模式输出TRG0设置为更新事件
	  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
	
	  /* TIM3从模式配置 */
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR的SMS位
	  sSlaveConfig.InputTrigger = TIM_TS_ITR3;    // SMCR的TS位 从定时器TIM3的ITR3是TIM4（SH1）
//		sSlaveConfig.InputTrigger = TIM_TS_ITR0;    // SMCR的TS位 从定时器TIM3的ITRO是TIM1
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig);
	
		
		/* IC配置,获取ITR1(TRA) */
		TIM_IC_InitTypeDef sConfigIC_ST;
		sConfigIC_ST.ICFilter = 0;
		sConfigIC_ST.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_ST.ICSelection = TIM_ICSELECTION_TRC;  // TRA包含ITR0
		HAL_TIM_IC_Init(&htim3);
		HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC_ST, TIM_CHANNEL_1);
		HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
		
    /* ST 单脉冲-->ch1引脚 = GPIOC6 配置输出比较匹配来产生精确的脉冲信号 */
		TIM_OC_InitTypeDef sConfigOC_ST ; // 定义并初始化配置结构体
		sConfigOC_ST.OCMode = TIM_OCMODE_PWM2;     // PWM模式选择(1或2),PWM1即向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平；
		sConfigOC_ST.OCPolarity = TIM_OCPOLARITY_HIGH;  //有效值为高电平
		sConfigOC_ST.OCFastMode = TIM_OCFAST_ENABLE;  // 使用快速模式
		sConfigOC_ST.Pulse = 5 - 1;
//		sConfigOC_ST.Pulse = 637 - 1;
		HAL_TIM_OC_Init(&htim3);
		HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC_ST, TIM_CHANNEL_1);
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
		
		/* TIM3初始化为单脉冲模式 */
		HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE);

		/* GPIO配置,PA6 */
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_6 ;                 /* GPIOA6复用为TIM3-ch1 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* 无上拉下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;                /* 端口复用 */		
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		__HAL_TIM_ENABLE(&htim3);

}

/* 从定时器 TIM4 配置 -- SH1信号 */
void TIM4_Config(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();  // 使能 TIM2 时钟，挂载APB1，84MHz
	  __HAL_RCC_GPIOD_CLK_ENABLE();   // PD12 13 14 15

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 84 - 1;     // psc,设置预分频值84，1MHz 时钟,1us1个时钟周期记2次数（TIM_CLOCKDIVISION_DIV2）
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 100 - 1;     // arr,40us，ARR-CCR=Delay
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1个时钟周期记一次数，也可以
	  htim4.Init.RepetitionCounter = 0;   // 寄存器RCR
//	  HAL_TIM_Base_Init(&htim4);
	  
	  /* TIM4主模式配置 */
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2的MMS位 主模式输出TRG0设置为更新事件
	  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
	
	  /* TIM4从模式配置 */
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR的SMS位
	  sSlaveConfig.InputTrigger = TIM_TS_ITR1;    // SMCR的TS位 从定时器TIM4的ITR2是TIM3
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig);
	
		
		/* IC配置,获取ITR1(TRA) */
		TIM_IC_InitTypeDef sConfigIC_SH1;
		sConfigIC_SH1.ICFilter = 0;
		sConfigIC_SH1.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_SH1.ICSelection = TIM_ICSELECTION_TRC;  // TRA包含ITR0
		HAL_TIM_IC_Init(&htim4);
		HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC_SH1, TIM_CHANNEL_1);
		HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
		
    /* SH1 单脉冲-->ch1引脚 = GPIOB6 配置输出比较匹配来产生精确的脉冲信号 */
		TIM_OC_InitTypeDef sConfigOC_SH1 ; // 定义并初始化配置结构体
		sConfigOC_SH1.OCMode = TIM_OCMODE_PWM2;     // PWM模式选择(1或2),PWM2即向上计数时，一旦TIMx_CNT >= TIMx_CCR时通道1为有效电平，否则为无效电平；
		sConfigOC_SH1.OCPolarity = TIM_OCPOLARITY_HIGH;  //有效值为高电平
		sConfigOC_SH1.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式
		sConfigOC_SH1.Pulse = 69 - 1;   // TIMx_CCR
		HAL_TIM_OC_Init(&htim4);
		HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC_SH1, TIM_CHANNEL_1);
		HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
		
		/* TIM4初始化为单脉冲模式 */
		HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE);

		/* GPIO配置，PD12*/
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_12 ;                 /* GPIOD12复用为TIM3-ch1 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* 无上拉下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF2_TIM4;                /* 端口复用 */		
		HAL_GPIO_Init(GPIOD, &gpio_init_struct);
		
		__HAL_TIM_ENABLE(&htim4);
}


/* 从定时器 TIM5 配置 -- SH2信号 */
void TIM5_Config(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();  // 使能 TIM2 时钟，挂载APB1 = 84MHz
	  __HAL_RCC_GPIOA_CLK_ENABLE();   // PA0 1 2 3

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 84 - 1;     // psc,设置预分频值84，1MHz 时钟,1us1个时钟周期记2次数（TIM_CLOCKDIVISION_DIV2）
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 100 - 1;     // ARR
//	  htim5.Init.Period = 200 - 1;     // ARR
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1个时钟周期记一次数，也可以
	  htim5.Init.RepetitionCounter = 0;   // 寄存器RCR
//		HAL_TIM_Base_Init(&htim5);
	  
	  /* TIM5主模式配置 */
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2的MMS位 主模式输出TRG0设置为更新事件
	  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);
	
	  /* TIM5从模式配置 */
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR的SMS位
	  sSlaveConfig.InputTrigger = TIM_TS_ITR2;    // SMCR的TS位 从定时器TIM5的ITR2是TIM4（SH1）
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig);
	
		
		/* IC配置,获取ITR1(TRA) */
		TIM_IC_InitTypeDef sConfigIC_SH2;
		sConfigIC_SH2.ICFilter = 0;
		sConfigIC_SH2.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_SH2.ICSelection = TIM_ICSELECTION_TRC;  // TRA包含ITR0
		HAL_TIM_IC_Init(&htim5);
		HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC_SH2, TIM_CHANNEL_2);
		HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_2);
		
    /* SH2 单脉冲-->ch4引脚 = GPIOA3 配置输出比较匹配来产生精确的脉冲信号 */
		TIM_OC_InitTypeDef sConfigOC_SH1 ; // 定义并初始化配置结构体
		sConfigOC_SH1.OCMode = TIM_OCMODE_PWM2;     // PWM模式选择(1或2),PWM1即向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平；
		sConfigOC_SH1.OCPolarity = TIM_OCPOLARITY_HIGH;  //有效值为高电平
		sConfigOC_SH1.OCFastMode = TIM_OCFAST_DISABLE;  // 禁用快速模式
		sConfigOC_SH1.Pulse = 69 - 1;
//		sConfigOC_SH1.Pulse = 169 - 1;
		HAL_TIM_OC_Init(&htim5);
		HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC_SH1, TIM_CHANNEL_2);
		HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_2);
		
		/* TIM3初始化为单脉冲模式 */
		HAL_TIM_OnePulse_Init(&htim5, TIM_OPMODE_SINGLE);

		/* GPIO配置,PA1 */
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_1  ;                 /* GPIOA3复用为TIM5-ch4 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* 无上拉下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* 高速 */
    gpio_init_struct.Alternate = GPIO_AF2_TIM5;                /* 端口复用 */		
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		__HAL_TIM_ENABLE(&htim5);
		
}

/***************************************** ADC采样触发timer  TIM8CC1 *************************************************************/

/* TIM8 init function */
void MX_TIM8_Init(void)
{
	/*ADC采样时钟，预分频Period（ARR）与Pulse（CCR）的设置应该满足 ### CCR占比为5/16 ###：
 *(1)PD阵列CLK=1MHz时,TIM8预分频系数 = 21-1，8MHz；ARR = 16-1；CCR = 5-1;
 *(1)PD阵列CLK=1.25MHz时???????,TIM8预分频系数 = 21-1，8MHz；ARR = 16-1；CCR = 5-1;
 */
	__HAL_RCC_TIM8_CLK_ENABLE();  // 使能 TIM8 时钟，挂载APB2 = 168MHz
	__HAL_RCC_GPIOC_CLK_ENABLE();   // PC6

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 1 - 1;     // psc,设置预分频值21，8MHz 时钟
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 336 - 1;     // arr,40us，ARR-CCR=Delay
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1个时钟周期记一次数，也可以
	htim8.Init.RepetitionCounter = 256-1;   // 寄存器RCR
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim8);
	
	/* TIM8主模式配置 */
	TIM_MasterConfigTypeDef sMasterConfig;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2的MMS位 主模式输出TRG0设置为更新事件
	HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

	/* TIM8从模式配置 */
	TIM_SlaveConfigTypeDef sSlaveConfig;
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR的SMS位
//	sSlaveConfig.InputTrigger = TIM_TS_ITR1;    // SMCR的TS位 从定时器TIM8的ITR1是TIM2（SH3）
	sSlaveConfig.InputTrigger = TIM_TS_ITR2;    // SMCR的TS位 从定时器TIM8的ITR2是TIM4（SH1）
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig);

	
	/* IC配置,获取ITR1(TRA) */
	TIM_IC_InitTypeDef sConfigIC_ADC;
	sConfigIC_ADC.ICFilter = 0;
	sConfigIC_ADC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC_ADC.ICSelection = TIM_ICSELECTION_TRC;  // TRA包含ITR0
	HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC_ADC, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_1);
	
	/* SH2 单脉冲-->ch4引脚 = GPIOA3 配置输出比较匹配来产生精确的脉冲信号 */
	TIM_OC_InitTypeDef sConfigOC_ADC ; // 定义并初始化配置结构体
	sConfigOC_ADC.OCMode = TIM_OCMODE_PWM2;     // PWM模式选择(1或2),PWM1即向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为有效电平，否则为无效电平；
	sConfigOC_ADC.OCPolarity = TIM_OCPOLARITY_HIGH;  //有效值为高电平
	sConfigOC_ADC.OCFastMode = TIM_OCFAST_ENABLE;  // 禁用快速模式
	sConfigOC_ADC.Pulse = 288 - 1;
	HAL_TIM_OC_Init(&htim8);
	HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC_ADC, TIM_CHANNEL_1); 
	
	// 启动ch1的PWM波
	HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	/* GPIO配置,PA1 */
	GPIO_InitTypeDef gpio_init_struct;		
	gpio_init_struct.Pin = GPIO_PIN_6  ;                 /* GPIOA3复用为TIM5-ch4 */
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* 复用推完输出 */
	gpio_init_struct.Pull = GPIO_NOPULL;                                /* 无上拉下拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* 高速 */
	gpio_init_struct.Alternate = GPIO_AF3_TIM8;                /* 端口复用 */		
	HAL_GPIO_Init(GPIOC, &gpio_init_struct);
	
	__HAL_TIM_ENABLE(&htim8);

}


