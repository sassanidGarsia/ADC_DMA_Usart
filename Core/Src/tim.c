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

/* ����CLK��ʱ�� TIM9 ��� */
TIM_HandleTypeDef htim9;  
/* ����ʱ�� TIM1 ��� */
TIM_HandleTypeDef htim1;  
/* �Ӷ�ʱ�� TIM2/3/4/5 ��� */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;  
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
/* ADC�����ö�ʱ�� TIM8 ��� */
TIM_HandleTypeDef htim8; 

/* CLK���ɶ�ʱ�� TIM9 ���� -- 1MHz CLK ���*/
void TIM9_Config(void)
{
	__HAL_RCC_TIM9_CLK_ENABLE();  // ����APB2����ʱ��(PCLK2)168MHz,ʹ�� TIM9 ʱ��
	__HAL_RCC_GPIOA_CLK_ENABLE();   // TIM9ch1->GPIOE5

	// TIM9�Ļ�����ʼ��
	htim9.Instance = TIM9; 
	htim9.Init.Prescaler = 84 - 1;     // ȡֵ��Χ0~65536,����Ԥ��Ƶֵ83��2MHz ʱ��
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 2 - 1;        // ARR=2, PWM����1us
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim9.Init.RepetitionCounter = 0;   // �Ĵ���RCR
	HAL_TIM_Base_Init(&htim9);

	// CLK�źŵ�����Ƚ�ͨ����ʼ��
	TIM_OC_InitTypeDef sConfigOC_CLK ;
	/* ��PWM1��ģʽ�£�CCR = ��Ч��ƽ���Ҵ˴�������Ч��ƽΪ��       *
	 * ��PWM2��ģʽ�£�ARR - CCR + 1 = ��Ч��ƽ���Ҵ˴�������Ч��ƽΪ�� */
	// PWM1�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ
	sConfigOC_CLK.OCMode = TIM_OCMODE_PWM1; 
	sConfigOC_CLK.Pulse = 2 - 1;  // 50% ռ�ձȣ�0.5us�ߵ�ƽ
	sConfigOC_CLK.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC_CLK.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC_CLK, TIM_CHANNEL_2); 

	// ����ch2��PWM��
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	
	/* GPIO����,PA3 */
	GPIO_InitTypeDef gpio_init_struct;
	gpio_init_struct.Pin = GPIO_PIN_3;                 /* ͨ��y��GPIO�� */
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;               /* ����������� */
	gpio_init_struct.Pull = GPIO_NOPULL;                   /* ���������� */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;         /* ���� */
	gpio_init_struct.Alternate = GPIO_AF3_TIM9;            /* �˿ڸ��� */		
	HAL_GPIO_Init(GPIOA, &gpio_init_struct);

}


/* ����ʱ�� TIM1 ���� -- RESET */
void TIM1_Config(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();  // ����APB2����ʱ��(PCLK2)168MHz,ʹ�� TIM1 ʱ��
	  __HAL_RCC_GPIOE_CLK_ENABLE();

	  // TIM1�Ļ�����ʼ��
    htim1.Instance = TIM1; 
    htim1.Init.Prescaler = 168 - 1;     // ȡֵ��Χ0~65536,����Ԥ��Ƶֵ168-1��1MHz ʱ�� 
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim1.Init.Period = 2000 - 1;        // PWM����2ms
	  htim1.Init.Period = 20000 - 1;        // PWM����200ms
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  htim1.Init.RepetitionCounter = 0;   // �Ĵ���RCR
	  HAL_TIM_Base_Init(&htim1);
	  
	  // ��ģʽ����
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;// TIM_TRGO_RESET | TIM_TRGO_ENABLE | TIM_TRGO_OC1REF����
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //��ģʽ���TRG0����Ϊ�����¼�
	  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    // RESET�źŵ�����Ƚ�ͨ����ʼ��
    TIM_OC_InitTypeDef sConfigOC_RESET ;
    sConfigOC_RESET.OCMode = TIM_OCMODE_PWM1;  // PWM1�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ
    sConfigOC_RESET.Pulse = 1000 - 1;  // 25% ռ�ձȣ�500us�ߵ�ƽ
    sConfigOC_RESET.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC_RESET.OCFastMode = TIM_OCFAST_DISABLE;
	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC_RESET, TIM_CHANNEL_3); 
		HAL_TIM_PWM_Init(&htim1);
	
		// ����ch1��PWM��
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		
		
		/* GPIO����,PE13 */
		GPIO_InitTypeDef gpio_init_struct;
		gpio_init_struct.Pin = GPIO_PIN_13;                 /* PE13 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* ����������� */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* ���������� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                      /* ���� */
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;            /* �˿ڸ��� */		
		HAL_GPIO_Init(GPIOE, &gpio_init_struct);

}


		
/* �Ӷ�ʱ�� TIM2 ���� -- SH3�ź� */
void TIM2_Config(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();  // ʹ�� TIM2 ʱ�ӣ�����APB1, 84MHz
		__HAL_RCC_GPIOA_CLK_ENABLE();   // PA2-TIM2ch3
	  
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1;     // psc,����Ԥ��Ƶֵ84��1MHz ʱ��,1us
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 58- 1;     // arr,40us��ARR-CCR=Delay
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1��ʱ�����ڼ�һ������Ҳ����1��ʱ�����ڼ�2������TIM_CLOCKDIVISION_DIV2��
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  htim2.Init.RepetitionCounter = 0;   // �Ĵ���RCR
//	  HAL_TIM_Base_Init(&htim2);

	  
	  // TIM2��ģʽ����
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger =  TIM_TRGO_UPDATE;   //��ģʽ���TRG0����Ϊ�����¼�
	  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
	
	  // TIM2��ģʽ����
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	  sSlaveConfig.InputTrigger = TIM_TS_ITR0;    // �Ӷ�ʱ��TIM2��ITRO��TIM1
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig);
		
		/* IC����,��ȡITR0(TRA) */
		TIM_IC_InitTypeDef sConfigIC_SH3;
		sConfigIC_SH3.ICFilter = 0;
		sConfigIC_SH3.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_SH3.ICSelection = TIM_ICSELECTION_TRC;  // TRA����ITR0
		//TIM2->CCER &= ~TIM_CCER_CC1E_Msk;  // �Ƚ�CC1Eλ��0������ʹ��
		//TIM2->CCMR1 |= TIM_CCMR1_CC1S; // CC1S��CC1 ͨ������Ϊ���룬 IC1 ӳ�䵽 TRC �ϡ���ģʽ����ͨ�� TS λ�� TIMx_SMCR �Ĵ�����ѡ���ڲ���������ʱ��Ч
		HAL_TIM_IC_Init(&htim2);
		HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC_SH3, TIM_CHANNEL_3);
		HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_3);
		
		/* OC���ã�SH3 ������-->ch3���� = GPIOA2 */
		TIM_OC_InitTypeDef sConfigOC_SH3;
		sConfigOC_SH3.OCMode = TIM_OCMODE_PWM2;   // PWM2�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ
		sConfigOC_SH3.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC_SH3.OCFastMode = TIM_OCFAST_ENABLE;
		/* CCR2����Ϊ49-1����t<48Ϊ��Ч��ƽ(Tdelay)��t>=48Ϊ��Ч��ƽ(�ߵ�ƽ),���Ϊ 58 - 27 + 1 = 32us */
		sConfigOC_SH3.Pulse = 27 - 1;  
		HAL_TIM_OC_Init(&htim2);
		HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC_SH3, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3);
		
		/* ��Ҫ�ģ�����һ�������ͣ */
	  HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE); 
		
		/* GPIO����,PA2 */
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_2 ;                 /* GPIOA2 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* ����������� */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* ���������� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* ���� */
    gpio_init_struct.Alternate = GPIO_AF1_TIM2;                /* �˿ڸ��� */		
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    
		__HAL_TIM_ENABLE(&htim2);
}
	

/* �Ӷ�ʱ�� TIM3 ���� -- ST�ź� */
void TIM3_Config(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();  // ʹ�� TIM2 ʱ�ӣ�����APB1�� 84MHz
	  __HAL_RCC_GPIOA_CLK_ENABLE();   // PB4-TIM1ch1
	
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 21 - 1;     // psc,����Ԥ��Ƶֵ84��1MHz ʱ��,1us1��ʱ�����ڼ�2������TIM_CLOCKDIVISION_DIV2��
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 8 - 1;     // ARR-CCR + 1 = Delay
//    htim3.Init.Period = 640 - 1;     // ������60us��ARR-CCR=Delay
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1��ʱ�����ڼ�һ������Ҳ����
	  htim3.Init.RepetitionCounter = 0;   // �Ĵ���RCR
	  
	  /* TIM3��ģʽ���� */
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2��MMSλ ��ģʽ���TRG0����Ϊ�����¼�
	  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
	
	  /* TIM3��ģʽ���� */
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR��SMSλ
	  sSlaveConfig.InputTrigger = TIM_TS_ITR3;    // SMCR��TSλ �Ӷ�ʱ��TIM3��ITR3��TIM4��SH1��
//		sSlaveConfig.InputTrigger = TIM_TS_ITR0;    // SMCR��TSλ �Ӷ�ʱ��TIM3��ITRO��TIM1
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig);
	
		
		/* IC����,��ȡITR1(TRA) */
		TIM_IC_InitTypeDef sConfigIC_ST;
		sConfigIC_ST.ICFilter = 0;
		sConfigIC_ST.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_ST.ICSelection = TIM_ICSELECTION_TRC;  // TRA����ITR0
		HAL_TIM_IC_Init(&htim3);
		HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC_ST, TIM_CHANNEL_1);
		HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
		
    /* ST ������-->ch1���� = GPIOC6 ��������Ƚ�ƥ����������ȷ�������ź� */
		TIM_OC_InitTypeDef sConfigOC_ST ; // ���岢��ʼ�����ýṹ��
		sConfigOC_ST.OCMode = TIM_OCMODE_PWM2;     // PWMģʽѡ��(1��2),PWM1�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ��
		sConfigOC_ST.OCPolarity = TIM_OCPOLARITY_HIGH;  //��ЧֵΪ�ߵ�ƽ
		sConfigOC_ST.OCFastMode = TIM_OCFAST_ENABLE;  // ʹ�ÿ���ģʽ
		sConfigOC_ST.Pulse = 5 - 1;
//		sConfigOC_ST.Pulse = 637 - 1;
		HAL_TIM_OC_Init(&htim3);
		HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC_ST, TIM_CHANNEL_1);
		HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
		
		/* TIM3��ʼ��Ϊ������ģʽ */
		HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE);

		/* GPIO����,PA6 */
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_6 ;                 /* GPIOA6����ΪTIM3-ch1 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* ����������� */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* ���������� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* ���� */
    gpio_init_struct.Alternate = GPIO_AF2_TIM3;                /* �˿ڸ��� */		
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		__HAL_TIM_ENABLE(&htim3);

}

/* �Ӷ�ʱ�� TIM4 ���� -- SH1�ź� */
void TIM4_Config(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();  // ʹ�� TIM2 ʱ�ӣ�����APB1��84MHz
	  __HAL_RCC_GPIOD_CLK_ENABLE();   // PD12 13 14 15

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 84 - 1;     // psc,����Ԥ��Ƶֵ84��1MHz ʱ��,1us1��ʱ�����ڼ�2������TIM_CLOCKDIVISION_DIV2��
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 100 - 1;     // arr,40us��ARR-CCR=Delay
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1��ʱ�����ڼ�һ������Ҳ����
	  htim4.Init.RepetitionCounter = 0;   // �Ĵ���RCR
//	  HAL_TIM_Base_Init(&htim4);
	  
	  /* TIM4��ģʽ���� */
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2��MMSλ ��ģʽ���TRG0����Ϊ�����¼�
	  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
	
	  /* TIM4��ģʽ���� */
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR��SMSλ
	  sSlaveConfig.InputTrigger = TIM_TS_ITR1;    // SMCR��TSλ �Ӷ�ʱ��TIM4��ITR2��TIM3
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig);
	
		
		/* IC����,��ȡITR1(TRA) */
		TIM_IC_InitTypeDef sConfigIC_SH1;
		sConfigIC_SH1.ICFilter = 0;
		sConfigIC_SH1.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_SH1.ICSelection = TIM_ICSELECTION_TRC;  // TRA����ITR0
		HAL_TIM_IC_Init(&htim4);
		HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC_SH1, TIM_CHANNEL_1);
		HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
		
    /* SH1 ������-->ch1���� = GPIOB6 ��������Ƚ�ƥ����������ȷ�������ź� */
		TIM_OC_InitTypeDef sConfigOC_SH1 ; // ���岢��ʼ�����ýṹ��
		sConfigOC_SH1.OCMode = TIM_OCMODE_PWM2;     // PWMģʽѡ��(1��2),PWM2�����ϼ���ʱ��һ��TIMx_CNT >= TIMx_CCRʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ��
		sConfigOC_SH1.OCPolarity = TIM_OCPOLARITY_HIGH;  //��ЧֵΪ�ߵ�ƽ
		sConfigOC_SH1.OCFastMode = TIM_OCFAST_DISABLE;  // ���ÿ���ģʽ
		sConfigOC_SH1.Pulse = 69 - 1;   // TIMx_CCR
		HAL_TIM_OC_Init(&htim4);
		HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC_SH1, TIM_CHANNEL_1);
		HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
		
		/* TIM4��ʼ��Ϊ������ģʽ */
		HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE);

		/* GPIO���ã�PD12*/
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_12 ;                 /* GPIOD12����ΪTIM3-ch1 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* ����������� */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* ���������� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* ���� */
    gpio_init_struct.Alternate = GPIO_AF2_TIM4;                /* �˿ڸ��� */		
		HAL_GPIO_Init(GPIOD, &gpio_init_struct);
		
		__HAL_TIM_ENABLE(&htim4);
}


/* �Ӷ�ʱ�� TIM5 ���� -- SH2�ź� */
void TIM5_Config(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();  // ʹ�� TIM2 ʱ�ӣ�����APB1 = 84MHz
	  __HAL_RCC_GPIOA_CLK_ENABLE();   // PA0 1 2 3

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 84 - 1;     // psc,����Ԥ��Ƶֵ84��1MHz ʱ��,1us1��ʱ�����ڼ�2������TIM_CLOCKDIVISION_DIV2��
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 100 - 1;     // ARR
//	  htim5.Init.Period = 200 - 1;     // ARR
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1��ʱ�����ڼ�һ������Ҳ����
	  htim5.Init.RepetitionCounter = 0;   // �Ĵ���RCR
//		HAL_TIM_Base_Init(&htim5);
	  
	  /* TIM5��ģʽ���� */
	  TIM_MasterConfigTypeDef sMasterConfig;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2��MMSλ ��ģʽ���TRG0����Ϊ�����¼�
	  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);
	
	  /* TIM5��ģʽ���� */
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR��SMSλ
	  sSlaveConfig.InputTrigger = TIM_TS_ITR2;    // SMCR��TSλ �Ӷ�ʱ��TIM5��ITR2��TIM4��SH1��
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	  sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	  HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig);
	
		
		/* IC����,��ȡITR1(TRA) */
		TIM_IC_InitTypeDef sConfigIC_SH2;
		sConfigIC_SH2.ICFilter = 0;
		sConfigIC_SH2.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC_SH2.ICSelection = TIM_ICSELECTION_TRC;  // TRA����ITR0
		HAL_TIM_IC_Init(&htim5);
		HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC_SH2, TIM_CHANNEL_2);
		HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_2);
		
    /* SH2 ������-->ch4���� = GPIOA3 ��������Ƚ�ƥ����������ȷ�������ź� */
		TIM_OC_InitTypeDef sConfigOC_SH1 ; // ���岢��ʼ�����ýṹ��
		sConfigOC_SH1.OCMode = TIM_OCMODE_PWM2;     // PWMģʽѡ��(1��2),PWM1�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ��
		sConfigOC_SH1.OCPolarity = TIM_OCPOLARITY_HIGH;  //��ЧֵΪ�ߵ�ƽ
		sConfigOC_SH1.OCFastMode = TIM_OCFAST_DISABLE;  // ���ÿ���ģʽ
		sConfigOC_SH1.Pulse = 69 - 1;
//		sConfigOC_SH1.Pulse = 169 - 1;
		HAL_TIM_OC_Init(&htim5);
		HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC_SH1, TIM_CHANNEL_2);
		HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_2);
		
		/* TIM3��ʼ��Ϊ������ģʽ */
		HAL_TIM_OnePulse_Init(&htim5, TIM_OPMODE_SINGLE);

		/* GPIO����,PA1 */
		GPIO_InitTypeDef gpio_init_struct;		
		gpio_init_struct.Pin = GPIO_PIN_1  ;                 /* GPIOA3����ΪTIM5-ch4 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* ����������� */
    gpio_init_struct.Pull = GPIO_NOPULL;                                /* ���������� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* ���� */
    gpio_init_struct.Alternate = GPIO_AF2_TIM5;                /* �˿ڸ��� */		
		HAL_GPIO_Init(GPIOA, &gpio_init_struct);
		
		__HAL_TIM_ENABLE(&htim5);
		
}

/***************************************** ADC��������timer  TIM8CC1 *************************************************************/

/* TIM8 init function */
void MX_TIM8_Init(void)
{
	/*ADC����ʱ�ӣ�Ԥ��ƵPeriod��ARR����Pulse��CCR��������Ӧ������ ### CCRռ��Ϊ5/16 ###��
 *(1)PD����CLK=1MHzʱ,TIM8Ԥ��Ƶϵ�� = 21-1��8MHz��ARR = 16-1��CCR = 5-1;
 *(1)PD����CLK=1.25MHzʱ???????,TIM8Ԥ��Ƶϵ�� = 21-1��8MHz��ARR = 16-1��CCR = 5-1;
 */
	__HAL_RCC_TIM8_CLK_ENABLE();  // ʹ�� TIM8 ʱ�ӣ�����APB2 = 168MHz
	__HAL_RCC_GPIOC_CLK_ENABLE();   // PC6

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 1 - 1;     // psc,����Ԥ��Ƶֵ21��8MHz ʱ��
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 336 - 1;     // arr,40us��ARR-CCR=Delay
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   //1��ʱ�����ڼ�һ������Ҳ����
	htim8.Init.RepetitionCounter = 256-1;   // �Ĵ���RCR
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim8);
	
	/* TIM8��ģʽ���� */
	TIM_MasterConfigTypeDef sMasterConfig;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;  //CR2��MMSλ ��ģʽ���TRG0����Ϊ�����¼�
	HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

	/* TIM8��ģʽ���� */
	TIM_SlaveConfigTypeDef sSlaveConfig;
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;   // SMCR��SMSλ
//	sSlaveConfig.InputTrigger = TIM_TS_ITR1;    // SMCR��TSλ �Ӷ�ʱ��TIM8��ITR1��TIM2��SH3��
	sSlaveConfig.InputTrigger = TIM_TS_ITR2;    // SMCR��TSλ �Ӷ�ʱ��TIM8��ITR2��TIM4��SH1��
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ETRPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;   /*!< Input trigger filter   This parameter can be a number between Min_Data = 0x0 and Max_Data = 0xF  */
	HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig);

	
	/* IC����,��ȡITR1(TRA) */
	TIM_IC_InitTypeDef sConfigIC_ADC;
	sConfigIC_ADC.ICFilter = 0;
	sConfigIC_ADC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC_ADC.ICSelection = TIM_ICSELECTION_TRC;  // TRA����ITR0
	HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC_ADC, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_1);
	
	/* SH2 ������-->ch4���� = GPIOA3 ��������Ƚ�ƥ����������ȷ�������ź� */
	TIM_OC_InitTypeDef sConfigOC_ADC ; // ���岢��ʼ�����ýṹ��
	sConfigOC_ADC.OCMode = TIM_OCMODE_PWM2;     // PWMģʽѡ��(1��2),PWM1�����ϼ���ʱ��һ��TIMx_CNT<TIMx_CCR1ʱͨ��1Ϊ��Ч��ƽ������Ϊ��Ч��ƽ��
	sConfigOC_ADC.OCPolarity = TIM_OCPOLARITY_HIGH;  //��ЧֵΪ�ߵ�ƽ
	sConfigOC_ADC.OCFastMode = TIM_OCFAST_ENABLE;  // ���ÿ���ģʽ
	sConfigOC_ADC.Pulse = 288 - 1;
	HAL_TIM_OC_Init(&htim8);
	HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC_ADC, TIM_CHANNEL_1); 
	
	// ����ch1��PWM��
	HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

	/* GPIO����,PA1 */
	GPIO_InitTypeDef gpio_init_struct;		
	gpio_init_struct.Pin = GPIO_PIN_6  ;                 /* GPIOA3����ΪTIM5-ch4 */
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;                            /* ����������� */
	gpio_init_struct.Pull = GPIO_NOPULL;                                /* ���������� */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                      /* ���� */
	gpio_init_struct.Alternate = GPIO_AF3_TIM8;                /* �˿ڸ��� */		
	HAL_GPIO_Init(GPIOC, &gpio_init_struct);
	
	__HAL_TIM_ENABLE(&htim8);

}


