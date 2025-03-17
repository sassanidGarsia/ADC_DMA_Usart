/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
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
#include "dma.h"
#include "adc.h"
#include "usart.h"

/* USER CODE BEGIN 0 */
extern ADC_HandleTypeDef hadc1;

DMA_HandleTypeDef hdma_adc1;
//uint32_t dma_buffer;
extern uint32_t g_adc_dma_buf[256];   /* ADC DMA BUF 地址 */
extern volatile uint8_t g_adc_dma_sta;
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
	
	/* ADC1 DMA Init */
	hdma_adc1.Instance = DMA2_Stream0;
	hdma_adc1.Init.Channel = DMA_CHANNEL_0;
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc1.Init.Mode = DMA_NORMAL;
	hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);

  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	__HAL_DMA_ENABLE_IT(&hdma_adc1, DMA_IT_TC);  // 使能 DMA 传输完成中断
	
	__HAL_DMA_ENABLE(&hdma_adc1);
//	//  !!!!
//	HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&ADC1->DR, (uint32_t)&g_adc_dma_buf, sizeof(uint16_t));

}
void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  printf("Enter XferCpltCallback\r\n");  // 添加入口打印  
	if(hdma == &hdma_adc1)
	{
			printf("ADC1 DMA Complete\r\n");
			g_adc_dma_sta = 1;
			// 暂时注释掉Stop
			// HAL_ADC_Stop_DMA(&hadc1);
	}
	else
	{
			printf("Unknown DMA Complete\r\n");
	}
}


