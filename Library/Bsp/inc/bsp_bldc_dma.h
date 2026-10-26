/*
 * @Description: BLDC 模式 DMA 板级接口
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-04 16:39:50
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-04 16:41:31
 * @detail: 
 */
#ifndef __BSP_BLDC_DMA_H__
#define __BSP_BLDC_DMA_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: BLDC 模式 ADC DMA 初始化（DMA2 Stream4 循环搬运 ADC 转换结果）
 * @return {void} 无
 */
VOID Bsp_Dma_Bldc_Init(VOID)
{

    return;
}

#ifdef __cplusplus
}
#endif
#endif
