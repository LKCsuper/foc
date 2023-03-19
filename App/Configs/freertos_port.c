/*
 * @Description: 
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-11 00:07:31
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ULONG ulFreeRTOSRunTimeTicks = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 配置运行统计
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void configureTimerForRunTimeStats(void)
{
	ulFreeRTOSRunTimeTicks = 0;
	Bsp_Tim_RunCount();
}

/**
 * @description: 获取freertos运行count
 * @detail: 
 * @return {*}
 * @author: lkc
 */
unsigned long getRunTimeCounterValue(void)
{
	return ulFreeRTOSRunTimeTicks;
}

#ifdef __cplusplus
}
#endif
