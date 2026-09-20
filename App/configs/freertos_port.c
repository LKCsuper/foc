/*
 * @Description: FreeRTOS 运行时统计计时器移植层
 * @Author: lkc
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
 * @description: 配置 FreeRTOS 运行时统计计时器并清零计数。
 * @return {void} 无
 * @author: lkc
 */
void configureTimerForRunTimeStats(void)
{
	ulFreeRTOSRunTimeTicks = 0;
	Bsp_Tim_RunCount();
}

/**
 * @description: 读取 FreeRTOS 运行时统计 tick 计数器。
 * @return {unsigned long} 累计运行时 tick 数
 * @author: lkc
 */
unsigned long getRunTimeCounterValue(void)
{
	return ulFreeRTOSRunTimeTicks;
}

#ifdef __cplusplus
}
#endif
