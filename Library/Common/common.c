/*
 * @Description: 通用运行时辅助（断言失败停机处理）
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2022-11-19 09:57:21
 * @LastEditors: lkc
 * @LastEditTime: 2023-03-19 21:22:40
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
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 可能在串口和调度器初始化之前调用，不依赖日志、RTOS 或动态内存。
 * @return {void} 无
 */
void Common_AssertFailed(void)
{
    __disable_irq();
    for (;;) {
    }
}

#ifdef __cplusplus
}
#endif
