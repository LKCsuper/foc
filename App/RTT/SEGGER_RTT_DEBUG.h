/*
 * @Description: 项目 RTT 调试初始化接口；SEGGER 驱动实现保持在原供应商文件中。
 */
#ifndef __SEGGER_RTT_DEBUG_H__
#define __SEGGER_RTT_DEBUG_H__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 初始化项目 RTT 调试入口。
 * @return {void} 无
 */
VOID RTT_Debug_Init(VOID);
#ifdef __cplusplus
}
#endif
#endif
