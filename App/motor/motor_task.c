/*
 * @Description: 电机后台任务占位（原 ChibiOS/VESC 任务已废弃）
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
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: PID 控制后台任务入口。
 * @detail: 当前速度/电流环已在 Motor_DmaInt 中运行，本函数保留为空实现。
 * @return {*}
 * @author: lkc
 */
VOID Motor_PidControl(VOID)
{
    return;
}

#ifdef __cplusplus
}
#endif
