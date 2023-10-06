/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-06 17:37:56
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-06 17:41:58
 * @detail: 
 */
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
 * @description: 重启设备
 * @detail: 
 * @return {*}
 * @author: lkc
 */
VOID CMD_Reset(VOID)
{
    Bsp_Misc_Reset();
    return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                reset, CMD_Reset, reset);
#ifdef __cplusplus
}
#endif