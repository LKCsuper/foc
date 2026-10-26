/*
 * @Description: Flash 参数存储相关 Shell 命令（替代 EasyFlash 版本）
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-06 17:43:41
 * @LastEditors: lkc
 * @LastEditTime: 2025-07-13
 * @detail: 基于 flash_storage v2 固定配置/校准格式，不保存运行状态
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#if MOTOR_CFG_CMD
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 保存当前电机参数到 Flash
 * @detail: 写入 Sector 10/11 A/B 槽，无需先 setenv 再 saveenv
 * @param {uint8_t} argc argc，取值约定见函数职责
 * @param {char} *
 * @param {char **} argv 命令参数字符串数组
 * @return {int} 本次计算或查询的结果
 * @author: lkc
 */
int Cmd_Saveparam(int argc, char **argv)
{
    if (motor.ol_enable || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive()) {
        printf("Stop motor and finish calibration before saving.\r\n");
        return -1;
    }

    if (FlashStorage_Save(&motor)) {
        printf("Parameters saved to Flash A/B slots.\r\n");
    } else {
        printf("Save failed!\r\n");
    }
    return 0;
}
/* Cmd_Saveparam 注册在 command.c 命令表中 */
#endif /* MOTOR_CFG_CMD */

#ifdef __cplusplus
}
#endif
