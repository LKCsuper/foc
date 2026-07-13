/*
 * @Description: Flash 参数存储相关 Shell 命令（替代 EasyFlash 版本）
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-06 17:43:41
 * @LastEditors: lkc
 * @LastEditTime: 2025-07-13
 * @detail: 基于 flash_storage 直接结构体读写，不再依赖 EasyFlash
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
 * @description: 保存当前电机参数和检测结果到 Flash
 * @detail: 写入 Sector 10/11 A/B 槽，无需先 setenv 再 saveenv
 * @param {uint8_t} argc
 * @param {char} *
 * @return {*}
 * @author: lkc
 */
int Cmd_Saveparam(int argc, char **argv)
{
    motor_detect_result_t detect;

    DETECT_GetResult(&detect);

    if (FlashStorage_Save(&motor, &detect)) {
        printf("Parameters saved to Flash A/B slots.\r\n");
    } else {
        printf("Save failed!\r\n");
    }
    return 0;
}
/* Cmd_Saveparam 注册在 command.c 命令表中 */

#ifdef __cplusplus
}
#endif
