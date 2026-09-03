/*
 * @Description: AB 编码器 Shell 命令
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
 * @description: 打印编码器计数值、机械角和电角度
 * @return {*}
 * @author: lkc
 */
static void Encoder_PrintStatus(void)
{
    PRINTF("Encoder count=%u / %u\r\n",
           (unsigned)Motor_Encoder_GetCount(),
           (unsigned)BSP_ENCODER_COUNTS);
    PRINTF("  mech=%.3f rad (%.1f deg)\r\n",
           Motor_Encoder_GetMechAngle(),
           Motor_Encoder_GetMechAngle() * 180.0f / (float)M_PI);
    PRINTF("  elec=%.3f rad (%.1f deg)\r\n",
           Motor_Encoder_GetElecAngle(),
           Motor_Encoder_GetElecAngle() * 180.0f / (float)M_PI);
}

/**
 * @description: AB 编码器：show / reset / detect
 * @param {int} argc
 * @param {char} *argv[]
 * @return {int} 0 成功，-1 失败
 * @author: lkc
 */
int Cmd_Motor_Encoder(int argc, char *argv[])
{
    if (argc < 2 || strcmp(argv[1], "show") == 0) {
        Encoder_PrintStatus();
        return 0;
    }
    if (strcmp(argv[1], "reset") == 0) {
        Motor_Encoder_Reset();
        PRINTF("Encoder count cleared\r\n");
        Encoder_PrintStatus();
        return 0;
    }
    if (strcmp(argv[1], "detect") == 0) {
        Motor_Encoder_Detect();
        return 0;
    }

    PRINTF("usage: encoder [show|reset|detect]\r\n");
    return -1;
}

/* Cmd_Motor_Encoder 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
