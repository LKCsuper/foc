/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-09-07 21:13:14
 * @LastEditors: lkc
 * @LastEditTime: 2023-09-07 21:14:45
 * @detail: 
 */
#ifndef _INCLUDES_H
#define _INCLUDES_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    OPEN_ENC_NONE,
    OPEN_ENC_DETECT,
    OPEN_ENC_NUM,
}OPEN_LOOP_TYPE;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 
 * @detail: 
 * @param {UCHAR} cTaskStat 任务状态
 * @param {UCHAR} cContStat 控制状态
 * @param {UCHAR} cPrecentage 百分比
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Motor_Encoder(int cTaskStat)
{
    /* 销毁和创建任务 */
    switch (cTaskStat)
    {
        case OPEN_ENC_DETECT:
        {
            
            break;
        }
        default:
            break;
    }

    return;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
                encoder, Cmd_Motor_Encoder, encoder task);
#ifdef __cplusplus
}
#endif
#endif
