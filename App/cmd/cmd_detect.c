/*
 * @Description: 
 * @Version: 2.0
 * @Date: 2023-03-21 23:47:18
 * @LastEditors: lkc
 * @detail: 
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum tagDetectType
{
    DETECT_NONE,
    DETECT_OFFSET,
    DETECT_NUM,
}DETECT_TYPE;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 检测
 * @detail: 
 * @param {ULONG} ulDetectType
 * @return {*}
 * @author: lkc
 */
VOID Cmd_Detect(ULONG ulDetectType)
{
    if (ulDetectType < 1)
    {
        PRINTF("Param less\n");
    }
    PRINTF("\n==================================================\n");
    PRINTF("Detect Start: ");

    switch (ulDetectType)
    {
        case DETECT_OFFSET:
            osDelay(100);
            PRINTF("Detect Offset\n");
            DETECT_Offset(FALSE);
            break;
        default:
            break;
    }
    PRINTF("Detect End\n");
    PRINTF("==================================================\n");

    return;
}


SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC),
detect, 
Cmd_Detect, 
detect 1 : current and volt offset 电流电压偏移
);

#ifdef __cplusplus
}
#endif
