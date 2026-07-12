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
    DETECT_HALL,
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
int Cmd_Detect(int argc, char *argv[])
{
    ULONG detect_type = DETECT_OFFSET;
    ULONG result = 0U;

    /*
     * Letter Shell 的 SHELL_TYPE_CMD_MAIN 会传入 argc/argv。旧代码把它导出为
     * SHELL_TYPE_CMD_FUNC，导致第一个参数被当成“检测类型”，出现随机的 51 并跳过了真正的校准。
     * 不带参数时默认检测电流/电压零点；传入 1 也表示同一个功能。
     */
    if (argc >= 2) {
        detect_type = (ULONG)atoi(argv[1]);
    }

    if (detect_type != DETECT_OFFSET && detect_type != DETECT_HALL) {
        PRINTF("Usage: detect [1=offset | 2=hall]\n");
        return -1;
    }

    PRINTF("\n==================================================\n");
    PRINTF("Detect Start: ");

    switch (detect_type)
    {
        case DETECT_OFFSET:
        {
            PRINTF("Detect Offset\n");
            result = DETECT_Offset(FALSE);
            break;
        }
        case DETECT_HALL:
        {
            PRINTF("Detect Hall\n");
            DETECT_Hall();
            break;
        }
        default:
            break;
    }

    PRINTF("Detect End\n");
    PRINTF("==================================================\n");

    return (result == 1U || detect_type == DETECT_HALL) ? 0 : -1;
}


SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
detect, 
Cmd_Detect, 
detect 1 : current and volt offset 电流电压偏移
);

#ifdef __cplusplus
}
#endif
