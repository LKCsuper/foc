/*
 * @Description: 偏移校准与霍尔检测命令
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
typedef enum tagDetectType {
    DETECT_NONE,
    DETECT_OFFSET,
    DETECT_HALL,
} DETECT_TYPE;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 打印 detect 命令用法
 * @return {*}
 * @author: lkc
 */
static void Detect_PrintUsage(void)
{
    PRINTF("Usage:\n");
    PRINTF("  detect | detect offset | detect 1\n");
    PRINTF("  detect hall | detect 2\n");
}

/**
 * @description: 解析 detect 子命令
 * @param {int} argc
 * @param {char} *argv[]
 * @return {DETECT_TYPE}
 * @author: lkc
 */
static DETECT_TYPE Detect_ParseType(int argc, char *argv[])
{
    if (argc < 2) {
        return DETECT_OFFSET;
    }
    if (strcmp(argv[1], "1") == 0 || strcmp(argv[1], "offset") == 0) {
        return DETECT_OFFSET;
    }
    if (strcmp(argv[1], "2") == 0 || strcmp(argv[1], "hall") == 0) {
        return DETECT_HALL;
    }
    return DETECT_NONE;
}

/**
 * @description: 电流/电压偏移校准，或霍尔表检测
 * @param {int} argc
 * @param {char} *argv[]
 * @return {int} 0 成功，-1 失败
 * @author: lkc
 */
int Cmd_Detect(int argc, char *argv[])
{
    DETECT_TYPE detect_type = Detect_ParseType(argc, argv);
    bool success = false;

    if (detect_type == DETECT_NONE) {
        Detect_PrintUsage();
        return -1;
    }

    PRINTF("\n==================================================\n");
    PRINTF("Detect Start\n");

    switch (detect_type) {
    case DETECT_OFFSET:
        success = DETECT_Offset(FALSE) == 1U;
        break;
    case DETECT_HALL:
        DETECT_Hall();
        success = true;
        break;
    default:
        break;
    }

    PRINTF("Detect End: %s\n", success ? "SUCCESS" : "FAILED");
    PRINTF("==================================================\n");

    return success ? 0 : -1;
}

/* Cmd_Detect 注册在 command.c 命令表中 */

#ifdef __cplusplus
}
#endif
