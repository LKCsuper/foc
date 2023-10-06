/*
 * @Description: 
 * @Version: 1.0
 * @Author: lkc
 * @Date: 2023-10-06 17:43:41
 * @LastEditors: lkc
 * @LastEditTime: 2023-10-06 18:05:48
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
 * @description: 设置环境变量
 * @detail: 
 * @param {uint8_t} argc
 * @param {char} *
 * @return {*}
 * @author: lkc
 */
static int Cmd_Setenv(uint8_t argc, char **argv) {
    uint8_t i;
    if (argc > 3) {
        /* environment variable value string together */
        for (i = 0; i < argc - 2; i++) {
            argv[2 + i][strlen(argv[2 + i])] = ' ';
        }
    }
    if (argc == 1) {
        printf("Please input: setenv <key> [value]\n");
    } else if (argc == 2) {
        ef_set_env(argv[1], NULL);
    } else {
        ef_set_env(argv[1], argv[2]);
    }
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                setenv, Cmd_Setenv, setenv);

/**
 * @description: 打印环境变量
 * @detail: 
 * @param {uint8_t} argc
 * @param {char} *
 * @return {*}
 * @author: lkc
 */
static int Cmd_Printenv(uint8_t argc, char **argv) {
    ef_print_env();
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                printenv, Cmd_Printenv, printenv);

/**
 * @description: 保存环境变量
 * @detail: 
 * @param {uint8_t} argc
 * @param {char} *
 * @return {*}
 * @author: lkc
 */
static int Cmd_Saveenv(uint8_t argc, char **argv) {
    ef_save_env();
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                saveenv, Cmd_Saveenv, saveenv);

/**
 * @description: 获取值
 * @detail: 
 * @param {uint8_t} argc
 * @param {char} *
 * @return {*}
 * @author: lkc
 */
static int Cmd_Getvalue(uint8_t argc, char **argv) {
    char *value = NULL;
    value = ef_get_env(argv[1]);
    if (value) {
        printf("The %s value is %s.\n", argv[1], value);
    } else {
        printf("Can't find %s.\n", argv[1]);
    }
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                getvalue, Cmd_Getvalue, getvalue);

/**
 * @description: 复位环境变量
 * @detail: 
 * @param {uint8_t} argc
 * @param {char} *
 * @return {*}
 * @author: lkc
 */
static int Cmd_Resetenv(uint8_t argc, char **argv) 
{
    ef_env_set_default();

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                resetenv, Cmd_Resetenv, resetenv);
#ifdef __cplusplus
}
#endif