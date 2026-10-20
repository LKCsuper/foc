/*
 * @Description: 开环命令共享接口：配置快照、提交结果提示与状态页
 * @Author: lkc
 * @detail: 供无感、速度环等命令文件复用，只能在 Shell 任务上下文调用。
 */
#ifndef CMD_OPENLOOP_H
#define CMD_OPENLOOP_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor.h"
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 捕获当前配置快照，供命令修改后整体提交。
 * @param {motor_config_t *} cfg 输出配置
 * @return {bool} 捕获成功返回 true；cfg 为空或读取失败返回 false
 */
bool Cmd_Ol_CaptureConfig(motor_config_t *cfg);

/**
 * @description: 按配置提交结果打印失败原因；成功时不输出。
 * @param {motor_result_t} result 配置提交返回值
 * @return {void} 无
 */
void Cmd_Ol_PrintConfigResult(motor_result_t result);

/**
 * @description: 格式化并输出开环、电流环、无感与保护状态页。
 * @detail: 使用共享静态缓冲区，已有输出进行中时直接返回。
 * @return {void} 无
 */
void Cmd_Ol_PrintStatus(void);

#ifdef __cplusplus
}
#endif
#endif
