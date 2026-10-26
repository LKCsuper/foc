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
#include <stddef.h>
#include "motor.h"
#include "cmd_args.h"
/* Private typedef -----------------------------------------------------------*/

/**
 * @description: float 配置项命令的限幅方式。
 */
typedef enum {
    CMD_CONFIG_CLAMP_NONE = 0, /* 原值提交，由 Motor_SetConfig 校验 */
    CMD_CONFIG_CLAMP_MIN,      /* 低于 min 时取 min */
    CMD_CONFIG_CLAMP_ABS,      /* 取绝对值 */
} cmd_config_clamp_t;

/**
 * @description: 单个 float 配置项命令描述，配合 Cmd_ConfigFloat 使用。
 * @detail: show_format / set_format 各含一个浮点格式符，保持旧命令的输出文本以兼容 Scripts/ 解析。
 */
typedef struct {
    size_t offset;                       /* offsetof(motor_config_t, 字段)，字段须为 float */
    cmd_config_clamp_t clamp;            /* 限幅方式 */
    float min;                           /* CMD_CONFIG_CLAMP_MIN 的下限 */
    const char *show_format;             /* 无参数时的显示/用法文本 */
    const char *set_format;              /* 提交成功后的回显文本 */
    void (*adjust)(motor_config_t *cfg); /* 提交前联动修正其他字段，可为空 */
    void (*applied)(void);               /* 提交成功后的副作用（如复位速度环），可为空 */
} cmd_config_float_t;
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
 * @description: 单个 float 配置项命令的通用实现（显示 / 严格解析 / 限幅 / 整体提交 / 回显）。
 * @detail: 只能在 Shell 任务上下文调用；非法数字返回 -1 且不修改配置。
 * @param {const cmd_config_float_t *} spec 字段描述，不可为空
 * @param {int} argc 参数个数
 * @param {char *} argv[] 参数列表，argv[1] 为新值
 * @return {int} 0 成功，-1 参数或提交失败
 */
int Cmd_ConfigFloat(const cmd_config_float_t *spec, int argc, char *argv[]);

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
