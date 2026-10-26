/* 网页调试接入：只在任务上下文调用，不进入 FOC ISR。 */
#ifndef CMD_STUDIO_H
#define CMD_STUDIO_H
#include <stdbool.h>
#include "motor_config.h"
#if MOTOR_CFG_CMD
/**
 * @description: studio 命令入口 — 解析并执行网页调试子命令。
 * @param {int} argc       参数个数
 * @param {char *[]} argv  参数表；子命令在 argv[1]
 * @return {int} 0 成功，负数为 MOTOR_ERR_* 错误码
 */
int Cmd_Studio(int argc, char *argv[]);
/**
 * @description: 查询网页调试模式是否已开启。
 * @return {bool} true=已开启
 */
bool Cmd_StudioIsActive(void);
/**
 * @description: 查询网页端是否持有电机控制租约。
 * @return {bool} true=网页端持有控制权
 */
bool Cmd_StudioOwnsControl(void);
/**
 * @description: Plot 后台任务轮询：检查租约超时、消费目标并每 20 ms 发送遥测。
 * @return {void} 无
 */
void Cmd_StudioPoll(void);
/**
 * @description: 校验并写入网页目标帧到单槽邮箱，仅在命令任务上下文调用。
 * @param {const char *} line  目标帧文本
 * @return {void} 无
 */
void Cmd_StudioTargetLine(const char *line);
/**
 * @description: 确保串口绘图后台任务存在，供 studio 复用其采样节拍。
 * @return {bool} true=任务已存在或创建成功
 */
bool Cmd_SerialPlotEnsureTask(void);
#else
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @return {bool} 本实现固定返回 false
 */
static inline bool Cmd_StudioIsActive(void) { return false; }
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @return {bool} 本实现固定返回 false
 */
static inline bool Cmd_StudioOwnsControl(void) { return false; }
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @return {void} 无
 */
static inline void Cmd_StudioPoll(void) {}
#endif
#endif
