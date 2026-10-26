/*
 * @Description: 速度环命令共享接口：测试序列中的按键中止检测
 * @Author: lkc
 */
#ifndef CMD_SPEED_H
#define CMD_SPEED_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_config.h"
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 测试过程中是否收到任意按键；收到后清空剩余输入。
 * @detail: 仅在命令任务占用测试序列时调用，非阻塞。
 * @return {bool} true 表示用户要求停止
 */
#if MOTOR_CFG_CMD_SPEED
/**
 * @description: 测试过程中是否收到任意按键。
 * @return {bool} true=要停止
 */
bool Cmd_Speed_StopKey(void);
#else
/**
 * @description: 模块关闭时的兼容空实现；不执行对应功能。
 * @return {bool} 本实现固定返回 false
 */
static inline bool Cmd_Speed_StopKey(void) { return false; }
#endif

#ifdef __cplusplus
}
#endif
#endif
