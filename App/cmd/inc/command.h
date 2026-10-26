/*
 * @Description: 精简命令行框架 — 替代 letter-shell
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2025-07-13
 * @detail:
 *   静态命令表 + 最简 tokenizer + 串口回显，去掉 linker section 自动发现、
 *   保留方向键命令历史，去掉 linker section 自动发现、Tab 补全和用户认证。
 */
#ifndef _COMMAND_H
#define _COMMAND_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "motor_config.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/

/* 命令函数统一签名：argc/argv 风格，返回 0 表示成功 */
typedef int (*cmd_func_t)(int argc, char *argv[]);

/* 命令表条目 */
typedef struct {
    const char *name;
    cmd_func_t  func;
    const char *desc;
} cmd_entry_t;

/**
 * @description: 把上电以来的时基格式化为 `[T+秒.毫秒s]`，便于截图对照命令间隔。
 * @param {char *} buf 输出缓冲，不可为空
 * @param {unsigned} buf_size 缓冲字节数，建议不少于 16
 * @return {void} 无
 */
void Cmd_FormatUptime(char *buf, unsigned buf_size);

#if MOTOR_CFG_CMD
/**
 * @description: 初始化命令行框架。
 * @detail: 创建串口命令消息队列并启动命令处理任务。
 * @return {void} 无
 * @author: lkc
 */
void Command_Init(void);
#else
/**
 * @description: 命令行关闭：不建队列、不建任务；串口接收中断看到空队列句柄后丢弃输入。
 * @return {void} 无
 */
static inline void Command_Init(void) {}
#endif

typedef struct {
    ULONG len;
    UCHAR data[USART_RX_BUF_SIZE];
} CMD_QUEUE_ITEM;

extern QueueHandle_t queueCmdHandle;

#ifdef __cplusplus
}
#endif
#endif
