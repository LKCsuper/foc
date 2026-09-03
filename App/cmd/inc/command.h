/*
 * @Description: 精简命令行框架 — 替代 letter-shell
 * @Version: 2.0
 * @Author: lkc
 * @Date: 2025-07-13
 * @detail:
 *   静态命令表 + 最简 tokenizer + 串口回显，去掉 linker section 自动发现、
 *   历史记录、Tab 补全、用户认证等嵌入式调试用不上的功能，约 200 行。
 */
#ifndef _COMMAND_H
#define _COMMAND_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 命令函数统一签名：argc/argv 风格，返回 0 表示成功 */
typedef int (*cmd_func_t)(int argc, char *argv[]);

/* 命令表条目 */
typedef struct {
    const char *name;
    cmd_func_t  func;
    const char *desc;
} cmd_entry_t;

/**
 * @description: 初始化命令行框架。
 * @detail: 创建串口命令消息队列并启动命令处理任务。
 * @return {*}
 * @author: lkc
 */
void Command_Init(void);

typedef struct {
    ULONG len;
    UCHAR data[USART_RX_BUF_SIZE];
} CMD_QUEUE_ITEM;

extern QueueHandle_t queueCmdHandle;

#ifdef __cplusplus
}
#endif
#endif
