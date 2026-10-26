/*
 * @Description: 命令行参数严格解析（纯 C、无平台依赖，可在主机测试）
 * @Author: lkc
 * @detail: 替代 atof/atoi：整串必须是合法数字，否则返回 false，避免 "abc" 被静默当成 0 写入配置。
 */
#ifndef CMD_ARGS_H
#define CMD_ARGS_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/* Private function prototypes -----------------------------------------------*/

/**
 * @description: 把整个字符串解析为有限 float。
 * @detail: 允许前导空白（strtof 行为），不允许尾随字符；NaN/Inf 与溢出均拒绝。失败时不修改 *out。
 * @param {const char *} text 参数文本，可为空（返回 false）
 * @param {float *} out 输出值，不可为空
 * @return {bool} 解析成功返回 true
 */
static inline bool Cmd_ArgFloat(const char *text, float *out)
{
    char *end;
    float value;

    if (text == NULL || out == NULL || *text == '\0') return false;
    errno = 0;
    value = strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !isfinite(value)) return false;
    *out = value;
    return true;
}

/**
 * @description: 把整个字符串解析为 uint32（十进制，或 0x 前缀十六进制）。
 * @detail: 拒绝负号、尾随字符和超出 32 位的值。失败时不修改 *out。
 * @param {const char *} text 参数文本，可为空（返回 false）
 * @param {uint32_t *} out 输出值，不可为空
 * @return {bool} 解析成功返回 true
 */
static inline bool Cmd_ArgU32(const char *text, uint32_t *out)
{
    const char *p = text;
    char *end;
    unsigned long value;

    if (text == NULL || out == NULL) return false;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0' || *p == '-' || *p == '+') return false;
    errno = 0;
    value = strtoul(p, &end, 0);
    if (end == p || *end != '\0' || errno == ERANGE) return false;
#if ULONG_MAX > 0xFFFFFFFFUL
    if (value > 0xFFFFFFFFUL) return false; /* 主机 64 位 long；MCU 上 long 为 32 位，ERANGE 已覆盖 */
#endif
    *out = (uint32_t)value;
    return true;
}

#ifdef __cplusplus
}
#endif
#endif
