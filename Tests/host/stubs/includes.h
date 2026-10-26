/* host 测试公共头：标准库与 flash_storage 依赖的最小包含。 */
#ifndef HOST_INCLUDES_H
#define HOST_INCLUDES_H
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "flash_storage.h"
/* Private macro -------------------------------------------------------------*/
#define PRINTF(...) ((void)0)
#ifdef ERROR
#undef ERROR
#endif
#define ERROR(...) ((void)0)
/**
 * @description: 主机测试替身：计算数据的 CRC-32C (Castagnoli) 校验值
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const uint8_t *} data 数据缓冲
 * @param {uint32_t} length 负载长度 (byte)
 * @return {uint32_t} * @author: lkc
 */
uint32_t utils_crc32c(const uint8_t *data, uint32_t length);
/**
 * @description: 主机测试替身：霍尔标定是否占用 FOC 中断路径
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} true 表示标定中，motor_it 走专用电流环
 */
bool Motor_Hall_DetectIsActive(void);
#endif
