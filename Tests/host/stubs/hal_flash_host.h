/*
 * @Description: Host 侧 Hal_Flash 测试辅助声明
 * @Author: lkc
 */
#ifndef HOST_HAL_FLASH_HOST_H
#define HOST_HAL_FLASH_HOST_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

/**
 * @description: 主机测试替身：在主机进程中映射模拟 Flash 槽，供真实存储代码读写。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本次处理的判定结果，具体条件见职责与实现说明
 */
bool Host_Flash_MapSlots(void);
/**
 * @description: 主机测试替身：解除主机模拟 Flash 槽的映射。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本次处理的判定结果，具体条件见职责与实现说明
 */
bool Host_Flash_UnmapSlots(void);
/**
 * @description: 主机测试替身：查询模拟 Flash 的累计擦除次数。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {int} 本次计算或查询的结果
 */
int Host_Flash_EraseCount(void);
/**
 * @description: 主机测试替身：设置模拟 Flash 剩余可写次数，用于注入写入中断。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {int} n n 参数
 * @return {void} 无
 */
void Host_Flash_SetWritesLeft(int n);

#ifdef __cplusplus
}
#endif
#endif
