/*
 * @Description: Host 侧 Hal_Flash 实现（VirtualAlloc 模拟双槽）
 * @Author: lkc
 */
/* Includes ------------------------------------------------------------------*/
#include <windows.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal_flash.h"
/* Private define ------------------------------------------------------------*/
#define HOST_FLASH_SLOT_COUNT 2U
#define HOST_FLASH_SLOT_SIZE  (128U * 1024U)
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t *s_slot_base[HOST_FLASH_SLOT_COUNT];
static int s_writes_left = -1;
static int s_erases;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 返回已累计的擦除次数（测试辅助）
 * @return {int} 擦除次数
 * @author: lkc
 */
int Host_Flash_EraseCount(void)
{
    return s_erases;
}

/**
 * @description: 设置剩余可写字节数（-1 表示不限；0 立即失败）
 * @param {int} n  剩余写入次数
 * @return {*}
 * @author: lkc
 */
void Host_Flash_SetWritesLeft(int n)
{
    s_writes_left = n;
}

/**
 * @description: 在固定地址映射双槽模拟 Flash
 * @return {bool} true=映射成功
 * @author: lkc
 */
bool Host_Flash_MapSlots(void)
{
    void *mem;

    /* 与历史测试一致：在 0x080C0000 连续映射 256KB（两槽）。 */
    mem = VirtualAlloc((void *)(uintptr_t)0x080C0000U, HOST_FLASH_SLOT_SIZE * HOST_FLASH_SLOT_COUNT,
                       MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (mem != (void *)(uintptr_t)0x080C0000U) {
        return false;
    }
    s_slot_base[0] = (uint8_t *)mem;
    s_slot_base[1] = (uint8_t *)mem + HOST_FLASH_SLOT_SIZE;
    memset(mem, 0xff, HOST_FLASH_SLOT_SIZE * HOST_FLASH_SLOT_COUNT);
    s_erases = 0;
    s_writes_left = -1;
    return true;
}

/**
 * @description: 解除双槽映射
 * @return {bool} true=解除成功或本就未映射
 * @author: lkc
 */
bool Host_Flash_UnmapSlots(void)
{
    if (s_slot_base[0] == NULL) {
        return true;
    }
    if (!VirtualFree(s_slot_base[0], 0, MEM_RELEASE)) {
        return false;
    }
    s_slot_base[0] = NULL;
    s_slot_base[1] = NULL;
    return true;
}

/**
 * @description: 返回参数持久化可用的槽数量
 * @return {uint8_t} 槽个数；当前双槽方案为 2
 * @author: lkc
 */
uint8_t Hal_Flash_SlotCount(void)
{
    return (uint8_t)HOST_FLASH_SLOT_COUNT;
}

/**
 * @description: 查询指定槽的基址与容量
 * @param {uint8_t} index           槽下标，范围 [0, SlotCount)
 * @param {hal_flash_slot_t} *out   输出槽描述，不可为空
 * @return {bool} true=成功；false=下标非法或 out 为空
 * @author: lkc
 */
bool Hal_Flash_GetSlot(uint8_t index, hal_flash_slot_t *out)
{
    if (out == NULL || index >= HOST_FLASH_SLOT_COUNT || s_slot_base[index] == NULL) {
        return false;
    }
    out->base = (uintptr_t)s_slot_base[index];
    out->size = HOST_FLASH_SLOT_SIZE;
    return true;
}

/**
 * @description: 擦除整槽（擦后应为全 0xFF，具体由介质决定）
 * @param {uint8_t} index  槽下标
 * @return {bool} true=擦除成功
 * @author: lkc
 */
bool Hal_Flash_EraseSlot(uint8_t index)
{
    if (index >= HOST_FLASH_SLOT_COUNT || s_slot_base[index] == NULL) {
        return false;
    }
    memset(s_slot_base[index], 0xff, HOST_FLASH_SLOT_SIZE);
    ++s_erases;
    return true;
}

/**
 * @description: 向 Flash 地址编程一段数据并回读校验
 * @param {uintptr_t} addr         起始地址，须落在某槽范围内
 * @param {const uint8_t} *data    待写数据，不可为空（size>0 时）
 * @param {uint32_t} size          字节数
 * @return {bool} true=写入且回读一致
 * @author: lkc
 */
bool Hal_Flash_Program(uintptr_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)(uintptr_t)addr;

    if (size == 0U) {
        return true;
    }
    if (data == NULL || s_slot_base[0] == NULL) {
        return false;
    }

    for (i = 0; i < size; i++) {
        if (s_writes_left == 0) {
            return false;
        }
        if (s_writes_left > 0) {
            --s_writes_left;
        }
        p[i] &= data[i];
    }
    return true;
}
