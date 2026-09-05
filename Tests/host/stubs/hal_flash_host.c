/*
 * @Description: Host 侧 Hal_Flash 实现（VirtualAlloc 模拟双槽）
 * @Author: lkc
 */
#include <windows.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal_flash.h"

#define HOST_FLASH_SLOT_COUNT 2U
#define HOST_FLASH_SLOT_SIZE  (128U * 1024U)

static uint8_t *s_slot_base[HOST_FLASH_SLOT_COUNT];
static int s_writes_left = -1;
static int s_erases;

int Host_Flash_EraseCount(void)
{
    return s_erases;
}

void Host_Flash_SetWritesLeft(int n)
{
    s_writes_left = n;
}

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

uint8_t Hal_Flash_SlotCount(void)
{
    return (uint8_t)HOST_FLASH_SLOT_COUNT;
}

bool Hal_Flash_GetSlot(uint8_t index, hal_flash_slot_t *out)
{
    if (out == NULL || index >= HOST_FLASH_SLOT_COUNT || s_slot_base[index] == NULL) {
        return false;
    }
    out->base = (uintptr_t)s_slot_base[index];
    out->size = HOST_FLASH_SLOT_SIZE;
    return true;
}

bool Hal_Flash_EraseSlot(uint8_t index)
{
    if (index >= HOST_FLASH_SLOT_COUNT || s_slot_base[index] == NULL) {
        return false;
    }
    memset(s_slot_base[index], 0xff, HOST_FLASH_SLOT_SIZE);
    ++s_erases;
    return true;
}

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
