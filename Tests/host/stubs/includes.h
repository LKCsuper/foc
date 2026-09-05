#ifndef HOST_INCLUDES_H
#define HOST_INCLUDES_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "flash_storage.h"
#define PRINTF(...) ((void)0)
#ifdef ERROR
#undef ERROR
#endif
#define ERROR(...)        ((void)0)
#define FLASH_Sector_10   10
#define FLASH_Sector_11   11
#define FLASH_FLAG_EOP    1
#define FLASH_FLAG_OPERR  2
#define FLASH_FLAG_WRPERR 4
#define FLASH_FLAG_PGAERR 8
#define FLASH_FLAG_PGPERR 16
#define FLASH_FLAG_PGSERR 32
#define VoltageRange_3    3
#define REG8(addr)        (*(volatile uint8_t *)(uintptr_t)(addr))
typedef enum {
    FLASH_COMPLETE,
    FLASH_ERROR
} FLASH_Status;
void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_ClearFlag(uint32_t flags);
FLASH_Status FLASH_EraseSector(uint16_t sector, int voltage);
FLASH_Status FLASH_ProgramByte(uint32_t address, uint8_t value);
uint32_t utils_crc32c(const uint8_t *data, uint32_t length);
bool Motor_Hall_DetectIsActive(void);
#endif
