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
#define ERROR(...) ((void)0)
uint32_t utils_crc32c(const uint8_t *data, uint32_t length);
bool Motor_Hall_DetectIsActive(void);
#endif
