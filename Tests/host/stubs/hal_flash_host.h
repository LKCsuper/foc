/*
 * @Description: Host 侧 Hal_Flash 测试辅助声明
 * @Author: lkc
 */
#ifndef HOST_HAL_FLASH_HOST_H
#define HOST_HAL_FLASH_HOST_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool Host_Flash_MapSlots(void);
bool Host_Flash_UnmapSlots(void);
int Host_Flash_EraseCount(void);
void Host_Flash_SetWritesLeft(int n);

#ifdef __cplusplus
}
#endif
#endif
