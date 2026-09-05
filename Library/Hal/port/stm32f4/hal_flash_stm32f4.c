/*
 * @Description: STM32F4 参数 Flash 移植层（扇区 10/11 双槽）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#ifndef HAL_INTERNAL
#define HAL_INTERNAL
#endif
#include "includes.h"
#include "hal_flash.h"
/* Private define ------------------------------------------------------------*/
#define HAL_FLASH_SLOT_COUNT     (2U)
#define HAL_FLASH_SLOT_SIZE      (128U * 1024U)
#define HAL_FLASH_SLOT_A_ADDR    (0x080C0000U)
#define HAL_FLASH_SLOT_B_ADDR    (0x080E0000U)
#define HAL_FLASH_SLOT_A_SECTOR  FLASH_Sector_10
#define HAL_FLASH_SLOT_B_SECTOR  FLASH_Sector_11
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const struct {
    uint32_t addr;
    uint16_t sector;
} s_hal_flash_slots[HAL_FLASH_SLOT_COUNT] = {
    {HAL_FLASH_SLOT_A_ADDR, HAL_FLASH_SLOT_A_SECTOR},
    {HAL_FLASH_SLOT_B_ADDR, HAL_FLASH_SLOT_B_SECTOR},
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 返回参数持久化可用的槽数量。
 * @return {uint8_t} 固定为 2
 */
uint8_t Hal_Flash_SlotCount(void)
{
    return (uint8_t)HAL_FLASH_SLOT_COUNT;
}

/**
 * @description: 查询指定槽的基址与容量。
 * @param {uint8_t} index 槽下标
 * @param {hal_flash_slot_t *} out 输出
 * @return {bool} true=成功
 */
bool Hal_Flash_GetSlot(uint8_t index, hal_flash_slot_t *out)
{
    if (out == NULL || index >= HAL_FLASH_SLOT_COUNT) {
        return false;
    }
    out->base = (uintptr_t)s_hal_flash_slots[index].addr;
    out->size = HAL_FLASH_SLOT_SIZE;
    return true;
}

/**
 * @description: 擦除整槽对应扇区。
 * @param {uint8_t} index 槽下标
 * @return {bool} true=FLASH_COMPLETE
 */
bool Hal_Flash_EraseSlot(uint8_t index)
{
    FLASH_Status status;

    if (index >= HAL_FLASH_SLOT_COUNT) {
        return false;
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                    FLASH_FLAG_PGSERR);
    status = FLASH_EraseSector(s_hal_flash_slots[index].sector, VoltageRange_3);
    FLASH_Lock();
    return status == FLASH_COMPLETE;
}

/**
 * @description: 逐字节编程并回读校验。
 * @param {uintptr_t} addr 起始地址
 * @param {const uint8_t *} data 数据
 * @param {uint32_t} size 长度 (byte)
 * @return {bool} true=全部成功
 */
bool Hal_Flash_Program(uintptr_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    FLASH_Status status;
    uint32_t a = (uint32_t)addr;

    if (size == 0U) {
        return true;
    }
    if (data == NULL) {
        return false;
    }

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                    FLASH_FLAG_PGSERR);

    for (i = 0; i < size; i++, a++) {
        status = FLASH_ProgramByte(a, data[i]);
        if (status != FLASH_COMPLETE || REG8(a) != data[i]) {
            FLASH_Lock();
            return false;
        }
    }

    FLASH_Lock();
    return true;
}

#ifdef __cplusplus
}
#endif
