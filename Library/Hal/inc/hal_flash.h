/*
 * @Description: 参数 Flash 双槽擦写硬件抽象（平台无关契约）
 * @Author: lkc
 */
#ifndef MOTOR_HAL_FLASH_H
#define MOTOR_HAL_FLASH_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 单个参数存储槽的物理描述（基址与容量由移植层提供）
 */
typedef struct {
    uintptr_t base; /* 槽起始地址（字节寻址） */
    uint32_t size;  /* 槽容量 (byte) */
} hal_flash_slot_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 返回参数持久化可用的槽数量
 * @return {uint8_t} 槽个数；当前双槽方案为 2
 * @author: lkc
 */
uint8_t Hal_Flash_SlotCount(void);

/**
 * @description: 查询指定槽的基址与容量
 * @param {uint8_t} index           槽下标，范围 [0, SlotCount)
 * @param {hal_flash_slot_t} *out   输出槽描述，不可为空
 * @return {bool} true=成功；false=下标非法或 out 为空
 * @author: lkc
 */
bool Hal_Flash_GetSlot(uint8_t index, hal_flash_slot_t *out);

/**
 * @description: 擦除整槽（擦后应为全 0xFF，具体由介质决定）
 * @param {uint8_t} index  槽下标
 * @return {bool} true=擦除成功
 * @author: lkc
 */
bool Hal_Flash_EraseSlot(uint8_t index);

/**
 * @description: 向 Flash 地址编程一段数据并回读校验
 * @param {uintptr_t} addr         起始地址，须落在某槽范围内
 * @param {const uint8_t} *data    待写数据，不可为空（size>0 时）
 * @param {uint32_t} size          字节数
 * @return {bool} true=写入且回读一致
 * @author: lkc
 */
bool Hal_Flash_Program(uintptr_t addr, const uint8_t *data, uint32_t size);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
