/*
 * @Description: 电机参数双槽 Flash 持久化
 * @Author: lkc
 */
#ifndef _FLASH_STORAGE_H_
#define _FLASH_STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
#include "motor_detect.h"
#include "motor_settings.h"

/* The linker application region ends at 0x080C0000. Sectors 10 and 11 are
 * reserved as two independent parameter slots so an interrupted save leaves
 * the previous slot intact. */
#define FLASH_STORAGE_SLOT_A_ADDR    0x080C0000U
#define FLASH_STORAGE_SLOT_B_ADDR    0x080E0000U
#define FLASH_STORAGE_MAGIC          0x464C4B43U
#define FLASH_STORAGE_FORMAT_VERSION 2U
#define FLASH_STORAGE_COMMITTED      0xA55AA55AU

/* v1 整体 MOTOR_PARAM_S 格式不迁移：拒绝加载，使用默认配置并重新校准。 */
typedef struct {
    uint32_t magic;
    uint32_t format_version;
    uint32_t sequence;
    uint32_t crc32;
    uint32_t payload_size;
    uint8_t payload[MOTOR_SETTINGS_WIRE_SIZE];
    uint32_t committed;
} flash_storage_t;

/**
 * @description: 从最新有效 Flash 槽加载电机参数与检测结果。
 * @param {MOTOR_PARAM_S} *motor              输出配置/校准；必须已初始化且停机，不改命令和实时状态
 * @param {motor_detect_result_t} *detect     兼容输出，清零并设置结果版本；v2 不保存已停用的 R/L/磁链结果
 * @return {bool} true=加载成功；false=无有效槽，调用方应使用默认参数
 * @author: lkc
 */
bool FlashStorage_Load(MOTOR_PARAM_S *motor, motor_detect_result_t *detect);

/**
 * @description: 将电机参数与检测结果保存到当前未使用的 Flash 槽。
 * @param {const MOTOR_PARAM_S} *motor              待保存配置/校准；必须停机且没有校准占用
 * @param {const motor_detect_result_t} *detect     兼容参数；v2 不保存此对象
 * @return {bool} true=保存并校验成功
 * @author: lkc
 */
bool FlashStorage_Save(const MOTOR_PARAM_S *motor, const motor_detect_result_t *detect);

#ifdef __cplusplus
}
#endif

#endif
