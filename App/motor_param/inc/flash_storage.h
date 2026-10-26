/*
 * @Description: 电机参数双槽 Flash 持久化
 * @Author: lkc
 */
#ifndef _FLASH_STORAGE_H_
#define _FLASH_STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"
#include "motor_settings.h"
#include "motor_anticogging.h"
/* Private define ------------------------------------------------------------*/

/* 槽物理地址由 Hal_Flash 提供；本头仅约定逻辑存储格式。 */
#define FLASH_STORAGE_MAGIC          0x464C4B43U
#define FLASH_STORAGE_FORMAT_VERSION    4U
#define FLASH_STORAGE_FORMAT_VERSION_V3 3U
#define FLASH_STORAGE_FORMAT_VERSION_V2 2U
#define FLASH_STORAGE_COMMITTED      0xA55AA55AU
#define FLASH_STORAGE_ANTICOG_MAGIC   0x434F4731U
#define FLASH_STORAGE_ANTICOG_SIZE    (8U + MOTOR_ANTICOGGING_BINS * 4U)
#define FLASH_STORAGE_PAYLOAD_SIZE    (MOTOR_SETTINGS_WIRE_SIZE + FLASH_STORAGE_ANTICOG_SIZE)

/* Private typedef -----------------------------------------------------------*/
/* v1 整体 MOTOR_PARAM_S 格式不迁移：拒绝加载，使用默认配置并重新校准。 */
typedef struct {
    uint32_t magic;
    uint32_t format_version;
    uint32_t sequence;
    uint32_t crc32;
    uint32_t payload_size;
    uint8_t payload[FLASH_STORAGE_PAYLOAD_SIZE];
    uint32_t committed;
} flash_storage_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 从最新有效 Flash 槽加载电机配置与校准。
 * @param {MOTOR_PARAM_S} *motor              输出配置/校准；必须已初始化且停机，不改命令和实时状态
 * @return {bool} true=加载成功；false=无有效槽，调用方应使用默认参数
 * @author: lkc
 */
bool FlashStorage_Load(MOTOR_PARAM_S *motor);

/**
 * @description: 将电机配置与校准保存到当前未使用的 Flash 槽。
 * @param {const MOTOR_PARAM_S} *motor              待保存配置/校准；必须停机且没有校准占用
 * @return {bool} true=保存并校验成功
 * @author: lkc
 */
bool FlashStorage_Save(const MOTOR_PARAM_S *motor);

#ifdef __cplusplus
}
#endif

#endif
