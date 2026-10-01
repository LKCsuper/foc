/*
 * @Description: 配置与校准 Flash v2 双槽持久化（擦写经 Hal_Flash）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "flash_storage.h"
#include "hal_flash.h"
#include "motor_detect.h"
#include "motor_hall.h"
#include "motor_math.h"
#include "motor_anticogging.h"
/* Private define ------------------------------------------------------------*/
#ifndef FLASH_STORAGE_PRINTF
#define FLASH_STORAGE_PRINTF(...) ((void)0)
#endif
#ifndef FLASH_STORAGE_ERROR
#define FLASH_STORAGE_ERROR(...) ((void)0)
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static bool flash_storage_sequence_newer(uint32_t lhs, uint32_t rhs);
static bool flash_storage_valid(const flash_storage_t *storage);
static bool flash_storage_get_slot(uint8_t index, hal_flash_slot_t *slot);
static const flash_storage_t *flash_storage_current(uint8_t *out_index);
static bool flash_storage_anticog_decode(const uint8_t *payload, float *table, bool *enabled, bool *calibrated);
/* Private functions ---------------------------------------------------------*/
static flash_storage_t s_write_storage;
static float s_anticogging_table[MOTOR_ANTICOGGING_BINS];

/**
 * @description: 按小端读取 16 位无符号数。
 * @param {const uint8_t} *p  至少 2 字节，不可为空
 * @return {uint16_t} 读出的值
 * @author: lkc
 */
static uint16_t flash_get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * @description: 按小端读取 32 位无符号数。
 * @param {const uint8_t} *p  至少 4 字节，不可为空
 * @return {uint32_t} 读出的值
 * @author: lkc
 */
static uint32_t flash_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
 * @description: 按小端写入 16 位无符号数。
 * @param {uint8_t} *p     至少 2 字节，不可为空
 * @param {uint16_t} value 待写入的值
 * @return {void} 无
 * @author: lkc
 */
static void flash_put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

/**
 * @description: 按小端写入 32 位无符号数。
 * @param {uint8_t} *p     至少 4 字节，不可为空
 * @param {uint32_t} value 待写入的值
 * @return {void} 无
 * @author: lkc
 */
static void flash_put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

/**
 * @description: 从参数载荷尾部解出抗齿槽表。
 * @detail: 魔数、桶数须匹配。使能但未标定、或任一点非有限且超出 ±0.5 V 时拒绝。
 *          table、enabled、calibrated 可分别传空，空指针只跳过对应输出。
 * @param {const uint8_t} *payload  参数区起点，其后为抗齿槽块，不可为空
 * @param {float} *table            输出补偿电压 (V)，可为空
 * @param {bool} *enabled           输出使能，可为空
 * @param {bool} *calibrated        输出是否已标定，可为空
 * @return {bool} true=块格式和数值都合法
 * @author: lkc
 */
static bool flash_storage_anticog_decode(const uint8_t *payload, float *table, bool *enabled, bool *calibrated)
{
    const uint8_t *p = payload + MOTOR_SETTINGS_WIRE_SIZE;
    uint16_t i;
    if (flash_get_u32(p) != FLASH_STORAGE_ANTICOG_MAGIC ||
        flash_get_u16(p + 4U) != MOTOR_ANTICOGGING_BINS || p[6] > 1U || p[7] > 1U || (p[6] != 0U && p[7] == 0U)) {
        return false;
    }
    if (enabled != NULL) *enabled = p[6] != 0U;
    if (calibrated != NULL) *calibrated = p[7] != 0U;
    p += 8U;
    for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++, p += 4U) {
        uint32_t bits = flash_get_u32(p);
        float value;
        memcpy(&value, &bits, sizeof(value));
        if (!isfinite(value) || value < -0.5f || value > 0.5f) return false;
        if (table != NULL) table[i] = value;
    }
    return true;
}

/**
 * @description: 按 32 位序号比较新旧，lhs 比 rhs 新则返回 true。
 * @detail: 采用有符号差值 (int32_t)(lhs-rhs) 判断，可正确处理序号回绕。
 * @param {uint32_t} lhs 一侧序号
 * @param {uint32_t} rhs 另一侧序号
 * @return {bool} true=lhs 更新
 * @author: lkc
 */
static bool flash_storage_sequence_newer(uint32_t lhs, uint32_t rhs)
{
    return (int32_t)(lhs - rhs) > 0;
}

/**
 * @description: 查询 Hal 槽；失败返回 false。
 * @param {uint8_t} index 槽下标（0 或 1）
 * @param {hal_flash_slot_t *} slot 输出槽描述，不可为空
 * @return {bool} 查询成功返回 true
 * @author: lkc
 */
static bool flash_storage_get_slot(uint8_t index, hal_flash_slot_t *slot)
{
    return Hal_Flash_GetSlot(index, slot);
}

/**
 * @description: 校验 Flash 槽数据是否有效。
 * @detail: magic/格式版本/长度/提交标记匹配，且固定 payload 的 CRC32 正确。
 * @param {const flash_storage_t *} storage Flash 槽首地址，不可为空
 * @return {bool} true=槽内容有效
 * @author: lkc
 */
static bool flash_storage_valid(const flash_storage_t *storage)
{
    uint32_t crc;
    uint32_t committed;
    uint32_t expected_size;
    motor_settings_t settings;

    expected_size = storage->format_version == FLASH_STORAGE_FORMAT_VERSION ? FLASH_STORAGE_PAYLOAD_SIZE :
                    storage->format_version == FLASH_STORAGE_FORMAT_VERSION_V3 ? MOTOR_SETTINGS_WIRE_SIZE :
                    storage->format_version == FLASH_STORAGE_FORMAT_VERSION_V2 ? MOTOR_SETTINGS_WIRE_SIZE_V2 : 0U;
    if (storage->magic != FLASH_STORAGE_MAGIC || storage->payload_size != expected_size || expected_size == 0U) {
        return false;
    }
    memcpy(&committed, storage->payload + storage->payload_size, sizeof(committed));
    if (committed != FLASH_STORAGE_COMMITTED) return false;

    crc = utils_crc32c((uint8_t *)storage->payload, storage->payload_size);
    if (crc != storage->crc32 ||
        !Motor_Settings_Decode(storage->payload,
                               storage->format_version == FLASH_STORAGE_FORMAT_VERSION_V2 ?
                                   MOTOR_SETTINGS_WIRE_SIZE_V2 : MOTOR_SETTINGS_WIRE_SIZE,
                               &settings)) {
        return false;
    }
    return storage->format_version != FLASH_STORAGE_FORMAT_VERSION ||
           flash_storage_anticog_decode(storage->payload, NULL, NULL, NULL);
}

/**
 * @description: 返回双槽中序号最新且校验有效的一侧。
 * @param {uint8_t *} out_index 可选，输出当前槽下标；两侧无效时不写
 * @return {const flash_storage_t *} 有效槽指针；两侧都无效时返回 NULL
 * @author: lkc
 */
static const flash_storage_t *flash_storage_current(uint8_t *out_index)
{
    hal_flash_slot_t slot_a;
    hal_flash_slot_t slot_b;
    const flash_storage_t *a;
    const flash_storage_t *b;
    bool a_valid;
    bool b_valid;

    if (!flash_storage_get_slot(0U, &slot_a) || !flash_storage_get_slot(1U, &slot_b)) {
        return NULL;
    }

    a = (const flash_storage_t *)(uintptr_t)slot_a.base;
    b = (const flash_storage_t *)(uintptr_t)slot_b.base;
    a_valid = flash_storage_valid(a);
    b_valid = flash_storage_valid(b);

    if (a_valid && b_valid) {
        if (flash_storage_sequence_newer(b->sequence, a->sequence)) {
            if (out_index != NULL) {
                *out_index = 1U;
            }
            return b;
        }
        if (out_index != NULL) {
            *out_index = 0U;
        }
        return a;
    }
    if (a_valid) {
        if (out_index != NULL) {
            *out_index = 0U;
        }
        return a;
    }
    if (b_valid) {
        if (out_index != NULL) {
            *out_index = 1U;
        }
        return b;
    }
    return NULL;
}

/**
 * @description: 从最新有效槽加载配置/校准。
 * @detail: 无有效槽时返回 false，由调用方回退默认参数。
 * @param {MOTOR_PARAM_S *} motor 目标电机对象，须停机中，不可为空
 * @return {bool} 加载并应用成功返回 true
 * @author: lkc
 */
bool FlashStorage_Load(MOTOR_PARAM_S *motor)
{
    const flash_storage_t *storage;
    motor_settings_t settings;

    if (motor == NULL || motor->ol_enable) {
        return false;
    }
    storage = flash_storage_current(NULL);
    if (storage == NULL) {
        FLASH_STORAGE_PRINTF("FlashStorage: no valid slot, using defaults\r\n");
        return false;
    }

    if (!Motor_Settings_Decode(storage->payload,
                               storage->format_version == FLASH_STORAGE_FORMAT_VERSION_V2 ?
                                   MOTOR_SETTINGS_WIRE_SIZE_V2 : MOTOR_SETTINGS_WIRE_SIZE,
                               &settings) ||
        !Motor_Settings_Apply(motor, &settings)) {
        return false;
    }
    Motor_Anticogging_Clear();
    if (storage->format_version == FLASH_STORAGE_FORMAT_VERSION) {
        bool enabled;
        bool calibrated;
        if (!flash_storage_anticog_decode(storage->payload, s_anticogging_table, &enabled, &calibrated)) {
            return false;
        }
        if (calibrated && !Motor_Anticogging_Import(s_anticogging_table, MOTOR_ANTICOGGING_BINS, enabled)) return false;
    }
    FLASH_STORAGE_PRINTF("FlashStorage: loaded slot sequence %u\r\n", storage->sequence);
    return true;
}

/**
 * @description: 把配置/校准保存到当前未使用的一侧槽。
 * @detail: 双槽交替写入；先编码再擦除；主体写完后写 committed；写毕回读整体校验。
 * @param {const MOTOR_PARAM_S *} motor 来源电机对象，须停机中，不可为空
 * @return {bool} 保存并通过回读校验返回 true
 * @author: lkc
 */
bool FlashStorage_Save(const MOTOR_PARAM_S *motor)
{
    const flash_storage_t *current;
    flash_storage_t *storage = &s_write_storage;
    motor_settings_t settings;
    hal_flash_slot_t target_slot;
    uint8_t current_index;
    uint8_t target_index;
    uint32_t target_addr;
    uint32_t committed_offset = (uint32_t)offsetof(flash_storage_t, payload) + FLASH_STORAGE_PAYLOAD_SIZE;
    const uint32_t committed = FLASH_STORAGE_COMMITTED;

    if (motor == NULL || motor->ol_enable || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive()) {
        return false;
    }
    if (Hal_Flash_SlotCount() < 2U) {
        return false;
    }
    if (!Hal_Flash_GetSlot(0U, &target_slot) || committed_offset + sizeof(uint32_t) > target_slot.size) {
        return false;
    }

    /* 先验证并编码，非法参数不能触发擦除。调用方必须串行调用存储接口。 */
    if (!Motor_Settings_Capture(motor, &settings)) {
        return false;
    }
    memset(storage, 0xFF, sizeof(*storage));
    if (!Motor_Settings_Encode(&settings, storage->payload, MOTOR_SETTINGS_WIRE_SIZE)) {
        return false;
    }
    {
        uint8_t *p = storage->payload + MOTOR_SETTINGS_WIRE_SIZE;
        bool enabled = false;
        uint16_t i;
        bool calibrated = Motor_Anticogging_Export(s_anticogging_table, MOTOR_ANTICOGGING_BINS, &enabled);
        if (!calibrated) {
            memset(s_anticogging_table, 0, sizeof(s_anticogging_table));
            enabled = false;
        }
        flash_put_u32(p, FLASH_STORAGE_ANTICOG_MAGIC);
        flash_put_u16(p + 4U, MOTOR_ANTICOGGING_BINS);
        p[6] = enabled ? 1U : 0U;
        p[7] = calibrated ? 1U : 0U;
        p += 8U;
        for (i = 0U; i < MOTOR_ANTICOGGING_BINS; i++, p += 4U) {
            uint32_t bits;
            memcpy(&bits, &s_anticogging_table[i], sizeof(bits));
            flash_put_u32(p, bits);
        }
    }

    current = flash_storage_current(&current_index);
    if (current != NULL) {
        target_index = (uint8_t)(1U - current_index);
    } else {
        target_index = 0U;
    }
    if (!Hal_Flash_GetSlot(target_index, &target_slot)) {
        return false;
    }
    target_addr = (uint32_t)target_slot.base;

    storage->magic = FLASH_STORAGE_MAGIC;
    storage->format_version = FLASH_STORAGE_FORMAT_VERSION;
    storage->sequence = current == NULL ? 1U : current->sequence + 1U;
    storage->payload_size = FLASH_STORAGE_PAYLOAD_SIZE;
    storage->crc32 = utils_crc32c(storage->payload, FLASH_STORAGE_PAYLOAD_SIZE);

    if (!Hal_Flash_EraseSlot(target_index) ||
        !Hal_Flash_Program((uintptr_t)target_addr, (const uint8_t *)storage, committed_offset) ||
        !Hal_Flash_Program((uintptr_t)(target_addr + committed_offset), (const uint8_t *)&committed,
                           sizeof(uint32_t))) {
        FLASH_STORAGE_ERROR("FlashStorage: erase/program failed\r\n");
        return false;
    }

    if (!flash_storage_valid((const flash_storage_t *)(uintptr_t)target_addr)) {
        FLASH_STORAGE_ERROR("FlashStorage: final verification failed\r\n");
        return false;
    }

    FLASH_STORAGE_PRINTF("FlashStorage: saved slot sequence %u\r\n", storage->sequence);
    return true;
}

#ifdef __cplusplus
}
#endif
