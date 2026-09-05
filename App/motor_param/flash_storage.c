/*
 * @Description: 配置与校准 Flash v2 双槽持久化（旧检测参数仅兼容）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"
#include <stddef.h>

#define FLASH_STORAGE_SLOT_SIZE     (128U * 1024U)
/* 扇区号仅 port/BSP 可见；勿放进 flash_storage.h，避免 App 头依赖 STM32 FLASH 宏 */
#define FLASH_STORAGE_SLOT_A_SECTOR FLASH_Sector_10
#define FLASH_STORAGE_SLOT_B_SECTOR FLASH_Sector_11

/**
 * @description: 按 32 位序号比较新旧，lhs 比 rhs 新则返回 true
 * @detail: 采用有符号差值 (int32_t)(lhs-rhs) 判断，可正确处理序号回绕
 * @param {uint32_t} lhs
 * @param {uint32_t} rhs
 * @return {*}
 * @author: lkc
 */
static bool flash_storage_sequence_newer(uint32_t lhs, uint32_t rhs)
{
    return (int32_t)(lhs - rhs) > 0;
}

/**
 * @description: 校验 Flash 槽数据是否有效
 * @detail: magic/格式版本/长度/提交标记匹配，且固定 payload 的 CRC32 正确
 * @param {const flash_storage_t} *storage
 * @return {*}
 * @author: lkc
 */
static bool flash_storage_valid(const flash_storage_t *storage)
{
    uint32_t crc;
    motor_settings_t settings;

    if (storage->magic != FLASH_STORAGE_MAGIC || storage->format_version != FLASH_STORAGE_FORMAT_VERSION ||
        storage->payload_size != MOTOR_SETTINGS_WIRE_SIZE || storage->committed != FLASH_STORAGE_COMMITTED) {
        return false;
    }

    crc = utils_crc32c(storage->payload, MOTOR_SETTINGS_WIRE_SIZE);
    return crc == storage->crc32 && Motor_Settings_Decode(storage->payload, storage->payload_size, &settings);
}

/**
 * @description: 擦除一个 Flash 扇区
 * @detail: 失败时记录错误日志并返回 false
 * @param {uint16_t} sector
 * @return {*}
 * @author: lkc
 */
static bool flash_storage_erase(uint16_t sector)
{
    FLASH_Status status;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                    FLASH_FLAG_PGSERR);
    status = FLASH_EraseSector(sector, VoltageRange_3);
    FLASH_Lock();

    if (status != FLASH_COMPLETE) {
        ERROR("FlashStorage: sector erase failed, status=%d\r\n", status);
        return false;
    }
    return true;
}

/**
 * @description: 将 data 从 addr 起逐字节编程到 Flash
 * @detail: 每字节写入后回读校验；任一步失败即锁定并返回 false
 * @param {uint32_t} addr
 * @param {const uint8_t} *data
 * @param {uint32_t} size
 * @return {*}
 * @author: lkc
 */
static bool flash_storage_write_bytes(uint32_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    FLASH_Status status;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                    FLASH_FLAG_PGSERR);

    for (i = 0; i < size; i++, addr++) {
        status = FLASH_ProgramByte(addr, data[i]);
        if (status != FLASH_COMPLETE || REG8(addr) != data[i]) {
            ERROR("FlashStorage: write failed at 0x%08X, status=%d\r\n", addr, status);
            FLASH_Lock();
            return false;
        }
    }

    FLASH_Lock();
    return true;
}

/**
 * @description: 返回双槽中序号最新且校验有效的一侧
 * @detail: 两侧都无效时返回 NULL
 * @return {*}
 * @author: lkc
 */
static const flash_storage_t *flash_storage_current(void)
{
    const flash_storage_t *a = (const flash_storage_t *)FLASH_STORAGE_SLOT_A_ADDR;
    const flash_storage_t *b = (const flash_storage_t *)FLASH_STORAGE_SLOT_B_ADDR;
    bool a_valid = flash_storage_valid(a);
    bool b_valid = flash_storage_valid(b);

    if (a_valid && b_valid) {
        return flash_storage_sequence_newer(b->sequence, a->sequence) ? b : a;
    }
    if (a_valid)
        return a;
    if (b_valid)
        return b;
    return NULL;
}

/**
 * @description: 从最新有效槽加载配置/校准，清空兼容检测结果
 * @detail: 无有效槽时返回 false，由调用方回退默认参数
 * @param {MOTOR_PARAM_S} *motor
 * @param {motor_detect_result_t} *detect
 * @return {*}
 * @author: lkc
 */
bool FlashStorage_Load(MOTOR_PARAM_S *motor, motor_detect_result_t *detect)
{
    const flash_storage_t *storage;
    motor_settings_t settings;

    if (motor == NULL || detect == NULL || motor->ol_enable)
        return false;
    storage = flash_storage_current();
    if (storage == NULL) {
        PRINTF("FlashStorage: no valid slot, using defaults\r\n");
        return false;
    }

    if (!Motor_Settings_Decode(storage->payload, storage->payload_size, &settings) ||
        !Motor_Settings_Apply(motor, &settings)) {
        return false;
    }
    memset(detect, 0, sizeof(*detect));
    detect->version = MOTOR_DETECT_RESULT_VERSION;
    PRINTF("FlashStorage: loaded slot sequence %u\r\n", storage->sequence);
    return true;
}

/**
 * @description: 把配置/校准保存到当前未使用的一侧槽
 * @detail: 双槽交替写入，降低掉电损坏风险；填 magic/序号并计算 CRC，擦除后先写主体、
 *          最后写 committed 提交字段，写毕回读整体校验
 * @param {const MOTOR_PARAM_S} *motor
 * @param {const motor_detect_result_t} *detect
 * @return {*}
 * @author: lkc
 */
bool FlashStorage_Save(const MOTOR_PARAM_S *motor, const motor_detect_result_t *detect)
{
    const flash_storage_t *current;
    flash_storage_t storage;
    motor_settings_t settings;
    uint32_t target_addr;
    uint16_t target_sector;
    uint32_t committed_offset = (uint32_t)offsetof(flash_storage_t, committed);
    const uint32_t committed = FLASH_STORAGE_COMMITTED;

    if (motor == NULL || detect == NULL || motor->ol_enable || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive() ||
        sizeof(storage) > FLASH_STORAGE_SLOT_SIZE) {
        return false;
    }

    /* 先验证并编码，非法参数不能触发擦除。调用方必须串行调用存储接口。 */
    if (!Motor_Settings_Capture(motor, &settings)) {
        return false;
    }
    memset(&storage, 0xFF, sizeof(storage));
    if (!Motor_Settings_Encode(&settings, storage.payload, MOTOR_SETTINGS_WIRE_SIZE)) {
        return false;
    }
    current = flash_storage_current();
    if (current == (const flash_storage_t *)FLASH_STORAGE_SLOT_A_ADDR) {
        target_addr = FLASH_STORAGE_SLOT_B_ADDR;
        target_sector = FLASH_STORAGE_SLOT_B_SECTOR;
    } else {
        target_addr = FLASH_STORAGE_SLOT_A_ADDR;
        target_sector = FLASH_STORAGE_SLOT_A_SECTOR;
    }

    storage.magic = FLASH_STORAGE_MAGIC;
    storage.format_version = FLASH_STORAGE_FORMAT_VERSION;
    storage.sequence = current == NULL ? 1U : current->sequence + 1U;
    storage.payload_size = MOTOR_SETTINGS_WIRE_SIZE;
    storage.crc32 = utils_crc32c(storage.payload, MOTOR_SETTINGS_WIRE_SIZE);

    if (!flash_storage_erase(target_sector) ||
        !flash_storage_write_bytes(target_addr, (const uint8_t *)&storage, committed_offset) ||
        !flash_storage_write_bytes(target_addr + committed_offset, (const uint8_t *)&committed, sizeof(uint32_t))) {
        return false;
    }

    if (!flash_storage_valid((const flash_storage_t *)(uintptr_t)target_addr)) {
        ERROR("FlashStorage: final verification failed\r\n");
        return false;
    }

    PRINTF("FlashStorage: saved slot sequence %u\r\n", storage.sequence);
    return true;
}

#ifdef __cplusplus
}
#endif
