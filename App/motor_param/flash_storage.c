#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"
#include <stddef.h>

#define FLASH_STORAGE_SLOT_SIZE (128U * 1024U)

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
 * @detail: magic/格式版本/已提交标记匹配，且 motor+detect 区域 CRC32 与槽内记录一致
 * @param {const flash_storage_t} *storage
 * @return {*}
 * @author: lkc
 */
static bool flash_storage_valid(const flash_storage_t *storage)
{
    uint32_t crc;

    if (storage->magic != FLASH_STORAGE_MAGIC ||
        storage->format_version != FLASH_STORAGE_FORMAT_VERSION ||
        storage->committed != FLASH_STORAGE_COMMITTED) {
        return false;
    }

    crc = utils_crc32c((const uint8_t *)&storage->motor,
                       sizeof(storage->motor) + sizeof(storage->detect));
    return crc == storage->crc32;
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
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
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
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

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
    if (a_valid) return a;
    if (b_valid) return b;
    return NULL;
}

/**
 * @description: 从最新有效槽加载电机参数与检测结果到输出指针
 * @detail: 无有效槽时返回 false，由调用方回退默认参数
 * @param {MOTOR_PARAM_S} *motor
 * @param {motor_detect_result_t} *detect
 * @return {*}
 * @author: lkc
 */
bool FlashStorage_Load(MOTOR_PARAM_S *motor, motor_detect_result_t *detect)
{
    const flash_storage_t *storage;

    if (motor == NULL || detect == NULL) return false;
    storage = flash_storage_current();
    if (storage == NULL) {
        PRINTF("FlashStorage: no valid slot, using defaults\r\n");
        return false;
    }

    memcpy(motor, &storage->motor, sizeof(*motor));
    memcpy(detect, &storage->detect, sizeof(*detect));
    PRINTF("FlashStorage: loaded slot sequence %u\r\n", storage->sequence);
    return true;
}

/**
 * @description: 把电机参数与检测结果保存到当前未使用的一侧槽
 * @detail: 双槽交替写入，天然防掉电损坏；填 magic/序号并计算 CRC，擦除后先写主体、
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
    uint32_t target_addr;
    uint16_t target_sector;
    uint32_t committed_offset = (uint32_t)offsetof(flash_storage_t, committed);
    const uint32_t committed = FLASH_STORAGE_COMMITTED;

    if (motor == NULL || detect == NULL || sizeof(storage) > FLASH_STORAGE_SLOT_SIZE) {
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

    memset(&storage, 0xFF, sizeof(storage));
    storage.magic = FLASH_STORAGE_MAGIC;
    storage.format_version = FLASH_STORAGE_FORMAT_VERSION;
    storage.sequence = current == NULL ? 1U : current->sequence + 1U;
    memcpy(&storage.motor, motor, sizeof(*motor));
    memcpy(&storage.detect, detect, sizeof(*detect));
    storage.crc32 = utils_crc32c((const uint8_t *)&storage.motor,
                                 sizeof(storage.motor) + sizeof(storage.detect));

    if (!flash_storage_erase(target_sector) ||
        !flash_storage_write_bytes(target_addr, (const uint8_t *)&storage, committed_offset) ||
        !flash_storage_write_bytes(target_addr + committed_offset,
                                   (const uint8_t *)&committed,
                                   sizeof(uint32_t))) {
        return false;
    }

    if (!flash_storage_valid((const flash_storage_t *)target_addr)) {
        ERROR("FlashStorage: final verification failed\r\n");
        return false;
    }

    PRINTF("FlashStorage: saved slot sequence %u\r\n", storage.sequence);
    return true;
}

#ifdef __cplusplus
}
#endif
