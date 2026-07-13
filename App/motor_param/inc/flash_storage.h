#ifndef _FLASH_STORAGE_H_
#define _FLASH_STORAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* The linker application region ends at 0x080C0000. Sectors 10 and 11 are
 * reserved as two independent parameter slots so an interrupted save leaves
 * the previous slot intact. */
#define FLASH_STORAGE_SLOT_A_ADDR    0x080C0000U
#define FLASH_STORAGE_SLOT_B_ADDR    0x080E0000U
#define FLASH_STORAGE_SLOT_A_SECTOR  FLASH_Sector_10
#define FLASH_STORAGE_SLOT_B_SECTOR  FLASH_Sector_11
#define FLASH_STORAGE_MAGIC          0x464C4B43U
#define FLASH_STORAGE_FORMAT_VERSION 1U
#define FLASH_STORAGE_COMMITTED      0xA55AA55AU

typedef struct {
    uint32_t magic;
    uint32_t format_version;
    uint32_t sequence;
    uint32_t crc32;
    MOTOR_PARAM_S motor;
    motor_detect_result_t detect;
    uint32_t committed;
} flash_storage_t;

bool FlashStorage_Load(MOTOR_PARAM_S *motor, motor_detect_result_t *detect);
bool FlashStorage_Save(const MOTOR_PARAM_S *motor, const motor_detect_result_t *detect);

#ifdef __cplusplus
}
#endif

#endif
