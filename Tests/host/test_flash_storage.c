/* Windows host memory simulates Flash; no device is accessed. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "flash_storage.h"
#include "hal_flash.h"
#include "hal_flash_host.h"

static bool detecting;

bool Motor_Detect_IsActive(void)
{
    return detecting;
}

bool Motor_Hall_DetectIsActive(void)
{
    return detecting;
}

uint32_t utils_crc32c(uint8_t *data, uint32_t length)
{
    uint32_t crc = ~0U;
    unsigned i;
    while (length--) {
        crc ^= *data++;
        for (i = 0; i < 8; ++i) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0x82f63b78U : 0U);
        }
    }
    return ~crc;
}

int main(void)
{
    MOTOR_PARAM_S m = {0}, out = {0};
    motor_detect_result_t detect = {0};
    flash_storage_t *a;
    flash_storage_t *b;
    hal_flash_slot_t slot_a;
    hal_flash_slot_t slot_b;
    int erase_before;

    assert(Host_Flash_MapSlots());
    assert(Hal_Flash_GetSlot(0U, &slot_a) && Hal_Flash_GetSlot(1U, &slot_b));
    a = (flash_storage_t *)(uintptr_t)slot_a.base;
    b = (flash_storage_t *)(uintptr_t)slot_b.base;

    m.ol_max_rpm = 3000;
    m.ol_pole_pairs = 2;
    m.ol_foc_freq = 30000;
    m.speed_loop_hz = 1000;
    memset(m.hall_table, 0xff, sizeof(m.hall_table));
    assert(!FlashStorage_Load(&out, &detect));
    m.ol_current_kp = 0.2f;
    assert(FlashStorage_Save(&m, &detect));
    assert(a->format_version == 2 && a->payload_size == MOTOR_SETTINGS_WIRE_SIZE);
    assert(FlashStorage_Load(&out, &detect) && out.ol_current_kp == 0.2f);
    out.ol_target_rpm = 987;
    out.ol_vd_int = 456;
    m.ol_current_kp = 0.3f;
    assert(FlashStorage_Save(&m, &detect));
    assert(b->sequence == 2);
    assert(FlashStorage_Load(&out, &detect) && out.ol_current_kp == 0.3f);
    assert(out.ol_target_rpm == 987 && out.ol_vd_int == 456);
    /* Every interrupted byte write, including commit marker, retains previous slot. */
    m.ol_current_kp = 0.4f;
    for (int n = 0; n < (int)sizeof(flash_storage_t); ++n) {
        Host_Flash_SetWritesLeft(n);
        assert(!FlashStorage_Save(&m, &detect));
    }
    Host_Flash_SetWritesLeft(-1);
    assert(FlashStorage_Save(&m, &detect));
    assert(FlashStorage_Load(&out, &detect) && out.ol_current_kp == 0.4f);
    /* A checksum alone cannot make a NaN configuration valid. */
    a->payload[0] = 0;
    a->payload[1] = 0;
    a->payload[2] = 0xc0;
    a->payload[3] = 0x7f;
    a->crc32 = utils_crc32c(a->payload, MOTOR_SETTINGS_WIRE_SIZE);
    assert(FlashStorage_Load(&out, &detect) && out.ol_current_kp == 0.3f);
    assert(FlashStorage_Save(&m, &detect));
    a->payload[0] ^= 1; /* CRC failure must fall back to B. */
    assert(FlashStorage_Load(&out, &detect) && out.ol_current_kp == 0.3f);
    erase_before = Host_Flash_EraseCount();
    m.ol_enable = true;
    assert(!FlashStorage_Save(&m, &detect) && Host_Flash_EraseCount() == erase_before);
    m.ol_enable = false;
    detecting = true;
    assert(!FlashStorage_Save(&m, &detect) && Host_Flash_EraseCount() == erase_before);
    detecting = false;
    m.ol_pole_pairs = NAN;
    assert(!FlashStorage_Save(&m, &detect) && Host_Flash_EraseCount() == erase_before);
    b->format_version = 1;
    assert(!FlashStorage_Load(&out, &detect));
    b->format_version = 2;
    b->payload_size--;
    assert(!FlashStorage_Load(&out, &detect));
    assert(Host_Flash_UnmapSlots());
    puts("PASS: real Flash storage code, A/B fallback, interrupted writes, version/length and save guards");
    return 0;
}
