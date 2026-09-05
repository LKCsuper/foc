/* Windows host memory simulates Flash; no device is accessed. */
#include <windows.h>
#include <assert.h>
#include <math.h>
#include "includes.h"

static int writes_left = -1;
static int erases;
static bool detecting;
void FLASH_Unlock(void)
{
}
void FLASH_Lock(void)
{
}
void FLASH_ClearFlag(uint32_t flags)
{
    (void)flags;
}
FLASH_Status FLASH_EraseSector(uint16_t sector, int voltage)
{
    (void)voltage;
    assert(sector == 10 || sector == 11);
    memset((void *)(uintptr_t)(sector == 10 ? FLASH_STORAGE_SLOT_A_ADDR : FLASH_STORAGE_SLOT_B_ADDR), 0xff, 128 * 1024);
    ++erases;
    return FLASH_COMPLETE;
}
FLASH_Status FLASH_ProgramByte(uint32_t address, uint8_t value)
{
    if (writes_left == 0) {
        return FLASH_ERROR;
    }
    if (writes_left > 0) {
        --writes_left;
    }
    REG8(address) &= value;
    return FLASH_COMPLETE;
}
bool Motor_Detect_IsActive(void)
{
    return detecting;
}
bool Motor_Hall_DetectIsActive(void)
{
    return detecting;
}
uint32_t utils_crc32c(const uint8_t *data, uint32_t length)
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
    flash_storage_t *a = (void *)(uintptr_t)FLASH_STORAGE_SLOT_A_ADDR;
    flash_storage_t *b = (void *)(uintptr_t)FLASH_STORAGE_SLOT_B_ADDR;
    int erase_before;
    assert(VirtualAlloc(a, 256 * 1024, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE) == a);
    memset(a, 0xff, 256 * 1024);
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
        writes_left = n;
        assert(!FlashStorage_Save(&m, &detect));
        assert(FlashStorage_Load(&out, &detect) && out.ol_current_kp == 0.3f);
    }
    writes_left = -1;
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
    erase_before = erases;
    m.ol_enable = true;
    assert(!FlashStorage_Save(&m, &detect) && erases == erase_before);
    m.ol_enable = false;
    detecting = true;
    assert(!FlashStorage_Save(&m, &detect) && erases == erase_before);
    detecting = false;
    m.ol_pole_pairs = NAN;
    assert(!FlashStorage_Save(&m, &detect) && erases == erase_before);
    b->format_version = 1;
    assert(!FlashStorage_Load(&out, &detect));
    b->format_version = 2;
    b->payload_size--;
    assert(!FlashStorage_Load(&out, &detect));
    assert(VirtualFree(a, 0, MEM_RELEASE));
    puts("PASS: real Flash storage code, A/B fallback, interrupted writes, version/length and save guards");
    return 0;
}
