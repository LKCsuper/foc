/*
 * Flash v2 线格式金标准：用固定输入编码，逐字节对比改为字段表（motor_config_fields.inc）之前
 * 的旧实现输出。改字段表顺序/类型会让本测试失败——那意味着存储格式变了，必须提升格式版本。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_settings.h"

/* 由重构前的 motor_settings.c 对 fill() 生成的 228 字节编码 */
static const char *const k_golden_hex =
    "00803b450000a03f0000c03f0000e03f0000004000606a460000e0400000304000004040000050400000604001000000"
    "000080400000884000009040010000000000a040f90300000000403f01000000000000000000c8400000d0400000d840"
    "00007a4401000000000000000000f84000000041000004410000084100000c4100001041000014410000a04000001c41"
    "0000204100002441000028410000003f000030410000803e0500000000000045001000450020004500000000cdcccc3d"
    "cdcc4c3e00000080cdcc4cbdcdccccbd0019324b647d96ff0000c03f0100000001000000";

static void fill(motor_settings_t *s)
{
    unsigned k = 0, i;
    memset(s, 0, sizeof(*s));
#define MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max) set_##kind(s->config.name, k); k++;
#define set_FLOAT(field, idx) ((field) = 1.0f + (float)(idx) * 0.25f)
#define set_BOOL(field, idx) ((field) = ((idx) % 2U) != 0U)
#define set_U16(field, idx) ((field) = (uint16_t)(1000U + (idx)))
#define set_INT(field, idx) ((field) = 5)
#include "motor_config_fields.inc"
#undef MOTOR_CONFIG_FIELD
    s->config.ol_pole_pairs = 7.0f;
    s->config.ol_foc_freq = 15000.0f;
    s->config.speed_loop_hz = 1000.0f;
    s->config.ol_max_rpm = 3000.0f;
    s->config.sensorless_fallback_rpm = 5.0f;
    s->config.sensorless_speed_filter = 0.25f;
    s->config.sensorless_phase_tolerance = 0.5f;
    s->config.hfi_start_min_confidence = 0.75f;
    for (i = 0; i < 3U; ++i) {
        s->calibration.current_offset[i] = 2048.0f + (float)i;
        s->calibration.voltage_offset[i] = 0.1f * (float)i;
        s->calibration.undriven_offset[i] = -0.05f * (float)i;
    }
    for (i = 0; i < MOTOR_HALL_TABLE_SIZE; ++i) s->calibration.hall_table[i] = (uint8_t)(i * 25U);
    s->calibration.hall_table[7] = 255U;
    s->calibration.encoder_offset_rad = 1.5f;
    s->calibration.encoder_inverted = true;
    s->calibration.encoder_calibrated = true;
}

int main(void)
{
    motor_settings_t settings, decoded;
    uint8_t data[MOTOR_SETTINGS_WIRE_SIZE], again[MOTOR_SETTINGS_WIRE_SIZE];
    char hex[2 * MOTOR_SETTINGS_WIRE_SIZE + 1];
    unsigned i;

    fill(&settings);
    assert(Motor_Settings_Encode(&settings, data, sizeof(data)));
    for (i = 0; i < sizeof(data); ++i) sprintf(&hex[2 * i], "%02x", data[i]);
#ifdef GOLDEN_PRINT
    printf("%s\n", hex);
    return 0;
#endif
    assert(strlen(k_golden_hex) == 2U * MOTOR_SETTINGS_WIRE_SIZE);
    if (strcmp(hex, k_golden_hex) != 0) {
        printf("wire format changed!\n got %s\nwant %s\n", hex, k_golden_hex);
        return 1;
    }
    assert(Motor_Settings_Decode(data, sizeof(data), &decoded));
    assert(Motor_Settings_Encode(&decoded, again, sizeof(again)));
    assert(memcmp(data, again, sizeof(data)) == 0);
    /* v2 旧槽（216 字节，无编码器校准）仍能解码 */
    assert(Motor_Settings_Decode(data, MOTOR_SETTINGS_WIRE_SIZE_V2, &decoded));
    assert(!decoded.calibration.encoder_calibrated);
    printf("settings wire format: golden %u bytes match, round trip and v2 decode passed\n",
           (unsigned)MOTOR_SETTINGS_WIRE_SIZE);
    return 0;
}
