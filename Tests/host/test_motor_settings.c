/* 参数设置编解码主机测试：校验、状态隔离与运行中配置约束 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_settings.h"
#include "flash_storage.h"

static int save_calls;
static int load_mode;
static motor_settings_t saved;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：从最新有效 Flash 槽加载电机配置与校准。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} m 电机实例
 * @return {bool} true=加载成功；false=无有效槽，调用方应使用默认参数
 */
bool FlashStorage_Load(MOTOR_PARAM_S *m)
{
    if (load_mode) {
        return Motor_Settings_Apply(m, &saved);
    }
    return false;
}
/**
 * @description: 主机测试替身：把配置/校准保存到当前未使用的一侧槽。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} m 兼容参数，本实现不使用
 * @return {bool} 本实现固定返回 true
 */
bool FlashStorage_Save(const MOTOR_PARAM_S *m)
{
    (void)m;
    ++save_calls;
    return true;
}
/**
 * @description: 主机测试替身：上电初始化默认单实例速度环，并向保护模块登记无效反馈
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Speed_Init(void)
{
}
/**
 * @description: 主机测试替身：上电初始化默认单实例观测器。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
void Motor_Sensorless_Init(void)
{
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_settings_t settings, decoded, before;
    MOTOR_PARAM_S target;
    uint8_t data[MOTOR_SETTINGS_WIRE_SIZE], other[MOTOR_SETTINGS_WIRE_SIZE];
    unsigned i;
    uint32_t revision;

    /* Real production defaults must pass validation, including repeated initialization. */
    motor.ol_enable = true;
    motor.ol_vd_int = 90.0f;
    motor.ia = 123.0f;
    motor.control_config_revision = 200U;
    MOTOR_Param_Init();
    assert(motor.control_config_revision == 201U);
    assert(!motor.ol_enable && motor.ol_vd_int == 0.0f && motor.ia == 0.0f);
    assert(save_calls == 0);
    assert(Motor_Settings_Capture(&motor, &settings));
    assert(Motor_Settings_Encode(&settings, data, sizeof(data)));
    /* First schema word is the selected motor's max RPM in little-endian IEEE754. */
    assert(memcmp(data, &motor.ol_max_rpm, sizeof(float)) == 0);
    assert(Motor_Settings_Decode(data, sizeof(data), &decoded));
    assert(Motor_Settings_Encode(&decoded, other, sizeof(other)));
    assert(memcmp(data, other, sizeof(data)) == 0);

    motor.control_config_revision ^= 0x12345678U;
    assert(Motor_Settings_Capture(&motor, &decoded));
    assert(Motor_Settings_Encode(&decoded, other, sizeof(other)));
    assert(memcmp(data, other, sizeof(data)) == 0);

    /* Commands and transient state cannot alter the persisted payload. */
    motor.ol_enable = true;
    motor.ol_target_rpm = -987.0f;
    motor.ol_iq_set = 7.0f;
    motor.ol_vd_int = 888.0f;
    motor.ia = 99.0f;
    assert(Motor_Settings_Capture(&motor, &decoded));
    assert(Motor_Settings_Encode(&decoded, other, sizeof(other)));
    assert(memcmp(data, other, sizeof(data)) == 0);

    target = motor;
    assert(!Motor_Settings_Apply(&target, &settings));
    assert(target.ol_vd_int == 888.0f);
    target.ol_enable = false;
    assert(Motor_Settings_Apply(&target, &settings));
    assert(target.ol_target_rpm == -987.0f && target.ol_iq_set == 7.0f && target.ia == 99.0f);

    /* Reject invalid settings and malformed wire data without partial publication. */
    before = decoded;
    for (i = 0; i < sizeof(data); ++i) {
        if (i == MOTOR_SETTINGS_WIRE_SIZE_V2) {
            continue;
        }
        assert(!Motor_Settings_Decode(data, i, &decoded));
        assert(memcmp(&decoded, &before, sizeof(decoded)) == 0);
    }
    assert(!Motor_Settings_Decode(NULL, sizeof(data), &decoded));
    assert(!Motor_Settings_Capture(NULL, &decoded));
    decoded.config.ol_pole_pairs = 1.5f;
    memset(other, 0xa5, sizeof(other));
    assert(!Motor_Settings_Encode(&decoded, other, sizeof(other)));
    assert(other[0] == 0xa5);
    decoded = settings;
    decoded.config.ol_current_kp = NAN;
    assert(!Motor_Settings_Apply(&target, &decoded));
    decoded = settings;
    decoded.config.ol_foc_freq = INFINITY;
    assert(!Motor_Settings_Encode(&decoded, other, sizeof(other)));
    decoded = settings;
    decoded.config.hall_extra_samples = -1;
    assert(!Motor_Settings_Encode(&decoded, other, sizeof(other)));
    memcpy(other, data, sizeof(data));
    other[0] = 0;
    other[1] = 0;
    other[2] = 0xc0;
    other[3] = 0x7f;
    assert(!Motor_Settings_Decode(other, sizeof(other), &decoded));
    memcpy(other, data, sizeof(data));
    other[sizeof(other) - 1] = 201;
    assert(!Motor_Settings_Decode(other, sizeof(other), &decoded));

    /* Preserve unsigned Hall encodings above 127 as well as invalid sentinel 255. */
    settings.calibration.hall_table[1] = 180;
    settings.calibration.current_offset[0] = 2045.0f;
    assert(Motor_Settings_Encode(&settings, data, sizeof(data)));
    assert(Motor_Settings_Decode(data, sizeof(data), &decoded));
    assert(decoded.calibration.hall_table[1] == 180);
    assert(decoded.calibration.hall_table[0] == 255);

    /* Current development policy keeps code gains but restores calibration. */
    saved = settings;
    saved.config.ol_current_kp = 0.9f;
    load_mode = 1;
    MOTOR_Param_Init();
    assert(motor.currentOffset[0] == 2045.0f);
    assert((uint8_t)motor.hall_table[1] == 180);
    assert(motor.ol_current_kp == 0.12f);
    assert(!motor.ol_enable && motor.ol_vd_int == 0.0f && motor.ia == 0.0f);
    assert(save_calls == 0);

    /* 运行中允许在线增益，禁止极对数；完整 Apply 仍要求停机。 */
    assert(Motor_Settings_Capture(&motor, &settings));
    motor.ol_enable = true;
    settings.config.ol_current_kp = 0.35f;
    assert(Motor_Settings_ApplyConfig(&motor, &settings.config));
    assert(motor.ol_current_kp == 0.35f);
    settings.config.ol_pole_pairs = 4.0f;
    assert(!Motor_Settings_ApplyConfig(&motor, &settings.config));
    assert(motor.ol_pole_pairs == MOTOR_FIXED_POLE_PAIRS);
    assert(!Motor_Settings_Apply(&motor, &settings));
    motor.ol_enable = false;
    settings.config.ol_pole_pairs = MOTOR_FIXED_POLE_PAIRS;
    assert(Motor_Settings_Apply(&motor, &settings));

    revision = motor.control_config_revision;
    assert(Motor_Settings_ApplyConfig(&motor, &settings.config));
    assert(motor.control_config_revision != revision);
    revision = motor.control_config_revision;
    settings.config.sensorless_motor_r = NAN;
    assert(!Motor_Settings_ApplyConfig(&motor, &settings.config));
    assert(motor.control_config_revision == revision);
    motor.control_config_revision = UINT32_MAX;
    assert(Motor_Settings_ApplyWeakening(&motor, 1.0f, 0.8f, 3.0f));
    assert(motor.control_config_revision == 1U && motor.fw_current_max == 1.0f);
    assert(!Motor_Settings_ApplyWeakening(&motor, NAN, 0.8f, 3.0f));
    assert(!Motor_Settings_ApplyWeakening(&motor, 1.0f, 0.1f, 3.0f));
    assert(!Motor_Settings_ApplyWeakening(&motor, 1.0f, 0.8f, -1.0f));
    assert(motor.control_config_revision == 1U && motor.fw_current_max == 1.0f);
    revision = motor.control_config_revision;
    MOTOR_Param_Init();
    assert(motor.control_config_revision != revision);
    puts("PASS: settings codec, validation, state isolation and initialization");
    return 0;
}
