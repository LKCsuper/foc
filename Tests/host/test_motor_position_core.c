/* 位置控制核心命令与模式切换主机测试 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_position_core.h"

/**
 * @description: 用 64 位数学参考核对累计计数在有符号边界及回绕处的位置换算。
 * @return {void} 无；任一位置与参考不一致则断言失败
 */
static void check_count_wrap(void)
{
    const uint32_t boundaries[] = {0U, 1U, 0x7fffffffU, 0x80000000U, 0x80000001U, UINT32_MAX};
    const uint32_t resolutions[] = {3U, 3600U, 4096U};
    motor_position_config_t config = {3600U, true, 2.0f, 5.0f,
        {1.0f, 100.0f, 0.1f, 0.2f, 2.0f, 1000.0f, 0.0f, 0.05f}};
    motor_position_runtime_t runtime;
    motor_position_state_t snapshot;
    uint32_t random = 1U;
    unsigned r, origin, i;
    for (r = 0U; r < sizeof(resolutions) / sizeof(resolutions[0]); ++r) {
        config.counts_per_rev = resolutions[r];
        for (origin = 0U; origin < sizeof(boundaries) / sizeof(boundaries[0]); ++origin) {
            assert(Motor_PositionCore_Init(&runtime, &config, boundaries[origin], 0.3f));
            for (i = 0U; i < 4102U; ++i) {
                uint32_t delta;
                int64_t signed_delta;
                float expected;
                random = random * 1664525U + 1013904223U;
                delta = i < 6U ? boundaries[i] : random;
                signed_delta = delta <= INT32_MAX ? (int64_t)delta : (int64_t)delta - 4294967296LL;
                expected = (float)signed_delta * 360.0f / (float)config.counts_per_rev;
                Motor_PositionCore_Snapshot(&runtime, &config, boundaries[origin] + delta, true, &snapshot);
                assert(snapshot.position_deg == expected);
            }
        }
    }
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_position_config_t config = {3600U, true, 2.0f, 5.0f,
        {1.0f, 100.0f, 0.1f, 0.2f, 2.0f, 1000.0f, 0.0f, 0.05f}};
    motor_position_runtime_t a, b;
    motor_position_state_t snapshot;
    motor_position_input_t input = {10U, true, true, true, 0.0f, 0.01f};
    motor_position_output_t output;
    assert(Motor_PositionCore_Init(&a, &config, UINT32_MAX - 9U, 0.3f));
    assert(Motor_PositionCore_Init(&b, &config, 10U, 0.3f));
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(!output.owns_speed_command && !output.hold_active);
    assert(fabsf(a.state.position_deg - 2.0f) < 1e-6f);
    assert(!Motor_PositionCore_Enable(&a, true, false));
    assert(Motor_PositionCore_Enable(&a, true, true));
    assert(Motor_PositionCore_SetTarget(&a, 30.0f));
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.owns_speed_command && output.hold_active && !output.voltage_hold);
    assert(output.speed_rpm == 10.0f && a.control.velocity_integrator_v == 0.0f); /* 斜坡首拍10RPM */
    assert(!b.state.enabled && b.state.target_deg == 0.0f);
    input.accumulated_count = 285U; /* 相对零点29.5deg，进入电压保持 */
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.voltage_hold && output.speed_rpm == 0.0f);
    input.accumulated_count = 260U; /* 误差3deg，位于滞环内 */
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.voltage_hold);
    Motor_PositionCore_Snapshot(&a, &config, 10U, true, &snapshot);
    assert(snapshot.error_deg == 28.0f && a.state.position_deg == 27.0f);
    input.accumulated_count = 200U; /* 误差9deg，退出电压保持 */
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(!output.voltage_hold && output.speed_rpm > 0.0f);
    input.feedback_valid = false;
    assert(Motor_PositionCore_Step(&a, &config, &input, &output));
    assert(output.owns_speed_command && !output.hold_active && output.speed_rpm == 0.0f);
    Motor_PositionCore_Zero(&a, 100U);
    Motor_PositionCore_Snapshot(&a, &config, 90U, true, &snapshot);
    assert(snapshot.position_deg == -1.0f);
    assert(!Motor_PositionCore_Move(&a, NAN));
    assert(!Motor_PositionCore_SetMaxRpm(&a, 101.0f, 100.0f));
    config.counts_per_rev = 0U;
    assert(!Motor_PositionCore_Init(&a, &config, 0U, 0.3f));
    assert(a.state.origin_count == 100);
    check_count_wrap();
    puts("Position command/mode standalone tests passed");
    return 0;
}
