/* 半周期门控：方向、窗口、重复放行与时间回绕 */
#include <assert.h>
#include <stdio.h>
#include "motor_foc_divider.h"

#define ARR 5600U
#define US(x) ((x) * 168U)

static bool gate_at(motor_foc_phase_gate_t *g, const motor_foc_phase_gate_config_t *c, bool down, uint32_t since_top,
                    uint32_t now)
{
    return Motor_FocDivider_PhaseGate(g, c, down, ARR - since_top, ARR, now);
}

int main(void)
{
    const motor_foc_phase_gate_config_t cfg = {US(8U), US(22U), 40U};
    motor_foc_phase_gate_t g = {0};
    uint32_t t = 1000U, k, runs = 0U;

    /* 正常节拍：顶部后约 10 us 进入的向下计数回调放行，向上计数回调不放行 */
    assert(gate_at(&g, &cfg, true, US(10U), t));
    assert(!Motor_FocDivider_PhaseGate(&g, &cfg, false, US(10U), ARR, t + 33U));
    assert(gate_at(&g, &cfg, true, US(10U), t + 67U));
    /* 被拖后约 30 us 才进入：放弃本拍 */
    assert(!gate_at(&g, &cfg, true, US(30U), t + 200U));
    /* 刚越过顶部（DMA 可能在写新一组）的旧回调：放弃；随后正常回调放行 */
    assert(!gate_at(&g, &cfg, true, US(3U), t + 300U));
    assert(gate_at(&g, &cfg, true, US(10U), t + 307U));
    /* 同一半周期第二次进入：重复保护拒绝 */
    assert(!gate_at(&g, &cfg, true, US(12U), t + 309U));
    /* 窗口边界 */
    assert(gate_at(&g, &cfg, true, US(8U), t + 400U));
    assert(gate_at(&g, &cfg, true, US(22U), t + 500U));
    assert(!gate_at(&g, &cfg, true, US(22U) + 1U, t + 600U));
    assert(!gate_at(&g, &cfg, true, US(8U) - 1U, t + 700U));
    /* 非法计数 */
    assert(!Motor_FocDivider_PhaseGate(&g, &cfg, true, ARR + 1U, ARR, t + 800U));
    /* 微秒时间戳回绕 */
    g.seeded = false;
    assert(gate_at(&g, &cfg, true, US(10U), 0xFFFFFFF0U));
    assert(gate_at(&g, &cfg, true, US(10U), 0xFFFFFFF0U + 67U));
    /* 连续 1000 个完整周期（每周期一上一下两个回调）：恰好放行 1000 次 */
    g.seeded = false;
    for (k = 0; k < 1000U; ++k) {
        runs += gate_at(&g, &cfg, true, US(10U), 5000U + k * 67U);
        runs += Motor_FocDivider_PhaseGate(&g, &cfg, false, US(30U), ARR, 5000U + k * 67U + 50U);
    }
    assert(runs == 1000U);
    g.seeded = false;
    assert(!Motor_FocDivider_ControlExpired(&g, true, false, 100U, 200U));
    assert(!Motor_FocDivider_ControlExpired(&g, true, false, 300U, 200U));
    assert(Motor_FocDivider_ControlExpired(&g, true, false, 301U, 200U));
    assert(Motor_FocDivider_ControlExpired(&g, true, true, 302U, 200U));
    assert(!Motor_FocDivider_ControlExpired(&g, false, false, 302U, 200U));
    assert(!Motor_FocDivider_ControlExpired(&g, true, true, 0xFFFFFFF0U, 200U));
    assert(!Motor_FocDivider_ControlExpired(&g, true, true, 0xFFFFFFF0U + 67U, 200U));
    assert(!Motor_FocDivider_ControlExpired(&g, true, false, 0xFFFFFFF0U + 267U, 200U));
    assert(Motor_FocDivider_ControlExpired(&g, true, false, 0xFFFFFFF0U + 268U, 200U));
    assert(Motor_FocDivider_ControlExpired(&g, true, true, 0U, 0U));
    puts("motor foc phase gate: direction, window, repeat guard, deadline and wrap passed");
    return 0;
}
