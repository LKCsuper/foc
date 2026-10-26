/* 相位边沿捕获主机测试：触发、停止与保留窗口 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_diag_phase.h"

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：进入短临界区（屏蔽 IRQ），返回进入前的 PRIMASK。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
static uint32_t Motor_Crit_Enter(void) { return 0U; }
/**
 * @description: 主机测试替身：退出短临界区；仅当进入前 IRQ 开启时恢复。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} key 兼容参数，本实现不使用
 * @return {void} 无
 */
static void Motor_Crit_Exit(uint32_t key) { (void)key; }
#include "phase_edge_under_test.inc"

/**
 * @description: 注入一个带时间信息的测试事件。
 * @param {uint32_t} time time 参数
 * @param {bool} full full 参数
 * @param {bool} down down 参数
 * @return {void} 无
 */
static void send_event(uint32_t time, bool full, bool down)
{
    bool active = g_motor_phase_active != 0U;
    if (active) Motor_Timing_PhaseIrqBegin(time - 1U);
    Motor_Timing_PhaseEdgeNote(time, 200U, down ? 16U : 0U,
                           full ? 0x00010000U : 1U, full ? 1U : 0U, true);
    if (time == 3828U) Motor_Timing_PhaseMarkStatus();
    if (active) Motor_Timing_PhaseIrqEnd(time + 5U, 0x20U, 0x202020U, 15U, 1U);
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    uint32_t index;
    Motor_Timing_PhaseEdgeArm();
    assert(g_motor_phase_active == 1U && g_motor_phase_debug.count == 0U);
    Motor_Timing_PhaseEdgeNote(0U, 0U, 0U, 0U, 0U, false);
    assert(g_motor_phase_active == 1U && g_motor_phase_debug.seen == 0U);
    for (index = 0U; index < 100U; index++) send_event(index * 33U, index % 2U == 0U, index % 2U != 0U);
    assert(g_motor_phase_active == 1U && g_motor_phase_debug.count == 64U);
    send_event(3300U, true, true);
    assert(g_motor_phase_active == 2U && g_motor_phase_debug.trigger_sequence == 101U);
    for (index = 0U; index < 15U; index++) send_event(3333U + index * 33U, index % 2U != 0U, index % 2U != 0U);
    assert(g_motor_phase_active == 2U);
    send_event(3828U, false, false);
    assert(g_motor_phase_active == 0U && g_motor_phase_debug.reason == 1U);
    assert(g_motor_phase_debug.seen == 117U);
    assert(g_motor_phase_debug.events[g_motor_phase_debug.head].sequence == 54U);
    {
        motor_phase_event_t *last = &g_motor_phase_debug.events[(g_motor_phase_debug.head + 63U) % 64U];
        assert((last->flags & 2U) != 0U);
        assert(last->irq_exit_us - last->irq_entry_us == 6U);
        assert(last->dma_status == 0x20U && last->adc_status == 0x202020U);
        assert(last->dma_remaining == 15U && last->irq_pending == 1U);
        assert((last->flags & 4U) != 0U);
    }
    send_event(4000U, true, false);
    assert(g_motor_phase_debug.seen == 117U);
    Motor_Timing_PhaseEdgeStop();
    assert(g_motor_phase_debug.reason == 1U);

    Motor_Timing_PhaseEdgeArm();
    send_event(100U, true, false);
    send_event(166U, false, true);
    assert(g_motor_phase_active == 1U && g_motor_phase_debug.trigger_sequence == 0U);
    Motor_Timing_PhaseEdgeNote(200U, 0U, 0U, 0U, 0U, false);
    assert(g_motor_phase_active == 0U && g_motor_phase_debug.reason == 2U);
    assert(g_motor_phase_debug.count == 2U);
    Motor_Timing_PhaseEdgeArm();
    Motor_Timing_PhaseIrqBegin(100U);
    Motor_Timing_PhaseIrqEnd(105U, 0U, 0U, 0U, 0U);
    assert(g_motor_phase_debug.count == 0U);
    Motor_Timing_PhaseEdgeNote(200U, 0U, 0U, 0U, 1U, true);
    assert((g_motor_phase_debug.events[0].flags & 2U) == 0U);
    Motor_Timing_PhaseIrqEnd(205U, 1U, 1U, 1U, 1U);
    assert(g_motor_phase_debug.events[0].irq_exit_us == 0U);
    Motor_Timing_PhaseMarkStatus();
    assert((g_motor_phase_debug.events[0].flags & 4U) == 0U);
    Motor_Timing_PhaseIrqBegin(300U);
    Motor_Timing_PhaseEdgeNote(302U, 0U, 0U, 0U, 1U, true);
    Motor_Timing_PhaseMarkStatus();
    Motor_Timing_PhaseIrqEnd(350U, 0U, 0U, 15U, 0U);
    assert((g_motor_phase_debug.events[1].flags & 7U) == 7U);
    Motor_Timing_PhaseEdgeReserve3000();
    send_event(100U, true, false);
    assert(g_motor_phase_active == 3U && g_motor_phase_debug.seen == 0U);
    Motor_Timing_PhaseEdgePoll(false, false, 100U);
    assert(g_motor_phase_active == 3U);
    Motor_Timing_PhaseEdgePoll(true, true, 200U);
    Motor_Timing_PhaseEdgePoll(true, true, 1000199U);
    assert(g_motor_phase_active == 3U);
    Motor_Timing_PhaseEdgePoll(true, false, 1000200U);
    Motor_Timing_PhaseEdgePoll(true, true, 0xfffffff0U);
    Motor_Timing_PhaseEdgePoll(true, true, (uint32_t)(0xfffffff0U + 1000000U));
    assert(g_motor_phase_active == 1U);
    send_event(2000000U, true, false);
    assert(g_motor_phase_debug.count == 1U);
    Motor_Timing_PhaseEdgeReserve3000();
    Motor_Timing_PhaseEdgePoll(true, false, 100U);
    Motor_Timing_PhaseEdgePoll(false, false, 200U);
    assert(g_motor_phase_active == 0U && g_motor_phase_debug.reason == 2U);

    Motor_Timing_PhaseEdgeArm();
    send_event(0xfffffff0U, true, false);
    send_event((uint32_t)(0xfffffff0U + 10000000U), true, false);
    assert(g_motor_phase_active == 0U && g_motor_phase_debug.reason == 3U);
    assert(g_motor_phase_debug.count == 2U);
    puts("Phase edge capture tests passed");
    return 0;
}
