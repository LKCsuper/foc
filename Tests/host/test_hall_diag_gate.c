/* Hall 诊断入口：非 Hall/停止路径不进入浮点累计，重新启用仍进入原主体。 */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

#define MOTOR_FEEDBACK_SRC_HALL 3
#define MOTOR_OL_MODE_IF 1
static struct { bool ol_enable; int ol_mode; } motor;
static bool s_hall_diag_running;
static int source;
static unsigned calls;

/**
 * @description: 测试替身，返回本场景反馈源。
 * @return {int} 当前反馈源
 */
static int Motor_Feedback_GetSourceDefault(void) { return source; }
/**
 * @description: 测试替身，记录进入原统计主体并标记累计运行。
 * @return {void} 无
 */
static void Motor_Hall_DiagAccumulate(void) { calls++; s_hall_diag_running = true; }
#include "hall_diag_gate_under_test.inc"

/**
 * @description: 验证停机、各反馈源、模式切换及重新进入 Hall 的门控行为。
 * @return {int} 通过返回 0
 */
int main(void)
{
    const struct { bool enabled; int source; int mode; bool accumulates; } cases[] = {
        {true, 3, 1, true}, {false, 3, 1, false}, {true, 3, 1, true},
        {true, 0, 1, false}, {true, 1, 1, false}, {true, 2, 1, false},
        {true, 3, 0, false}, {true, 3, 1, true}
    };
    unsigned i;
    for (i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        unsigned before = calls;
        motor.ol_enable = cases[i].enabled;
        motor.ol_mode = cases[i].mode;
        source = cases[i].source;
        Motor_Hall_DiagUpdate();
        assert(calls == before + (cases[i].accumulates ? 1U : 0U));
        assert(s_hall_diag_running == cases[i].accumulates);
    }
    puts("test_hall_diag_gate: pass");
    return 0;
}
