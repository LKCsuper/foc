#include <assert.h>
#include <stdio.h>
#include "motor_foc_divider.h"
#include "motor_alignment.h"
#include "motor_if_start.h"

int main(void)
{
    motor_foc_divider_t da;
    motor_foc_divider_t db;
    motor_alignment_t align_a;
    motor_alignment_t align_b;
    motor_alignment_config_t align_cfg;
    motor_alignment_output_t out;
    motor_if_start_t if_a;
    motor_if_start_t if_b;
    motor_if_start_config_t if_cfg;
    motor_if_start_input_t if_in;
    unsigned i;
    unsigned run_a = 0;
    unsigned run_b = 0;

    Motor_FocDivider_Init(&da, 2);
    Motor_FocDivider_Init(&db, 3);

    for (i = 0; i < 12U; ++i) {
        if (Motor_FocDivider_ShouldRun(&da)) {
            ++run_a;
        }
        if (Motor_FocDivider_ShouldRun(&db)) {
            ++run_b;
        }
    }
    assert(run_a == 6U);
    assert(run_b == 4U);

    /* 对齐双实例：不同电流/时长互不影响。 */
    MotorAlignment_Reset(&align_a);
    MotorAlignment_Reset(&align_b);
    align_cfg.enabled = true;
    align_cfg.current = 0.5f;
    align_cfg.duration = 0.003f;
    align_cfg.angle = 0.0f;
    assert(MotorAlignment_Step(&align_a, &align_cfg, 0.001f, &out) == MOTOR_ALIGNMENT_STARTED);
    assert(align_a.running);
    assert(!align_b.running);
    (void)MotorAlignment_Step(&align_a, &align_cfg, 0.001f, &out);
    assert(align_a.running);
    MotorAlignment_MarkComplete(&align_b);
    assert(align_b.complete);
    assert(!align_a.complete);
    (void)MotorAlignment_Step(&align_a, &align_cfg, 0.001f, &out);
    assert(align_a.complete);
    assert(!align_a.running);

    /* I/F 启动双实例：首周期相位注入隔离。 */
    MotorIfStart_Reset(&if_a);
    MotorIfStart_Reset(&if_b);
    if_cfg.min_rpm = 100.0f;
    if_cfg.phase_boost = 0.4f;
    if_cfg.min_iq_scale = 0.2f;
    MotorIfStart_NormalizeConfig(&if_cfg);
    if_in.target_rpm = 500.0f;
    if_in.current_rpm = 0.0f;
    if_in.rotor_position_known = false;
    MotorIfStart_Step(&if_a, &if_cfg, &if_in);
    assert(if_a.phase_boost != 0.0f);
    assert(if_b.phase_boost == 0.0f);
    assert(if_a.first_cycle == false);
    assert(if_b.first_cycle == true);
    MotorIfStart_SetNeutral(&if_b);
    assert(if_b.iq_scale == 1.0f);
    assert(if_a.iq_scale < 1.0f);

    printf("test_motor_startup: pass\n");
    return 0;
}
