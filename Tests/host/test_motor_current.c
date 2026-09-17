#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_current.h"
#include "motor_param.h"

int main(void)
{
    MOTOR_PARAM_S a;
    MOTOR_PARAM_S b;
    motor_current_target_ctx_t ctx;
    motor_dq_voltage_t va;
    motor_dq_voltage_t vb;
    float id_t;
    float iq_t;
    float dt = 0.001f;
    float v_limit = 12.0f;
    unsigned i;

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.ol_current_kp = 0.2f;
    a.ol_current_ki = 30.0f;
    a.ol_iq_set = 1.0f;
    a.ol_id_set = 0.0f;
    a.ol_align_current = 0.5f;
    b = a;

    memset(&ctx, 0, sizeof(ctx));
    /* 对齐目标：Id=align，Iq=0。 */
    ctx.alignment_running = true;
    ctx.alignment_current_scale = 1.0f;
    ctx.if_iq_scale = 0.5f;
    ctx.soft_current_scale = 1.0f;
    ctx.speed_loop_active = false;
    ctx.speed_iq_target = 2.0f;
    Motor_Current_GetTargets(&a, &ctx, &id_t, &iq_t);
    assert(fabsf(id_t - 0.5f) < 1e-6f);
    assert(fabsf(iq_t) < 1e-6f);

    /* 对齐阶段忽略弱磁 Id。 */
    ctx.id_fw = -2.0f;
    Motor_Current_GetTargets(&a, &ctx, &id_t, &iq_t);
    assert(fabsf(id_t - 0.5f) < 1e-6f);
    ctx.id_fw = 0.0f;

    /* I/F 缩放目标。 */
    ctx.alignment_running = false;
    ctx.alignment_current_scale = 0.0f;
    ctx.if_iq_scale = 0.5f;
    Motor_Current_GetTargets(&a, &ctx, &id_t, &iq_t);
    assert(fabsf(iq_t - 0.5f) < 1e-6f);

    /* 运行区叠加弱磁负 Id。 */
    ctx.id_fw = -1.5f;
    Motor_Current_GetTargets(&a, &ctx, &id_t, &iq_t);
    assert(fabsf(id_t + 1.5f) < 1e-6f);
    ctx.id_fw = 0.0f;

    /* 速度环目标优先。 */
    ctx.speed_loop_active = true;
    ctx.speed_iq_target = 1.25f;
    Motor_Current_GetTargets(&a, &ctx, &id_t, &iq_t);
    assert(fabsf(iq_t - 1.25f) < 1e-6f);

    /* 拖转交接中点：定角 Id 下降，速度环 Iq 同比例上升。 */
    ctx.alignment_current_scale = 0.5f;
    ctx.soft_current_scale = 0.5f;
    Motor_Current_GetTargets(&a, &ctx, &id_t, &iq_t);
    assert(fabsf(id_t - 0.25f) < 1e-6f);
    assert(fabsf(iq_t - 0.625f) < 1e-6f);

    /* 双实例 PI：不同 Iq 反馈，积分互不影响。 */
    ctx.alignment_running = false;
    ctx.alignment_current_scale = 0.0f;
    ctx.if_iq_scale = 1.0f;
    ctx.soft_current_scale = 1.0f;
    ctx.speed_loop_active = false;
    a.ia = 0.0f;
    a.ib = 0.0f;
    a.ic = 0.0f;
    b.ia = 0.0f;
    b.ib = 0.0f;
    b.ic = 0.0f;
    /* theta=0 时 Id≈Ialpha, Iq≈Ibeta；用纯 q 轴电流近似：设 ib/ic 造 Ibeta。 */
    a.ol_iq_set = 1.0f;
    b.ol_iq_set = 1.0f;
    for (i = 0; i < 20U; ++i) {
        /* A 反馈 Iq 偏低 → 更大 vq；B 已跟上目标。 */
        a.Id = 0.0f;
        a.Iq = 0.2f;
        b.Id = 0.0f;
        b.Iq = 1.0f;
        va = Motor_Current_RunPi(&a, 0.0f, 1.0f, dt, v_limit);
        vb = Motor_Current_RunPi(&b, 0.0f, 1.0f, dt, v_limit);
        (void)va;
        (void)vb;
    }
    assert(a.ol_vq_int > b.ol_vq_int);
    assert(fabsf(a.ol_vd_int) < 1e-3f);
    assert(fabsf(b.ol_vd_int) < 1e-3f);
    assert(fabsf(b.ol_vq_int) < 0.05f);

    /* Step：Clarke/Park 后 Id/Iq 写入 motor，再出电压。 */
    memset(&a, 0, sizeof(a));
    a.ol_current_kp = 0.2f;
    a.ol_current_ki = 0.0f;
    a.ol_iq_set = 0.0f;
    a.ol_id_set = 0.5f;
    a.ia = 0.5f;
    a.ib = -0.25f;
    a.ic = -0.25f;
    ctx.alignment_running = false;
    ctx.alignment_current_scale = 0.0f;
    ctx.if_iq_scale = 1.0f;
    ctx.speed_loop_active = false;
    va = Motor_Current_Step(&a, 0.0f, &ctx, dt, v_limit, &id_t, &iq_t);
    assert(fabsf(id_t - 0.5f) < 1e-6f);
    assert(fabsf(iq_t) < 1e-6f);
    /* theta=0、对称三相 → Id≈ia 平均方向，接近 0.5；误差≈0 → vd 很小。 */
    assert(fabsf(a.Id - 0.5f) < 0.05f);
    assert(fabsf(va.vd) < 0.05f);

    /* 反电势前馈：ωe·ψf 加到 vq；ωe=0 时电压不变。仅转子 dq。 */
    {
        motor_dq_voltage_t ff = {1.0f, 2.0f};
        motor_dq_voltage_t z = {1.0f, 2.0f};

        Motor_Current_ApplySpeedFeedforward(&z, 0.0f, 0.0008f, 0.02573f, 0.0f, 0.6f, 18.0f);
        assert(fabsf(z.vd - 1.0f) < 1e-6f);
        assert(fabsf(z.vq - 2.0f) < 1e-6f);
        Motor_Current_ApplySpeedFeedforward(&ff, 62.832f, 0.0008f, 0.02573f, 0.0f, 0.6f, 18.0f);
        assert(ff.vq > 2.0f);
        assert(ff.vd < 1.0f);
    }

    printf("test_motor_current: pass\n");
    return 0;
}
