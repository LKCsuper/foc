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

    /* VESC LONGEST_ZERO：高 CCR 相用另外两相重构；窗口足够时不改。 */
    {
        float ia = 1.0f;
        float ib = -0.4f;
        float ic = -0.2f;

        assert(!Motor_Current_ReconstructLongestZero(&ia, &ib, &ic, 2000U, 1800U, 1600U, 5600U));
        assert(fabsf(ia - 1.0f) < 1e-6f);
        /* 空载 3000 峰值 CCR≈4864：门槛 500 时仍用三相实测。 */
        assert(!Motor_Current_ReconstructLongestZero(&ia, &ib, &ic, 4864U, 3105U, 735U, 5600U));
        Motor_Current_ResetReconstructDiag();
        assert(Motor_Current_ReconstructLongestZero(&ia, &ib, &ic, 5200U, 1800U, 1600U, 5600U));
        assert(fabsf(ia - 0.6f) < 1e-6f);
        {
            int last = 0;
            uint32_t samples = 0U;
            uint32_t hits = 0U;
            uint32_t hit_a = 0U;
            uint32_t hit_b = 0U;
            uint32_t hit_c = 0U;

            Motor_Current_GetReconstructDiag(&last, &samples, &hits, &hit_a, &hit_b, &hit_c);
            assert(last == 1);
            assert(samples == 1U);
            assert(hits == 1U);
            assert(hit_a == 1U && hit_b == 0U && hit_c == 0U);
        }
        assert(fabsf(ib + 0.4f) < 1e-6f);
        ia = 0.3f;
        ib = 1.2f;
        ic = -0.5f;
        assert(Motor_Current_ReconstructLongestZero(&ia, &ib, &ic, 2000U, 5200U, 1900U, 5600U));
        assert(fabsf(ib - 0.2f) < 1e-6f);
        assert(!Motor_Current_ReconstructLongestZero(&ia, &ib, &ic, 5200U, 5200U, 1000U, 5600U));
        assert(!Motor_Current_ReconstructLongestZero(&ia, &ib, &ic, 5000U, 1000U, 1000U, 400U));
    }

    /* 切换坐标+目标时，P+积分合成的静止电压必须连续，涵盖正反转和跨圈。 */
    {
        const float deltas[] = {1.5707963f, -1.5707963f, 0.07f, -0.07f, 6.23f};
        for (i = 0; i < sizeof(deltas) / sizeof(deltas[0]); ++i) {
            float delta = deltas[i], c = cosf(delta), s = sinf(delta);
            float old_d = 0.23f, old_q = -0.11f, kp = 0.12f;
            float before_d = -1.55f + kp * (0.0f - old_d);
            float before_q = 1.06f + kp * (0.30f - old_q);
            float after_d, after_q;
            memset(&a, 0, sizeof(a));
            a.ol_current_kp = kp;
            a.ol_vd_int = -1.55f; a.ol_vq_int = 1.06f;
            Motor_Current_TransportPi(&a, delta, 0.0f, 0.30f, 0.0f, 0.15f);
            after_d = a.ol_vd_int + kp * (0.0f - (c * old_d + s * old_q));
            after_q = a.ol_vq_int + kp * (0.15f - (-s * old_d + c * old_q));
            assert(fabsf(c * after_d - s * after_q - before_d) < 1e-5f);
            assert(fabsf(s * after_d + c * after_q - before_q) < 1e-5f);
        }
        /* 实际Step必须在目标输出指针覆盖旧目标之前迁移积分。 */
        memset(&a, 0, sizeof(a)); memset(&ctx, 0, sizeof(ctx));
        a.ol_current_kp = 0.12f; a.ol_vd_int = -1.55f; a.ol_vq_int = 1.06f;
        a.iq_target = 0.30f;
        ctx.soft_current_scale = 1.0f; ctx.speed_loop_active = true;
        ctx.speed_iq_target = 0.15f; ctx.frame_correction = 1.5707963f;
        va = Motor_Current_Step(&a, ctx.frame_correction, &ctx, 0.0f, 4.0f,
                                &a.id_target, &a.iq_target);
        assert(fabsf(-va.vq + 1.55f) < 1e-5f);
        assert(fabsf(va.vd - (1.06f + 0.12f * 0.30f)) < 1e-5f);
        assert(a.iq_target == 0.15f);
    }
    {
        unsigned direction;
        for (direction = 0U; direction < 2U; direction++) {
            float angle = direction == 0U ? 3.10f : -3.10f;
            float delta = direction == 0U ? 0.007f : -0.007f;
            float feedback_d = 0.23f, feedback_q = -0.11f;
            float target_q = 0.5f;
            float before_d, before_q, expected_alpha, expected_beta;
            unsigned step;
            memset(&a, 0, sizeof(a));
            a.ol_current_kp = 0.12f;
            a.ol_vd_int = -0.8f;
            a.ol_vq_int = 1.1f;
            before_d = a.ol_vd_int - a.ol_current_kp * feedback_d;
            before_q = a.ol_vq_int + a.ol_current_kp * (target_q - feedback_q);
            expected_alpha = cosf(angle) * before_d - sinf(angle) * before_q;
            expected_beta = sinf(angle) * before_d + cosf(angle) * before_q;
            for (step = 0U; step < 300U; step++) {
                float next_target_q = step < 150U ? 0.5f : 0.0f;
                float rotated_d = cosf(delta) * feedback_d + sinf(delta) * feedback_q;
                float rotated_q = -sinf(delta) * feedback_d + cosf(delta) * feedback_q;
                float after_d, after_q;
                Motor_Current_TransportPi(&a, delta, 0.0f, target_q, 0.0f, next_target_q);
                angle = remainderf(angle + delta, 6.283185307f);
                feedback_d = rotated_d;
                feedback_q = rotated_q;
                target_q = next_target_q;
                after_d = a.ol_vd_int - a.ol_current_kp * feedback_d;
                after_q = a.ol_vq_int + a.ol_current_kp * (target_q - feedback_q);
                assert(fabsf(cosf(angle) * after_d - sinf(angle) * after_q - expected_alpha) < 1e-4f);
                assert(fabsf(sinf(angle) * after_d + cosf(angle) * after_q - expected_beta) < 1e-4f);
            }
        }
    }
    printf("test_motor_current: pass\n");
    return 0;
}
