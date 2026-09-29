/*
 * @Description: 编码器 ODrive PLL、回绕与零点有效主机测试
 * @Author: lkc
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_encoder.h"
#include "motor_param.h"

MOTOR_PARAM_S motor;

void Hal_Encoder_Init(void)
{
}
uint16_t Hal_Encoder_GetCount(void)
{
    return 0U;
}
void Hal_Encoder_SetCount(uint16_t count)
{
    (void)count;
}
uint32_t Hal_Encoder_CountsPerRev(void)
{
    return 4096U;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void expect_rpm(const motor_encoder_runtime_t *rt, float expected, float tol)
{
    float rpm = Motor_Encoder_GetMechRpmState(rt);

    if (!Motor_Encoder_IsSpeedValidState(rt) || fabsf(rpm - expected) >= tol) {
        printf("rpm %.4f expected %.4f valid %d\n", rpm, expected, Motor_Encoder_IsSpeedValidState(rt) ? 1 : 0);
        assert(0);
    }
}

int main(void)
{
    motor_encoder_runtime_t a;
    motor_encoder_runtime_t b;
    const uint32_t cpr = 4096U;
    const float dt = 0.0001f;
    uint16_t count = 0U;
    float elec;
    int i;

    Motor_Encoder_RuntimeInit(&a, cpr, false);
    Motor_Encoder_RuntimeInit(&b, cpr, false);

    /* 首拍：锚定 PLL，速度无效 */
    Motor_Encoder_UpdateState(&a, 0U, dt);
    assert(!Motor_Encoder_IsSpeedValidState(&a));
    assert(!Motor_Encoder_IsZeroValidState(&a));

    /* 每 1 ms 加 1 count：1000 count/s → 14.648 RPM。30 rad/s 低带宽下留 500 ms 收敛。 */
    for (i = 1; i <= 5000; ++i) {
        if ((i % 10) == 0) {
            count++;
        }
        Motor_Encoder_UpdateState(&a, count, dt);
    }
    expect_rpm(&a, 14.6484375f, 0.5f);

    /* 30 rad/s 低速带宽下，计数停住 1 s 后转速收敛到近零 snap。 */
    for (i = 0; i < 10000; ++i) {
        Motor_Encoder_UpdateState(&a, count, dt);
    }
    assert(Motor_Encoder_IsSpeedValidState(&a));
    assert(fabsf(Motor_Encoder_GetMechRpmState(&a)) < 1e-3f);

    /* 无新帧时仅预测；超过 2 ms 后速度失效且标记样本陈旧。 */
    for (i = 0; i < 21; ++i) {
        Motor_Encoder_PredictState(&a, dt);
    }
    assert(!Motor_Encoder_IsSpeedValidState(&a));
    assert(a.sample_stale);

    /* 回绕：从 4050 正向跨过 0，速度保持为正。 */
    Motor_Encoder_RuntimeInit(&a, cpr, false);
    Motor_Encoder_UpdateState(&a, 4050U, dt);
    for (i = 1; i <= 500; ++i) {
        Motor_Encoder_UpdateState(&a, (uint16_t)((4050 + i) % 4096), dt);
    }
    assert(Motor_Encoder_IsSpeedValidState(&a));
    assert(Motor_Encoder_GetMechRpmState(&a) > 1.0f);

    /* 零点标定后，原始计数对应的电角≈0。标定角不走 PLL。 */
    Motor_Encoder_MarkZeroState(&a, 100U, 2.0f);
    assert(Motor_Encoder_IsZeroValidState(&a));
    elec = Motor_Encoder_GetElecAngleState(&a, 100U, 2.0f);
    assert(fabsf(elec) < 1e-4f || fabsf(elec - 2.0f * (float)M_PI) < 1e-4f);

    /* 双实例隔离：b 静止时 a 的跨零速度还在。 */
    Motor_Encoder_UpdateState(&b, 0U, dt);
    for (i = 0; i < 50; ++i) {
        Motor_Encoder_UpdateState(&b, 0U, dt);
    }
    assert(Motor_Encoder_IsSpeedValidState(&b));
    assert(fabsf(Motor_Encoder_GetMechRpmState(&b)) < 1e-3f);
    assert(Motor_Encoder_GetMechRpmState(&a) > 1.0f);

    /* 非法 dt：速度无效，但不清零点 */
    Motor_Encoder_UpdateState(&a, 200U, 1.0f);
    assert(!Motor_Encoder_IsSpeedValidState(&a));
    assert(Motor_Encoder_IsZeroValidState(&a));

    /* 方向取反后，计数减小对应正转速。 */
    Motor_Encoder_RuntimeInit(&b, cpr, true);
    Motor_Encoder_UpdateState(&b, 500U, dt);
    for (i = 1; i <= 500; ++i) {
        Motor_Encoder_UpdateState(&b, (uint16_t)(500 - i), dt);
    }
    assert(Motor_Encoder_GetMechRpmState(&b) > 1.0f);

    /* 不可能的单拍跳变被拒绝并置诊断，不把速度拉飞。 */
    Motor_Encoder_RuntimeInit(&b, cpr, false);
    Motor_Encoder_UpdateState(&b, 0U, dt);
    Motor_Encoder_UpdateState(&b, 1000U, dt);
    assert(!Motor_Encoder_IsSpeedValidState(&b));
    assert(Motor_Encoder_IsSignalFaultState(&b));
    assert(Motor_Encoder_GetInvalidCountState(&b) == 1U);
    assert(fabsf(Motor_Encoder_GetMechRpmState(&b)) < 1e-3f);
    assert(b.last_count == 0U);
    assert(fabsf(Motor_Encoder_GetMechAngleState(&b, b.last_count)) < 1e-6f);

    Motor_Encoder_ClearZeroState(&a);
    assert(!Motor_Encoder_IsZeroValidState(&a));
    assert(!Motor_Encoder_IsSpeedValidState(&a));

    puts("test_motor_encoder: pass");
    return 0;
}
