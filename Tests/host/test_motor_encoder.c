/*
 * @Description: 编码器速度/回绕/零点有效主机测试
 * @Author: lkc
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
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

int main(void)
{
    motor_encoder_runtime_t a;
    motor_encoder_runtime_t b;
    const uint32_t cpr = 4096U;
    float elec;

    Motor_Encoder_RuntimeInit(&a, cpr, false);
    Motor_Encoder_RuntimeInit(&b, cpr, false);

    /* 首拍：建立历史，速度无效 */
    Motor_Encoder_UpdateState(&a, 0U, 0.01f);
    assert(!Motor_Encoder_IsSpeedValidState(&a));
    assert(!Motor_Encoder_IsZeroValidState(&a));

    /* +41 count / 0.1 s ≈ 0.1 圈/s ≈ 6 RPM */
    Motor_Encoder_UpdateState(&a, 41U, 0.1f);
    assert(Motor_Encoder_IsSpeedValidState(&a));
    assert(fabsf(Motor_Encoder_GetMechRpmState(&a) - 6.0f) < 0.05f);

    /* 回绕：从 4000 到 100，正向跨零 */
    Motor_Encoder_UpdateState(&a, 4000U, 0.01f);
    Motor_Encoder_UpdateState(&a, 100U, 0.01f);
    assert(Motor_Encoder_IsSpeedValidState(&a));
    assert(Motor_Encoder_GetMechRpmState(&a) > 0.0f);

    /* 零点标定后电角≈0，angle 可用 */
    Motor_Encoder_MarkZeroState(&a, 100U, 2.0f);
    assert(Motor_Encoder_IsZeroValidState(&a));
    elec = Motor_Encoder_GetElecAngleState(&a, 100U, 2.0f);
    assert(fabsf(elec) < 1e-4f || fabsf(elec - 2.0f * (float)M_PI) < 1e-4f);

    /* 双实例隔离 */
    Motor_Encoder_UpdateState(&b, 0U, 0.001f);
    Motor_Encoder_UpdateState(&b, 0U, 0.001f); /* 静止：rpm≈0 且 valid */
    assert(Motor_Encoder_IsSpeedValidState(&b));
    assert(fabsf(Motor_Encoder_GetMechRpmState(&b)) < 1e-3f);
    assert(Motor_Encoder_GetMechRpmState(&a) > 0.0f);

    /* 非法 dt：速度无效，但不清零点 */
    Motor_Encoder_UpdateState(&a, 200U, 1.0f);
    assert(!Motor_Encoder_IsSpeedValidState(&a));
    assert(Motor_Encoder_IsZeroValidState(&a));

    Motor_Encoder_ClearZeroState(&a);
    assert(!Motor_Encoder_IsZeroValidState(&a));

    puts("test_motor_encoder: pass");
    return 0;
}
