/*
 * @Description: 偏移校准非阻塞核心主机测试
 * @Author: lkc
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_detect_offset.h"
#include "motor_param.h"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_detect_offset_t st;
    MOTOR_PARAM_S motor;
    uint32_t i;
    const uint32_t n = 10U;

    memset(&motor, 0, sizeof(motor));
    Motor_Detect_OffsetReset(&st, true, n, 4096.0f);

    /* 相 A：电流中点 2048，电压 1.0 V */
    for (i = 0U; i < n; i++) {
        bool full = Motor_Detect_OffsetFeedDriven(&st, 2048.0f, 1.0f);
        assert((i + 1U < n) ? !full : full);
    }
    assert(Motor_Detect_OffsetNextPhase(&st));

    /* 相 B：2050，1.1 V */
    for (i = 0U; i < n; i++) {
        (void)Motor_Detect_OffsetFeedDriven(&st, 2050.0f, 1.1f);
    }
    assert(Motor_Detect_OffsetNextPhase(&st));

    /* 相 C：2046，0.9 V */
    for (i = 0U; i < n; i++) {
        (void)Motor_Detect_OffsetFeedDriven(&st, 2046.0f, 0.9f);
    }
    assert(!Motor_Detect_OffsetNextPhase(&st));

    Motor_Detect_OffsetFinalizeDriven(&st);
    assert(fabsf(st.current_offset[0] - 2048.0f) < 1e-4f);
    assert(fabsf(st.current_offset[1] - 2050.0f) < 1e-4f);
    assert(fabsf(st.current_offset[2] - 2046.0f) < 1e-4f);
    /* v_avg = 1.0；偏置 0 / +0.1 / -0.1 */
    assert(fabsf(st.volt_offset[0]) < 1e-5f);
    assert(fabsf(st.volt_offset[1] - 0.1f) < 1e-5f);
    assert(fabsf(st.volt_offset[2] + 0.1f) < 1e-5f);
    assert(!st.offset_warn);

    for (i = 0U; i < n; i++) {
        bool full = Motor_Detect_OffsetFeedUndriven(&st, 1.0f, 1.2f, 0.8f);
        assert((i + 1U < n) ? !full : full);
    }
    Motor_Detect_OffsetFinalizeUndriven(&st);
    /* 相对均值 1.0：0 / +0.2 / -0.2 */
    assert(fabsf(st.volt_undriven_offset[0]) < 1e-5f);
    assert(fabsf(st.volt_undriven_offset[1] - 0.2f) < 1e-5f);
    assert(fabsf(st.volt_undriven_offset[2] + 0.2f) < 1e-5f);

    Motor_Detect_OffsetApply(&st, &motor, true);
    assert(fabsf(motor.currentOffset[1] - 2050.0f) < 1e-4f);
    assert(fabsf(motor.voltundrivenOffset[1] - 0.2f) < 1e-5f);
    assert(Motor_Detect_OffsetIsStored(&motor));

    /* 三相同时采样应得到与逐相同样的平均值。 */
    Motor_Detect_OffsetReset(&st, false, n, 4096.0f);
    for (i = 0U; i < n; i++) {
        bool full = Motor_Detect_OffsetFeedDrivenTriple(&st, 2048.0f, 2050.0f, 2046.0f, 1.0f, 1.1f, 0.9f);
        assert((i + 1U < n) ? !full : full);
    }
    Motor_Detect_OffsetFinalizeDriven(&st);
    assert(fabsf(st.current_offset[0] - 2048.0f) < 1e-4f);
    assert(fabsf(st.current_offset[1] - 2050.0f) < 1e-4f);
    assert(fabsf(st.current_offset[2] - 2046.0f) < 1e-4f);
    assert(fabsf(st.i_max[0] - st.i_min[0]) < 1e-4f);
    assert(fabsf(st.i_max[1] - st.i_min[1]) < 1e-4f);
    assert(fabsf(st.i_max[2] - st.i_min[2]) < 1e-4f);
    assert(fabsf(st.sum_max - st.sum_min) < 1e-4f);

    /* 未校准（全 0）或越界不能当作已存偏移 */
    {
        MOTOR_PARAM_S empty;
        MOTOR_PARAM_S bad;

        memset(&empty, 0, sizeof(empty));
        assert(!Motor_Detect_OffsetIsStored(&empty));
        assert(!Motor_Detect_OffsetIsStored(NULL));
        bad = motor;
        bad.currentOffset[2] = 0.0f;
        assert(!Motor_Detect_OffsetIsStored(&bad));
        bad.currentOffset[2] = 4000.0f;
        assert(!Motor_Detect_OffsetIsStored(&bad));
    }

    /* 严重偏离中点应告警 */
    Motor_Detect_OffsetReset(&st, false, 4U, 4096.0f);
    for (i = 0U; i < 4U; i++) {
        (void)Motor_Detect_OffsetFeedDriven(&st, 100.0f, 0.0f);
    }
    (void)Motor_Detect_OffsetNextPhase(&st);
    for (i = 0U; i < 4U; i++) {
        (void)Motor_Detect_OffsetFeedDriven(&st, 100.0f, 0.0f);
    }
    (void)Motor_Detect_OffsetNextPhase(&st);
    for (i = 0U; i < 4U; i++) {
        (void)Motor_Detect_OffsetFeedDriven(&st, 100.0f, 0.0f);
    }
    Motor_Detect_OffsetFinalizeDriven(&st);
    assert(st.offset_warn);

    {
        float original[3], values[3];
        memcpy(original, motor.currentOffset, sizeof(original));
        memcpy(values, original, sizeof(values));
        values[0] += 0.2f;
        values[2] = NAN;
        assert(!Motor_Detect_OffsetSetCurrentRam(&motor, values));
        assert(memcmp(original, motor.currentOffset, sizeof(original)) == 0);
        values[2] = original[2];
        motor.ol_enable = true;
        assert(!Motor_Detect_OffsetSetCurrentRam(&motor, values));
        motor.ol_enable = false;
        values[2] += 2.0f;
        assert(!Motor_Detect_OffsetSetCurrentRam(&motor, values));
        values[2] = original[2];
        assert(Motor_Detect_OffsetSetCurrentRam(&motor, values));
        assert(Motor_Detect_OffsetSetCurrentRam(&motor, original));
        assert(memcmp(original, motor.currentOffset, sizeof(original)) == 0);
        assert(!Motor_Detect_OffsetSetCurrentRam(NULL, values));
    }
    puts("test_motor_detect_offset: pass");
    return 0;
}
