/* 速度前馈拟合主机测试：V/RPM 斜率与截距，拒绝饱和数据 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_speed_ff_detect.h"
/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_speed_ff_fit_t fit;
    motor_speed_ff_result_t result;
    const float rpm[] = {300, 600, 900, 1200, -300, -600, -900, -1200};
    unsigned i;
    Motor_SpeedFfFit_Init(&fit);
    assert(!Motor_SpeedFfFit_Add(&fit, 0.0f, 0.0f));
    for (i = 0U; i < sizeof(rpm) / sizeof(rpm[0]); i++) {
        float sign = rpm[i] > 0.0f ? 1.0f : -1.0f;
        assert(Motor_SpeedFfFit_Add(&fit, rpm[i], sign * (0.08f + 0.00308f * fabsf(rpm[i]))));
    }
    assert(Motor_SpeedFfFit_Solve(&fit, &result));
    assert(fabsf(result.slope_v_per_rpm - 0.00308f) < 1.0e-6f);
    assert(fabsf(result.boost_v - 0.08f) < 1.0e-5f);
    /* 单精度累计量的平方差存在消减误差；这里只验证拟合残差仍远小于实机门槛。 */
    assert(result.rms_error_v < 0.01f && result.points == 8U);

    /* 高转速饱和数据会拟合出负截距，必须拒绝，不能错误报告 PASS。 */
    Motor_SpeedFfFit_Init(&fit);
    assert(Motor_SpeedFfFit_Add(&fit, 651.1f, 1.662f));
    assert(Motor_SpeedFfFit_Add(&fit, 1299.4f, 3.423f));
    assert(Motor_SpeedFfFit_Add(&fit, 1940.4f, 5.472f));
    assert(Motor_SpeedFfFit_Add(&fit, 2414.9f, 7.314f));
    assert(!Motor_SpeedFfFit_Solve(&fit, &result));
    puts("test_motor_speed_ff_detect: pass");
    return 0;
}
