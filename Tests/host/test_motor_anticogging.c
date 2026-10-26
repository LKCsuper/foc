/* 齿槽补偿：学习/完成/使能与清空状态机及查询电压。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_anticogging.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    motor_anticogging_status_t status;
    unsigned bin;

    Motor_Anticogging_Init();
    assert(Motor_Anticogging_Start(1U));
    Motor_Anticogging_SetLearnTarget(1.0f, 0.02f);
    {
        float theta;
        float vd;
        float vq;
        assert(Motor_Anticogging_GetLearnTargets(&theta, &vd, &vq));
        assert(fabsf(theta - 1.0f) < 1e-6f);
        assert(fabsf(vd) < 1e-6f);
        assert(fabsf(vq - 0.02f) < 1e-6f);
    }
    for (bin = 0U; bin < MOTOR_ANTICOGGING_BINS; bin++) {
        float angle = ((float)bin + 0.5f) * 2.0f * (float)M_PI /
                      (float)MOTOR_ANTICOGGING_BINS;
        assert(Motor_Anticogging_Record((uint16_t)bin, 0.01f + 0.02f * sinf(84.0f * angle)));
    }
    assert(Motor_Anticogging_Finish());
    Motor_Anticogging_GetStatus(&status);
    assert(!status.learning && status.calibrated && status.enabled);
    assert(status.min_samples == 1U);
    assert(status.total_samples == MOTOR_ANTICOGGING_BINS);
    assert(status.min_voltage < -0.013f && status.max_voltage > 0.013f);
    assert(Motor_Anticogging_SetEnabled(false));
    assert(fabsf(Motor_Anticogging_GetVoltage(0.5f * (float)M_PI)) < 1e-6f);
    assert(Motor_Anticogging_SetEnabled(true));
    Motor_Anticogging_Clear();
    assert(!Motor_Anticogging_SetEnabled(true));
    puts("test_motor_anticogging: pass");
    return 0;
}
