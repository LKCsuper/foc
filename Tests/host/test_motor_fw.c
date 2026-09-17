#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_fw.h"

int main(void)
{
    motor_fw_t fw;
    float id;
    float dt = 0.001f;
    unsigned i;

    Motor_Fw_Reset(&fw);
    assert(fabsf(fw.id) < 1e-6f);

    /* 电压未到门槛：保持 0 */
    id = Motor_Fw_Step(&fw, 2.0f, 0.80f, 1000.0f, 5.0f, 5.0f, 13.9f, dt);
    assert(fabsf(id) < 1e-6f);

    /* 电压顶满：斜坡到 −current_max */
    Motor_Fw_Reset(&fw);
    for (i = 0; i < 500U; ++i) {
        id = Motor_Fw_Step(&fw, 2.0f, 0.80f, 8.0f, 12.0f, 8.0f, 13.9f, dt);
    }
    assert(fabsf(id + 2.0f) < 1e-3f);

    /* current_max=0 立即关闭 */
    id = Motor_Fw_Step(&fw, 0.0f, 0.80f, 8.0f, 12.0f, 8.0f, 13.9f, dt);
    assert(fabsf(id) < 1e-6f);

    /* 滞回：略低于 start 但不低于 exit 时保持 */
    Motor_Fw_Reset(&fw);
    fw.id = -1.0f;
    id = Motor_Fw_Step(&fw, 2.0f, 0.80f, 1000.0f, 10.8f, 0.0f, 13.9f, dt);
    assert(fabsf(id + 1.0f) < 1e-3f);

    /* 明显退出 */
    id = Motor_Fw_Step(&fw, 2.0f, 0.80f, 1000.0f, 5.0f, 0.0f, 13.9f, dt);
    assert(fabsf(id) < 1e-3f);

    /* 3 A 电源：Iq 占满后弱磁余量为 0；Iq=0.8 A 时约剩 2.37 A */
    assert(fabsf(Motor_Fw_IdRoom(2.5f, MOTOR_FW_I_BUDGET)) < 1e-6f);
    assert(fabsf(Motor_Fw_IdRoom(0.8f, MOTOR_FW_I_BUDGET) - sqrtf(2.5f * 2.5f - 0.8f * 0.8f)) < 1e-5f);
    assert(Motor_Fw_IdRoom(0.0f, MOTOR_FW_I_BUDGET) > 2.49f);

    printf("test_motor_fw: PASS\n");
    return 0;
}
