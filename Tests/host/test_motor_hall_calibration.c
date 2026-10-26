/* 霍尔标定核心状态机主机测试：参数校验、对齐爬升、扫角建表、实例隔离、时间回绕与各退出路径。 */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_hall_calibration.h"

/**
 * @description: 断言标定已结束（非活动态）且电流/角度目标清零。
 * @param {const motor_hall_calibration_t *} s s 参数
 * @return {void} 无
 */
static void stopped(const motor_hall_calibration_t *s)
{
    assert(!Motor_Hall_CalibrationIsActive(s));
    assert(s->id_target == 0.0f && s->iq_target == 0.0f && s->theta_rad == 0.0f);
}

/**
 * @description: 以 25 ms 步进推进扫角直至终态，验证正反转步数、表编码与各扇区采样门限。
 * @param {motor_hall_calibration_t *} s s 参数
 * @param {uint32_t *} now 当前时刻 (tick)
 * @return {void} 无
 */
static void finish_sweep(motor_hall_calibration_t *s, uint32_t *now)
{
    unsigned forward = 0, reverse = 0;
    while (Motor_Hall_CalibrationIsActive(s)) {
        motor_hall_calibration_stage_t stage = s->stage;
        int hall = (int)(s->step % 360U) / 60 + 1;
        *now += 25U;
        Motor_Hall_CalibrationStep(s, *now, hall, false, false);
        if (stage == MOTOR_HALL_CAL_FORWARD) ++forward;
        if (stage == MOTOR_HALL_CAL_REVERSE) ++reverse;
    }
    assert(forward == 722U && reverse == 722U);
    assert(s->stage == MOTOR_HALL_CAL_DONE);
    stopped(s);
    assert(s->table[0] == 255U && s->table[7] == 255U);
    for (unsigned i = 1; i <= 6; ++i) {
        /* 扇区中心=(i-1)*60+29.5°；保留 +90° 转子 d 轴修正。 */
        float degrees = fmodf((float)(i - 1U) * 60.0f + 29.5f + 90.0f, 360.0f);
        assert(fabsf((float)s->table[i] - degrees * 200.0f / 360.0f) < 1.1f);
        assert(s->samples[i] >= 240U);
    }
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @param {int} argc 命令参数个数
 * @param {char **} argv 兼容参数，本实现不使用
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(int argc, char **argv)
{
    motor_hall_calibration_t a = {0}, b = {0}, snapshot;
    uint32_t now = UINT32_MAX - 500U;
    (void)argv;
    if (argc > 1) { Motor_Hall_CalibrationStep(NULL, 0, 1, false, false); return 1; }
    assert(!Motor_Hall_CalibrationStart(&a, NAN, 0, 120000)); stopped(&a);
    assert(!Motor_Hall_CalibrationStart(&a, INFINITY, 0, 120000));
    assert(!Motor_Hall_CalibrationStart(&a, -1, 0, 120000));
    assert(!Motor_Hall_CalibrationStart(&a, 0, 0, 120000));
    assert(!Motor_Hall_CalibrationStart(&a, 1, 0, 0));
    assert(!Motor_Hall_CalibrationStart(&a, 1, 0, UINT32_MAX));
    assert(Motor_Hall_CalibrationStart(&a, FLT_MAX, 0, 120000));
    for (unsigned i = 1; i <= 800; ++i) {
        Motor_Hall_CalibrationStep(&a, i, 1, false, false);
        assert(isfinite(a.id_target));
    }
    assert(Motor_Hall_CalibrationStart(&a, 0.6f, now, 120000));
    assert(Motor_Hall_CalibrationStart(&b, 1.2f, now, 120000));
    snapshot = b;
    Motor_Hall_CalibrationStep(&a, now, 1, false, false);
    assert(a.step == 0U);
    for (unsigned i = 1; i <= 800; ++i) {
        ++now;
        Motor_Hall_CalibrationStep(&a, now, 1, false, false);
        assert(fabsf(a.id_target - 0.6f * (float)i / 800.0f) < 1e-6f);
        assert(a.iq_target == 0.0f && a.theta_rad == 0.0f);
    }
    assert(memcmp(&snapshot, &b, sizeof(b)) == 0); /* 独立实例隔离 */
    assert(a.stage == MOTOR_HALL_CAL_HOLD);
    now += 599U;
    Motor_Hall_CalibrationStep(&a, now, 1, false, false);
    assert(a.stage == MOTOR_HALL_CAL_HOLD);
    ++now;
    Motor_Hall_CalibrationStep(&a, now, 1, false, false);
    assert(a.stage == MOTOR_HALL_CAL_FORWARD && a.iq_target == 0.6f && a.id_target == 0);
    finish_sweep(&a, &now);
    snapshot = a;
    Motor_Hall_CalibrationStep(&a, now + 1000U, -1, true, true);
    assert(a.stage == MOTOR_HALL_CAL_DONE && memcmp(a.table, snapshot.table, 8) == 0);

    for (unsigned stage = MOTOR_HALL_CAL_ALIGN; stage <= MOTOR_HALL_CAL_REVERSE; ++stage) {
        assert(Motor_Hall_CalibrationStart(&a, 1, 100, 1000));
        a.stage = (motor_hall_calibration_stage_t)stage;
        Motor_Hall_CalibrationStep(&a, 100, 1, true, false);
        assert(a.stage == MOTOR_HALL_CAL_CANCELLED); stopped(&a);
        assert(Motor_Hall_CalibrationStart(&a, 1, 100, 1000));
        a.stage = (motor_hall_calibration_stage_t)stage;
        Motor_Hall_CalibrationStep(&a, 1100, 1, true, true);
        assert(a.stage == MOTOR_HALL_CAL_FAULT); stopped(&a);
    }
    assert(Motor_Hall_CalibrationStart(&a, 1, UINT32_MAX - 100U, 200));
    Motor_Hall_CalibrationStep(&a, 98, 1, false, false);
    assert(a.stage == MOTOR_HALL_CAL_ALIGN && a.step == 1U); /* 迟到只推进一步 */
    Motor_Hall_CalibrationStep(&a, 99, 1, false, false);
    assert(a.stage == MOTOR_HALL_CAL_TIMEOUT); stopped(&a);
    assert(Motor_Hall_CalibrationStart(&a, 1, 0, 120000));
    a.stage = MOTOR_HALL_CAL_FORWARD;
    Motor_Hall_CalibrationStep(&a, 25, 8, false, false);
    assert(a.stage == MOTOR_HALL_CAL_INVALID); stopped(&a);
    puts("Hall calibration core: timing, table, isolation, wrap, abort and timeout passed");
    return 0;
}
