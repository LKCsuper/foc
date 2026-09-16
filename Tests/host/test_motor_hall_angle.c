/*
 * @Description: 霍尔表解码与 Hall-PLL 转速估计主机测试
 * @Author: lkc
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_hall_angle.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const int k_hall_seq[6] = {1, 3, 2, 6, 4, 5};
/* 实机 Hall 每 32 次 ADC 触发更新一次，测试须使用相同 dt。 */
static const float k_hall_dt = 32.0f / 30000.0f;

static void fill_ready_table(int8_t table[MOTOR_HALL_TABLE_SIZE])
{
    int i;

    for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
        table[i] = MOTOR_HALL_TABLE_INVALID;
    }
    /* 6 个有效扇区：约每 60° → entry = deg*200/360 */
    table[1] = (int8_t)(uint8_t)(0.0f * 200.0f / 360.0f);
    table[3] = (int8_t)(uint8_t)(60.0f * 200.0f / 360.0f);
    table[2] = (int8_t)(uint8_t)(120.0f * 200.0f / 360.0f);
    table[6] = (int8_t)(uint8_t)(180.0f * 200.0f / 360.0f);
    table[4] = (int8_t)(uint8_t)(240.0f * 200.0f / 360.0f);
    table[5] = (int8_t)(uint8_t)(300.0f * 200.0f / 360.0f);
    /* 0、7 无效 */
}

static void drive_hall_rpm(motor_hall_angle_t *st, const int8_t *table, float pp, float rpm, float duration_s,
                           int *seq_index)
{
    float sector_period;
    float sector_age = 0.0f;
    float t;
    int dir;

    assert(rpm != 0.0f);
    dir = rpm > 0.0f ? 1 : -1;
    sector_period = 10.0f / (pp * fabsf(rpm));
    for (t = 0.0f; t < duration_s; t += k_hall_dt) {
        Motor_Hall_AngleUpdate(st, table, k_hall_seq[*seq_index], pp, k_hall_dt);
        sector_age += k_hall_dt;
        if (sector_age >= sector_period) {
            sector_age -= sector_period;
            *seq_index = (*seq_index + dir + 6) % 6;
        }
    }
}

int main(void)
{
    int8_t table[MOTOR_HALL_TABLE_SIZE];
    motor_hall_angle_t st;
    float theta;
    float expect;
    int seq;

    fill_ready_table(table);
    assert(Motor_Hall_TableReady(table));
    assert(Motor_Hall_DecodeAngle(table, 1, &theta));
    assert(fabsf(theta) < 1e-4f || fabsf(theta - 2.0f * (float)M_PI) < 1e-4f);
    assert(!Motor_Hall_DecodeAngle(table, 0, &theta));

    table[1] = MOTOR_HALL_TABLE_INVALID;
    assert(!Motor_Hall_TableReady(table));
    fill_ready_table(table);

    /* 六个有效状态若全映射到同一电角，不得误报标定成功。 */
    table[1] = table[2] = table[3] = table[4] = table[5] = table[6] = 13;
    assert(!Motor_Hall_TableReady(table));
    fill_ready_table(table);

    Motor_Hall_AngleReset(&st);
    Motor_Hall_AngleUpdate(&st, table, 1, 2.0f, 0.01f);
    assert(st.angle_valid);
    assert(st.speed_valid);
    assert(fabsf(st.mech_rpm_fast) < 1e-3f);

    /* 停留超时 → 停转，速度有效为 0 */
    Motor_Hall_AngleUpdate(&st, table, 1, 2.0f, 0.25f);
    assert(st.angle_valid);
    assert(st.speed_valid);
    assert(fabsf(st.mech_rpm) < 1e-4f);
    assert(fabsf(st.mech_rpm_fast) < 1e-4f);

    /* 1 极对、200 RPM：PLL 与六边沿窗口都应收敛到约 200，而不是单边沿台阶。 */
    Motor_Hall_AngleReset(&st);
    seq = 0;
    drive_hall_rpm(&st, table, 1.0f, 200.0f, 2.0f, &seq);
    assert(st.angle_valid);
    assert(st.speed_valid);
    expect = 200.0f;
    assert(fabsf(st.mech_rpm - expect) < 5.0f);
    assert(fabsf(st.mech_rpm_fast - expect) < 8.0f);
    /* 200RPM 位于渐入区，电角不应再钉死在扇区中心。 */
    assert(st.period_count == MOTOR_HALL_SPEED_WINDOW);

    /* 2 对极 72 RPM：速度环用的 PLL 应贴着 72，不能在边沿间掉到 0。 */
    Motor_Hall_AngleReset(&st);
    seq = 0;
    drive_hall_rpm(&st, table, 2.0f, 72.0f, 1.2f, &seq);
    assert(fabsf(st.mech_rpm_fast - 72.0f) < 4.0f);
    assert(fabsf(st.mech_rpm - 72.0f) < 4.0f);
    /* 低于插值门限：电角保持当前扇区中心。 */
    assert(Motor_Hall_DecodeAngle(table, st.last_state, &theta));
    assert(fabsf(st.theta_rad - theta) < 1e-4f ||
           fabsf(fabsf(st.theta_rad - theta) - 2.0f * (float)M_PI) < 1e-4f);

    /* 同扇区继续推进若干 ADC 拍，PLL 速度不得塌成边沿周期测速那样的台阶。 */
    {
        float rpm_a = st.mech_rpm_fast;
        int hold_state = st.last_state;
        int i;

        for (i = 0; i < 18; i++) {
            Motor_Hall_AngleUpdate(&st, table, hold_state, 2.0f, k_hall_dt);
        }
        assert(fabsf(st.mech_rpm_fast - rpm_a) < 8.0f);
        assert(fabsf(st.mech_rpm_fast) > 50.0f);
    }

    /* 反转 −72 RPM。 */
    Motor_Hall_AngleReset(&st);
    seq = 0;
    drive_hall_rpm(&st, table, 2.0f, -72.0f, 1.2f, &seq);
    assert(st.direction == -1);
    assert(fabsf(st.mech_rpm_fast + 72.0f) < 4.0f);

    /* 高速也不能因六扇区计数回绕而锁到错误速度。 */
    Motor_Hall_AngleReset(&st);
    seq = 0;
    drive_hall_rpm(&st, table, 2.0f, 800.0f, 2.0f, &seq);
    assert(!st.pll_low_speed_active);
    assert(fabsf(st.mech_rpm_fast - 800.0f) < 40.0f);

    /* 从高速降至低速时重新以当前周期速度初始化 PLL，不得沿用高速误锁状态。 */
    drive_hall_rpm(&st, table, 2.0f, 72.0f, 2.0f, &seq);
    assert(st.pll_low_speed_active);
    assert(fabsf(st.mech_rpm_fast - 72.0f) < 8.0f);

    /* ADC 节拍在相邻扇区来回跳：不得把 PLL 速度打飞，也不得在回绕里卡死。 */
    Motor_Hall_AngleReset(&st);
    Motor_Hall_AngleUpdate(&st, table, 1, 2.0f, k_hall_dt);
    {
        int i;

        for (i = 0; i < 30000; i++) {
            Motor_Hall_AngleUpdate(&st, table, (i & 1) ? 1 : 3, 2.0f, k_hall_dt);
        }
        assert(fabsf(st.mech_rpm_fast) < 20000.0f);
    }

    /* 非法扇区：角无效 */
    Motor_Hall_AngleUpdate(&st, table, 0, 2.0f, k_hall_dt);
    assert(!st.angle_valid);
    assert(!st.speed_valid);

    puts("test_motor_hall_angle: pass");
    return 0;
}
