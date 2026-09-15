/*
 * @Description: 霍尔表解码与转速估计主机测试
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

int main(void)
{
    int8_t table[MOTOR_HALL_TABLE_SIZE];
    motor_hall_angle_t st;
    float theta;
    float expect;

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
    assert(!st.speed_valid);

    /* 停留超时 → 停转，速度有效为 0 */
    Motor_Hall_AngleUpdate(&st, table, 1, 2.0f, 0.25f);
    assert(st.angle_valid);
    assert(st.speed_valid);
    assert(fabsf(st.mech_rpm) < 1e-4f);

    /* 换向 1→3（固定 1/6 电角周）/ 0.05s，1 极对 → 200RPM */
    Motor_Hall_AngleReset(&st);
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.01f);
    Motor_Hall_AngleUpdate(&st, table, 3, 1.0f, 0.05f);
    assert(st.angle_valid);
    assert(st.speed_valid);
    expect = 200.0f;
    assert(fabsf(st.mech_rpm - expect) < 1.0f);
    assert(fabsf(st.mech_rpm_raw - expect) < 1.0f);
    /* 快速支路按标定表量化后的实际角差测速：60° 被编码为 33/200 周，即 198 RPM。 */
    assert(fabsf(st.mech_rpm_fast - 198.0f) < 1.0f);
    /* 200RPM 位于渐入区：扇区中心与完整插值各占一部分。 */
    assert(fabsf(st.theta_rad - (5.0f * (float)M_PI / 18.0f)) < 0.02f);
    Motor_Hall_AngleUpdate(&st, table, 3, 1.0f, 0.025f);
    assert(fabsf(st.theta_rad - ((float)M_PI / 3.0f)) < 0.02f);

    /* 六个略有抖动的边沿周期采用总时间平均，而非平均瞬时 RPM。 */
    Motor_Hall_AngleReset(&st);
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.01f);
    Motor_Hall_AngleUpdate(&st, table, 3, 1.0f, 0.05f);
    Motor_Hall_AngleUpdate(&st, table, 2, 1.0f, 0.04f);
    Motor_Hall_AngleUpdate(&st, table, 6, 1.0f, 0.06f);
    Motor_Hall_AngleUpdate(&st, table, 4, 1.0f, 0.05f);
    Motor_Hall_AngleUpdate(&st, table, 5, 1.0f, 0.04f);
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.06f);
    assert(st.period_count == MOTOR_HALL_SPEED_WINDOW);
    assert(fabsf(st.mech_rpm - 200.0f) < 1.0f);
    assert(fabsf(st.mech_rpm_raw - (10.0f / 0.06f)) < 1.0f);

    /* 边沿迟到时快速值立即受年龄上限约束，不等待 0.2s 停转超时。 */
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.10f);
    assert(fabsf(st.mech_rpm_fast) <= 100.1f);

    /* 反转同样按速度渐入；200RPM 时边界偏移为完整值的 1/3。 */
    Motor_Hall_AngleReset(&st);
    Motor_Hall_AngleUpdate(&st, table, 3, 1.0f, 0.01f);
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.05f);
    assert(st.direction == -1);
    assert(fabsf(st.theta_rad - ((float)M_PI / 18.0f)) < 0.02f);
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.025f);
    assert(fabsf(st.theta_rad) < 0.02f);

    /* 非法扇区：角无效 */
    Motor_Hall_AngleUpdate(&st, table, 0, 1.0f, 0.01f);
    assert(!st.angle_valid);
    assert(!st.speed_valid);

    puts("test_motor_hall_angle: pass");
    return 0;
}
