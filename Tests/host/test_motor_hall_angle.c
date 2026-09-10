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

    /* 换向 1→3（+60° 电角）/ 0.05s → elec = 1/6 / 0.05 rps */
    Motor_Hall_AngleReset(&st);
    Motor_Hall_AngleUpdate(&st, table, 1, 1.0f, 0.01f);
    Motor_Hall_AngleUpdate(&st, table, 3, 1.0f, 0.05f);
    assert(st.angle_valid);
    assert(st.speed_valid);
    expect = (33.0f / 200.0f / 0.05f) * 60.0f; /* entry 33 ≈ 60° */
    assert(fabsf(st.mech_rpm - expect) < 1.0f);

    /* 非法扇区：角无效 */
    Motor_Hall_AngleUpdate(&st, table, 0, 1.0f, 0.01f);
    assert(!st.angle_valid);
    assert(!st.speed_valid);

    puts("test_motor_hall_angle: pass");
    return 0;
}
