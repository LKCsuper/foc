/*
 * @Description: UVW 线序判定主机单测（霍尔步进，无硬件）
 * @Author: lkc
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "motor_phase_order.h"

int main(void)
{
    /* 标准正序：1→3→2→6→4→5 */
    assert(Motor_PhaseOrder_HallStep(1, 3) == 1);
    assert(Motor_PhaseOrder_HallStep(3, 2) == 1);
    assert(Motor_PhaseOrder_HallStep(2, 6) == 1);
    assert(Motor_PhaseOrder_HallStep(5, 1) == 1);
    /* 反序 */
    assert(Motor_PhaseOrder_HallStep(3, 1) == -1);
    assert(Motor_PhaseOrder_HallStep(1, 5) == -1);
    /* 无效 / 同态 / 跳步 */
    assert(Motor_PhaseOrder_HallStep(1, 1) == 0);
    assert(Motor_PhaseOrder_HallStep(0, 1) == 0);
    assert(Motor_PhaseOrder_HallStep(1, 2) == 0);

    assert(Motor_PhaseOrder_Judge(5, 50.0f, 3) == MOTOR_PHASE_ORDER_OK);
    assert(Motor_PhaseOrder_Judge(-5, 50.0f, 3) == MOTOR_PHASE_ORDER_SWAP_UV);
    assert(Motor_PhaseOrder_Judge(-5, -50.0f, 3) == MOTOR_PHASE_ORDER_OK);
    assert(Motor_PhaseOrder_Judge(5, -50.0f, 3) == MOTOR_PHASE_ORDER_SWAP_UV);
    assert(Motor_PhaseOrder_Judge(1, 50.0f, 3) == MOTOR_PHASE_ORDER_NO_MOTION);
    assert(Motor_PhaseOrder_Judge(5, 0.0f, 3) == MOTOR_PHASE_ORDER_INVALID);
    assert(strcmp(Motor_PhaseOrder_ResultName(MOTOR_PHASE_ORDER_SWAP_UV), "SWAP_UV") == 0);
    printf("test_motor_phase_order: OK\n");
    return 0;
}
