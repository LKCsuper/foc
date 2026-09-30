/*
 * @Description: UVW 线序检测：开环正转与霍尔扇区方向对照
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "motor_phase_order.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef MOTOR_PHASE_ORDER_HOST_TEST
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "hal_motor.h"
#include "motor.h"
#include "motor_detect.h"
#include "motor_feedback.h"
#include "motor_hall.h"
#include "motor_param.h"
#include "motor_protect.h"
#include "motor_settings.h"
#endif

/* Private define ------------------------------------------------------------*/
#ifndef MOTOR_PHASE_ORDER_HOST_TEST
#define MOTOR_PHASE_ORDER_RPM         (50.0f)
#define MOTOR_PHASE_ORDER_DURATION_MS (2500U)
#define MOTOR_PHASE_ORDER_POLL_MS     (20U)
#define MOTOR_PHASE_ORDER_MIN_STEPS   (3)
#define MOTOR_PHASE_ORDER_BOOST_V     (3.5f)
#endif

/*
 * 标准六步霍尔序（一种常见接线）：1→3→2→6→4→5→1
 * 扇区号仅用于相邻步进符号；不依赖 hall_table 标定。
 */
static const int8_t s_hall_to_sector[8] = {
    -1, /* 0 无效 */
    0,  /* 1 */
    2,  /* 2 */
    1,  /* 3 */
    4,  /* 4 */
    5,  /* 5 */
    3,  /* 6 */
    -1, /* 7 无效 */
};

/**
 * @description: 相邻两次霍尔状态的扇区步进
 * @param {int} prev_hall  上一霍尔
 * @param {int} curr_hall  当前霍尔
 * @return {int32_t} +1 / -1 / 0
 * @author: lkc
 */
int32_t Motor_PhaseOrder_HallStep(int prev_hall, int curr_hall)
{
    int8_t a;
    int8_t b;
    int diff;

    if (prev_hall < 0 || prev_hall > 7 || curr_hall < 0 || curr_hall > 7) {
        return 0;
    }
    a = s_hall_to_sector[prev_hall];
    b = s_hall_to_sector[curr_hall];
    if (a < 0 || b < 0 || a == b) {
        return 0;
    }
    diff = (int)b - (int)a;
    if (diff > 3) {
        diff -= 6;
    }
    if (diff < -3) {
        diff += 6;
    }
    /* 只认单扇区步进，跳步当作噪声丢掉 */
    if (diff == 1 || diff == -1) {
        return (int32_t)diff;
    }
    return 0;
}

/**
 * @description: 由方向得分与命令转速符号判定线序
 * @param {int32_t} dir_delta       有符号方向得分
 * @param {float} commanded_rpm     开环目标转速 (RPM)
 * @param {int32_t} min_steps       运动门限
 * @return {motor_phase_order_result_t} 判定结果
 * @author: lkc
 */
motor_phase_order_result_t Motor_PhaseOrder_Judge(int32_t dir_delta, float commanded_rpm, int32_t min_steps)
{
    int32_t abs_delta;
    int32_t gate;

    if (!isfinite(commanded_rpm) || commanded_rpm == 0.0f) {
        return MOTOR_PHASE_ORDER_INVALID;
    }
    gate = (min_steps < 1) ? 1 : min_steps;
    abs_delta = (dir_delta >= 0) ? dir_delta : -dir_delta;
    if (abs_delta < gate) {
        return MOTOR_PHASE_ORDER_NO_MOTION;
    }
    /* 正转命令期望霍尔正序步进；反向则 U/V 与软件正方向相反 */
    if ((commanded_rpm > 0.0f && dir_delta > 0) || (commanded_rpm < 0.0f && dir_delta < 0)) {
        return MOTOR_PHASE_ORDER_OK;
    }
    return MOTOR_PHASE_ORDER_SWAP_UV;
}

/**
 * @description: 结论转短字符串
 * @param {motor_phase_order_result_t} result  结论
 * @return {const char} * 静态名
 * @author: lkc
 */
const char *Motor_PhaseOrder_ResultName(motor_phase_order_result_t result)
{
    switch (result) {
        case MOTOR_PHASE_ORDER_OK:
            return "OK";
        case MOTOR_PHASE_ORDER_SWAP_UV:
            return "SWAP_UV";
        case MOTOR_PHASE_ORDER_NO_MOTION:
            return "NO_MOTION";
        case MOTOR_PHASE_ORDER_FAULT:
            return "FAULT";
        case MOTOR_PHASE_ORDER_BUSY:
            return "BUSY";
        case MOTOR_PHASE_ORDER_INVALID:
            return "INVALID";
        default:
            return "UNKNOWN";
    }
}

#ifndef MOTOR_PHASE_ORDER_HOST_TEST
/**
 * @description: 板上执行 UVW 线序检测
 * @param {motor_phase_order_report_t} *report  输出报告
 * @return {motor_phase_order_result_t} 结论
 * @author: lkc
 */
motor_phase_order_result_t Motor_PhaseOrder_Detect(motor_phase_order_report_t *report)
{
    motor_command_t cmd;
    motor_settings_t settings;
    motor_feedback_source_t fb_saved;
    int hall_prev;
    int hall_now;
    int32_t dir_score;
    uint32_t waited_ms;
    float boost_saved;
    int mode_saved;
    bool sl_saved;
    bool align_saved;
    bool hfi_saved;
    motor_phase_order_result_t result;

    if (report == NULL) {
        return MOTOR_PHASE_ORDER_INVALID;
    }
    memset(report, 0, sizeof(*report));
    report->commanded_rpm = MOTOR_PHASE_ORDER_RPM;
    report->duration_s = (float)MOTOR_PHASE_ORDER_DURATION_MS / 1000.0f;
    report->boost_v = MOTOR_PHASE_ORDER_BOOST_V;

    if (motor.ol_enable || Motor_Detect_IsActive() || Motor_Hall_DetectIsActive()) {
        report->result = MOTOR_PHASE_ORDER_BUSY;
        return report->result;
    }
    if (Motor_Protect_IsFaultLatched()) {
        report->result = MOTOR_PHASE_ORDER_FAULT;
        return report->result;
    }
    if (!Motor_Settings_Capture(&motor, &settings) || Motor_GetCommand(&motor, &cmd) != MOTOR_OK) {
        report->result = MOTOR_PHASE_ORDER_INVALID;
        return report->result;
    }

    mode_saved = cmd.mode;
    boost_saved = settings.config.ol_v_boost;
    sl_saved = settings.config.sensorless_enable;
    align_saved = settings.config.ol_align_enable;
    hfi_saved = settings.config.hfi_start_enable;
    fb_saved = Motor_GetFeedbackSource();

    settings.config.ol_v_boost = MOTOR_PHASE_ORDER_BOOST_V;
    settings.config.sensorless_enable = false;
    settings.config.ol_align_enable = false;
    settings.config.hfi_start_enable = false;
    if (Motor_SetConfig(&motor, &settings.config) != MOTOR_OK) {
        report->result = MOTOR_PHASE_ORDER_INVALID;
        return report->result;
    }
    (void)Motor_SetFeedbackSource(MOTOR_FEEDBACK_SRC_OPENLOOP);

    cmd.mode = MOTOR_OL_MODE_VF;
    cmd.target_rpm = MOTOR_PHASE_ORDER_RPM;
    cmd.enable = false;
    cmd.iq_set = 0.0f;
    cmd.id_set = 0.0f;
    cmd.speed_enable = false;
    (void)Motor_SetCommand(&motor, &cmd);

    printf("PHASE ORDER: VF +%.0fRPM boost=%.1fV for %.1fs (watch hall)\r\n",
           (double)MOTOR_PHASE_ORDER_RPM, (double)MOTOR_PHASE_ORDER_BOOST_V, (double)report->duration_s);

    Motor_Protect_ClearFault();
    hall_prev = Motor_Hall_Read(false, 0);
    dir_score = 0;
    printf("PHASE ORDER: hall_start=%d (hand-turn: show should change 1..6)\r\n", hall_prev);

    cmd.enable = true;
    if (Motor_SetCommand(&motor, &cmd) != MOTOR_OK) {
        report->result = MOTOR_PHASE_ORDER_INVALID;
        goto restore;
    }

    waited_ms = 0U;
    while (waited_ms < MOTOR_PHASE_ORDER_DURATION_MS) {
        if (Motor_Protect_IsFaultLatched()) {
            report->dir_delta = dir_score;
            report->result = MOTOR_PHASE_ORDER_FAULT;
            printf("PHASE ORDER aborted by fault: %s\r\n",
                   Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
            (void)Motor_RequestStop(&motor);
            goto restore;
        }
        osDelay(pdMS_TO_TICKS(MOTOR_PHASE_ORDER_POLL_MS));
        waited_ms += MOTOR_PHASE_ORDER_POLL_MS;
        hall_now = Motor_Hall_Read(false, 0);
        dir_score += Motor_PhaseOrder_HallStep(hall_prev, hall_now);
        if (hall_now >= 1 && hall_now <= 6) {
            hall_prev = hall_now;
        }
    }
    printf("PHASE ORDER: hall_end=%d\r\n", hall_prev);

    (void)Motor_RequestStop(&motor);
    Hal_Motor_PwmStop();

    report->dir_delta = dir_score;
    result = Motor_PhaseOrder_Judge(report->dir_delta, MOTOR_PHASE_ORDER_RPM, MOTOR_PHASE_ORDER_MIN_STEPS);
    report->result = result;

    printf("PHASE ORDER: hall_dir=%ld result=%s\r\n", (long)report->dir_delta,
           Motor_PhaseOrder_ResultName(result));
    if (result == MOTOR_PHASE_ORDER_OK) {
        printf("PHASE ORDER: UVW matches software positive direction\r\n");
    } else if (result == MOTOR_PHASE_ORDER_SWAP_UV) {
        printf("PHASE ORDER: reverse hall sequence — swap motor U and V, then retry\r\n");
    } else if (result == MOTOR_PHASE_ORDER_NO_MOTION) {
        printf("PHASE ORDER: little hall motion — free shaft / raise boost / check HALL\r\n");
    }

restore:
    (void)Motor_RequestStop(&motor);
    if (Motor_Settings_Capture(&motor, &settings)) {
        settings.config.ol_v_boost = boost_saved;
        settings.config.sensorless_enable = sl_saved;
        settings.config.ol_align_enable = align_saved;
        settings.config.hfi_start_enable = hfi_saved;
        (void)Motor_SetConfig(&motor, &settings.config);
    }
    if (Motor_GetCommand(&motor, &cmd) == MOTOR_OK) {
        cmd.mode = mode_saved;
        cmd.enable = false;
        cmd.target_rpm = 0.0f;
        (void)Motor_SetCommand(&motor, &cmd);
    }
    (void)Motor_SetFeedbackSource(fb_saved);
    return report->result;
}
#endif /* MOTOR_PHASE_ORDER_HOST_TEST */

#ifdef __cplusplus
}
#endif

