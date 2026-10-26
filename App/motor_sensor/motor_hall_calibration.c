/* 时间驱动霍尔标定；所有状态由调用者持有。 */
/* Includes ------------------------------------------------------------------*/
#include "motor_hall_calibration.h"
#include "common_assert.h"
#include "motor_math.h"
#include "motor_foc.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define HALL_CAL_ALIGN_STEPS 800U /* 对齐电流爬升步数（每步约 1 ms，共约 0.8 s） */
#define HALL_CAL_HOLD_MS 600U     /* 对齐完成后保持时间 (ms) */
#define HALL_CAL_STEP_MS 25U      /* 扫角每步保持时间 (ms) */
#define HALL_CAL_DEGREES 360U     /* 每圈扫角度步数 */
#define HALL_CAL_LOOPS 2U         /* 正转、反转各扫圈数 */

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 置标定终态并清零电流/角度目标，供调用者安全撤销输出权并停机。
 * @param {motor_hall_calibration_t *} state 标定状态，不可为空
 * @param {motor_hall_calibration_stage_t} stage 目标终态（DONE/CANCELLED/FAULT/TIMEOUT/INVALID）
 * @return {void} 无
 */
static void Hall_CalibrationFinish(motor_hall_calibration_t *state, motor_hall_calibration_stage_t stage)
{
    state->stage = stage;
    state->theta_rad = 0.0f;
    state->id_target = 0.0f;
    state->iq_target = 0.0f;
}

/**
 * @description: 查询标定状态机是否仍处于活动阶段（占用标定输出）。
 * @param {const motor_hall_calibration_t *} state  标定状态，不可为空
 * @return {bool} true=处于 ALIGN..REVERSE 活动态
 */
bool Motor_Hall_CalibrationIsActive(const motor_hall_calibration_t *state)
{
    COMMON_ASSERT(state != NULL);
    return state->stage >= MOTOR_HALL_CAL_ALIGN && state->stage <= MOTOR_HALL_CAL_REVERSE;
}

/**
 * @description: 开始/重新开始霍尔标定状态机。
 * @detail: current 须为有限正电流 (A)；now_ms 与后续 Step 共用同一毫秒时基；timeout_ms 须在 1..INT32_MAX。
 *          失败返回 false 并置 INVALID。调用者应先停机再独占输出。
 * @param {motor_hall_calibration_t *} state  标定状态，不可为空
 * @param {float} current                     标定电流 (A)
 * @param {uint32_t} now_ms                   开始时刻 (ms)
 * @param {uint32_t} timeout_ms               总超时 (ms)
 * @return {bool} true=已进入 ALIGN 阶段；false=参数非法
 */
bool Motor_Hall_CalibrationStart(motor_hall_calibration_t *state, float current, uint32_t now_ms,
                                 uint32_t timeout_ms)
{
    COMMON_ASSERT(state != NULL);
    memset(state, 0, sizeof(*state));
    memset(state->table, 255, sizeof(state->table));
    if (!isfinite(current) || current <= 0.0f || timeout_ms == 0U || timeout_ms > INT32_MAX) {
        state->stage = MOTOR_HALL_CAL_INVALID;
        return false;
    }
    state->stage = MOTOR_HALL_CAL_ALIGN;
    state->current = current;
    state->started_ms = state->changed_ms = now_ms;
    state->timeout_ms = timeout_ms;
    return true;
}

/**
 * @description: 保留原实现的 >30 样本门限、圆均值与 200/360 编码。
 * +Iq 慢爬时转子 d 轴跟随强制 Park 角的 +90° 电流矢量，因此表保存该修正角。
 * @param {motor_hall_calibration_t *} state 实例状态
 * @return {void} 无
 */
static void Hall_CalibrationBuildTable(motor_hall_calibration_t *state)
{
    unsigned i;
    for (i = 0; i < 8U; ++i) {
        if (state->samples[i] > 30U) {
            float angle = RAD2DEG_f(atan2f(state->sin_sum[i], state->cos_sum[i])) + 90.0f;
            utils_norm_angle(&angle);
            state->table[i] = (uint8_t)(angle * 200.0f / 360.0f);
        }
    }
}

/**
 * @description: 非阻塞推进标定状态机一步。
 * @detail: hall 为本次采集的 0..7 原始读数，仅扫角到期时消费。fault 优先于 cancel，均优先于超时。
 *          终态幂等且清零电流/角度目标；调用者须在终态撤销标定输出并停机。
 * @param {motor_hall_calibration_t *} state  标定状态，不可为空
 * @param {uint32_t} now_ms                   本次时刻 (ms)
 * @param {int} hall                          本次读取的霍尔状态 0..7
 * @param {bool} cancel                       true=请求取消
 * @param {bool} fault                        true=发生保护故障
 * @return {void} 无
 */
void Motor_Hall_CalibrationStep(motor_hall_calibration_t *state, uint32_t now_ms, int hall,
                                bool cancel, bool fault)
{
    uint32_t delay_ms;
    float sine;
    float cosine;
    COMMON_ASSERT(state != NULL);
    state->sampled_forward = false;
    if (!Motor_Hall_CalibrationIsActive(state)) return;
    if (fault || cancel || (uint32_t)(now_ms - state->started_ms) >= state->timeout_ms) {
        Hall_CalibrationFinish(state, fault ? MOTOR_HALL_CAL_FAULT :
                                      cancel ? MOTOR_HALL_CAL_CANCELLED : MOTOR_HALL_CAL_TIMEOUT);
        return;
    }
    delay_ms = state->stage == MOTOR_HALL_CAL_ALIGN ? 1U :
               state->stage == MOTOR_HALL_CAL_HOLD ? HALL_CAL_HOLD_MS : HALL_CAL_STEP_MS;
    if ((uint32_t)(now_ms - state->changed_ms) < delay_ms) return;
    /* 不追赶迟到的调度；每个目标都需要实际保持完整的等待时间。 */
    state->changed_ms = now_ms;
    if (state->stage == MOTOR_HALL_CAL_ALIGN) {
        ++state->step;
        state->id_target = state->current * ((float)state->step / (float)HALL_CAL_ALIGN_STEPS);
        if (state->step == HALL_CAL_ALIGN_STEPS) state->stage = MOTOR_HALL_CAL_HOLD;
        return;
    }
    if (state->stage == MOTOR_HALL_CAL_HOLD) {
        state->stage = MOTOR_HALL_CAL_FORWARD;
        state->step = 0U;
        state->id_target = 0.0f;
        state->iq_target = state->current;
        return;
    }
    if (hall < 0 || hall > 7) {
        Hall_CalibrationFinish(state, MOTOR_HALL_CAL_INVALID);
        return;
    }
    sincos_f32(state->theta_rad, &sine, &cosine);
    state->sin_sum[hall] += sine;
    state->cos_sum[hall] += cosine;
    ++state->samples[hall];
    state->sampled_forward = state->stage == MOTOR_HALL_CAL_FORWARD;
    if (state->stage == MOTOR_HALL_CAL_FORWARD) {
        if (state->step < HALL_CAL_DEGREES) {
            ++state->step;
        } else if (++state->loop < HALL_CAL_LOOPS) {
            state->step = 0U;
        } else {
            state->stage = MOTOR_HALL_CAL_REVERSE;
            state->loop = 0U;
        }
    } else {
        if (state->step > 0U) {
            --state->step;
        } else if (++state->loop < HALL_CAL_LOOPS) {
            state->step = HALL_CAL_DEGREES;
        } else {
            Hall_CalibrationBuildTable(state);
            Hall_CalibrationFinish(state, MOTOR_HALL_CAL_DONE);
            return;
        }
    }
    state->theta_rad = DEG2RAD_f((float)state->step);
}
