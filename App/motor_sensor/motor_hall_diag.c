/*
 * @Description: Hall I/F 电流诊断：扇区累计、最差帧、换向窗与 mean/RMS
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "common_assert.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx.h"
#include "motor_hall_diag.h"
#include "motor_param.h"
#include "motor_hall_angle.h"
#include "motor_current.h"
#include "motor_feedback.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static motor_hall_current_diag_sector_t s_hall_current_diag[8];
static motor_hall_current_worst_t s_hall_current_worst;
static motor_hall_current_worst_t s_hall_current_worst_id;
static motor_hall_current_edge_diag_t s_hall_edge_diag;
static float s_hall_prev_ia;
static float s_hall_prev_ib;
static float s_hall_prev_ic;
static int s_hall_prev_state = -1;
static uint8_t s_hall_diag_div;
static bool s_hall_diag_running;
static float s_hall_diag_target_rpm;
static uint16_t s_hall_isr_cnt;
static uint16_t s_hall_isr_arr;
/* Private function prototypes -----------------------------------------------*/
static void Motor_Hall_DiagFillFrame(motor_hall_current_worst_t *dst, float signed_phase_sum, float phase_sum,
                                     int hall_state, float id_error, bool first_half);
static void Motor_Hall_DiagCaptureBeat(float signed_phase_sum, float phase_sum, int hall_state);
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 记下本拍 TIM1 计数和 ARR，供扇区时间诊断。
 * @param {uint16_t} tim1_cnt  TIM1 当前计数
 * @param {uint16_t} tim1_arr  TIM1 自动重装值
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagNoteTimer(uint16_t tim1_cnt, uint16_t tim1_arr)
{
    s_hall_isr_cnt = tim1_cnt;
    s_hall_isr_arr = tim1_arr;
}

/**
 * @description: 在关中断下复制 8 个霍尔扇区的电流统计。
 * @param {motor_hall_current_diag_sector_t[]} sectors  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetSectors(motor_hall_current_diag_sector_t sectors[8])
{
    uint32_t primask;

    COMMON_ASSERT(sectors != NULL);
    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(sectors, s_hall_current_diag, sizeof(s_hall_current_diag));
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @description: 在关中断下复制三相和最大的那一拍快照。
 * @param {motor_hall_current_worst_t *} out  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetWorst(motor_hall_current_worst_t *out)
{
    uint32_t primask;

    COMMON_ASSERT(out != NULL);
    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_hall_current_worst;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @description: 在关中断下复制 Id 误差最大的那一拍快照。
 * @param {motor_hall_current_worst_t *} out  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetWorstId(motor_hall_current_worst_t *out)
{
    uint32_t primask;

    COMMON_ASSERT(out != NULL);
    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_hall_current_worst_id;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @description: 在关中断下复制霍尔换相边沿电流统计。
 * @param {motor_hall_current_edge_diag_t *} out  输出，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagGetEdge(motor_hall_current_edge_diag_t *out)
{
    uint32_t primask;

    COMMON_ASSERT(out != NULL);
    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_hall_edge_diag;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @description: 把本拍电流、电角、重构相写入锁存快照。
 * @param {motor_hall_current_worst_t} *dst 目标快照
 * @param {float} signed_phase_sum  三相电流代数和 (A)
 * @param {float} phase_sum         |Ia+Ib+Ic| (A)
 * @param {int} hall_state          Hall 状态 0..7
 * @param {float} id_error          |Id−目标| (A)
 * @param {bool} first_half         true=扇区前半
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Hall_DiagFillFrame(motor_hall_current_worst_t *dst, float signed_phase_sum, float phase_sum,
                                     int hall_state, float id_error, bool first_half)
{
    int recon_last = 0;
    float center = 0.0f;

    dst->signed_sum_a = signed_phase_sum;
    dst->abs_sum_a = phase_sum;
    dst->ia = motor.ia;
    dst->ib = motor.ib;
    dst->ic = motor.ic;
    dst->id = motor.Id;
    dst->iq = motor.Iq;
    dst->id_target = motor.id_target;
    dst->id_error = id_error;
    dst->prev_ia = s_hall_prev_ia;
    dst->prev_ib = s_hall_prev_ib;
    dst->prev_ic = s_hall_prev_ic;
    dst->theta_rad = motor.theta;
    (void)Motor_Hall_DecodeAngle(motor.hall_table, hall_state, &center);
    dst->sector_center_rad = center;
    Motor_Current_GetReconstructDiag(&recon_last, NULL, NULL, NULL, NULL, NULL);
    dst->recon_phase = (uint8_t)recon_last;
    dst->first_half = first_half ? 1U : 0U;
    dst->current_adc[0] = (uint16_t)motor.currentAdc[0];
    dst->current_adc[1] = (uint16_t)motor.currentAdc[1];
    dst->current_adc[2] = (uint16_t)motor.currentAdc[2];
    dst->duty[0] = motor.duty1;
    dst->duty[1] = motor.duty2;
    dst->duty[2] = motor.duty3;
    dst->tim1_cnt = s_hall_isr_cnt;
    dst->tim1_arr = s_hall_isr_arr;
    dst->hall_state = (uint8_t)hall_state;
    dst->edge_age_ms = Motor_Hall_AngleGetEdgeAge() * 1000.0f;
    dst->beats = s_hall_current_worst.beats;
}

/**
 * @description: 若本拍三相和或 |Id| 创本轮新高，则锁存电流、电角与重构相。
 * @detail: TIM1 计数须在 ISR 入口已写入 NoteTimer，避免 FOC 算完后再读。
 * @param {float} signed_phase_sum  三相电流代数和 (A)
 * @param {float} phase_sum         |Ia+Ib+Ic| (A)
 * @param {int} hall_state          Hall 状态 0..7
 * @return {void} 无
 * @author: lkc
 */
static void Motor_Hall_DiagCaptureBeat(float signed_phase_sum, float phase_sum, int hall_state)
{
    float id_error;
    float hall_rpm;
    float sector_progress;
    bool first_half;

    s_hall_current_worst.beats++;
    id_error = fabsf(motor.Id - motor.id_target);
    hall_rpm = fabsf(Motor_Hall_AngleGetMechRpm());
    sector_progress = Motor_Hall_AngleGetEdgeAge() * hall_rpm * motor.ol_pole_pairs * 0.1f;
    first_half = sector_progress < 0.5f;
    if (phase_sum > s_hall_current_worst.abs_sum_a) {
        Motor_Hall_DiagFillFrame(&s_hall_current_worst, signed_phase_sum, phase_sum, hall_state, id_error, first_half);
    }
    if (isfinite(id_error) && id_error > s_hall_current_worst_id.id_error) {
        Motor_Hall_DiagFillFrame(&s_hall_current_worst_id, signed_phase_sum, phase_sum, hall_state, id_error, first_half);
    }
    if (Motor_Hall_AngleGetEdgeAge() < MOTOR_HALL_DIAG_EDGE_AGE_S) {
        s_hall_edge_diag.edge_samples++;
        if (isfinite(id_error) && id_error > MOTOR_HALL_DIAG_ID_ERROR_A) {
            s_hall_edge_diag.edge_id_over++;
        }
    } else {
        s_hall_edge_diag.body_samples++;
        if (isfinite(id_error) && id_error > MOTOR_HALL_DIAG_ID_ERROR_A) {
            s_hall_edge_diag.body_id_over++;
        }
    }
    {
        int32_t id_mA = (int32_t)(motor.Id * 1000.0f);
        int32_t iq_mA = (int32_t)(motor.Iq * 1000.0f);
        int32_t iq_t_mA = (int32_t)(motor.iq_target * 1000.0f);
        int32_t ia_mA = (int32_t)(motor.ia * 1000.0f);
        int32_t ib_mA = (int32_t)(motor.ib * 1000.0f);
        int32_t ic_mA = (int32_t)(motor.ic * 1000.0f);

        s_hall_edge_diag.id_mA_sum += id_mA;
        s_hall_edge_diag.iq_mA_sum += iq_mA;
        s_hall_edge_diag.iq_target_mA_sum += iq_t_mA;
        s_hall_edge_diag.ia_mA_sum += ia_mA;
        s_hall_edge_diag.ib_mA_sum += ib_mA;
        s_hall_edge_diag.ic_mA_sum += ic_mA;
        s_hall_edge_diag.id_abs_mA_sum += (id_mA >= 0) ? (uint32_t)id_mA : (uint32_t)(-id_mA);
        s_hall_edge_diag.iq_abs_mA_sum += (iq_mA >= 0) ? (uint32_t)iq_mA : (uint32_t)(-iq_mA);
        s_hall_edge_diag.id2_mA2_sum += (uint64_t)((int64_t)id_mA * (int64_t)id_mA);
        s_hall_edge_diag.iq2_mA2_sum += (uint64_t)((int64_t)iq_mA * (int64_t)iq_mA);
    }
    if (s_hall_prev_state >= 1 && hall_state != s_hall_prev_state) {
        s_hall_edge_diag.jump_samples++;
        if (isfinite(id_error) && id_error > s_hall_edge_diag.jump.id_error) {
            Motor_Hall_DiagFillFrame(&s_hall_edge_diag.jump, signed_phase_sum, phase_sum, hall_state, id_error,
                                     first_half);
        }
    }
    s_hall_prev_state = hall_state;
    s_hall_prev_ia = motor.ia;
    s_hall_prev_ib = motor.ib;
    s_hall_prev_ic = motor.ic;
}

/**
 * @description: 累计霍尔 I/F 的扇区电流和换相边沿统计。
 * @detail: 调用者已确认运行条件；独立函数避免其他反馈模式准备整套浮点统计栈帧。
 * @return {void} 无
 * @author: lkc
 */
static __attribute__((noinline)) void Motor_Hall_DiagAccumulate(void)
{
    motor_hall_current_diag_sector_t *sector;
    float signed_phase_sum;
    float phase_sum;
    float id_error;
    float sector_progress;
    float hall_rpm;
    bool first_half;
    int hall_state;

    if (!s_hall_diag_running || s_hall_diag_target_rpm != motor.ol_target_rpm) {
        memset(s_hall_current_diag, 0, sizeof(s_hall_current_diag));
        memset(&s_hall_current_worst, 0, sizeof(s_hall_current_worst));
        memset(&s_hall_current_worst_id, 0, sizeof(s_hall_current_worst_id));
        memset(&s_hall_edge_diag, 0, sizeof(s_hall_edge_diag));
        s_hall_prev_state = -1;
        s_hall_prev_ia = 0.0f;
        s_hall_prev_ib = 0.0f;
        s_hall_prev_ic = 0.0f;
        s_hall_diag_div = 0U;
        s_hall_diag_running = true;
        s_hall_diag_target_rpm = motor.ol_target_rpm;
    }
    if (!Motor_Hall_AngleIsSpeedValid() || fabsf(motor.ol_current_rpm - motor.ol_target_rpm) > 1.0f) return;
    hall_state = Motor_Hall_AngleGetState();
    signed_phase_sum = motor.ia + motor.ib + motor.ic;
    phase_sum = fabsf(signed_phase_sum);
    if (isfinite(phase_sum)) {
        Motor_Hall_DiagCaptureBeat(signed_phase_sum, phase_sum, hall_state);
    }
    if (++s_hall_diag_div < MOTOR_HALL_DIAG_DIVIDER) return;
    s_hall_diag_div = 0U;
    if (hall_state < 1 || hall_state > 6) return;
    id_error = fabsf(motor.Id - motor.id_target);
    if (!isfinite(phase_sum) || !isfinite(id_error)) return;
    sector = &s_hall_current_diag[hall_state];
    sector->samples++;
    sector->signed_phase_sum_milliamps += (int32_t)(signed_phase_sum * 1000.0f);
    sector->current_adc_sum[0] += (uint32_t)motor.currentAdc[0];
    sector->current_adc_sum[1] += (uint32_t)motor.currentAdc[1];
    sector->current_adc_sum[2] += (uint32_t)motor.currentAdc[2];
    hall_rpm = fabsf(Motor_Hall_AngleGetMechRpm());
    sector_progress = Motor_Hall_AngleGetEdgeAge() * hall_rpm * motor.ol_pole_pairs * 0.1f;
    first_half = sector_progress < 0.5f;
    if (first_half) {
        sector->first_half_samples++;
    } else {
        sector->second_half_samples++;
    }
    if (phase_sum > sector->max_abs_phase_sum_a) {
        sector->max_abs_phase_sum_a = phase_sum;
    }
    if (id_error > sector->max_abs_id_error_a) {
        sector->max_abs_id_error_a = id_error;
    }
    if (phase_sum > MOTOR_HALL_DIAG_PHASE_SUM_A) {
        sector->phase_sum_over++;
        if (signed_phase_sum > 0.0f) {
            sector->phase_sum_positive_over++;
        } else {
            sector->phase_sum_negative_over++;
        }
    }
    if (id_error > MOTOR_HALL_DIAG_ID_ERROR_A) {
        sector->id_error_over++;
        if (first_half) {
            sector->first_half_id_over++;
        } else {
            sector->second_half_id_over++;
        }
    }
}

/**
 * @description: 在霍尔 I/F 运行时累计扇区电流和换相边沿统计。
 * @detail: 未使能、反馈不是霍尔或不是 I/F 时停止累计；满足条件后才进入统计主体。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Hall_DiagUpdate(void)
{
    if (!motor.ol_enable || Motor_Feedback_GetSourceDefault() != MOTOR_FEEDBACK_SRC_HALL ||
        motor.ol_mode != MOTOR_OL_MODE_IF) {
        s_hall_diag_running = false;
        return;
    }
    Motor_Hall_DiagAccumulate();
}

#ifdef __cplusplus
}
#endif
