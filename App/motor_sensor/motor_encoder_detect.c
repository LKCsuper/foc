/*
 * @Description: 编码器方向与电角零点标定（阻塞式应用流程）
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor_encoder.h"
#include "motor_param.h"
#include "motor_math.h"
#include "motor.h"
#include "motor_protect.h"
#include "hal_motor.h"
#include "hal_sensor.h"
#include "motor_mt6701.h"
#include "motor_anticogging.h"
#include "flash_storage.h"
/* Private variables ---------------------------------------------------------*/
static struct {
    volatile bool active;
    volatile float theta_rad;
    volatile float id_target;
} s_encoder_detect;
/* Private functions ---------------------------------------------------------*/
/**
 * @description: 把角度折到 [0, 2π)。
 * @param {float} angle 输入角度 (rad)
 * @return {float} 折绕后的角度 (rad)
 */
static float Motor_EncoderDetect_Wrap2Pi(float angle)
{
    while (angle >= (2.0f * (float)M_PI)) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < 0.0f) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

/**
 * @description: 计算单圈计数差，并按半圈折绕。
 * @param {uint16_t} now   当前计数
 * @param {uint16_t} last  起始计数
 * @param {uint32_t} cpr   每圈计数；0 时按 1 处理
 * @return {int32_t} 有符号计数差
 */
static int32_t Motor_EncoderDetect_DeltaCounts(uint16_t now, uint16_t last, uint32_t cpr)
{
    int32_t delta;
    int32_t half;

    if (cpr == 0U) {
        cpr = 1U;
    }
    delta = (int32_t)now - (int32_t)last;
    half = (int32_t)(cpr / 2U);
    if (delta > half) {
        delta -= (int32_t)cpr;
    } else if (delta < -half) {
        delta += (int32_t)cpr;
    }
    return delta;
}

/**
 * @description: 查询编码器电角零点对齐是否正在占用控制输出。
 * @return {bool} true=对齐进行中，本拍输出由检测目标决定
 * @author: lkc
 */
bool Motor_Encoder_DetectIsActive(void)
{
    return s_encoder_detect.active;
}

/**
 * @description: 读取编码器对齐阶段的固定电角和 Id 目标。
 * @param {float *} theta_rad  输出电角度 (rad)，不可为空
 * @param {float *} id_target  输出 Id 目标 (A)，不可为空
 * @param {float *} iq_target  输出 Iq 目标 (A)，不可为空；对齐时为 0
 * @return {bool} true=对齐进行中且三个指针都已写出
 * @author: lkc
 */
bool Motor_Encoder_DetectGetTargets(float *theta_rad, float *id_target, float *iq_target)
{
    if (!s_encoder_detect.active || theta_rad == NULL || id_target == NULL || iq_target == NULL) {
        return false;
    }
    *theta_rad = s_encoder_detect.theta_rad;
    *id_target = s_encoder_detect.id_target;
    *iq_target = 0.0f;
    return true;
}

/**
 * @description: 无 Z 相时标定编码器方向和电角零点。
 * @detail: 阻塞在应用任务里调用，不在控制中断里跑。先在电角 0 爬升 Id，再扫过 π/2
 *          判断计数方向；随后正反向各走一圈机械角，用角度差的正余弦均值求零点。
 *          成功后写入运行时，按 save_to_flash 决定是否保存 Flash。电机已运行、故障锁存或标定电流超出
 *          (0, 0.5] A 时直接拒绝。
 * @param {bool} save_to_flash true=尝试保存，false=仅应用到 RAM
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_DetectWithSave(bool save_to_flash)
{
    float pole_pairs = motor.ol_pole_pairs;
    float current = motor.ol_iq_set;
    float offset_rad = 0.0f;
    int32_t direction_delta = 0;
    uint16_t direction_start;
    bool inverted = false;
    float measured;
    float forced;
    float diff;
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    const uint32_t scan_points = 140U;
    const float scan_range = 2.0f * (float)M_PI * pole_pairs;
    uint32_t step;
    bool success = false;

    if (!Motor_MT6701_IsValid()) {
        printf("Encoder detect rejected: MT6701 %s data invalid\r\n", Motor_MT6701_InterfaceName());
        return;
    }
    if (motor.ol_enable || Motor_Protect_IsFaultLatched() || current <= 0.0f || current > 0.5f) {
        printf("Encoder detect rejected: stop motor, clear fault, set oliq 0.1..0.5A\r\n");
        return;
    }

    printf("Encoder detect: align theta=0 Id=%.2fA\r\n", (double)current);
    Motor_Protect_ClearFault();
    motor.ol_enable = true;
    (void)Motor_ClearRuntimeControl(&motor);
    s_encoder_detect.id_target = 0.0f;
    s_encoder_detect.theta_rad = 0.0f;
    s_encoder_detect.active = true;
    Hal_Motor_PwmStart();

    for (step = 0U; step < 500U; step++) {
        if (Motor_Protect_IsFaultLatched()) {
            break;
        }
        s_encoder_detect.id_target = current * (float)(step + 1U) / 500.0f;
        osDelay(pdMS_TO_TICKS(1U));
    }
    if (!Motor_Protect_IsFaultLatched()) {
        osDelay(pdMS_TO_TICKS(700U));
        direction_start = Hal_Encoder_GetCount();
        for (step = 0U; step < 1000U; step++) {
            if (Motor_Protect_IsFaultLatched() || !Motor_MT6701_IsValid()) {
                break;
            }
            s_encoder_detect.theta_rad = (0.5f * (float)M_PI) * (float)(step + 1U) / 1000.0f;
            osDelay(pdMS_TO_TICKS(1U));
        }
        direction_delta = Motor_EncoderDetect_DeltaCounts(Hal_Encoder_GetCount(), direction_start,
                                                           Hal_Encoder_CountsPerRev());
        if (!Motor_Protect_IsFaultLatched() && Motor_MT6701_IsValid() &&
            (direction_delta > 100 || direction_delta < -100)) {
            inverted = direction_delta < 0;
            Motor_Encoder_ApplyCalibration(0.0f, inverted, false);
            printf("Encoder direction: delta=%ld inverted=%d\r\n", (long)direction_delta, inverted ? 1 : 0);
            for (step = 1000U; step > 0U; step--) {
                if (Motor_Protect_IsFaultLatched()) {
                    break;
                }
                s_encoder_detect.theta_rad = (0.5f * (float)M_PI) * (float)(step - 1U) / 1000.0f;
                osDelay(pdMS_TO_TICKS(1U));
            }
            if (!Motor_Protect_IsFaultLatched()) {
                osDelay(pdMS_TO_TICKS(300U));
                printf("Encoder offset scan: forward/back one mechanical turn\r\n");
                for (step = 0U; step < scan_points; step++) {
                    s_encoder_detect.theta_rad = scan_range * (float)(step + 1U) / (float)scan_points;
                    osDelay(pdMS_TO_TICKS(50U));
                    if (Motor_Protect_IsFaultLatched() || !Motor_MT6701_IsValid()) {
                        break;
                    }
                    measured = Motor_Encoder_GetMechAngle() * pole_pairs;
                    forced = fmodf(s_encoder_detect.theta_rad, 2.0f * (float)M_PI);
                    diff = measured - forced;
                    sin_sum += sinf(diff);
                    cos_sum += cosf(diff);
                }
                for (step = scan_points; step > 0U && !Motor_Protect_IsFaultLatched(); step--) {
                    s_encoder_detect.theta_rad = scan_range * (float)(step - 1U) / (float)scan_points;
                    osDelay(pdMS_TO_TICKS(50U));
                    if (!Motor_MT6701_IsValid()) {
                        break;
                    }
                    measured = Motor_Encoder_GetMechAngle() * pole_pairs;
                    forced = fmodf(s_encoder_detect.theta_rad, 2.0f * (float)M_PI);
                    diff = measured - forced;
                    sin_sum += sinf(diff);
                    cos_sum += cosf(diff);
                }
                success = !Motor_Protect_IsFaultLatched() && Motor_MT6701_IsValid() &&
                          (fabsf(sin_sum) + fabsf(cos_sum) > 1.0f);
                if (success) {
                    offset_rad = Motor_EncoderDetect_Wrap2Pi(atan2f(sin_sum, cos_sum));
                    Motor_Encoder_ApplyCalibration(offset_rad, inverted, true);
                    printf("Encoder offset average: %.3f rad (%.1f deg)\r\n", (double)offset_rad,
                           (double)(offset_rad * 180.0f / (float)M_PI));
                }
            }
        } else {
            printf("Encoder direction FAILED: movement delta=%ld\r\n", (long)direction_delta);
        }
    }

    s_encoder_detect.active = false;
    s_encoder_detect.theta_rad = 0.0f;
    s_encoder_detect.id_target = 0.0f;
    (void)Motor_RequestStop(&motor);
    Hal_Motor_PwmStop();
    (void)Motor_ClearRuntimeControl(&motor);

    if (!success) {
        printf("Encoder detect FAILED: %s\r\n", Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
        return;
    }

    motor.encoder_offset_rad = offset_rad;
    motor.encoder_inverted = inverted;
    motor.encoder_calibrated = true;
    /* 电角零点或方向变化后，旧机械角齿槽表不再对应当前安装关系。 */
    Motor_Anticogging_Clear();
    printf("Encoder detect: stale cogging table cleared; run encoder cog learn again\r\n");
    if (!save_to_flash) {
        printf("Encoder calibration applied to RAM only\r\n");
    } else if (FlashStorage_Save(&motor)) {
        printf("Encoder calibration saved to flash\r\n");
    } else {
        printf("Encoder calibration save FAILED\r\n");
    }
    printf("Encoder detect: count=%u mech=%.3f rad elec0=%.3f rad zero_ok=%d\r\n",
           (unsigned)Motor_Encoder_GetCount(), Motor_Encoder_GetMechAngle(), Motor_Encoder_GetElecAngle(),
           Motor_Encoder_IsZeroValid() ? 1 : 0);
}

/**
 * @description: 无 Z 相时标定编码器方向和电角零点，并尝试保存 Flash。
 * @detail: 转调 Motor_Encoder_DetectWithSave(true)。阻塞在应用任务里调用，不在控制中断里跑。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Encoder_Detect(void)
{
    Motor_Encoder_DetectWithSave(true);
}

#ifdef __cplusplus
}
#endif
