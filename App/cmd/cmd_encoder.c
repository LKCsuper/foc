/*
 * @Description: MT6701 编码器 Shell 命令
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ENCODER_WATCH_PERIOD_MS (100U)
#define ENCODER_WATCH_DEFAULT_S (2.0f)
#define ENCODER_WATCH_MAX_S     (5.0f)
#define ENCODER_COG_DEFAULT_PASSES (2U)
#define ENCODER_COG_MAX_PASSES     (5U)
#define ENCODER_COG_STEP_MS        (2U)
#define ENCODER_COG_POS_KP_V_RAD   (20.0f)
#define ENCODER_COG_POS_KI_V_RAD_S (30.0f)
#define ENCODER_COG_VEL_KD_V_RPM   (0.005f)
#define ENCODER_COG_VQ_LIMIT_V     (1.50f)
#define ENCODER_COG_POS_COUNTS     (16.0f)
#define ENCODER_COG_VEL_THRESH_RPM (0.5f)
#define ENCODER_COG_STABLE_SAMPLES (20U)
#define ENCODER_COG_POINT_TIMEOUT  (5000U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 打印编码器角度、转速和 MT6701 帧状态。
 * @return {void} 无
 * @author: lkc
 */
static void Encoder_PrintStatus(void)
{
    motor_mt6701_status_t mt;

    Motor_MT6701_GetStatus(&mt);
    PRINTF("Encoder count=%u / %u\r\n",
           (unsigned)Motor_Encoder_GetCount(),
           (unsigned)Hal_Encoder_CountsPerRev());
    PRINTF("  mech=%.3f rad (%.1f deg)\r\n",
           Motor_Encoder_GetMechAngle(),
           Motor_Encoder_GetMechAngle() * 180.0f / (float)M_PI);
    PRINTF("  elec=%.3f rad (%.1f deg)\r\n",
           Motor_Encoder_GetElecAngle(),
           Motor_Encoder_GetElecAngle() * 180.0f / (float)M_PI);
    PRINTF("  rpm=%.2f raw=%.2f valid=%d edge_age=%.4f s sample_age=%.4f s\r\n",
           Motor_Encoder_GetMechRpm(), Motor_Encoder_GetRawMechRpm(),
           Motor_Encoder_IsSpeedValid() ? 1 : 0, Motor_Encoder_GetEdgeAge(), Motor_Encoder_GetSampleAge());
    PRINTF("  zero_valid=%d signal_fault=%d rejected=%lu\r\n",
           Motor_Encoder_IsZeroValid() ? 1 : 0, Motor_Encoder_IsSignalFault() ? 1 : 0,
           (unsigned long)Motor_Encoder_GetInvalidCount());
    PRINTF("  %s@%luHz valid=%d ok=%lu err=%lu xfer_err=%lu crc_err=%lu status_err=%lu consecutive=%lu mag=0x%X last_bad_mag=0x%X\r\n",
           Motor_MT6701_InterfaceName(), (unsigned long)MOTOR_MT6701_POLL_HZ, mt.valid ? 1 : 0,
           (unsigned long)mt.success_count, (unsigned long)mt.error_count,
           (unsigned long)mt.transfer_error_count, (unsigned long)mt.crc_error_count,
           (unsigned long)mt.status_error_count, (unsigned long)mt.consecutive_errors,
           (unsigned)mt.magnetic_status, (unsigned)mt.last_bad_magnetic_status);
}

/**
 * @description: 短时对比 MT6701 原始计数窗口转速与 PLL 转速。
 * @detail: raw_rpm 用 TIM5 微秒时基（Hal_Motor_TimeUs，1 us/tick）计算窗口时长。
 *          FOC ISR 占用高时 RTOS tick 会变慢，按 tick 计时会把转速读高。
 *          Shell 打印耗时计入该真实间隔。采样节拍仍用 osDelay，不进 ISR。
 *          时长小于等于 0 时用 2 s，超过 5 s 则截断，避免长时挡住停机命令。
 * @param {float} duration_s  观察时长 (s)，按 RTOS tick 睡眠
 * @return {void} 无
 * @author: lkc
 */
static void Encoder_Watch(float duration_s)
{
    uint32_t last_accumulated = Motor_Encoder_GetAccumulatedCount();
    uint32_t last_us = Hal_Motor_TimeUs();
    uint32_t samples;
    uint32_t i;
    int32_t cpr = (int32_t)Hal_Encoder_CountsPerRev();

    if (duration_s <= 0.0f) {
        duration_s = ENCODER_WATCH_DEFAULT_S;
    } else if (duration_s > ENCODER_WATCH_MAX_S) {
        duration_s = ENCODER_WATCH_MAX_S;
    }
    samples = (uint32_t)(duration_s * 1000.0f / (float)ENCODER_WATCH_PERIOD_MS);
    if (samples < 1U) {
        samples = 1U;
    }
    PRINTF("Encoder watch %.1fs TIM5: count delta dt_s raw_rpm pll_rpm valid\r\n", (double)duration_s);
    for (i = 0U; i < samples; i++) {
        uint16_t count;
        uint32_t accumulated;
        uint32_t now_us;
        uint32_t elapsed_us;
        int32_t delta;
        float elapsed_s;
        float raw_rpm;

        osDelay(pdMS_TO_TICKS(ENCODER_WATCH_PERIOD_MS));
        now_us = Hal_Motor_TimeUs();
        count = Motor_Encoder_GetCount();
        accumulated = Motor_Encoder_GetAccumulatedCount();
        delta = (int32_t)(accumulated - last_accumulated);
        elapsed_us = now_us - last_us;
        if (elapsed_us == 0U) {
            elapsed_s = 0.0f;
            raw_rpm = 0.0f;
        } else {
            elapsed_s = (float)elapsed_us * 1.0e-6f;
            raw_rpm = (float)delta * 60.0f / ((float)cpr * elapsed_s);
        }
        PRINTF("  %5u %+6ld dt=%7.4f raw=%8.2f pll=%8.2f valid=%d\r\n", (unsigned)count, (long)delta,
               (double)elapsed_s, (double)raw_rpm, (double)Motor_Encoder_GetMechRpm(),
               Motor_Encoder_IsSpeedValid() ? 1 : 0);
        last_accumulated = accumulated;
        last_us = now_us;
    }
}

/**
 * @description: 打印齿槽学习进度、标定使能状态和补偿电压范围。
 * @return {void} 无
 * @author: lkc
 */
static void Encoder_CoggingStatus(void)
{
    motor_anticogging_status_t status;
    Motor_Anticogging_GetStatus(&status);
    PRINTF("Cogging: learning=%d calibrated=%d enabled=%d bins=%u progress=%u/%u samples=%lu range=%.3f..%.3fV\r\n",
           status.learning ? 1 : 0, status.calibrated ? 1 : 0, status.enabled ? 1 : 0,
           (unsigned)MOTOR_ANTICOGGING_BINS, (unsigned)status.min_samples,
           (unsigned)status.target_samples, (unsigned long)status.total_samples,
           (double)status.min_voltage, (double)status.max_voltage);
}

/**
 * @description: 把机械角折回 [0, 2π)。
 * @param {float} angle  机械角 (rad)
 * @return {float} 折回后的机械角 (rad)
 * @author: lkc
 */
static float Encoder_CoggingWrap(float angle)
{
    while (angle < 0.0f) angle += 2.0f * (float)M_PI;
    while (angle >= 2.0f * (float)M_PI) angle -= 2.0f * (float)M_PI;
    return angle;
}

/**
 * @description: 计算目标相对当前机械角的最短有符号误差。
 * @param {float} target   目标机械角 (rad)
 * @param {float} current  当前机械角 (rad)
 * @return {float} 误差 (rad)，范围 (-π, π]
 * @author: lkc
 */
static float Encoder_CoggingDelta(float target, float current)
{
    float delta = Encoder_CoggingWrap(target) - Encoder_CoggingWrap(current);
    if (delta > (float)M_PI) delta -= 2.0f * (float)M_PI;
    if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;
    return delta;
}

/**
 * @description: 用电压位置保持扫描每个齿槽桶，记下维持该位置的积分 Vq。
 * @detail: 位置 PI 直接产生 Vq，并以编码器速度作阻尼。要求电机已停、零点有效且无故障。
 *          失败时停学习并恢复进入前的命令，且保持停机。
 * @param {uint16_t} passes  正反向扫描次数，调用方须已限制在 1～5
 * @return {bool} true=各桶都已记满并收表
 * @author: lkc
 */
static bool Encoder_CoggingLearnPosition(uint16_t passes)
{
    motor_command_t saved;
    motor_command_t calibration;
    uint16_t start_bin;
    float vq_integrator = 0.0f;
    float vq_command = 0.0f;
    float control_sign = 1.0f;
    float pos_threshold = ENCODER_COG_POS_COUNTS * 2.0f * (float)M_PI /
                          (float)Hal_Encoder_CountsPerRev();
    uint16_t pass;
    bool ok = false;

    if (Motor_GetCommand(&motor, &saved) != MOTOR_OK || saved.enable || !Motor_Encoder_IsZeroValid() ||
        !Motor_MT6701_IsValid() || Motor_Protect_IsFaultLatched()) {
        PRINTF("Cogging learn requires stopped motor, valid encoder zero and no fault\r\n");
        return false;
    }
    if (!Motor_Anticogging_Start(passes)) return false;

    calibration = saved;
    calibration.enable = true;
    calibration.mode = MOTOR_OL_MODE_IF;
    calibration.target_rpm = 0.0f;
    calibration.iq_set = 0.0f;
    calibration.id_set = 0.0f;
    calibration.speed_enable = false;
    Motor_Anticogging_SetLearnTarget(Motor_Encoder_GetElecAngle(), 0.0f);
    if (Motor_SetCommand(&motor, &calibration) != MOTOR_OK) goto cleanup;

    PRINTF("Cogging control direction: motor config sign=%+.0f\r\n", (double)control_sign);
    start_bin = (uint16_t)(Motor_Encoder_GetMechAngle() * (float)MOTOR_ANTICOGGING_BINS /
                           (2.0f * (float)M_PI));
    if (start_bin >= MOTOR_ANTICOGGING_BINS) start_bin = 0U;

    for (pass = 0U; pass < passes; pass++) {
        uint16_t k;
        bool reverse = (pass & 1U) != 0U;
        PRINTF("Cogging position scan: pass %u/%u direction=%s\r\n",
               (unsigned)(pass + 1U), (unsigned)passes, reverse ? "reverse" : "forward");
        for (k = 0U; k < MOTOR_ANTICOGGING_BINS; k++) {
            uint16_t bin = reverse ?
                               (uint16_t)((start_bin + MOTOR_ANTICOGGING_BINS - 1U - k) %
                                          MOTOR_ANTICOGGING_BINS) :
                               (uint16_t)((start_bin + k) % MOTOR_ANTICOGGING_BINS);
            float target = ((float)bin + 0.5f) * 2.0f * (float)M_PI /
                           (float)MOTOR_ANTICOGGING_BINS;
            uint16_t stable = 0U;
            uint16_t iteration;

            for (iteration = 0U; iteration < ENCODER_COG_POINT_TIMEOUT; iteration++) {
                float position_error;
                float velocity;

                if (Motor_Protect_IsFaultLatched() || !Motor_MT6701_IsValid()) goto cleanup;
                position_error = Encoder_CoggingDelta(target, Motor_Encoder_GetMechAngle());
                velocity = Motor_Encoder_GetMechRpm();
                vq_integrator += ENCODER_COG_POS_KI_V_RAD_S * position_error *
                                 ((float)ENCODER_COG_STEP_MS * 0.001f);
                if (vq_integrator > ENCODER_COG_VQ_LIMIT_V) vq_integrator = ENCODER_COG_VQ_LIMIT_V;
                if (vq_integrator < -ENCODER_COG_VQ_LIMIT_V) vq_integrator = -ENCODER_COG_VQ_LIMIT_V;
                vq_command = control_sign *
                             (vq_integrator + ENCODER_COG_POS_KP_V_RAD * position_error -
                              ENCODER_COG_VEL_KD_V_RPM * velocity);
                if (vq_command > ENCODER_COG_VQ_LIMIT_V) vq_command = ENCODER_COG_VQ_LIMIT_V;
                if (vq_command < -ENCODER_COG_VQ_LIMIT_V) vq_command = -ENCODER_COG_VQ_LIMIT_V;
                Motor_Anticogging_SetLearnTarget(Motor_Encoder_GetElecAngle(), vq_command);

                if (fabsf(position_error) <= pos_threshold &&
                    fabsf(velocity) <= ENCODER_COG_VEL_THRESH_RPM) {
                    stable++;
                    if (stable >= ENCODER_COG_STABLE_SAMPLES) break;
                } else {
                    stable = 0U;
                }
                osDelay(pdMS_TO_TICKS(ENCODER_COG_STEP_MS));
            }
            if (stable < ENCODER_COG_STABLE_SAMPLES ||
                !Motor_Anticogging_Record(bin, control_sign * vq_integrator)) {
                PRINTF("Cogging point timeout: bin=%u err=%.4frad rpm=%.2f vq=%.3fV\r\n",
                       (unsigned)bin, (double)Encoder_CoggingDelta(target, Motor_Encoder_GetMechAngle()),
                       (double)Motor_Encoder_GetMechRpm(), (double)vq_integrator);
                goto cleanup;
            }
        }
    }

    for (pass = 25U; pass > 0U; pass--) {
        Motor_Anticogging_SetLearnTarget(Motor_Encoder_GetElecAngle(),
                                         vq_command * (float)(pass - 1U) / 25.0f);
        osDelay(pdMS_TO_TICKS(ENCODER_COG_STEP_MS));
    }
    ok = Motor_Anticogging_Finish();

cleanup:
    if (!ok) Motor_Anticogging_Stop();
    saved.enable = false;
    (void)Motor_SetCommand(&motor, &saved);
    return ok;
}

/**
 * @description: 处理 encoder cog 子命令。
 * @detail: status 只打印。learn 要求电机停机、编码器零点有效且无故障；执行静态正反向逐点扫描。
 *          on 在未标定时失败。stop 不清除表，clear 清表。
 * @param {int} argc       参数个数，含命令名
 * @param {char *[]} argv  参数表；子命令在 argv[2]
 * @return {int} 0=已执行，-1=参数或前置条件不满足
 * @author: lkc
 */
static int Encoder_CoggingCommand(int argc, char *argv[])
{
    int passes = (int)ENCODER_COG_DEFAULT_PASSES;
    if (argc < 3 || strcmp(argv[2], "status") == 0) {
        Encoder_CoggingStatus();
        return 0;
    }
    if (strcmp(argv[2], "learn") == 0) {
        if (argc >= 4) passes = atoi(argv[3]);
        if (passes < 1 || passes > (int)ENCODER_COG_MAX_PASSES) {
            PRINTF("usage: encoder cog learn [passes 1..5]\r\n");
            return -1;
        }
        PRINTF("Cogging voltage-position learn: %d pass(es), motor will hold %u positions\r\n",
               passes, (unsigned)MOTOR_ANTICOGGING_BINS);
        if (!Encoder_CoggingLearnPosition((uint16_t)passes)) {
            PRINTF("Cogging position learn FAILED\r\n");
            return -1;
        }
        PRINTF("Cogging position learn complete\r\n");
        {
            motor_detect_result_t detect_compat = {0};
            detect_compat.version = MOTOR_DETECT_RESULT_VERSION;
            if (FlashStorage_Save(&motor, &detect_compat)) {
                PRINTF("Cogging table saved to flash\r\n");
            } else {
                PRINTF("Cogging table save FAILED; table remains active in RAM\r\n");
            }
        }
        Encoder_CoggingStatus();
        return 0;
    }
    if (strcmp(argv[2], "on") == 0 || strcmp(argv[2], "off") == 0) {
        bool enable = strcmp(argv[2], "on") == 0;
        if (!Motor_Anticogging_SetEnabled(enable)) {
            PRINTF("Cogging table not calibrated\r\n");
            return -1;
        }
        Encoder_CoggingStatus();
        return 0;
    }
    if (strcmp(argv[2], "clear") == 0) {
        Motor_Anticogging_Clear();
        PRINTF("Cogging table cleared\r\n");
        return 0;
    }
    if (strcmp(argv[2], "stop") == 0) {
        Motor_Anticogging_Stop();
        PRINTF("Cogging learn stopped\r\n");
        return 0;
    }
    PRINTF("usage: encoder cog [status|learn [passes 1..5]|on|off|stop|clear]\r\n");
    return -1;
}

/**
 * @description: MT6701 编码器命令：show / reset / detect / watch / cog。
 * @param {int} argc       参数个数，含命令名
 * @param {char *[]} argv  参数表；子命令在 argv[1]
 * @return {int} 0=已执行，-1=无法识别的子命令；cog 失败时透传其子命令返回值
 * @author: lkc
 */
int Cmd_Motor_Encoder(int argc, char *argv[])
{
    if (argc < 2 || strcmp(argv[1], "show") == 0) {
        Encoder_PrintStatus();
        return 0;
    }
    if (strcmp(argv[1], "reset") == 0) {
        Motor_Encoder_Reset();
        PRINTF("Encoder software zero cleared\r\n");
        Encoder_PrintStatus();
        return 0;
    }
    if (strcmp(argv[1], "detect") == 0) {
        Motor_Encoder_Detect();
        return 0;
    }
    if (strcmp(argv[1], "watch") == 0) {
        float duration_s = (argc >= 3) ? strtof(argv[2], NULL) : ENCODER_WATCH_DEFAULT_S;

        Encoder_Watch(duration_s);
        return 0;
    }
    if (strcmp(argv[1], "cog") == 0) {
        return Encoder_CoggingCommand(argc, argv);
    }

    PRINTF("usage: encoder [show|reset|detect|watch [seconds]|cog ...]\r\n");
    return -1;
}

/* Cmd_Motor_Encoder 注册在 command.c 命令表中 */
#ifdef __cplusplus
}
#endif
