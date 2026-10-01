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
typedef struct {
    uint16_t count;
    int32_t delta;
    uint32_t elapsed_us;
    int32_t raw_centi_rpm;
    int32_t pll_centi_rpm;
    bool valid;
} encoder_watch_sample_t;

/* Private define ------------------------------------------------------------*/
#define ENCODER_WATCH_PERIOD_MS (100U)
#define ENCODER_WATCH_DEFAULT_S (2.0f)
#define ENCODER_WATCH_MAX_S     (5.0f)
#define ENCODER_WATCH_MAX_SAMPLES ((uint32_t)(ENCODER_WATCH_MAX_S * 1000.0f / ENCODER_WATCH_PERIOD_MS))
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
static encoder_watch_sample_t s_encoder_watch_samples[ENCODER_WATCH_MAX_SAMPLES];
/* Private function prototypes -----------------------------------------------*/
static char *Encoder_AppendChar(char *dst, const char *end, char value);
static char *Encoder_AppendString(char *dst, const char *end, const char *value);
static char *Encoder_AppendU32(char *dst, const char *end, uint32_t value);
static char *Encoder_AppendI32(char *dst, const char *end, int32_t value, bool always_sign);
static char *Encoder_AppendFixed2(char *dst, const char *end, int32_t centi_value);
/* Private functions ---------------------------------------------------------*/

static char *Encoder_AppendChar(char *dst, const char *end, char value)
{
    if (dst < end) {
        *dst++ = value;
    }
    return dst;
}

static char *Encoder_AppendString(char *dst, const char *end, const char *value)
{
    while ((*value != '\0') && (dst < end)) {
        *dst++ = *value++;
    }
    return dst;
}

static char *Encoder_AppendU32(char *dst, const char *end, uint32_t value)
{
    char reversed[10];
    uint32_t count = 0U;

    do {
        reversed[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(reversed)));
    while ((count > 0U) && (dst < end)) {
        *dst++ = reversed[--count];
    }
    return dst;
}

static char *Encoder_AppendI32(char *dst, const char *end, int32_t value, bool always_sign)
{
    uint32_t magnitude;

    if (value < 0) {
        dst = Encoder_AppendChar(dst, end, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        if (always_sign) {
            dst = Encoder_AppendChar(dst, end, '+');
        }
        magnitude = (uint32_t)value;
    }
    return Encoder_AppendU32(dst, end, magnitude);
}

static char *Encoder_AppendFixed2(char *dst, const char *end, int32_t centi_value)
{
    uint32_t magnitude;

    if (centi_value < 0) {
        dst = Encoder_AppendChar(dst, end, '-');
        magnitude = (uint32_t)(-(centi_value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)centi_value;
    }
    dst = Encoder_AppendU32(dst, end, magnitude / 100U);
    dst = Encoder_AppendChar(dst, end, '.');
    dst = Encoder_AppendChar(dst, end, (char)('0' + ((magnitude / 10U) % 10U)));
    return Encoder_AppendChar(dst, end, (char)('0' + (magnitude % 10U)));
}

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
 *          循环用 TIM5 真实时间截止；转速用整数定点打印，避免每行浮点 printf
 *          把 100 ms 观察节拍拉长。采样等待仍在任务中，不进 ISR。
 *          时长小于等于 0 时用 2 s，超过 5 s 则截断，避免长时挡住停机命令。
 * @param {float} duration_s  观察时长 (s)，按 RTOS tick 睡眠
 * @return {void} 无
 * @author: lkc
 */
static void Encoder_Watch(float duration_s)
{
    uint32_t last_accumulated;
    uint32_t last_us;
    uint32_t start_us;
    uint32_t duration_us;
    uint32_t duration_tenths;
    uint32_t sample_count = 0U;
    uint32_t i;
    char line[128];
    int32_t cpr = (int32_t)Hal_Encoder_CountsPerRev();

    if (duration_s <= 0.0f) {
        duration_s = ENCODER_WATCH_DEFAULT_S;
    } else if (duration_s > ENCODER_WATCH_MAX_S) {
        duration_s = ENCODER_WATCH_MAX_S;
    }
    duration_us = (uint32_t)(duration_s * 1000000.0f);
    duration_tenths = (uint32_t)(duration_s * 10.0f + 0.5f);
    PRINTF("Encoder watch capturing %lu.%lus...\r\n",
           (unsigned long)(duration_tenths / 10U), (unsigned long)(duration_tenths % 10U));
    last_accumulated = Motor_Encoder_GetAccumulatedCount();
    last_us = Hal_Motor_TimeUs();
    start_us = last_us;
    do {
        uint32_t accumulated;
        uint32_t now_us;
        uint32_t elapsed_us;
        int32_t delta;
        float raw_rpm;
        uint32_t target_us = last_us + ENCODER_WATCH_PERIOD_MS * 1000U;

        while ((int32_t)(Hal_Motor_TimeUs() - target_us) < 0) {
            osDelay(1U);
        }
        now_us = Hal_Motor_TimeUs();
        accumulated = Motor_Encoder_GetAccumulatedCount();
        delta = (int32_t)(accumulated - last_accumulated);
        elapsed_us = now_us - last_us;
        raw_rpm = (elapsed_us == 0U) ? 0.0f :
                  (float)delta * 60000000.0f / ((float)cpr * (float)elapsed_us);

        if (sample_count < ENCODER_WATCH_MAX_SAMPLES) {
            encoder_watch_sample_t *sample = &s_encoder_watch_samples[sample_count++];
            sample->count = Motor_Encoder_GetCount();
            sample->delta = delta;
            sample->elapsed_us = elapsed_us;
            sample->raw_centi_rpm = (int32_t)(raw_rpm * 100.0f);
            sample->pll_centi_rpm = (int32_t)(Motor_Encoder_GetMechRpm() * 100.0f);
            sample->valid = Motor_Encoder_IsSpeedValid();
        }
        last_accumulated = accumulated;
        last_us = now_us;
    } while (((uint32_t)(last_us - start_us) < duration_us) &&
             (sample_count < ENCODER_WATCH_MAX_SAMPLES));

    {
        char *cursor = line;
        const char *end = line + sizeof(line);
        cursor = Encoder_AppendString(cursor, end, "Encoder watch ");
        cursor = Encoder_AppendU32(cursor, end, duration_tenths / 10U);
        cursor = Encoder_AppendChar(cursor, end, '.');
        cursor = Encoder_AppendU32(cursor, end, duration_tenths % 10U);
        cursor = Encoder_AppendString(cursor, end,
                                     "s TIM5: count delta dt_ms raw_rpm pll_rpm valid\r\n");
        (void)Bsp_Usart_Write((const UCHAR *)line, (ULONG)(cursor - line));
    }
    for (i = 0U; i < sample_count; i++) {
        const encoder_watch_sample_t *sample = &s_encoder_watch_samples[i];
        {
            char *cursor = line;
            const char *end = line + sizeof(line);
            cursor = Encoder_AppendString(cursor, end, "count=");
            cursor = Encoder_AppendU32(cursor, end, sample->count);
            cursor = Encoder_AppendString(cursor, end, " delta=");
            cursor = Encoder_AppendI32(cursor, end, sample->delta, true);
            cursor = Encoder_AppendString(cursor, end, " dt_ms=");
            cursor = Encoder_AppendU32(cursor, end, (sample->elapsed_us + 500U) / 1000U);
            cursor = Encoder_AppendString(cursor, end, " raw=");
            cursor = Encoder_AppendFixed2(cursor, end, sample->raw_centi_rpm);
            cursor = Encoder_AppendString(cursor, end, " pll=");
            cursor = Encoder_AppendFixed2(cursor, end, sample->pll_centi_rpm);
            cursor = Encoder_AppendString(cursor, end, " valid=");
            cursor = Encoder_AppendU32(cursor, end, sample->valid ? 1U : 0U);
            cursor = Encoder_AppendString(cursor, end, "\r\n");
            (void)Bsp_Usart_Write((const UCHAR *)line, (ULONG)(cursor - line));
        }
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
        if (FlashStorage_Save(&motor)) {
            PRINTF("Cogging table saved to flash\r\n");
        } else {
            PRINTF("Cogging table save FAILED; table remains active in RAM\r\n");
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
