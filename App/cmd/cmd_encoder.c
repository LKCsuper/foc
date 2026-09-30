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
    PRINTF("  %s@%luHz valid=%d ok=%lu err=%lu crc_err=%lu consecutive=%lu mag=0x%X\r\n",
           Motor_MT6701_InterfaceName(), (unsigned long)MOTOR_MT6701_POLL_HZ, mt.valid ? 1 : 0,
           (unsigned long)mt.success_count,
           (unsigned long)mt.error_count, (unsigned long)mt.crc_error_count,
           (unsigned long)mt.consecutive_errors, (unsigned)mt.magnetic_status);
}

/**
 * @description: 短时对比 MT6701 原始计数窗口转速与 PLL 转速。
 * @detail: Shell 打印耗时计入真实采样间隔，避免按固定 100 ms 计算导致 raw_rpm 偏高。
 *          不进 ISR。时长小于等于 0 时用 2 s，超过 5 s 则截断，避免长时挡住停机命令。
 * @param {float} duration_s  观察时长 (s)
 * @return {void} 无
 * @author: lkc
 */
static void Encoder_Watch(float duration_s)
{
    uint16_t last_count = Motor_Encoder_GetCount();
    TickType_t last_tick = xTaskGetTickCount();
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
    PRINTF("Encoder watch %.1fs: count delta raw_rpm pll_rpm valid\r\n", (double)duration_s);
    for (i = 0U; i < samples; i++) {
        uint16_t count;
        TickType_t now_tick;
        TickType_t elapsed_ticks;
        int32_t delta;
        float raw_rpm;

        osDelay(pdMS_TO_TICKS(ENCODER_WATCH_PERIOD_MS));
        now_tick = xTaskGetTickCount();
        count = Motor_Encoder_GetCount();
        delta = (int32_t)count - (int32_t)last_count;
        if (delta > cpr / 2) {
            delta -= cpr;
        } else if (delta < -(cpr / 2)) {
            delta += cpr;
        }
        if (motor.encoder_inverted) {
            delta = -delta;
        }
        elapsed_ticks = now_tick - last_tick;
        if (elapsed_ticks == 0U) {
            raw_rpm = 0.0f;
        } else {
            raw_rpm = (float)delta * (60.0f * (float)configTICK_RATE_HZ) /
                      ((float)cpr * (float)elapsed_ticks);
        }
        PRINTF("  %5u %+6ld raw=%8.2f pll=%8.2f valid=%d\r\n", (unsigned)count, (long)delta,
               (double)raw_rpm, (double)Motor_Encoder_GetMechRpm(), Motor_Encoder_IsSpeedValid() ? 1 : 0);
        last_count = count;
        last_tick = now_tick;
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
 * @description: 处理 encoder cog 子命令。
 * @detail: status 只打印。learn 要求电机已使能、速度环打开且反馈为编码器，每桶样本数 1..50。
 *          on 在未标定时失败。stop 不清除表，clear 清表。
 * @param {int} argc       参数个数，含命令名
 * @param {char *[]} argv  参数表；子命令在 argv[2]
 * @return {int} 0=已执行，-1=参数或前置条件不满足
 * @author: lkc
 */
static int Encoder_CoggingCommand(int argc, char *argv[])
{
    int turns = 10;
    if (argc < 3 || strcmp(argv[2], "status") == 0) {
        Encoder_CoggingStatus();
        return 0;
    }
    if (strcmp(argv[2], "learn") == 0) {
        if (argc >= 4) turns = atoi(argv[3]);
        if (!motor.ol_enable || !motor.speed_enable ||
            Motor_Feedback_GetSourceDefault() != MOTOR_FEEDBACK_SRC_ENCODER) {
            PRINTF("Cogging learn requires running encoder speed loop\r\n");
            return -1;
        }
        if (turns < 1 || turns > 50 || !Motor_Anticogging_Start((uint16_t)turns)) {
            PRINTF("usage: encoder cog learn [1..50]\r\n");
            return -1;
        }
        PRINTF("Cogging learn started: %d samples/bin, keep constant speed until complete\r\n", turns);
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
    PRINTF("usage: encoder cog [status|learn [1..50]|on|off|stop|clear]\r\n");
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
