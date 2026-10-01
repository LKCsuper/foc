/*
 * @Description: 电流/电压偏移校准。R/L/磁链等电机参数检测后期再做。
 * @Author: lkc
 *
 * 偏移累加/结算在 motor_detect_offset（无 RTOS）；本文件包装 PWM、延时与日志。
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor_detect.h"
#include "motor_detect_offset.h"
#include "flash_storage.h"
#include "motor_param.h"
#include "motor_sensorless.h"
#include "motor_hall.h"
#include "hal_motor.h"
#include "motor_protect.h"
#include "printf_color.h"
#include "motor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DETECT_OFFSET_ISR_TIMEOUT_MS (2000U)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * 校准任务会改 PWM 输出。置位后 DMA 中断仍采样换算，但不跑 FOC、不跑软件过流
 * （零偏未定时电流不可信）；ADC_Value[] 由 DMA 硬件持续刷新。
 * 电流零点改由 ISR 按 TIM1 方向分沿累加：Hall FOC 只用下降沿，任务 1 ms 抽
 * ADC_Value 会混入上升沿，导致约 2~3 计数共模偏差。
 */
static volatile bool s_detect_active = false;
static volatile bool s_offset_isr_enable = false;
static volatile uint32_t s_offset_down_n = 0U;
static volatile uint32_t s_offset_up_n = 0U;
static motor_detect_offset_t s_offset_down;
static motor_detect_offset_t s_offset_up;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 校准任务侧查询是否因保护锁存应中止。
 * @return {bool} true 应停采并 PwmStop
 * @author: lkc
 */
static bool Detect_ShouldAbort(void)
{
    return Motor_Protect_IsFaultLatched();
}

/**
 * @description: 将当前 RAM 中的配置/校准写入 Flash（须已停机且未占用功率桥）
 * @return {bool} true=保存成功
 * @author: lkc
 */
static bool Detect_SaveOffsetToFlash(void)
{
    if (!FlashStorage_Save(&motor)) {
        PRINTF("Detect offset: flash save failed\r\n");
        return false;
    }
    PRINTF("Detect offset: saved to flash\r\n");
    return true;
}

/**
 * @description: 打印某一沿的电流零点均值（ADC 计数）。
 * @param {const char *} tag 沿名称
 * @param {const motor_detect_offset_t *} st 累加状态
 * @return {*}
 */
static void Detect_PrintEdgeMeans(const char *tag, const motor_detect_offset_t *st)
{
    float n;

    if (tag == NULL || st == NULL || st->count == 0U) {
        PRINTF("Detect offset: %s n=0\r\n", (tag != NULL) ? tag : "?");
        return;
    }
    n = (float)st->count;
    PRINTF("Detect offset: %s n=%lu CURR %.2f %.2f %.2f\r\n", tag, (unsigned long)st->count,
           (double)(st->i_sum[0] / n), (double)(st->i_sum[1] / n), (double)(st->i_sum[2] / n));
    PRINTF("Detect offset: %s p2p %.1f %.1f %.1f sum_span %.1f (%.3f A)\r\n", tag,
           (double)(st->i_max[0] - st->i_min[0]), (double)(st->i_max[1] - st->i_min[1]),
           (double)(st->i_max[2] - st->i_min[2]), (double)(st->sum_max - st->sum_min),
           (double)((st->sum_max - st->sum_min) * Hal_Motor_CurrentScale()));
}

/**
 * @description: 等待 ISR 在升降沿各采满 samples_per_phase；零点采用下降沿。
 * @return {bool} false 表示保护锁存或超时
 */
static bool Detect_SampleIsrBlocking(void)
{
    uint32_t waited_ms = 0U;
    uint32_t need = s_offset_down.samples_per_phase;

    s_offset_down_n = 0U;
    s_offset_up_n = 0U;
    s_offset_isr_enable = true;
    while ((s_offset_down_n < need) || (s_offset_up_n < need)) {
        if (Detect_ShouldAbort()) {
            s_offset_isr_enable = false;
            return false;
        }
        osDelay(pdMS_TO_TICKS(1));
        waited_ms++;
        if (waited_ms >= DETECT_OFFSET_ISR_TIMEOUT_MS) {
            s_offset_isr_enable = false;
            PRINTF("Detect offset: ISR sample timeout down=%lu up=%lu need=%lu\r\n",
                   (unsigned long)s_offset_down_n, (unsigned long)s_offset_up_n, (unsigned long)need);
            return false;
        }
    }
    s_offset_isr_enable = false;
    osDelay(pdMS_TO_TICKS(1));
    return !Detect_ShouldAbort();
}

/**
 * @description: 校准三相电流零点和驱动时相电压偏移。
 * @detail: 三相 50% 中心对齐；电流零点由 ISR 按 TIM1 沿分别累加，写入下降沿均值
 *          （与 Hall FOC 顶部零矢量采样窗一致）。cal_undriven=true 时再采未驱动相电压。
 *          累加/结算走 Motor_Detect_Offset*；本函数负责停机、PWM、osDelay 与打印。
 *          成功后自动写入 Flash；中止路径不保存。
 * @param {bool} cal_undriven true 时额外校准未驱动相电压偏移
 * @return {uint32_t} 1 成功
 * @author: lkc
 */
uint32_t DETECT_Offset(bool cal_undriven)
{
    /*
     * 先占住互斥，再停机关通道。之后 DMA 中断只采样换算（不跑软件过流），不跑 FOC。
     */
    if (Motor_Protect_IsFaultLatched()) {
        PRINTF("Detect offset: blocked, fault=%s — run: olfault clear\r\n",
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
        return 0U;
    }
    Motor_Protect_ClearFault();

    s_offset_isr_enable = false;
    s_detect_active = true;
    (void)Motor_RequestStop(&motor);
    Motor_Sensorless_Reset();
    Hal_Motor_PwmStop();
    /* 清残留 dq/相电流，避免上一拍故障快照量影响后续判断 */
    motor.ia = 0.0f;
    motor.ib = 0.0f;
    motor.ic = 0.0f;
    motor.Id = 0.0f;
    motor.Iq = 0.0f;
    PRINTF("Detect offset: begin\r\n");

    Motor_Detect_OffsetReset(&s_offset_down, cal_undriven, MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES,
                             MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE);
    Motor_Detect_OffsetReset(&s_offset_up, false, MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES,
                             MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE);
    s_offset_down_n = 0U;
    s_offset_up_n = 0U;

    osDelay(pdMS_TO_TICKS(1000));
    if (Detect_ShouldAbort()) {
        goto abort_detect;
    }

    Hal_Motor_PwmSetSafe();
    Hal_Motor_PwmStart();
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: all phases 50%, ISR edge sample\r\n");
    if (!Detect_SampleIsrBlocking()) {
        goto abort_detect;
    }

    Hal_Motor_PwmStop();
    Detect_PrintEdgeMeans("down", &s_offset_down);
    Detect_PrintEdgeMeans("up", &s_offset_up);
    Motor_Detect_OffsetFinalizeDriven(&s_offset_down);
    Motor_Detect_OffsetApply(&s_offset_down, &motor, false);

    PRINTF("ADC_RAW [%d] [%d] [%d]\r\n", (int)Hal_Motor_CurrentAdc(0U), (int)Hal_Motor_CurrentAdc(1U),
           (int)Hal_Motor_CurrentAdc(2U));
    PRINTF("Current Offset [%f] [%f] [%f]\r\n", motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
    PRINTF("Volt Offset [%f] [%f] [%f]\r\n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);

    if (s_offset_down.offset_warn) {
        PRINTF("WARNING: current offset > %.0f%% ADC range — check hardware\r\n",
               (double)MOTOR_DETECT_OFFSET_WARN_PERCENT);
    }

    if (cal_undriven) {
        uint32_t i;

        PRINTF("Detect offset: undriven voltage\r\n");
        osDelay(pdMS_TO_TICKS(10));

        for (i = 0U; i < s_offset_down.samples_per_phase; i++) {
            if (Detect_ShouldAbort()) {
                goto abort_detect;
            }
            (void)Motor_Detect_OffsetFeedUndriven(&s_offset_down, Hal_Motor_SenseVolts(0U), Hal_Motor_SenseVolts(1U),
                                                  Hal_Motor_SenseVolts(2U));
            osDelay(pdMS_TO_TICKS(1));
        }
        Hal_Motor_PwmStop();
        Motor_Detect_OffsetFinalizeUndriven(&s_offset_down);
        Motor_Detect_OffsetApply(&s_offset_down, &motor, true);
    }

    Hal_Motor_PwmStop();
    s_offset_isr_enable = false;
    s_detect_active = false;
    PRINTF("Detect offset: done\r\n");
    (void)Detect_SaveOffsetToFlash();
    return 1U;

abort_detect:
    s_offset_isr_enable = false;
    Hal_Motor_PwmStop();
    s_detect_active = false;
    {
        float ipeak = fabsf(motor.ia);
        if (fabsf(motor.ib) > ipeak) {
            ipeak = fabsf(motor.ib);
        }
        if (fabsf(motor.ic) > ipeak) {
            ipeak = fabsf(motor.ic);
        }
        PRINTF("Detect offset: aborted (latched=%d fault=%d %s Ipeak=%.3fA)\r\n",
               Motor_Protect_IsFaultLatched() ? 1 : 0, (int)Motor_Protect_GetFault(),
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()), (double)ipeak);
    }
    return 0U;
}

/**
 * @description: 查询偏移校准是否占用功率桥。
 * @return {bool} true 时 ISR 只采样+保护，不跑 FOC
 * @author: lkc
 */
bool Motor_Detect_IsActive(void)
{
    return s_detect_active;
}

/**
 * @description: DMA 完成中断喂入一拍电流/电压，仅零偏 ISR 采集窗口内有效。
 * @param {bool} pwm_down true=TIM1 向下计数
 * @return {*}
 */
void Motor_Detect_IsrFeed(bool pwm_down)
{
    motor_detect_offset_t *st;
    volatile uint32_t *count_vis;

    if (!s_offset_isr_enable) {
        return;
    }
    if (pwm_down) {
        st = &s_offset_down;
        count_vis = &s_offset_down_n;
    } else {
        st = &s_offset_up;
        count_vis = &s_offset_up_n;
    }
    if (st->count >= st->samples_per_phase) {
        return;
    }
    (void)Motor_Detect_OffsetFeedDrivenTriple(st, (float)Hal_Motor_CurrentAdc(0U), (float)Hal_Motor_CurrentAdc(1U),
                                              (float)Hal_Motor_CurrentAdc(2U), Hal_Motor_SenseVolts(0U),
                                              Hal_Motor_SenseVolts(1U), Hal_Motor_SenseVolts(2U));
    *count_vis = st->count;
}

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表。
 * @return {bool} true 表示 6 个有效扇区标定成功
 * @author: lkc
 */
bool DETECT_Hall(void)
{
    uint8_t hall_tab[MOTOR_HALL_TABLE_SIZE];
    bool res;
    int i;

    /* I/F 扫角电流；PI 限流，避免开环电压过流 */
    Motor_Hall_Detect(1.2f, hall_tab, &res);

    if (res) {
        INFO("DETECT HALL Success.\r\n");
        for (i = 0; i < (int)MOTOR_HALL_TABLE_SIZE; i++) {
            PRINTF("%d ", (int)motor.hall_table[i]);
        }
        PRINTF("\r\n");
        for (i = 0; i < 6; i++) {
            PRINTF("%d ", motor.hallABC[i]);
        }
        PRINTF("\r\n");
    } else {
        PRINTF("DETECT HALL Failed.\r\n");
    }

    return res;
}

/**
 * @description: 上电偏移校准：已有 Flash 电流零点则跳过，否则检测并写入 Flash。
 * @detail: 校准期间 ISR 仍采样+软件保护，但不跑 FOC；失败时 fault 保持锁存。
 * @return {void} 无
 * @author: lkc
 */
void Detect_Init(void)
{
    if (Motor_Detect_OffsetIsStored(&motor)) {
        PRINTF("Detect_Init: skip, using stored offset\r\n");
        PRINTF("Current Offset [%f] [%f] [%f]\r\n", motor.currentOffset[0], motor.currentOffset[1],
               motor.currentOffset[2]);
        PRINTF("Volt Offset [%f] [%f] [%f]\r\n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);
        PRINTF("Undriven Offset [%f] [%f] [%f]\r\n", motor.voltundrivenOffset[0], motor.voltundrivenOffset[1],
               motor.voltundrivenOffset[2]);
        return;
    }

    if (DETECT_Offset(true) == 0U) {
        PRINTF("Detect_Init: offset aborted\r\n");
    }
}

#ifdef __cplusplus
}
#endif
