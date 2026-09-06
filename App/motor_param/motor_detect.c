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
#include <math.h>
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "motor_detect.h"
#include "motor_detect_offset.h"
#include "motor_param.h"
#include "motor_sensorless.h"
#include "motor_hall.h"
#include "hal_motor.h"
#include "motor_protect.h"
#include "printf_color.h"
#include "motor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * 校准任务会改 PWM 输出。置位后 DMA 中断仍采样换算，但不跑 FOC、不跑软件过流
 * （零偏未定时电流不可信）；ADC_Value[] 由 DMA 硬件持续刷新。
 */
static volatile bool s_detect_active = false;
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
 * @description: 对当前驱动相阻塞采集 samples 次，每次间隔 1 ms，喂入非阻塞核心。
 * @param {motor_detect_offset_t *} st 偏移状态
 * @param {uint8_t} phase 0=A，1=B，2=C
 * @return {bool} false 表示保护锁存已中止
 * @author: lkc
 */
static bool Detect_SamplePhaseBlocking(motor_detect_offset_t *st, uint8_t phase)
{
    uint32_t i;
    float i_adc;
    float v_sense;

    for (i = 0U; i < st->samples_per_phase; i++) {
        if (Detect_ShouldAbort()) {
            return false;
        }
        switch (phase) {
            case 0U:
                i_adc = (float)Hal_Motor_CurrentAdc(0U);
                v_sense = Hal_Motor_SenseVolts(0U);
                break;
            case 1U:
                i_adc = (float)Hal_Motor_CurrentAdc(1U);
                v_sense = Hal_Motor_SenseVolts(1U);
                break;
            default:
                i_adc = (float)Hal_Motor_CurrentAdc(2U);
                v_sense = Hal_Motor_SenseVolts(2U);
                break;
        }
        (void)Motor_Detect_OffsetFeedDriven(st, i_adc, v_sense);
        osDelay(pdMS_TO_TICKS(1));
    }
    return !Detect_ShouldAbort();
}

/**
 * @description: 校准三相电流零点和驱动时相电压偏移。
 * @detail: 50% 占空比依次使能 A/B/C 相采样。cal_undriven=true 时再采未驱动相电压。
 *          累加/结算走 Motor_Detect_Offset*；本函数负责停机、PWM、osDelay 与打印。
 * @param {bool} cal_undriven true 时额外校准未驱动相电压偏移
 * @return {uint32_t} 1 成功
 * @author: lkc
 */
uint32_t DETECT_Offset(bool cal_undriven)
{
    motor_detect_offset_t st;

    /*
     * 先占住互斥，再停机关通道。之后 DMA 中断只采样换算（不跑软件过流），不跑 FOC。
     */
    if (Motor_Protect_IsFaultLatched()) {
        PRINTF("Detect offset: blocked, fault=%s — run: olfault clear\r\n",
               Motor_Protect_GetFaultName(Motor_Protect_GetFault()));
        return 0U;
    }

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

    Motor_Detect_OffsetReset(&st, cal_undriven, MOTOR_DETECT_OFFSET_DEFAULT_SAMPLES,
                             MOTOR_DETECT_OFFSET_DEFAULT_ADC_SCALE);

    osDelay(pdMS_TO_TICKS(1000));
    if (Detect_ShouldAbort()) {
        goto abort_detect;
    }

    Hal_Motor_PwmSetSafe();

    Hal_Motor_EnablePhase(1U);
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: phase A\r\n");
    if (!Detect_SamplePhaseBlocking(&st, 0U)) {
        goto abort_detect;
    }

    (void)Motor_Detect_OffsetNextPhase(&st);
    Hal_Motor_EnablePhase(2U);
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: phase B\r\n");
    if (!Detect_SamplePhaseBlocking(&st, 1U)) {
        goto abort_detect;
    }

    (void)Motor_Detect_OffsetNextPhase(&st);
    Hal_Motor_EnablePhase(3U);
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: phase C\r\n");
    if (!Detect_SamplePhaseBlocking(&st, 2U)) {
        goto abort_detect;
    }

    Hal_Motor_PwmStop();
    Motor_Detect_OffsetFinalizeDriven(&st);
    Motor_Detect_OffsetApply(&st, &motor, false);

    PRINTF("ADC_RAW [%d] [%d] [%d]\r\n", (int)Hal_Motor_CurrentAdc(0U), (int)Hal_Motor_CurrentAdc(1U),
           (int)Hal_Motor_CurrentAdc(2U));
    PRINTF("Current Offset [%f] [%f] [%f]\r\n", motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
    PRINTF("Volt Offset [%f] [%f] [%f]\r\n", motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);

    if (st.offset_warn) {
        PRINTF("WARNING: current offset > %.0f%% ADC range — check hardware\r\n",
               (double)MOTOR_DETECT_OFFSET_WARN_PERCENT);
    }

    if (cal_undriven) {
        uint32_t i;

        PRINTF("Detect offset: undriven voltage\r\n");
        osDelay(pdMS_TO_TICKS(10));

        for (i = 0U; i < st.samples_per_phase; i++) {
            if (Detect_ShouldAbort()) {
                goto abort_detect;
            }
            (void)Motor_Detect_OffsetFeedUndriven(&st, Hal_Motor_SenseVolts(0U), Hal_Motor_SenseVolts(1U),
                                                  Hal_Motor_SenseVolts(2U));
            osDelay(pdMS_TO_TICKS(1));
        }
        Hal_Motor_PwmStop();
        Motor_Detect_OffsetFinalizeUndriven(&st);
        Motor_Detect_OffsetApply(&st, &motor, true);
    }

    Hal_Motor_PwmStop();
    s_detect_active = false;
    PRINTF("Detect offset: done\r\n");
    return 1U;

abort_detect:
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
 * @description: 开环转一圈，标定霍尔状态与电角度对应表。
 * @return {bool} true 表示 6 个有效扇区标定成功
 * @author: lkc
 */
bool DETECT_Hall(void)
{
    uint8_t hall_tab[MOTOR_HALL_TABLE_SIZE];
    bool res;
    int i;

    Motor_Hall_Detect(6.0f / 3.0f, hall_tab, &res);

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
 * @description: 上电做一次电流/电压偏移校准。
 * @detail: 校准期间 ISR 仍采样+软件保护，但不跑 FOC；失败时 fault 保持锁存。
 * @return {void} 无
 * @author: lkc
 */
void Detect_Init(void)
{
    if (DETECT_Offset(true) == 0U) {
        PRINTF("Detect_Init: offset aborted\r\n");
    }
}

#ifdef __cplusplus
}
#endif
