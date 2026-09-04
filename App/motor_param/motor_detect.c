/*
 * @Description: 电流/电压偏移校准。R/L/磁链等电机参数检测后期再做。
 * @Author: lkc
 */
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "includes.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MOTOR_DETECT_OFFSET_SAMPLES     (1000U) /* 每相采样次数 */
#define MOTOR_DETECT_OFFSET_MAX_PERCENT (5.0f)  /* 电流零点偏离 ADC 中点超过 5% 时打印警告 */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * 校准任务会改 PWM 输出。置位后 DMA 中断只更新 ADC_Value[]，不跑 FOC，
 * 避免双方抢写 PWM。
 */
static volatile bool s_detect_active = false;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @description: 对单个相累加电流和电压采样
 * @param {uint32_t} phase    0=A, 1=B, 2=C
 * @param {uint32_t} samples  采样次数
 * @param {float} *current_acc 电流累加
 * @param {float} *voltage_acc 电压累加
 * @return {*}
 * @author: lkc
 */
static void Detect_SamplePhase(uint32_t phase, uint32_t samples, float *current_acc, float *voltage_acc)
{
    uint32_t i;

    for (i = 0U; i < samples; i++) {
        switch (phase) {
        case 0:
            *current_acc += (float)Hal_Motor_CurrentAdc(0U);
            *voltage_acc += Hal_Motor_SenseVolts(0U);
            break;
        case 1:
            *current_acc += (float)Hal_Motor_CurrentAdc(1U);
            *voltage_acc += Hal_Motor_SenseVolts(1U);
            break;
        default:
            *current_acc += (float)Hal_Motor_CurrentAdc(2U);
            *voltage_acc += Hal_Motor_SenseVolts(2U);
            break;
        }
        osDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @description: 校准三相电流零点和驱动时相电压偏移
 * @detail: 50% 占空比依次使能 A/B/C 相采样。cal_undriven=true 时再采未驱动相电压。
 * @param {BOOL} cal_undriven  true 时额外校准未驱动相电压偏移
 * @return {ULONG} 1 成功
 * @author: lkc
 */
ULONG DETECT_Offset(BOOL cal_undriven)
{
    const uint32_t samples = MOTOR_DETECT_OFFSET_SAMPLES;
    float current_sum[3] = {0.0f, 0.0f, 0.0f};
    float voltage_sum[3] = {0.0f, 0.0f, 0.0f};
    float v_avg;
    float adc_range = 4096.0f;
    float abs_max;

    /*
     * 先占住互斥，再停机关桥。之后 DMA 中断只刷 ADC，不再改 PWM。
     */
    s_detect_active = true;
    motor.ol_enable = false;
    motor.ol_target_rpm = 0.0f;
    motor.ol_current_rpm = 0.0f;
    motor.ol_vd_int = 0.0f;
    motor.ol_vq_int = 0.0f;
    Motor_SpeedLoop_Reset();
    Motor_Sensorless_Reset();
    Hal_Motor_PwmStop();
    PRINTF("Detect offset: begin\r\n");

    osDelay(pdMS_TO_TICKS(1000));

    Hal_Motor_PwmSetSafe();

    Hal_Motor_EnablePhase(1U);
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: phase A\r\n");
    Detect_SamplePhase(0U, samples, &current_sum[0], &voltage_sum[0]);

    Hal_Motor_EnablePhase(2U);
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: phase B\r\n");
    Detect_SamplePhase(1U, samples, &current_sum[1], &voltage_sum[1]);

    Hal_Motor_EnablePhase(3U);
    osDelay(pdMS_TO_TICKS(1));
    PRINTF("Detect offset: phase C\r\n");
    Detect_SamplePhase(2U, samples, &current_sum[2], &voltage_sum[2]);

    Hal_Motor_PwmStop();

    motor.currentOffset[0] = current_sum[0] / (float)samples;
    motor.currentOffset[1] = current_sum[1] / (float)samples;
    motor.currentOffset[2] = current_sum[2] / (float)samples;

    voltage_sum[0] /= (float)samples;
    voltage_sum[1] /= (float)samples;
    voltage_sum[2] /= (float)samples;
    v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0f;

    motor.voltOffset[0] = voltage_sum[0] - v_avg;
    motor.voltOffset[1] = voltage_sum[1] - v_avg;
    motor.voltOffset[2] = voltage_sum[2] - v_avg;

    PRINTF("ADC_RAW [%d] [%d] [%d]\r\n",
           (int)Hal_Motor_CurrentAdc(0U), (int)Hal_Motor_CurrentAdc(1U),
           (int)Hal_Motor_CurrentAdc(2U));
    PRINTF("Current Offset [%f] [%f] [%f]\r\n",
           motor.currentOffset[0], motor.currentOffset[1], motor.currentOffset[2]);
    PRINTF("Volt Offset [%f] [%f] [%f]\r\n",
           motor.voltOffset[0], motor.voltOffset[1], motor.voltOffset[2]);

    abs_max = fabsf(motor.currentOffset[0] - adc_range * 0.5f);
    if (fabsf(motor.currentOffset[1] - adc_range * 0.5f) > abs_max) {
        abs_max = fabsf(motor.currentOffset[1] - adc_range * 0.5f);
    }
    if (fabsf(motor.currentOffset[2] - adc_range * 0.5f) > abs_max) {
        abs_max = fabsf(motor.currentOffset[2] - adc_range * 0.5f);
    }
    if (abs_max > adc_range * MOTOR_DETECT_OFFSET_MAX_PERCENT / 100.0f) {
        PRINTF("WARNING: current offset > %.0f%% ADC range — check hardware\r\n",
               (double)MOTOR_DETECT_OFFSET_MAX_PERCENT);
    }

    if (cal_undriven) {
        uint32_t i;

        PRINTF("Detect offset: undriven voltage\r\n");
        osDelay(pdMS_TO_TICKS(10));

        voltage_sum[0] = 0.0f;
        voltage_sum[1] = 0.0f;
        voltage_sum[2] = 0.0f;

        for (i = 0U; i < samples; i++) {
            v_avg = (Hal_Motor_SenseVolts(0U) +
                     Hal_Motor_SenseVolts(1U) +
                     Hal_Motor_SenseVolts(2U)) / 3.0f;
            voltage_sum[0] += Hal_Motor_SenseVolts(0U) - v_avg;
            voltage_sum[1] += Hal_Motor_SenseVolts(1U) - v_avg;
            voltage_sum[2] += Hal_Motor_SenseVolts(2U) - v_avg;
            osDelay(pdMS_TO_TICKS(1));
        }
        Hal_Motor_PwmStop();

        motor.voltundrivenOffset[0] = voltage_sum[0] / (float)samples;
        motor.voltundrivenOffset[1] = voltage_sum[1] / (float)samples;
        motor.voltundrivenOffset[2] = voltage_sum[2] / (float)samples;
    }

    Hal_Motor_PwmStop();
    s_detect_active = false;
    PRINTF("Detect offset: done\r\n");
    return 1U;
}

/**
 * @description: 查询偏移校准是否占用功率桥
 * @return {bool} true 时 FOC 中断必须立刻返回
 * @author: lkc
 */
bool Motor_Detect_IsActive(void)
{
    return s_detect_active;
}

/**
 * @description: 开环转一圈，标定霍尔状态与电角度对应表
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
 * @description: 上电做一次电流/电压偏移校准
 * @detail: 校准期间 ADC 中断里的 FOC 必须关闭
 * @return {*}
 * @author: lkc
 */
VOID Detect_Init(VOID)
{
    DETECT_Offset(TRUE);
}

#ifdef __cplusplus
}
#endif
