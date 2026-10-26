/* 传感器更新调度主机测试：编码器/霍尔/无感源切换与故障触发 */
/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensor_update.h"
#include "motor_protect.h"

static unsigned s_encoder_updates;
static unsigned s_reference_updates;
static float s_encoder_dt;
static bool s_encoder_fault;
static bool s_compare;
static bool s_detect_active;
static bool s_learning;
static bool s_fault_latched;
static unsigned s_fault_triggers;
static mc_fault_code s_fault_code;
static unsigned s_hall_reads;
static unsigned s_hall_updates;
static int s_hall_state = 5;
static const int8_t *s_hall_table;
static float s_hall_poles;
static float s_hall_dt;

/* Private functions ---------------------------------------------------------*/
/**
 * @description: 主机测试替身：用 HAL 当前计数与采样间隔更新默认实例转速
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} dt_s 本次推进时间间隔 (s)
 * @return {void} 无
 */
void Motor_Encoder_Update(float dt_s)
{
    s_encoder_updates++;
    s_encoder_dt = dt_s;
}
/**
 * @description: 主机测试替身：查询默认实例是否处于信号故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_encoder_fault
 */
bool Motor_Encoder_IsSignalFault(void) { return s_encoder_fault; }
/**
 * @description: 主机测试替身：按参考诊断周期更新编码器，不作为控制反馈。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} dt_s 本次推进时间间隔 (s)
 * @return {void} 无
 */
void Motor_Encoder_UpdateReference(float dt_s)
{
    s_reference_updates++;
    Motor_Encoder_Update(dt_s);
}
/**
 * @description: 主机测试替身：查询编码器对照采集是否打开。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_compare
 */
bool Motor_Encoder_IsCompareEnabled(void) { return s_compare; }
/**
 * @description: 主机测试替身：查询编码器电角零点对齐是否正在占用控制输出。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_detect_active
 */
bool Motor_Encoder_DetectIsActive(void) { return s_detect_active; }
/**
 * @description: 主机测试替身：查询是否正在学习。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_learning
 */
bool Motor_Anticogging_IsLearning(void) { return s_learning; }
/**
 * @description: 主机测试替身：查询默认实例是否已锁存故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 s_fault_latched
 */
bool Motor_Protect_IsFaultLatched(void) { return s_fault_latched; }
/**
 * @description: 主机测试替身：默认实例外部触发故障锁存。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {mc_fault_code} fault 故障条件或故障编码
 * @param {const MOTOR_PARAM_S *} motor 兼容参数，本实现不使用
 * @return {void} 无
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor)
{
    (void)motor;
    s_fault_triggers++;
    s_fault_code = fault;
}
/**
 * @description: 主机测试替身：读取三相霍尔状态（bit0=A, bit1=B, bit2=C）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} second_motor second_motor 参数
 * @param {int} samples 采样数量或采样缓冲
 * @return {int} 测试记录值 s_hall_state
 */
int Motor_Hall_Read(bool second_motor, int samples)
{
    assert(!second_motor);
    assert(samples == 3);
    s_hall_reads++;
    return s_hall_state;
}
/**
 * @description: 主机测试替身：默认实例更新
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const int8_t *} table 八项霍尔电角编码表
 * @param {int} hall_state hall_state 参数
 * @param {float} poles poles 参数
 * @param {float} dt_s 本次推进时间间隔 (s)
 * @return {void} 无
 */
void Motor_Hall_AngleUpdateDefault(const int8_t *table, int hall_state, float poles, float dt_s)
{
    s_hall_updates++;
    s_hall_table = table;
    assert(hall_state == s_hall_state);
    s_hall_poles = poles;
    s_hall_dt = dt_s;
}

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S motor;
    unsigned i;

    memset(&motor, 0, sizeof(motor));
    motor.ol_enable = true;
    motor.ol_pole_pairs = 7.0f;
    motor.hall_extra_samples = 3;

    s_encoder_fault = true;
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_ENCODER, 0.002f);
    assert(s_encoder_updates == 1U && fabsf(s_encoder_dt - 0.002f) < 1.0e-9f);
    assert(s_fault_triggers == 1U && s_fault_code == FAULT_CODE_ENCODER_SPI);
    assert(s_reference_updates == 0U);

    s_detect_active = true;
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_ENCODER, 0.002f);
    assert(s_fault_triggers == 1U);
    s_detect_active = false;
    s_learning = true;
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_ENCODER, 0.002f);
    assert(s_fault_triggers == 1U);

    s_learning = false;
    s_encoder_fault = false;
    for (i = 0U; i < 3U; i++) {
        Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_HALL, 0.001f);
    }
    assert(s_hall_reads == 0U && s_hall_updates == 0U);
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_HALL, 0.001f);
    assert(s_hall_reads == 1U && s_hall_updates == 1U);
    assert(s_hall_table == motor.hall_table && s_hall_poles == 7.0f);
    assert(fabsf(s_hall_dt - 0.004f) < 1.0e-9f);

    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_OPENLOOP, 0.001f);
    Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_HALL, 0.001f);
    assert(s_hall_updates == 1U);

    {
        unsigned before = s_encoder_updates;
        unsigned faults = s_fault_triggers;
        s_encoder_fault = true;
        Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_SENSORLESS, 0.001f);
        assert(s_encoder_updates == before);
        s_compare = true;
        Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_SENSORLESS, 0.001f);
        assert(s_encoder_updates == before + 1U);
        s_encoder_fault = false;
        for (i = 0U; i < 9U; i++) {
            Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_SENSORLESS, 0.0001f);
        }
        assert(s_reference_updates == 1U);
        Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_SENSORLESS, 0.00011f);
        assert(s_reference_updates == 2U);
        assert(fabsf(s_encoder_dt - 0.00101f) < 1.0e-8f);
        before = s_encoder_updates - 1U;
        assert(s_reference_updates == 2U);
        assert(s_fault_triggers == faults); /* 参考传感器失联不能接管或触发控制故障。 */
        s_compare = false;
        Motor_SensorUpdate_Step(&motor, MOTOR_FEEDBACK_SRC_OPENLOOP, 0.001f);
        assert(s_encoder_updates == before + 1U);
    }
    puts("test_motor_sensor_update: pass");
    return 0;
}
