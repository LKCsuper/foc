/*
 * @Description: 电机过流、欠压、堵转与失速保护接口（支持状态实例）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 保护只“判断并锁存原因”，不停 PWM；关断由 ISR/控制出口在故障后执行。
 *   2) 软件保护带去抖，不能代替硬件过流/短路关断。
 *   3) State API 不读无感/速度环全局；锁存诊断经 motor_protect_aux_t 由调用者注入。
 *   4) 无参包装走默认单实例，内部 FillAuxDefault 仍拉无感（过渡期兼容）。
 */
#ifndef MOTOR_PROTECT_H
#define MOTOR_PROTECT_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "motor_protect_core.h"
#include "motor_param.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/




/**
 * @description: 过流去抖检测（三相电流峰值）。
 * @detail: 只推进过流计时并返回是否满窗，不锁存故障、不停 PWM。
 *          state 或 motor 为空返回 false。软件去抖不能代替硬件过流关断。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   采样电流 (A)
 * @param {float} dt                      本拍时间 (s)，通常为 ADC 周期
 * @return {bool} true=条件已持续超过 over_current_time
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrentState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 欠压去抖检测。
 * @detail: 仅在使能且目标转速非零时累计，只返回是否满窗，不锁存、不停 PWM。
 *          state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   母线电压 (V) 与使能来源
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=欠压已持续超过 under_voltage_time
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltageState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 堵转去抖检测。
 * @detail: 高目标、近零实测速、大电流，且速度反馈有效时才累计。
 *          只返回是否满窗，不锁存、不停 PWM。state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   目标转速 (RPM) 与电流 (A)
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=堵转条件已持续超过 stall_time
 * @author: lkc
 */
bool Motor_Protect_CheckStallState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 失速去抖检测。
 * @detail: 斜坡跟上目标后，曾达约 50% 再跌破 speed_loss_ratio 才累计。
 *          斜坡未到位不算失速。只返回是否满窗，不锁存、不停 PWM。
 *          state 或 motor 为空返回 false。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   目标与斜坡转速 (RPM)
 * @param {float} dt                      本拍时间 (s)
 * @return {bool} true=失速条件已持续超过 speed_loss_time
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLossState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 按优先级跑完保护并在首次触发时锁存故障与快照。
 * @detail: 优先级：过流 > 欠压 > 堵转 > 失速。已锁存则直接返回 true，不覆盖首因。
 *          state 或 motor 为空返回 false。aux 可为 NULL。不关 PWM。
 * @param {motor_protect_state_t *} state 状态对象；空则忽略
 * @param {const MOTOR_PARAM_S *} motor   电机实时量
 * @param {float} dt                      本拍时间 (s)
 * @param {const motor_protect_aux_t *} aux 锁存用辅助快照；可为 NULL
 * @return {bool} true=当前已锁存故障（含本拍新锁存）
 * @author: lkc
 */
bool Motor_Protect_RunState(motor_protect_state_t *state, const MOTOR_PARAM_S *motor, float dt,
                            const motor_protect_aux_t *aux);

/**
 * @description: 由外部模块直接锁存故障。
 * @detail: 不覆盖已有首因，不关 PWM。state 与 motor 不可为空，违反约定触发 COMMON_ASSERT。
 *          fault 为 FAULT_CODE_NONE 时忽略。aux 可为 NULL。
 * @param {motor_protect_state_t *} state 状态对象，不可为空
 * @param {mc_fault_code} fault           故障码；FAULT_CODE_NONE 忽略
 * @param {const MOTOR_PARAM_S *} motor   用于填充快照，不可为空
 * @param {const motor_protect_aux_t *} aux 辅助快照；可为 NULL
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_TriggerFaultState(motor_protect_state_t *state, mc_fault_code fault,
                                     const MOTOR_PARAM_S *motor, const motor_protect_aux_t *aux);









/**
 * @description: 初始化默认单实例保护模块。
 * @detail: 清故障锁存、计时和首发采样证据。config 为 NULL 时用代码默认阈值。
 *          不访问硬件，可在上电初始化调用。
 * @param {const motor_protect_config_t *} config 自定义参数；NULL 用默认
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_Init(const motor_protect_config_t *config);

/**
 * @description: 更新默认实例的速度反馈。
 * @detail: 只写默认单实例。valid 为 false 时清堵转/失速计时及曾达速标志。
 * @param {float} rpm  机械转速 (RPM)
 * @param {bool} valid 反馈是否可信
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid);

/**
 * @description: 默认实例完整保护步进，仅首次故障同拍采集无感/速度辅助快照。
 * @detail: 先判定再锁存，已锁存时不再读无感全局。motor 为空返回 false。
 *          首发锁存后置采样待采集，由 FOC ISR 调 CaptureFaultSample。不关 PWM。
 * @param {const MOTOR_PARAM_S *} motor 电机实时参数；空则忽略
 * @param {float} dt                    本拍时间 (s)，ADC 路径约 1/30 kHz
 * @return {bool} true=已锁存故障
 * @author: lkc
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 默认实例外部触发故障锁存。
 * @detail: 同拍采集无感/速度辅助快照，不覆盖已有首因，不关 PWM。
 *          motor 不可为空，违反约定触发 COMMON_ASSERT。FAULT_CODE_NONE 忽略。
 *          不武装首发 ADC 采样采集。
 * @param {mc_fault_code} fault         故障码；FAULT_CODE_NONE 忽略
 * @param {const MOTOR_PARAM_S *} motor 快照来源，不可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor);

/**
 * @description: 清除默认实例故障锁存、计时和首发采样证据。
 * @detail: 保留配置。不访问硬件，不停 PWM。
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_ClearFault(void);

/**
 * @description: 查询默认实例是否已锁存故障。
 * @detail: 锁存且故障码不是 FAULT_CODE_NONE 才返回 true。
 * @return {bool} true=已锁存有效故障
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatched(void);

/**
 * @description: 读取默认实例当前故障码。
 * @detail: 未锁存时为 FAULT_CODE_NONE。不访问硬件。
 * @return {mc_fault_code} 故障码
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFault(void);

/**
 * @description: 故障码转可读短名，供 Shell 与日志使用。
 * @detail: 返回静态字符串，生命周期与程序相同。未知码返回 "UNKNOWN"。
 * @param {mc_fault_code} fault 故障码
 * @return {const char *} 静态短名，如 "MOTOR_STALL"
 * @author: lkc
 */
const char *Motor_Protect_GetFaultName(mc_fault_code fault);

/**
 * @description: 拷贝默认实例故障快照。
 * @detail: 停机后仍保留至 ClearFault。snapshot 为空则忽略，不阻塞。
 * @param {motor_protect_snapshot_t *} snapshot 输出缓冲；可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_GetSnapshot(motor_protect_snapshot_t *snapshot);

/**
 * @description: 首发默认保护故障的 RAM 采样证据。
 * @detail: adc 与 configured_offset 按 CURR1/2/3 排列，current_abc 按逻辑 A/B/C。
 *          valid 只表示已复制，不保证 DMA 帧一致或 ADC 物理准确。
 *          offset 为配置值，非法时 Sense 会另行回退。
 */
typedef struct {
    bool valid;
    float adc[3], configured_offset[3], current_abc[3];
    float scale_a_per_count, trigger_dt_s, over_current_elapsed_s;
    float over_current_threshold_a, over_current_delay_s;
    uint32_t entry_us;
    uint16_t callback_count, callback_period, callback_cr1;
} motor_protect_sample_t;

/**
 * @description: 首发默认保护故障后复制一帧已有采样证据。
 * @detail: 由 FOC ISR 在待采集且已锁存时调用一次。只复制 motor 里已有 ADC 与电流，
 *          不重新访问 ADC，不改保护和输出。motor 为空、未待采集或未锁存则返回。
 *          adc 与 configured_offset 按 CURR1/2/3，current_abc 按逻辑 A/B/C。
 *          valid 只表示已复制，不证明 DMA 帧一致或 ADC 物理准确。
 *          entry_us 是控制入口时间；count、period、cr1 是回调期顺序读数，
 *          不是采样孔径或锁存证明。外部触发故障不武装此采集。
 * @param {const MOTOR_PARAM_S *} motor 已换算的电流 (A) 与 ADC 帧；空则忽略
 * @param {float} scale_a_per_count     电流比例 (A/count)
 * @param {uint32_t} entry_us           控制入口时间 (us)
 * @param {uint16_t} count              回调期定时器计数值
 * @param {uint16_t} period             回调期定时器周期
 * @param {uint16_t} cr1                回调期定时器 CR1
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_CaptureFaultSample(const MOTOR_PARAM_S *motor, float scale_a_per_count, uint32_t entry_us,
                                      uint16_t count, uint16_t period, uint16_t cr1);

/**
 * @description: 查询首发故障采样是否仍待采集。
 * @detail: FOC ISR 仅在返回 true 时读取回调元数据。首发采集完成后返回 false，
 *          故障保持期间不再重复取时序。
 * @return {bool} true=待采集
 * @author: lkc
 */
bool Motor_Protect_FaultSampleIsPending(void);

/**
 * @description: 读取首发故障采样快照。
 * @detail: 停机后的应用上下文调用，不阻塞。sample 为空则忽略。无数据时 valid 为 false。
 * @param {motor_protect_sample_t *} sample 输出缓冲；可为空
 * @return {void} 无
 * @author: lkc
 */
void Motor_Protect_GetFaultSample(motor_protect_sample_t *sample);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
