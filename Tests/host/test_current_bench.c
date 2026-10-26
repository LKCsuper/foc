/* 停机电流/三角基准：各阻塞源与配置错误拒绝，成功路径不改写电机对象。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "motor_param.h"
#include "motor_current.h"
#include "motor_pid.h"
#include "motor_control.h"
#include "motor_feedback.h"
#include "motor_foc.h"

typedef struct { bool enable; float target_rpm; } motor_command_t;
#define MOTOR_OK 0
MOTOR_PARAM_S motor;
static MOTOR_PARAM_S test_motor;
static motor_command_t command;
static unsigned blocked, stopped, calls, lines, depth;
static int get_result;
static unsigned resets, begins, ends;
static unsigned probe_calls, capture_calls;
static bool detailed;
static bool arm_fail;
static unsigned math_calls;
/**
 * @description: 主机测试替身：从电机对象捕获当前命令字段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} instance instance 参数
 * @param {motor_command_t *} out 输出缓冲或结果
 * @return {int} 测试记录值 get_result
 */
static int Motor_GetCommand(MOTOR_PARAM_S *instance, motor_command_t *out)
{ assert(instance == &motor); *out = command; return get_result; }
/**
 * @description: 主机测试替身：查询位置环是否使能。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 blocked == 1U
 */
static bool Motor_Position_IsEnabled(void) { return blocked == 1U; }
/**
 * @description: 主机测试替身：查询关节阻抗是否使能。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 blocked == 2U
 */
static bool Motor_Joint_IsEnabled(void) { return blocked == 2U; }
/**
 * @description: 主机测试替身：查询 BLDC 是否占用主 PWM 出口。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 blocked == 3U
 */
static bool Motor_Bldc_IsSelected(void) { return blocked == 3U; }
/**
 * @description: 主机测试替身：偏移校准时 FOC 不得改占空比编排（ISR 仍采样+软件保护）。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 blocked == 4U
 */
static bool Motor_Detect_IsActive(void) { return blocked == 4U; }
/**
 * @description: 主机测试替身：查询默认实例是否已锁存故障。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 blocked == 5U
 */
static bool Motor_Protect_IsFaultLatched(void) { return blocked == 5U; }
/**
 * @description: 主机测试替身：读取默认单实例请求的反馈来源
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {motor_feedback_source_t} 测试记录值 blocked == 6U ? MOTOR_FEEDBACK_SRC_ENCODER : MOTOR_FEEDBACK_SRC_SENSORLESS
 */
motor_feedback_source_t Motor_Feedback_GetSourceDefault(void) { return blocked == 6U ? MOTOR_FEEDBACK_SRC_ENCODER : MOTOR_FEEDBACK_SRC_SENSORLESS; }
/**
 * @description: 主机测试替身：初始化控制实例（启动链复位到可启动状态）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_control_t *} ctrl ctrl 参数
 * @return {void} 无
 */
void Motor_Control_Init(motor_control_t *ctrl) { memset(ctrl, 0, sizeof(*ctrl)); }
/**
 * @description: 主机测试替身：停机软件实例的 I/F 外层逐项诊断，不输出 PWM。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {motor_control_t *} ctrl ctrl 参数
 * @param {MOTOR_PARAM_S *} instance instance 参数
 * @param {motor_current_target_ctx_t *} ctx ctx 参数
 * @param {unsigned} mode mode 参数
 * @return {motor_control_result_t} 测试记录值 result
 */
motor_control_result_t Motor_Control_IfDiagnosticStep(motor_control_t *ctrl, MOTOR_PARAM_S *instance,
                                                     motor_current_target_ctx_t *ctx, unsigned mode)
{ motor_control_result_t result = {0}; assert(ctrl && instance != &motor && ctx && mode <= 8U); calls++; return result; }
/**
 * @description: 主机测试替身：停止节拍和 DMA 回调累加。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Motor_Timing_CadenceStop(void) { stopped++; }
/**
 * @description: 主机测试替身：停止交接窗口继续写入。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Motor_Timing_HandoffStop(void) { stopped++; }
/**
 * @description: 主机测试替身：清零节拍、回调和分段耗时窗口，并重新允许采集。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Motor_Timing_CadenceReset(void) { resets++; }
/**
 * @description: 主机测试替身：关闭 DMA 回调耗时统计。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Motor_Timing_CallbackStop(void) { }
/**
 * @description: 主机测试替身：开关电流环内部的细分计时。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {bool} enabled 是否启用
 * @return {void} 无
 */
static void Motor_Timing_ProfileSetDetailed(bool enabled) { detailed = enabled; }
/**
 * @description: 主机测试替身：推进分段耗时分频，并决定本拍是否采样。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 测试记录值 !arm_fail
 */
static bool Motor_Timing_ProfileBegin(void) { assert(depth == 1U && detailed); begins++; return !arm_fail; }
/**
 * @description: 主机测试替身：清除本拍分段采样标志。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {void} 无
 */
static void Motor_Timing_ProfileEnd(void) { assert(depth == 1U); ends++; }
/**
 * @description: 主机测试替身：查询本拍细分计时是否有效。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
static bool Motor_Timing_ProfileDetailedIsActive(void) { probe_calls++; return false; }
/**
 * @description: 主机测试替身：判断本电流控制拍是否应记录交接样本。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {bool} 本实现固定返回 false
 */
static bool Motor_Timing_HandoffDue(void) { capture_calls++; return false; }
/**
 * @description: 主机测试替身：进入短临界区（屏蔽 IRQ），返回进入前的 PRIMASK。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 本实现固定返回 0U
 */
static uint32_t Motor_Crit_Enter(void) { assert(depth == 0U); depth++; return 0U; }
/**
 * @description: 主机测试替身：退出短临界区；仅当进入前 IRQ 开启时恢复。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {uint32_t} key 进入临界区时保存的中断状态
 * @return {void} 无
 */
static void Motor_Crit_Exit(uint32_t key) { assert(key == 0U && depth == 1U); depth--; }
/**
 * @description: 主机测试替身：读取电机通用微秒时基（自由运行，非 Hall 专用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @return {uint32_t} 测试记录值 now++
 */
static uint32_t Hal_Motor_TimeUs(void) { static uint32_t now; return now++; }
/**
 * @description: 记录测试中的逐行文本输出。
 * @param {const char *} format 兼容参数，本实现不使用
 * @param {...} ... 与格式字符串匹配的可变参数
 * @return {void} 无
 */
static void print_line(const char *format, ...) { (void)format; assert(depth == 0U); lines++; }
#define PRINTF print_line
/**
 * @description: 主机测试替身：同一角度共用正弦表索引与插值比例，计算 sin 与 cos。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @param {float *} sine sine 参数
 * @param {float *} cosine cosine 参数
 * @return {void} 无
 */
void sincos_f32(float angle, float *sine, float *cosine)
{ assert(depth == 1U); math_calls++; *sine = angle; *cosine = 1.0f; }
/**
 * @description: 主机测试替身：以象限缩减和泰勒多项式计算同角度正余弦，供后端对照。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @param {float *} sine sine 参数
 * @param {float *} cosine cosine 参数
 * @return {void} 无
 */
void sincos_taylor_f32(float angle, float *sine, float *cosine)
{ sincos_f32(angle, sine, cosine); }
/**
 * @description: 主机测试替身：只供停机对照，调用用户提供的原始正余弦函数，不修改第三方文件。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} angle angle 参数
 * @param {int} terms terms 参数
 * @param {float *} sine sine 参数
 * @param {float *} cosine cosine 参数
 * @return {void} 无
 */
void Motor_ReferenceTrigRaw(float angle, int terms, float *sine, float *cosine)
{ assert(terms == 3 || terms == 5 || terms == 7 || terms == 9 || terms == 12); sincos_f32(angle, sine, cosine); }
/**
 * @description: 主机测试替身：三相电流变换到 αβ，再变换到 d/q，写回反馈字段。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} instance instance 参数
 * @param {float} theta 强制电角度 (rad)
 * @return {void} 无
 */
void Motor_Current_FeedbackUpdateDq(MOTOR_PARAM_S *instance, float theta)
{ assert(instance != &motor && theta == 0.6f); instance->Id = 0.1f; instance->Iq = 0.2f; calls++; }
/**
 * @description: 主机测试替身：按工况选择 Id/Iq 目标，再按 MOTOR_FW_I_BUDGET 限幅。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {const MOTOR_PARAM_S *} instance instance 参数
 * @param {const motor_current_target_ctx_t *} ctx ctx 参数
 * @param {float *} id D 轴电流 (A)
 * @param {float *} iq Q 轴电流 (A)
 * @return {void} 无
 */
void Motor_Current_GetTargets(const MOTOR_PARAM_S *instance, const motor_current_target_ctx_t *ctx, float *id, float *iq)
{ assert(instance != &motor && ctx->soft_current_scale == 1.0f); *id = 0.0f; *iq = 1.0f; calls++; }
/**
 * @description: 主机测试替身：单轴电流 PI 控制器（d/q 轴通用）
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {float} target 目标机械转速 (RPM)
 * @param {float} measured measured 参数
 * @param {float *} integral integral 参数
 * @param {float} kp 电流比例增益 (V/A)
 * @param {float} ki 电流积分增益 (V/(A·s))
 * @param {float} dt 本次推进时间间隔 (s)
 * @param {float} limit limit 参数
 * @return {float} 测试记录值 *integral + target - measured
 */
float Motor_Current_PI_Run(float target, float measured, float *integral, float kp, float ki, float dt, float limit)
{ assert(kp == 0.12f && ki == 18.0f && dt > 0.0f && limit == 4.0f); calls++; return *integral + target - measured; }
/**
 * @description: 主机测试替身：对 vd/vq 做合成矢量限幅，饱和时回写 PI 积分。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} instance instance 参数
 * @param {motor_dq_voltage_t *} voltage 施加的电压 (V)
 * @param {float} id 兼容参数，本实现不使用
 * @param {float} iq 兼容参数，本实现不使用
 * @param {float} limit limit 参数
 * @return {void} 无
 */
void Motor_Current_LimitVoltage(MOTOR_PARAM_S *instance, motor_dq_voltage_t *voltage, float id, float iq, float limit)
{ assert(instance != &motor && voltage && limit == 4.0f); (void)id; (void)iq; calls++; }
/**
 * @description: 主机测试替身：运行 d/q 双轴电流 PI，并做矢量限幅。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} instance instance 参数
 * @param {float} id D 轴电流 (A)
 * @param {float} iq Q 轴电流 (A)
 * @param {float} dt 本次推进时间间隔 (s)
 * @param {float} limit limit 参数
 * @return {motor_dq_voltage_t} 测试记录值 voltage
 */
motor_dq_voltage_t Motor_Current_RunPi(MOTOR_PARAM_S *instance, float id, float iq, float dt, float limit)
{ motor_dq_voltage_t voltage = {id, iq}; assert(instance != &motor && dt > 0.0f && limit == 4.0f); calls++; return voltage; }
/**
 * @description: 主机测试替身：电流环一步：反馈变换、取目标、可选坐标交接、PI 及矢量限幅。
 * @detail: 仅供本文件的主机测试使用；模拟行为由函数体及测试断言限定。
 * @param {MOTOR_PARAM_S *} instance instance 参数
 * @param {float} theta 强制电角度 (rad)
 * @param {const motor_current_target_ctx_t *} ctx ctx 参数
 * @param {float} dt 本次推进时间间隔 (s)
 * @param {float} limit limit 参数
 * @param {float *} id D 轴电流 (A)
 * @param {float *} iq Q 轴电流 (A)
 * @return {motor_dq_voltage_t} 限幅后的 vd/vq (V)；motor 为空时为 0
 */
motor_dq_voltage_t Motor_Current_Step(MOTOR_PARAM_S *instance, float theta, const motor_current_target_ctx_t *ctx,
                                     float dt, float limit, float *id, float *iq)
{ assert(theta == 0.6f && ctx && ((id && iq) || (!id && !iq))); return Motor_Current_RunPi(instance, id ? *id : 0.0f, iq ? *iq : 1.0f, dt, limit); }
#include "current_bench_under_test.inc"
#include "trig_bench_under_test.inc"

/**
 * @description: 运行本文件的主机断言测试；失败时终止进程。
 * @return {int} 正常结束返回 0；断言失败会终止进程
 */
int main(void)
{
    MOTOR_PARAM_S before;
    unsigned index;
    for (index = 1U; index <= 6U; ++index) {
        blocked = index;
        assert(ol_current_bench() == -1 && calls == 0U && stopped == 0U);
    }
    blocked = 0U;
    command.enable = true; assert(ol_current_bench() == -1);
    command.enable = false; command.target_rpm = 300.0f; assert(ol_current_bench() == -1);
    command.target_rpm = 0.0f; motor.ol_enable = true; assert(ol_current_bench() == -1);
    motor.ol_enable = false; get_result = -1; assert(ol_current_bench() == -1);
    get_result = MOTOR_OK; before = motor; lines = 0U;
    assert(ol_current_bench() == 0);
    assert(calls == 16898U && stopped == 4U && lines == 49U && depth == 0U);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    for (index = 1U; index <= 5U; ++index) {
        blocked = index;
        assert(ol_trig_bench() == -1 && math_calls == 0U);
    }
    blocked = 0U; lines = 0U;
    assert(ol_trig_bench() == 0);
    assert(math_calls == 10752U && lines == 25U && depth == 0U);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    assert(probe_calls == 1024U && capture_calls == 1024U);
    assert(resets == 2U && begins == 128U && ends == 128U && !detailed);
    arm_fail = true;
    assert(ol_current_bench() == -1);
    assert(begins == 384U && ends == 128U && !detailed && depth == 0U);
    assert(memcmp(&before, &motor, sizeof(motor)) == 0);
    puts("Stopped current benchmark tests passed");
    return 0;
}
