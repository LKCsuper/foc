/*
 * @Description: 开环/I/F 主控制编排与输出仲裁（不含 HAL PWM）
 * @Author: lkc
 *
 * 学习要点：
 *   1) 本模块决定本拍电压矢量，不写定时器寄存器。
 *   2) SAFE=中点占空比请求（非关桥）；OUTPUT_DQ=vd/vq/theta；关通道用 Hal_Motor_PwmStop。
 *   3) 电流环电角经 Motor_Feedback 选择；启动链状态在 motor_control_t。
 *   4) motor_it：采样、保护、分频、Step、应用 result。
 *   5) 速度 PI 不在 Step 内积分；ISR 只置允许标志并预置 Iq，任务调用 Run。
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "motor_param.h"
#include "motor_output_core.h"
#include "motor_alignment.h"
#include "motor_if_start.h"
#include "motor_hfi_start.h"
#include "motor_fw.h"
#include "motor_current.h"
/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/**
 * @description: 启动链状态（对齐 + I/F 派生 + HFI）
 */
typedef struct {
    /** 启动阶段：电流爬升 → 可选定角保持 → 平滑拖转。 */
    enum {
        MOTOR_STARTUP_CURRENT_RAMP = 0,
        MOTOR_STARTUP_ANGLE_HOLD,
        MOTOR_STARTUP_DRAG_RAMP
    } phase;
    motor_alignment_t alignment;
    motor_if_start_t if_start;
    motor_hfi_start_t hfi_start;
    float soft_current_scale; /* 0..1，定角电流/开环电压爬升 */
    float drag_current_scale; /* 0..1，Id 到 Iq 的平滑交接 */
} motor_control_startup_t;

/* 规整后字段的原始字节快照；逐字段比较，不比较结构体填充字节。 */
typedef struct {
#define MOTOR_NORMALIZE_FIELD(name) uint32_t name;
#include "motor_control_normalize_fields.inc"
#undef MOTOR_NORMALIZE_FIELD
} motor_control_normalized_config_t;

/**
 * @description: 控制编排实例（不持久化）
 */
typedef struct {
    motor_control_startup_t startup;
    motor_fw_t fw;           /* 弱磁负 Id 状态 */
    int last_mode;           /* 上一控制模式；-1 表示尚未运行 */
    bool speed_loop_latched; /* 速度 PI 已接管；跌回门槛以下才释放给启动 Iq */
    float current_frame_correction; /* 本拍无感渐变附加角，排除正常转速积分(rad) */
    motor_control_normalized_config_t normalized_config; /* 上次规整后字段；直接修改也在下一拍识别 */
    bool normalized_config_valid; /* Init 清零，首次运行必须完整规整 */
} motor_control_t;

/**
 * @description: 停机软件实例的 I/F 外层逐项诊断，不输出 PWM。
 * @detail: 仅供停机命令对独立实例调用；固定 24 V、15 kHz、反馈角取 ol_theta。
 *          mode 0=空入口，1=选角，2=给定反馈选角，3=上下文，4=关闭弱磁步进，
 *          5=电压上限，6=RunIf，7=给定反馈 RunIf，8=同上下文电流 Step。
 *          不运行启动、观测器或速度任务；会修改传入实例及共享 PI 诊断统计。
 * @param {motor_control_t *} ctrl 独立控制状态，不可为空
 * @param {MOTOR_PARAM_S *} motor 独立电机参数，不可为空
 * @param {motor_current_target_ctx_t *} ctx 独立目标上下文，不可为空
 * @param {unsigned} mode 诊断模式 0～8
 * @return {motor_control_result_t} 仅供校验使用的结果，调用方不得应用到硬件
 */
motor_control_result_t Motor_Control_IfDiagnosticStep(motor_control_t *ctrl, MOTOR_PARAM_S *motor,
                                                     motor_current_target_ctx_t *ctx, unsigned mode);

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/**
 * @description: 初始化控制实例（启动链复位到可启动状态）
 * @param {motor_control_t} *ctrl  控制实例
 * @return {void} 无
 * @author: lkc
 */
void Motor_Control_Init(motor_control_t *ctrl);

/**
 * @description: 复位启动链与速度/无感/电流积分显示（不清用户目标命令）
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   电机对象
 * @return {void} 无
 * @author: lkc
 */
void Motor_Control_ResetRuntime(motor_control_t *ctrl, MOTOR_PARAM_S *motor);

/**
 * @description: 计算电压限幅：min(ol_vq_limit, Vbus/sqrt(3))，且非负
 * @param {const MOTOR_PARAM_S} *motor  含 ol_vq_limit
 * @param {float} v_bus                 母线电压 (V)
 * @return {float} 限幅 (V)
 * @author: lkc
 */
float Motor_Control_GetVoltageLimit(const MOTOR_PARAM_S *motor, float v_bus);

/**
 * @description: 判断本拍是否允许运行开环/I/F
 * @param {const MOTOR_PARAM_S} *motor  使能与目标
 * @param {float} v_bus                 母线电压 (V)
 * @param {bool} hall_detect_active     霍尔标定占用时放宽“目标转速非零”
 * @return {bool} true=可继续控制
 * @author: lkc
 */
bool Motor_Control_IsReady(const MOTOR_PARAM_S *motor, float v_bus, bool hall_detect_active);

/**
 * @description: 钳位开环/无感相关配置字段到合法范围
 * @param {MOTOR_PARAM_S} *motor  被原地修改
 * @return {void} 无
 * @author: lkc
 */
void Motor_Control_NormalizeConfig(MOTOR_PARAM_S *motor);

/**
 * @description: 按斜坡更新 ol_current_rpm。速度环用 VESC 25000 ERPM/s；开环用 olramp
 * @param {MOTOR_PARAM_S} *motor                  斜坡状态
 * @param {float} dt                              控制周期 (s)
 * @param {bool} sensorless_takeover_complete     true=允许冲向最终目标
 * @return {void} 无
 * @author: lkc
 */
void Motor_Control_UpdateSpeed(MOTOR_PARAM_S *motor, float dt, bool sensorless_takeover_complete);

/**
 * @description: 用 I/F 启动模块的 angle_rpm 积分电角度
 * @param {motor_control_t} *ctrl  控制实例
 * @param {MOTOR_PARAM_S} *motor   写 ol_theta
 * @param {float} dt               控制周期 (s)
 * @return {void} 无
 * @author: lkc
 */
void Motor_Control_UpdateTheta(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 推进一拍开环/I/F/V/F/霍尔标定编排，返回输出请求
 * @detail: 不调用 HAL；未就绪时复位并请求 SAFE。欠压、过流等保护不在这里判定
 * @param {motor_control_t} *ctrl   控制实例
 * @param {MOTOR_PARAM_S} *motor    电机对象
 * @param {float} v_bus             母线电压 (V)
 * @param {float} dt                FOC 控制周期 (s)
 * @param {bool} hall_detect_active 霍尔标定是否占用功率桥
 * @return {motor_control_result_t} 本拍输出仲裁
 * @author: lkc
 */
motor_control_result_t Motor_Control_Step(motor_control_t *ctrl, MOTOR_PARAM_S *motor, float v_bus, float dt,
                                          bool hall_detect_active);

/**
 * @description: 查询是否处于预对齐
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {bool} true=对齐进行中
 * @author: lkc
 */
bool Motor_Control_IsAligning(const motor_control_t *ctrl);

/**
 * @description: 只读查询I/F拖转电流交接是否完成；不要求跳过对齐时未使用的定角软起比例。空对象返回false。
 * @param {const motor_control_t} *ctrl 控制实例
 * @return {bool} true=拖转电流交接已完成
 */
bool Motor_Control_IsIfDragSettled(const motor_control_t *ctrl);

/**
 * @description: 本次对齐已进行时间
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {float} 时长 (s)
 * @author: lkc
 */
float Motor_Control_GetAlignElapsed(const motor_control_t *ctrl);

/**
 * @description: HFI 是否在跑
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {bool} true=扫描中
 * @author: lkc
 */
bool Motor_Control_HfiIsRunning(const motor_control_t *ctrl);

/**
 * @description: HFI 是否已成功
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {bool} true=识别成功
 * @author: lkc
 */
bool Motor_Control_HfiSucceeded(const motor_control_t *ctrl);

/**
 * @description: HFI 识别电角度
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {float} 电角度 (rad)
 * @author: lkc
 */
float Motor_Control_HfiGetAngle(const motor_control_t *ctrl);

/**
 * @description: HFI 可信度
 * @param {const motor_control_t} *ctrl  控制实例
 * @return {float} 可信度 [0,1]
 * @author: lkc
 */
float Motor_Control_HfiGetConfidence(const motor_control_t *ctrl);

/**
 * @description: 故障仲裁：已锁存故障时强制 SAFE，禁止 DQ 覆盖
 * @detail: SAFE 为中点占空比请求，不等于关桥
 * @param {bool} fault_latched                     保护是否已锁存
 * @param {const motor_control_result_t} *proposed Control_Step 提议的输出
 * @return {motor_control_result_t} 最终允许应用的输出
 * @author: lkc
 */
motor_control_result_t Motor_Control_ArbitrateOutput(bool fault_latched, const motor_control_result_t *proposed);
/* Private functions ---------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
