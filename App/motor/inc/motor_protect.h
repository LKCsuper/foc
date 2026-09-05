/*
 * @Description: 电机过流、欠压、堵转与失速保护接口
 * @Author: lkc
 */
#ifndef _MOTOR_PROTECT_H
#define _MOTOR_PROTECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "motor_types.h"
#include "motor_param.h"
/*
 * 电机保护参数表。
 *
 * 每种保护都由“阈值 + 持续时间”组成：
 *   1. 检测值超过阈值时开始计时；
 *   2. 条件持续时间达到 xxx_time 后才确认故障；
 *   3. 中途恢复正常则计时清零。
 *
 * 这样可以过滤 ADC 毛刺和转速测量抖动，避免单个异常采样立即停机。
 */
typedef struct {
    float over_current;       /* 三相电流绝对值峰值上限，单位 A */
    float under_voltage;      /* 运行期间允许的最低母线电压，单位 V */
    float stall_target_rpm;   /* 启用堵转/失速判断的最低目标转速，单位 RPM */
    float stall_speed_rpm;    /* 低于该真实转速时认为转子接近静止，单位 RPM */
    float stall_current;      /* 堵转判断要求的最低负载电流，单位 A */
    float speed_loss_ratio;   /* 失速阈值：真实转速/目标转速 */
    float over_current_time;  /* 过流连续保持时间，单位 s */
    float under_voltage_time; /* 欠压连续保持时间，单位 s */
    float stall_time;         /* 堵转连续保持时间，单位 s */
    float speed_loss_time;    /* 失速连续保持时间，单位 s */
} motor_protect_config_t;

/*
 * 首发故障发生瞬间的数据快照。
 *
 * 电机停机以后，实时电流和母线电压会立即变化，只看 olstatus 往往已经
 * 看不到真正的触发条件。因此在第一次锁存故障时保存以下数据，直到执行
 * olfault clear 才清除。
 */
typedef struct {
    float current_peak; /* 故障瞬间 max(|Ia|,|Ib|,|Ic|)，单位 A */
    float bus_voltage;  /* 故障瞬间母线电压，单位 V */
    float target_rpm;   /* 用户下发的目标机械转速，单位 RPM */
    float ramp_rpm;     /* I/F 软件斜坡当时走到的转速，单位 RPM，不是真实反馈 */
    float measured_rpm; /* 有效传感器/观测器反馈，单位 RPM */
    float observer_rpm; /* 无论是否接管，都保存 PLL 当时估算的机械 RPM */
    float flux;         /* 观测器估算磁链，单位 Wb */
    float phase_error;  /* 观测角与 I/F 强制角的误差，单位 rad */
    float id;           /* 故障瞬间 D 轴反馈电流，单位 A */
    float iq;           /* 故障瞬间 Q 轴反馈电流，单位 A */
    float iq_command;   /* 故障瞬间使用的 Q 轴目标电流，单位 A */
    float vd_integrator;/* D 轴 PI 积分电压，单位 V */
    float vq_integrator;/* Q 轴 PI 积分电压，单位 V */
    float applied_voltage; /* 实际 α/β 输出电压矢量模长，单位 V */
    float lock_timer;   /* 无感四项条件连续满足时间，单位 s */
    float blend;        /* 无感接管比例，0=I/F，1=观测角 */
    int sensorless_state; /* motor_sensorless_state_t，避免停机复位后丢失 */
    bool speed_valid;   /* measured_rpm 在故障瞬间是否可信 */
    bool sensorless_takeover; /* 故障时是否已授权无感接管 */
    bool speed_loop_enabled;  /* 故障时速度 PI 是否已开启 */
    bool flux_ok;       /* 无感磁链接管条件 */
    bool observer_speed_ok; /* 无感转速接管条件 */
    bool direction_ok;  /* 无感方向接管条件 */
    bool phase_ok;      /* 无感相位接管条件 */
} motor_protect_snapshot_t;

/**
 * @description: 初始化保护模块。
 * @param {const motor_protect_config_t} *config  自定义保护参数；传 NULL 使用默认参数
 * @detail: 调用后清除故障码、锁存标志、所有计时器和速度反馈状态。
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_Init(const motor_protect_config_t *config);

/**
 * @description: 更新真实机械转速反馈。
 * @param {float} rpm    霍尔、编码器或观测器计算出的机械转速 (RPM)，可为负数
 * @param {bool} valid   true 表示 rpm 有效；false 表示暂时没有可靠速度反馈
 * @detail: valid=false 时堵转和失速保护被禁用，过流和欠压保护不受影响。
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid);

/**
 * @description: 检查过流故障条件是否已连续保持足够长时间。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据，含三相电流与母线电压
 * @param {float} dt                      本次与上次保护检查之间的时间 (s)
 * @return {bool} true=过流条件已确认；false=尚未触发
 * @author: lkc
 */
bool Motor_Protect_CheckOverCurrent(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 检查欠压故障条件是否已连续保持足够长时间。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                      保护检查周期 (s)
 * @return {bool} true=欠压条件已确认；false=尚未触发
 * @author: lkc
 */
bool Motor_Protect_CheckUnderVoltage(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 检查堵转故障条件是否已连续保持足够长时间。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                      保护检查周期 (s)
 * @return {bool} true=堵转条件已确认；false=尚未触发
 * @author: lkc
 */
bool Motor_Protect_CheckStall(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 检查失速故障条件是否已连续保持足够长时间。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                      保护检查周期 (s)
 * @return {bool} true=失速条件已确认；false=尚未触发
 * @author: lkc
 */
bool Motor_Protect_CheckSpeedLoss(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 保护统一入口，由 FOC 控制周期调用。
 * @detail: 依次检查四种故障，并把第一次发生的故障锁存下来。
 * @param {const MOTOR_PARAM_S} *motor  电机实时数据
 * @param {float} dt                      保护检查周期 (s)
 * @return {bool} true=已有锁存故障，上层必须停止本周期 FOC 输出；false=可以运行
 * @author: lkc
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt);

/**
 * @description: 由无感观测器等安全模块锁存一个已经确认的故障。
 * @detail: 只记录首发故障及快照；调用者或下一次 Motor_Protect_Run() 负责停 PWM。
 * @param {mc_fault_code} fault              要锁存的故障码；FAULT_CODE_NONE 时忽略
 * @param {const MOTOR_PARAM_S} *motor  用于生成故障快照；为空时忽略
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_TriggerFault(mc_fault_code fault, const MOTOR_PARAM_S *motor);

/**
 * @description: 人工解除故障锁存。
 * @detail: 调用前应先撤销运行指令并确认真实故障已经排除。
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_ClearFault(void);

/**
 * @description: 查询是否存在锁存故障。
 * @return {bool} true=已锁存故障
 * @author: lkc
 */
bool Motor_Protect_IsFaultLatched(void);

/**
 * @description: 获取锁存的首发故障码。
 * @return {mc_fault_code} 故障码；无故障时返回 FAULT_CODE_NONE
 * @author: lkc
 */
mc_fault_code Motor_Protect_GetFault(void);

/**
 * @description: 把故障码转换成可读名称，便于 Shell 直接显示原因。
 * @param {mc_fault_code} fault  待转换的故障码
 * @return {const char *} 故障名称字符串
 * @author: lkc
 */
const char *Motor_Protect_GetFaultName(mc_fault_code fault);

/**
 * @description: 获取首发故障发生瞬间的数据快照。
 * @param {motor_protect_snapshot_t} *snapshot  接收快照数据的结构体指针；为 NULL 时不执行任何操作
 * @return {*}
 * @author: lkc
 */
void Motor_Protect_GetSnapshot(motor_protect_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
