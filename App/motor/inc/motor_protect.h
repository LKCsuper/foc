#ifndef _MOTOR_PROTECT_H
#define _MOTOR_PROTECT_H

#ifdef __cplusplus
extern "C" {
#endif

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
 * 初始化保护模块。
 * @param config 自定义保护参数；传 NULL 表示继续使用源文件中的默认参数。
 * @note  调用后会清除故障码、锁存标志、所有计时器和速度反馈状态。
 */
void Motor_Protect_Init(const motor_protect_config_t *config);

/*
 * 更新真实机械转速反馈。
 * @param rpm   霍尔、编码器或观测器计算出的机械转速，单位 RPM，可为负数。
 * @param valid true 表示 rpm 有效；false 表示暂时没有可靠速度反馈。
 * @note  valid=false 时堵转和失速保护被禁用，过流和欠压保护不受影响。
 */
void Motor_Protect_SetSpeedFeedback(float rpm, bool valid);

/*
 * 以下四个函数只负责一种故障的判断，不负责停 PWM，也不直接锁存故障码。
 * @param motor 电机实时数据，包含三相电流、母线电压和目标转速。
 * @param dt    本次与上次保护检查之间的时间，单位秒。
 * @return true：该故障条件已经连续保持足够长时间；false：尚未触发。
 */
bool Motor_Protect_CheckOverCurrent(const MOTOR_PARAM_S *motor, float dt);
bool Motor_Protect_CheckUnderVoltage(const MOTOR_PARAM_S *motor, float dt);
bool Motor_Protect_CheckStall(const MOTOR_PARAM_S *motor, float dt);
bool Motor_Protect_CheckSpeedLoss(const MOTOR_PARAM_S *motor, float dt);

/*
 * 保护统一入口，由 FOC 控制周期调用。
 * 函数依次检查四种故障，并把第一次发生的故障锁存下来。
 * @return true：已有锁存故障，上层必须停止本周期 FOC 输出；false：可以运行。
 */
bool Motor_Protect_Run(const MOTOR_PARAM_S *motor, float dt);

/* 人工解除故障锁存；调用前应先撤销运行指令并确认真实故障已经排除。 */
void Motor_Protect_ClearFault(void);

/* 查询是否存在锁存故障。 */
bool Motor_Protect_IsFaultLatched(void);

/* 获取锁存的首发故障码；没有故障时返回 FAULT_CODE_NONE。 */
mc_fault_code Motor_Protect_GetFault(void);

#ifdef __cplusplus
}
#endif

#endif
