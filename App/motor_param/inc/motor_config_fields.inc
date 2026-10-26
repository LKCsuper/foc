/*
 * @Description: motor_config_t 字段总表（X-macro），配置结构体、校验、停机判断与 Flash 编解码都从这里展开。
 * @Author: lkc
 * @detail: 用法：包含前定义 MOTOR_CONFIG_FIELD(name, kind, nonneg, stop, word_max)，包含后 #undef。
 *   name     字段名，须与 MOTOR_PARAM_S 中同名字段一一对应
 *   kind     FLOAT / BOOL / U16 / INT：决定 C 类型与线格式（FLOAT=IEEE754 binary32，其余按 32 位小端字）
 *   nonneg   1=必须 >= 0（仅 FLOAT 生效；FLOAT 一律要求有限值）
 *   stop     1=运行中禁止修改（Motor_Settings_ConfigRequiresStop）
 *   word_max 非 FLOAT 字段解码时允许的最大字值；FLOAT 填 0U
 *   跨字段约束（极对数整数、min<=max 等）仍写在 motor_settings.c 的 settings_valid 中。
 *
 * 警告：本表的行顺序就是 Flash v2 线格式顺序（MOTOR_SETTINGS_WIRE_SIZE）。增删、改序、改 kind 都会改变
 *       存储格式，必须同时提升 Flash 格式版本并更新 test_motor_settings / test_flash_storage。
 * 本文件故意没有 include guard。
 */
MOTOR_CONFIG_FIELD(ol_max_rpm, FLOAT, 0, 0, 0U) /* max target speed RPM */
MOTOR_CONFIG_FIELD(ol_v_boost, FLOAT, 1, 0, 0U) /* V/F 启动补偿 V */
MOTOR_CONFIG_FIELD(ol_kv_f, FLOAT, 1, 0, 0U) /* V/F 斜率 V/Hz */
MOTOR_CONFIG_FIELD(ol_ramp_rpm_s, FLOAT, 1, 0, 0U) /* 软启动速率 RPM/s */
MOTOR_CONFIG_FIELD(ol_vq_limit, FLOAT, 1, 0, 0U) /* Vq 上限 V */
MOTOR_CONFIG_FIELD(ol_foc_freq, FLOAT, 0, 1, 0U) /* ADC-DMA 基准频率 Hz */
MOTOR_CONFIG_FIELD(ol_pole_pairs, FLOAT, 0, 1, 0U) /* 电机极对数 */
MOTOR_CONFIG_FIELD(ol_min_rpm, FLOAT, 1, 0, 0U) /* 旧启动策略兼容参数；平滑启动不作转速地板 */
MOTOR_CONFIG_FIELD(ol_current_kp, FLOAT, 1, 0, 0U) /* 开环电流环 Kp */
MOTOR_CONFIG_FIELD(ol_current_ki, FLOAT, 1, 0, 0U) /* 开环电流环 Ki */
MOTOR_CONFIG_FIELD(ol_start_phase_boost, FLOAT, 1, 0, 0U) /* 兼容旧启动方式的一拍相位补偿 rad */
MOTOR_CONFIG_FIELD(ol_align_enable, BOOL, 0, 1, 1U) /* true=每次 I/F 启动前执行预对齐 */
MOTOR_CONFIG_FIELD(ol_align_current, FLOAT, 1, 1, 0U) /* 对齐 D 轴电流 Id (A) */
MOTOR_CONFIG_FIELD(ol_align_time, FLOAT, 1, 1, 0U) /* 固定磁场保持时间 (s) */
MOTOR_CONFIG_FIELD(ol_align_angle, FLOAT, 0, 1, 0U) /* 固定对齐电角度 (rad) */
MOTOR_CONFIG_FIELD(hfi_start_enable, BOOL, 0, 1, 1U) /* true=I/F 启动前先扫描转子凸极轴 */
MOTOR_CONFIG_FIELD(hfi_start_voltage, FLOAT, 1, 1, 0U) /* 双极性注入电压幅值 V */
MOTOR_CONFIG_FIELD(hfi_start_cycles, U16, 0, 1, 65535U)
MOTOR_CONFIG_FIELD(hfi_start_min_confidence, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(speed_enable, BOOL, 0, 0, 1U)
MOTOR_CONFIG_FIELD(speed_allow_braking, BOOL, 0, 0, 1U)
MOTOR_CONFIG_FIELD(speed_kp, FLOAT, 1, 0, 0U)
MOTOR_CONFIG_FIELD(speed_ki, FLOAT, 1, 0, 0U)
MOTOR_CONFIG_FIELD(speed_iq_limit, FLOAT, 1, 0, 0U)
MOTOR_CONFIG_FIELD(speed_loop_hz, FLOAT, 0, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_enable, BOOL, 0, 0, 1U)
MOTOR_CONFIG_FIELD(sensorless_takeover, BOOL, 0, 0, 1U)
MOTOR_CONFIG_FIELD(sensorless_motor_r, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_motor_l, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_flux_linkage, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_observer_gain, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_pll_kp, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_pll_ki, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_transition_rpm, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_fallback_rpm, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_openloop_max_rpm, FLOAT, 1, 0, 0U)
MOTOR_CONFIG_FIELD(sensorless_lock_time, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_loss_time, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_blend_time, FLOAT, 1, 0, 0U)
MOTOR_CONFIG_FIELD(sensorless_phase_tolerance, FLOAT, 1, 0, 0U)
MOTOR_CONFIG_FIELD(sensorless_phase_offset, FLOAT, 0, 1, 0U)
MOTOR_CONFIG_FIELD(sensorless_speed_filter, FLOAT, 1, 1, 0U)
MOTOR_CONFIG_FIELD(hall_extra_samples, INT, 0, 1, 32U) /* 霍尔 GPIO 额外滤波次数 */
