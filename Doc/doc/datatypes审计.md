# datatypes.h / VESC 遗留类型审计

> 阶段 1 交付物 · 日期：2026-09-03  
> 目的：摸清 `datatypes.h` 与 VESC 配置体系在**当前主路径**中的实际用量，为参数体系统一提供依据。

---

## 1. 审计结论（摘要）

| 项 | 结论 |
|----|------|
| `datatypes.h` 行数 | **1464 行**（含大量 CAN/BMS/平衡车等未用类型） |
| 主控制路径运行时状态 | **`MOTOR_PARAM_S motor`**（`motor_param.c`），ISR 只用此结构 |
| VESC 运行时状态 | **`motor_all_state_t`** + **`mc_configuration`**，仅少数遗留路径引用 |
| 能否立刻删除 `datatypes.h` | **否** — 仍有活跃依赖，需分步迁移 |
| 发现的重要问题 | **霍尔标定 `Motor_Hall_Detect` 写 `motor_all_state_t`，主 ISR 读 `MOTOR_PARAM_S`，两路未打通** |

---

## 2. 两套参数体系对照

```
┌─────────────────────────────────────────────────────────────┐
│  MOTOR_PARAM_S  (motor)          ← 主路径 ★                  │
│  App/motor_param/inc/motor_param.h                          │
│  使用者：motor_it.c, motor_protect, motor_sensorless,       │
│           cmd_openloop, flash_storage                       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  motor_all_state_t (m_motor_1) + mc_configuration (mcconf1) │
│  定义嵌在 motor_param.h，类型来自 datatypes.h               │
│  使用者：motor_hall.c, motor_detect.c(DETECT_Hall),         │
│           motor_param.c(初始化), motor_task.c(注释/死代码)    │
└─────────────────────────────────────────────────────────────┘
```

**新人应只关注 `MOTOR_PARAM_S`。** VESC 体系是历史包袱，阶段 1 目标是收敛到前者。

---

## 3. `datatypes.h` 中「仍在使用」的类型

### 3.1 高频 / 必须保留（迁移前）

| 类型 | 用途 | 引用位置 |
|------|------|----------|
| `mc_fault_code` | 保护故障码 | `motor_protect.c/h`, `motor_detect.h` |
| `mc_configuration` | 电机配置大结构 | `motor_param.h`, `motor_param.c`, `motor_cal.c`, `motor_hall.c`, `motor_detect.c` |
| `motor_all_state_t` | VESC 电机运行时（定义在 `motor_param.h`） | `motor_hall.c`, `motor_detect.c`, `motor_param.c` |
| `mc_state` | `MC_STATE_OFF/RUNNING` 等 | `motor_hall.c` |
| `mc_control_mode` | `CONTROL_MODE_CURRENT/NONE` 等 | `motor_hall.c` |

### 3.2 低频 / 可改签名后删除对 mc_configuration 的依赖

| 符号 | 用途 | 引用位置 | 迁移建议 |
|------|------|----------|----------|
| `foc_pll_run(...)` | PLL 算法 | 已删除 | 无调用者；各观测器保留自身已验证的 PLL 状态更新 |
| `VESC_Calc_Foc_cc_kp_ki_gain(mc_configuration*)` | 根据 R/L 算 PI | `motor_cal.c` | 改为写 `MOTOR_PARAM_S` 的 `ol_current_kp/ki` |
| `foc_hall_table[8]` | 霍尔表 | `mc_configuration` 字段；`DETECT_Hall` 写入 | 迁到 `MOTOR_PARAM_S` 或独立 `motor_hall_config_t` |
| `m_hall_extra_samples` | 霍尔滤波次数 | `motor_hall.c` 读 `motor->m_conf->` | 迁到 `MOTOR_PARAM_S` 或宏 |

### 3.3 基本未用（可归档）

以下在 `App/` 内 **无有效引用**（仅定义或注释代码）：

- `HW_TYPE`, `mc_pwm_mode`, `mc_comm_mode`, `out_aux_mode`, `temp_sensor_type`
- `gpd_output_mode`, 大部分 `app_configuration` 相关
- CAN / BMS / IMU / 平衡车 / 踏频计等 VESC 扩展类型
- `motor_task.c` 里 `foc_run_pid_control_*` 路径（双电机 #2，且与当前 ISR 无关）

**建议：** 整体移动到 `Doc/VESC/archive/datatypes_full.h`，不参与编译；新建精简 `motor_types.h`（见 §5）。

---

## 4. 按文件依赖清单

| 文件 | 依赖 VESC 类型 | 主路径？ | 阶段 1 动作 |
|------|----------------|----------|-------------|
| `includes.h` | `#include datatypes.h`, `mcconf_default.h` | 是 | 最终改为 `motor_types.h` + 按需包含 |
| `motor_param.h` | 内含 `motor_all_state_t` 定义 | 是 | 将 VESC 结构移到 `motor_vesc_legacy.h` 或删除 |
| `motor_param.c` | `m_motor_1/2`, `mcconf1/2`, `motorCfg` | 部分 | 保留初始化至迁移完成；逐步删 |
| `motor_it.c` | 无（仅用 `MOTOR_PARAM_S motor`） | **是 ★** | 不动 |
| `motor_protect.c` | `mc_fault_code` | **是 ★** | 故障码迁入 `motor_types.h` |
| `motor_sensorless.c` | 无 VESC 全局 | **是 ★** | 不动 |
| `motor_hall.c` | `motor_all_state_t`, `mc_configuration`, `MC_STATE_*`, `CONTROL_MODE_*` | 标定路径 | **重写**：改用 `MOTOR_PARAM_S` + 现有开环/I-F API |
| `motor_detect.c` | `DETECT_Hall` 写 `m_conf->foc_hall_table` | 标定路径 | 霍尔表写入改到 `motor` 或 Flash 字段 |
| `motor_cal.c` | `mc_configuration` | 低 | 改签名或标记 deprecated |
| `motor_foc.c` | `foc_pll_run` 参数 | 低（未调用） | 改签名或 `#if 0` |
| `motor_task.c` | `motor_all_state_t` | **否（死代码）** | 删除或移归档 |
| `mcconf_default.h` | 填充 `mc_configuration` 默认值 | 间接 | 需要的默认值并入 `MOTOR_Param_Init` |
| `flash_storage.c` | `MOTOR_PARAM_S` + `motor_detect_result_t` | **是 ★** | 已用新体系，保持 |

---

## 5. 目标：`motor_types.h` 精简版（建议 < 150 行）

阶段 1 结束时新建 `App/motor/inc/motor_types.h`，仅保留：

```c
/* 故障码 — 从 datatypes.h 提取 mc_fault_code */
typedef enum { FAULT_CODE_NONE = 0, ... } mc_fault_code;

/* 霍尔表长度 */
#define MOTOR_HALL_TABLE_SIZE  (8U)

/* 可选：霍尔配置子结构，将来并入 MOTOR_PARAM_S */
typedef struct {
    int8_t  hall_table[MOTOR_HALL_TABLE_SIZE];  /* VESC 风格 0~200 或 -1 无效 */
    uint8_t hall_extra_samples;
} motor_hall_config_t;
```

**不再包含：** `mc_configuration`、`motor_all_state_t`、CAN/BMS 等。

---

## 6. 已知架构问题：霍尔标定与 ISR 脱节

### 6.1 现象

`Motor_Hall_Detect()`（`motor_hall.c`）通过 **`gpstMotor1`（`motor_all_state_t`）** 设置：

- `m_phase_override`, `m_id_set`, `m_iq_set`, `m_control_mode`, `m_state`

而 **`Motor_DmaInt()`（`motor_it.c`）** 只驱动 **`MOTOR_PARAM_S motor`**，**不读取** `motor_all_state_t` 任何字段。

### 6.2 影响

- `detect hall` 命令可能**无法按预期**在开环/电流闭环下转动电机（除非另有未审计的桥接代码）。
- 标定结果写入 `motor->m_conf->foc_hall_table`，与 `motor`（`MOTOR_PARAM_S`）和 Flash 主路径的关系不明确。

### 6.3 阶段 1 修复建议（优先）

1. 重写 `Motor_Hall_Detect`：用 `motor.ol_*` / `Motor_OpenLoop_*` 或专用检测例程驱动 ISR 认得的参数。
2. 霍尔表保存到 `MOTOR_PARAM_S` 新字段（如 `hall_table[8]`）并在 `MOTOR_Param_Init` / `saveparam` 中持久化。
3. 删除标定路径对 `motor_all_state_t` 的依赖。

---

## 7. 阶段 1 任务拆分（可执行）

| 序号 | 任务 | 预估 | 依赖 |
|------|------|------|------|
| 1.1 | 新建 `motor_types.h`，迁入 `mc_fault_code` | 0.5d | — |
| 1.2 | `motor_protect.h` 改 include `motor_types.h` | 0.5d | 1.1 |
| 1.3 | 复制 `datatypes.h` → `Doc/VESC/archive/datatypes_full.h`（仅归档，不删原文件） | 0.5d | — |
| 1.4 | 重写 `Motor_Hall_Detect` 对接 `MOTOR_PARAM_S` | 2d | 1.1 |
| 1.5 | `DETECT_Hall` 改写霍尔表到 `motor` + Flash | 0.5d | 1.4 |
| 1.6 | `foc_pll_run` 改 `(float kp, float ki)` 或删除 | 0.5d | — |
| 1.7 | `VESC_Calc_Foc_cc_kp_ki_gain` 改写 `MOTOR_PARAM_S` | 0.5d | — |
| 1.8 | 从 `motor_param.h` 移除 `motor_all_state_t` 定义 | 1d | 1.4, 1.5 |
| 1.9 | `includes.h`：`datatypes.h` → `motor_types.h` | 0.5d | 1.1~1.8 |
| 1.10 | 删除/归档 `motor_task.c` 死代码 | 0.5d | — |
| 1.11 | 跑通 [框架重构计划](./框架重构计划.md) 阶段 0 验收清单 | 1d | 全部 |

**合计约 1~1.5 周**（与重构计划一致）。

---

## 8. 迁移时禁止事项

- 不要在一次 PR 里同时改 HAL（阶段 2）和参数体系。
- 不要修改 `Library/` 第三方与 BSP 行为（除非霍尔标定必须动 PWM 接口）。
- 不要恢复完整 VESC `mc_configuration` 字段到 Flash（Flash 布局以 `MOTOR_PARAM_S` 为准）。

---

## 9. grep 命令（便于复审计）

```bash
# VESC 类型引用
rg "mc_configuration|motor_all_state_t|datatypes\.h|mcconf_default" App/

# 主路径确认
rg "MOTOR_PARAM_S|\\bmotor\\." App/motor/motor_it.c App/cmd/cmd_openloop.c

# 故障码
rg "mc_fault_code|FAULT_CODE_" App/

# 死代码
rg "gpstMotor1|m_motor_1" App/
```

---

## 10. 阶段 1 进度（2026-09-04）

| 任务 | 状态 |
|------|------|
| 1.1 新建 `motor_types.h`，迁入 `mc_fault_code` | ✅ 已完成 |
| 1.2 `motor_protect.h` / `motor_param.h` / `motor_detect.h` include `motor_types.h` | ✅ 已完成 |
| 1.3 归档 `datatypes_full.h` / `mcconf_default.h` | ✅ 已完成 → `Doc/VESC/archive/` |
| 1.4 重写 `Motor_Hall_Detect` 对接 `MOTOR_PARAM_S` | ✅ 已完成 |
| 1.5 `DETECT_Hall` 改写霍尔表到 `motor` | ✅ 已完成 |
| 1.6 `foc_pll_run` 改 `(pll_kp, pll_ki)` | ✅ 已完成 |
| 1.7 `Motor_Calc_FocCurrentKpKi` 写 `MOTOR_PARAM_S` | ✅ 已完成 |
| 1.8 移除 `motor_all_state_t` / `mc_configuration` 全局 | ✅ 已完成 |
| 1.9 `includes.h` 去掉 `datatypes.h` / `mcconf_default.h` | ✅ 已完成 |
| 1.10 清理 `motor_task.c` 死代码 | ✅ 已完成 |
| 1.11 上板验收清单 | ⏳ 待测 |

**注意：** `MOTOR_PARAM_S` 末尾新增 `hall_table` / `hall_extra_samples`。项目未发布，Flash 布局直接以当前结构为准，不升格式版本、不兼容旧槽。开发板上旧数据会因 CRC 不匹配被当作无效槽，上电走默认参数。

---

## 11. 相关文档

- [代码阅读指南](./代码阅读指南.md) — 主路径阅读顺序
- [框架重构计划](./框架重构计划.md) — 阶段 1 总览

---

## 附录：`mc_configuration` 中主路径可能用到的字段（待迁入 MOTOR_PARAM_S）

| 字段 | 当前读/写位置 | 建议迁入 |
|------|---------------|----------|
| `foc_hall_table[8]` | `DETECT_Hall`, `Motor_Hall_Detect` | `motor.hall_table[]` |
| `m_hall_extra_samples` | `Motor_Hall_Detect` | `motor.hall_extra_samples` |
| `foc_motor_r/l/flux_linkage` | `motor_param.c` 初始化 `gpstMotorCfg` | 已在 `MOTOR_PARAM_S` 有 `sensorless_*` 同类字段，需对齐命名 |
| `foc_pll_kp/ki` | 仅 `foc_pll_run`（未调用） | `motor.sensorless_pll_kp/ki` 或删除 |
| `foc_current_kp/ki` | `motor_cal.c` | `motor.ol_current_kp/ki` |

其余 `mc_configuration` 字段在当前开环/I-F/无感主路径中 **未发现引用**。
