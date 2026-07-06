# 开环 FOC（V/F 电压模式）学习笔记

## 1. 什么是开环 FOC？

FOC（Field-Oriented Control，磁场定向控制）的核心思想：**把三相交流电机的控制，转化为类似直流电机的控制**。

```
┌──────────────────────────────────────────────────────────────┐
│                      闭环 FOC（正常）                         │
│                                                              │
│  电流传感器 ──→ Clarke ──→ Park ──→ PI控制 ──→ 逆Park ──→ SVPWM │
│                  ↑                  ↑           │            │
│                  │                  │           ↓            │
│              三相电流         角度(编码器)    三相PWM          │
│                                                              │
│  关键：角度来自真实传感器，PI 自动调节电压                      │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│                      开环 FOC（当前）                         │
│                                                              │
│  虚拟角度 ──→ 逆Park ──→ SVPWM ──→ 三相PWM                    │
│     ↑                        ↑                              │
│     │                        │                              │
│  手动递增              固定Vd=0, Vq=V/F曲线自动                    │
│                                                              │
│  关键：角度是"假的"，电压是算出来的，没有反馈                      │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. 两个时代

### 2.1 旧方案（已废弃）：THETA_STEP + OPEN_LOOP_VQ

```c
#define OPEN_LOOP_VQ   5.0f       /* Vq 电压 */
#define THETA_STEP     0.001f     /* 每步角度增量 */
```

**调参体验**：改一次 → 编译 → 烧录 → 不行 → 循环。两个参数互相耦合，完全凭感觉。

### 2.2 新方案（当前）：V/F 曲线控制

**核心思想**：用户只设一个参数 `ol_target_rpm`（目标转速），其余全部自动计算。

```
用户设置: ol_target_rpm
         │
         ├──→ 软启动斜坡 → ol_current_rpm
         │         │
         │         ├──→ 角度递增 (theta_step)
         │         │
         │         └──→ V/F 曲线 → Vq = V_boost + Kv_f × f_electrical
         │
         ↓
   inverse_park(0, Vq, theta) → SVPWM → 电机转
```

**V/F 曲线公式**：

```
f_electrical = ol_current_rpm / 60 × pole_pairs    [Hz]
Vq = ol_v_boost + ol_kv_f × f_electrical           [V]
```

---

## 3. 参数速查

所有参数都在 `MOTOR_PARAM_S` 结构体中，改 [motor_param.c](../App/Motor_Param/motor_param.c) 即可生效。

### 核心参数（需要调的）

| 参数 | 默认值 | 作用 | 调法 |
|------|--------|------|------|
| `ol_target_rpm` | 80 | **目标转速** — 唯一需要常调的参数 | 先 80 转稳，再逐步加 |
| `ol_v_boost` | 4V | **启动补偿电压** — 克服静摩擦 + 绕组压降 | 不转→加大，太热→减小 |
| `ol_kv_f` | 0.05 | **V/F 斜率** — 高速时电压匹配反电动势 | 高速无力→加大 |
| `ol_ramp_rpm_s` | 300 | **软启动斜坡** — 转速变化速率 | 启动抖动→减小 |

### 系统参数（设好就不用动）

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `ol_vq_limit` | 18V | Vq 上限，防止过调制/过流 |
| `ol_foc_freq` | 30000 | FOC 控制频率 Hz（PWM频率/分频） |
| `ol_pole_pairs` | 4 | 电机极对数（8极=4对极） |
| `ol_min_rpm` | 60 | 最低转速地板，防止磁场转太慢推不动转子 |
| `ol_theta` | (内部) | 开环电角度，中断中自增 |
| `ol_current_rpm` | (内部) | 当前斜坡转速，只读 |

### V/F 曲线计算示例（4对极，24V 母线）

| 转速 RPM | 电频率 Hz | Vq | 调制深度 |
|----------|-----------|-----|----------|
| 60 (min) | 4 | 4.2V | 18% |
| 80 | 5.3 | 4.3V | 18% |
| 200 | 13.3 | 4.7V | 19% |
| 500 | 33.3 | 5.7V | 24% |
| 1000 | 66.7 | 7.3V | 31% |
| 2000 | 133 | 10.7V | 44% |
| 4000 | 267 | 17.3V | 72% |

---

## 4. 调参实战经验

### 4.1 调试流程

```
1. 设 ol_target_rpm = 80，ol_v_boost = 4.0，烧录
2. 电机完全不动 → ol_v_boost += 1V，直到能转（本电机 2V 不转、4V 能转）
3. 转一下就抖停 → ol_ramp_rpm_s 减半，目标转速也减半
4. 一直抖动不转 → ol_min_rpm 太低或太高，调回接近旧代码的等效值
5. 平稳转了 → 提 ol_target_rpm 加速，热了就提速（反电动势上来电流自然降）
6. 高速无力 → ol_kv_f += 0.01
```

### 4.2 故障诊断表

| 现象 | 根因 | 优先调 | 其次 |
|------|------|--------|------|
| 完全不动 | Vq 太小，克服不了静摩擦 | `ol_v_boost` ↑ | `ol_target_rpm` ↑ |
| 转一下→抖动→停 | 斜坡太陡，转子跟不上 | `ol_ramp_rpm_s` ↓↓ | `ol_target_rpm` ↓ |
| 一直抖不转 | 磁场转速与 Vq 不匹配 | `ol_min_rpm` 对齐旧等效值 | `ol_v_boost` ↑ |
| 能转但发热 | Vq 过大，电流超出需要 | 提速让反电动势抵消 | `ol_v_boost` ↓ |
| 高速转不起来 | V/F 斜率不够，反电动势抵消后净电压不足 | `ol_kv_f` ↑ | `ol_vq_limit` 检查 |
| 启动过流重启 | Vq 太大触发保护 | `ol_vq_limit` ↓ | `ol_v_boost` ↓ |

### 4.3 核心教训

**教训 1：ramp 不能从 0 起步**
- `ol_current_rpm` 初始为 0，磁场转极慢（<1 RPM），Vq = V_boost
- 等于给电机通"几乎静止的交流电"，推不动转子，原地抖
- 解决：加 `ol_min_rpm` 地板，首次运行直接跳到最低转速

**教训 2：最低转速要对齐旧代码**
- 旧代码 `THETA_STEP = 0.001` → 等效 ~72 RPM（4对极@30kHz）
- 一开始设 `MIN_OL_RPM = 200`，是原来的 3 倍，转子跟不上
- 降到 60 RPM 即可正常转
- 换算公式：`等效RPM = theta_step × FOC_freq / (2π) / pole_pairs × 60`

**教训 3：V_boost 要找最低有效值**
- 太小：推不动转子
- 太大：低速时电流过大，电机发热
- 二分法找：能转的最小值 + 1V 就是最佳值

**教训 4：低速大电压 = 发热**
- V/F 曲线在低速时 Vq ≈ V_boost，后面跟不上反电动势
- 开环没有电流环，多余电压全变热量
- 不要长时间在极低速运行，提转速后反电动势自然匹配

**教训 5：Flash 持久化陷阱**
- `MOTOR_PARAM_S` 整个存 Flash，加了新字段后第一次启动触发"大小不一致"→写默认值
- 之后改 `motor_param.c` 不再生效（Flash 已有旧默认值）
- 开发阶段：在 flash 加载后强制覆盖 V/F 参数（见 motor_param.c 末尾覆盖块）
- 生产阶段：去掉覆盖块，用 shell 命令调参后写 Flash

---

## 5. Shell 命令

```
olspeed 500       ← 设目标转速 500 RPM
olspeed           ← 查看当前全部状态
olboost 5.0       ← 设启动补偿电压
olkvf 0.06        ← 设 V/F 斜率
olramp 200        ← 设软启动斜坡
olvlimit 15       ← 设 Vq 上限
```

---

## 6. 相关代码索引

| 文件 | 内容 |
|------|------|
| [motor_it.c](../App/Motor/motor_it.c) | DMA 中断 + 开环 V/F 主逻辑（斜坡、角度递增、V/F 曲线、SVPWM） |
| [motor_param.h](../App/Motor_Param/inc/motor_param.h) | `MOTOR_PARAM_S` 结构体，所有 V/F 参数定义 |
| [motor_param.c](../App/Motor_Param/motor_param.c) | 默认值初始化 + Flash 加载后覆盖 |
| [cmd_openloop.c](../App/Cmd/cmd_openloop.c) | `olspeed`/`olboost`/`olkvf`/`olramp`/`olvlimit` shell 命令 |
| [motor_foc.c](../App/Motor/motor_foc.c) | `inverse_park()`, `svm_hqd()` |

---

## 7. 当前阶段总结：先用简洁开环验证，再考虑 V/F 参数化

### 7.1 当前现象

- 简洁版开环可以转。
- V/F 参数化版本也能转，但低速仍有一点抖动。
- 这说明基础链路大体是通的：
  - DMA 中断能进。
  - `inverse_park()` 和 `svm_hqd()` 能正常输出。
  - PWM 三相输出能推动电机。
  - 相序至少不是完全错误。

### 7.2 简洁版开环的价值

调试阶段，固定电压幅度 + 固定角度增量最直接：

```c
#define OPEN_LOOP_VQ      3.0f
#define OPEN_LOOP_STEP    0.001f

theta += OPEN_LOOP_STEP;
if (theta > M_2PI) {
    theta -= M_2PI;
}

inverse_park(0.0f, OPEN_LOOP_VQ, theta, &modAlpha, &modBeta);
svm_hqd(modAlpha, modBeta, vBus, TIM1->ARR, &duty1, &duty2, &duty3, &sector);
```

只需要看两个量：

| 参数 | 作用 | 调参判断 |
|------|------|----------|
| `OPEN_LOOP_VQ` | 电压幅度/力矩 | 卡住没力就加；嗡嗡发热就减 |
| `OPEN_LOOP_STEP` | 每次中断角度增量/等效转速 | 抖动失步就减；能稳定后再加 |

简洁版适合先确认：

- 电机能不能被 FOC/SVPWM 推起来。
- 相序方向是否正确。
- `inverse_park()` 角度方向是否符合预期。
- PWM 输出是否正常。

### 7.3 V/F 参数化版本的意义

V/F 版本不是为了让最初调试更简单，而是为了后续使用更方便：

- 可以用 `olspeed <rpm>` 设目标转速。
- 有 `ol_ramp_rpm_s` 软启动，减少突然给高速导致的失步。
- `Vq = ol_v_boost + ol_kv_f * f_elec`，速度升高时自动补一点电压。
- 有 `olstart` / `olstop` / `olmax` / `olmin` 等命令，现场调参更方便。
- 目标为 0 或未使能时，可以回安全占空比。

代价也明显：

- 参数更多，排查抖动更绕。
- `ol_foc_freq` 必须接近真实中断频率，否则 RPM 和实际角度速度不一致。
- 开环本身没有位置反馈，低速抖动无法完全避免。

### 7.4 目前建议

当前阶段建议先以简洁版为主：

1. 用固定 `OPEN_LOOP_VQ` 和 `OPEN_LOOP_STEP` 找到一组能稳定转的基础值。
2. 记录等效转速：

```text
等效机械 RPM = OPEN_LOOP_STEP * FOC_freq / (2π) / pole_pairs * 60
```

3. 确认相序、电角度方向、SVPWM 输出都没有问题。
4. 再把这组基础值迁移到 V/F 版本：
   - `ol_min_rpm` 对齐简洁版等效低速。
   - `ol_v_boost` 对齐简洁版能稳定启动的 `OPEN_LOOP_VQ`。
   - `ol_ramp_rpm_s` 先设小，例如 `10 RPM/s`。
   - `ol_kv_f` 先保守，例如 `0.15 V/Hz`，高速无力再加。

### 7.5 当前默认参数记录

当前偏稳的默认值：

```c
ol_enable      = true;
ol_target_rpm  = 80.0f;
ol_max_rpm     = 800.0f;
ol_v_boost     = 3.0f;
ol_kv_f        = 0.15f;
ol_ramp_rpm_s  = 10.0f;
ol_vq_limit    = 18.0f;
ol_foc_freq    = 30000.0f;
ol_pole_pairs  = 4.0f;
ol_min_rpm     = 30.0f;
```

### 7.6 重要提醒

`ol_foc_freq = 30000` 只是算法假定值，不一定等于真实 `Motor_DmaInt()` 控制频率。

如果真实控制频率不是 30kHz：

- 真实频率低于 30kHz：实际转速会低于目标。
- 真实频率高于 30kHz：实际转速会高于目标。

所以如果继续使用 V/F 版本，后面必须确认真实中断频率，或者用真实 `dt` 来积分角度。
