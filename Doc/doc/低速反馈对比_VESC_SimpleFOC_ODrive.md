# 低速反馈对比：VESC / SimpleFOC / ODrive

> 2026-09-16。对照本仓库 Hall 72 RPM 实机、现有编码器/无感代码，以及三家开源栈的低速做法。
> 目的是决定「测速和电角学谁」，不是换协议栈或改 VESC 外设基线。
>
> 配套：[FOC 实机测试交接](./FOC实机测试交接_2026-09-16.md) · [开环电流环调试说明](./开环电流环调试说明.md)

本文中的 **ODrive** 即口头说的 oldriver。本机默认 **2 对极**，机械转速与 VESC ERPM 的关系是：

```text
ERPM = 机械 RPM × 极对数
72 RPM  = 144 ERPM
150 RPM = 300 ERPM
500 ERPM（VESC 默认 Hall 插值门限）≈ 250 RPM
```

## 1. 先看本机事实

Hall 完整套件日志：`Doc/verify/hall_verify_535940d8-dirty_20260916-110721_allHall.log`。

| 工况 | 结果 | 说明 |
|------|------|------|
| 150→120→100→72 正转降速 | 脚本通过 | 72 RPM 的 filtered≈77，但 **fast/Meas 只有 56～63** |
| 停机直起 72 RPM | **失败** | filtered=41.2，Speed Iq 仅 0.095 A，**未反转** |
| 150→0 主动制动 | 通过 | 目标与 Speed Iq 归零 |
| 反转 −150→−72 | 通过 | −72 比正转 72 更稳 |
| 150→800 升速 | 通过 | 空载电气闭环成立 |
| 编码器 | 未接入 | `encoder show` 曾保持 0 |
| 无感接管 | 未整套实机验证 | 测试主动 `slenable 0` |

72 RPM、2 对极时 Hall 扇区约 **69 ms** 一次边沿（约 14 Hz）。新版低速 `mech_rpm_fast` 已接入 ODrive 式 Hall-PLL（`MOTOR_HALL_PLL_BW_HZ=10`，cpr=6）；超过 300 RPM 继续用原六边沿速度，降到 200 RPM 以下再切回 PLL，避免低带宽 PLL 在高速直接起转时误锁。低于 `MOTOR_HALL_INTERP_START_RPM`（150 RPM）电角仍钉在扇区中心。速度 PI 约 `Kp=0.006`、`Ki=0.003`。此版已通过主机测试、Keil 编译，并用 STM32F405RG / SWD 100 kHz 烧录成功；但首次 72 RPM 起转后 `olsnap` 和停机命令均无串口确认，实机控制效果**尚未验证通过**，测试已暂停并要求人工断电。

低速排序（对本机）：

```text
编码器 PLL > Hall PLL > I/F 开环拖 > 观测器闭环 > 没有 HFI 的无感零速
```

VESC HFI 是另一条「没编码器也要零速」的平行路，不替代 Hall 72 RPM 的修法。

## 2. Hall 低速

Hall 的根因是 **每电周期只有 6 个扇区**。72 RPM 上插值、测速、从停机起转是三件不同的事。

### 2.1 三家怎么做

**VESC**（`ThirtyLibrary/bldc/motor/foc_math.c` 的 `foc_correct_hall`）

- 边沿把角放到新旧扇区中点，再用上一扇区 `ω` 往前推。
- 低于 `foc_hall_interp_erpm`（默认 500 ERPM，本机约 **250 RPM**）**关掉插值**，钉在当前表值。注释：低速插值会在换向时锁偏 60°。
- 电角再按 `max(hall_rpm, interp_erpm)` 的 1.5 倍 **限速**，减轻 60° 跳变打电流环。
- 速度环默认吃 **电角 PLL**（`foc_pll_run`），不是 Hall 边沿周期。
- 速度指令低于 `s_pid_min_erpm` 时松轴，不在 Hall 分辨率上锁零速。

**SimpleFOC**（官方 `HallSensor.cpp`）

- `getSensorAngle()` 永远是 `扇区 × 60°`，**不插值**。
- 速度 = 上一扇区周期；刚换向或超过 2 倍周期无边沿 → 速度报 0。
- 速度 PID 默认 `P≈0.5、I≈10`，再加 `LPF_velocity.Tf≈5～10 ms`。靠大积分和低通把台阶糊过去。
- 低速常用直接给 `Uq`/`Iq`，不一定上速度闭环。

**ODrive**（`encoder.cpp`，Hall 与增量编码器同一套）

- Hall 当成 `cpr = 6 × 极对数` 的粗编码器。
- **临界阻尼 PLL** 同时出平滑位置和速度：
  ```text
  pll_kp = 2 × bandwidth
  pll_ki = 0.25 × kp²
  pos += vel·dt + kp·err·dt
  vel += ki·err·dt
  ```
- 扇区内用 PLL 速度插值，限制在当前 count 的 `[0, 1)`。
- `|vel|` 低于约 `0.5·dt·ki` 时速度 **snap 到 0**，插值钉在 **0.5（扇区中心）**，停转不漂角。
- 内环始终是 Iq。从 0 起转时速度误差大，电流顶到限幅，靠转矩过齿槽。
- 非法 Hall 状态可报 `ILLEGAL_HALL_STATE`。文档承认 Hall 低速不如编码器。

### 2.2 对照本机

| | 低速电角 | 测速 | 停机直起 72 |
|--|----------|------|-------------|
| VESC | &lt;250 RPM 不插值 + 限速 | 电角 PLL | 给 Iq，不硬锁 Hall 转速 |
| SimpleFOC | 永远 60° 台阶 | 扇区 Δt，换向/超时→0 | 大 I 或直接给转矩 |
| ODrive | PLL 插值；停转钉中心 | 同一 PLL | 误差顶满 Iq |
| 咱们已实测旧版 | &lt;150 RPM 钉中心 | 扇区角差 fast | 72 RPM 直起未稳定达速 |
| 咱们待上板版 | &lt;150 RPM 钉中心 | 低速 Hall-PLL；高速六边沿速度 | 尚未实机验证 |

直起失败不是反转，是：六边沿滤波还停在慢扇区、PI 看 fast≈56、电角钉中心、转矩对不齐。反转 −72 更好，说明扇区/齿槽不对称，单边沿测速更吃方向。

Hall 72 应学 **ODrive 的 Hall-PLL**（平滑角 + 平滑速度），停转/极低速保持 VESC 那种钉中心；不要先把 `INTERP_START` 降到 72，也不要把 SimpleFOC 的「不插值 + 大积分」当目标形态。

## 3. 编码器低速

编码器低速的问题和 Hall 不是一类：**不是 60° 太粗，而是每个控制拍 Δcount 经常是 0 或 1**。角已经够做电流环；用 `ΔN/Δt` 当转速会量化成废数据。

以 1000 线 AB、4 倍频（4000 CPR）、72 RPM、FOC 约 20 kHz 为例：

| 反馈 | 每圈边沿 | 72 RPM 边沿频率 | 每拍计数 |
|------|----------|-----------------|----------|
| Hall | 12 | ~14 Hz | 多拍才跳一格 |
| 4000 CPR 编码器 | 4000 | ~4800 Hz | 约 **0.24 count/拍** |

`Δcount/dt` 的量化步长约为 `60/(cpr·dt)`。20 kHz 上 4000 CPR 一步约 **300 RPM**，72 RPM 测不准。本仓库 `Motor_Encoder_UpdateState()` 正是这条。

### 3.1 三家怎么做

**ODrive**：Hall 和增量编码器 **同一 PLL**。CPR 从 6 变成几千，带宽可开到约 1 kHz。count 之间用 `vel` 外推；近零 snap 到 0。低速编码器仍然靠这套，只是台阶比 Hall 细两个数量级。

**VESC**：电角直接读正交计数，count 之间基本不插值。速度仍是 **电角 PLL**，不靠 Δcount。低速用编码器角，高过 `foc_sl_erpm` 交给观测器（`foc_correct_encoder`）。有 Z：找到 index 才绝对有感；只有 AB：低速先开环/观测器，转到无感转速再同步编码器。

**SimpleFOC**：增量用 `dθ/dt`，间隔太短就沿用上次速度，停转容易黏住。低速更认 SPI 磁编（AS5047 一类）的绝对角。

### 3.2 对照本机

| | 低速电角 | 低速速度 | 零点 |
|--|----------|----------|------|
| ODrive | count + PLL 插值 | 同一 PLL，近零打 0 | offset 标定，Z 可选 |
| VESC | 计数值 | 电角 PLL；高速切观测器 | 先找 Z，或 AB 高速同步 |
| SimpleFOC | 计数值 | Δθ/Δt + 低通 | 电压对齐 |
| 咱们 | 计数值 | **Δcount/dt** | Detect 把当前位置标 0 |

编码器一旦接入，72 RPM 电流环应明显好于 Hall（一格约 0.36° 电气，对 60°）。速度环必须和 Hall **共用 PLL**，不要再做一套边沿周期，也不要用逐拍 Δcount。引脚方案确认前不改 TIM/编码器底层，见交接文档第 6.3 节。

## 4. 无感低速

反电势跟转速成正比。72 RPM、2 对极时电频率只有 **2.4 Hz**，磁链观测器几乎看不见转子。三家都不是「把观测器调猛一点」，而是 **先开环拖起来，或改用凸极注入（HFI）**。

### 4.1 三家怎么做

**VESC** 两条路：

1. 经典无感：掉到开环门限以下用强制角拖转（lock → ramp → 保持），观测器一直算，够快再切回观测角。默认开环约 1500 ERPM（本机约 **750 RPM 机械**）。
2. HFI V1～V5：d 轴注高频，用凸极估零速角，再和观测器在 `foc_sl_erpm_hfi`（默认 3000 ERPM）交接。三家里 **唯一能认真做零速/低速无感** 的量产路径。要凸极，有噪声和调参成本。

**ODrive**：非线性磁链观测（Ortega）+ PLL，**没有 HFI**。文档写明不能停、不能在零速换向，最低大概几百 RPM。`sensorless_ramp` 开环爬升，收敛后切闭环。低速策略就是别在低速用它。

**SimpleFOC**：`velocity_openloop` / `angle_openloop` 积分虚拟角、给 `Uq`。没有可用的 Ortega/HFI 产品路径。

### 4.2 对照本机

当前路径接近工业常见做法，且比 ODrive 的开环电压爬升更稳：

- 低速 **I/F**（电流闭环 + 强制角）。72/150/300 RPM 开环已实机通过。
- 观测器可只观察不接管（`takeover` / `slenable`）。锁定后 blend 切观测角。
- 低于 `sensorless_fallback_rpm`（默认 300）退回 I/F；高于开环安全转速失锁则报故障，不在高速硬切回 I/F。
- `MotorHfiStart` 只是启动扫角，不是 VESC 那种持续 HFI 跟踪。
- 默认 `transition=500`、`fallback=300`。**72 RPM 本来就不该让观测器接管。**

| | 零速/72 RPM 无感 | 怎么起转 | 停转/反转 |
|--|------------------|----------|-----------|
| VESC HFI | 能做 | 注入估角 | 可以 |
| VESC 开环 | 不做闭环 | 强制角 + 电流 | 低速回开环 |
| ODrive | 不做 | 开环爬升到几百 RPM | 文档明确不支持 |
| SimpleFOC | 不做 | 电压开环一直拖 | 开环可以，无感不行 |
| 咱们 | 不做（I/F 拖） | I/F，HFI 只扫一次角 | 低速回 I/F |

无感下一阶段仍按交接：先只观测不接管，再分阶段闭环，考核点在 **300～500 RPM 以上**，不要拿 72 RPM 判定无感。

## 5. 结论：学谁、不学谁

不要换协议栈，也不要因为「参数不常规」去改 STM32F4 的 ADC/PWM/DMA 基线。

| 要解决的问题 | 更该学谁 | 不要先做的 |
|--------------|----------|------------|
| Hall 72 测速乱、速度环跟着跳 | **ODrive Hall-PLL** | 把插值门限降到 72 |
| 扇区跳变把电流打尖 | VESC 角度限速 | SimpleFOC 式永不插值 |
| 停机直起推不过齿槽 | 测速稳之前给够 Iq | 把 `Ki` 拉到 SimpleFOC 的 10 |
| 编码器低速速度量化 | 与 Hall **共用** ODrive 式 PLL | 逐拍 Δcount/dt 或 Hall 边沿周期 |
| 无感 72 RPM 闭环 | 不做；I/F 拖到 300+ 再接管 | 用观测器硬锁 72 |
| 没编码器也要零速无感 | 以后才考虑 VESC 持续 HFI | 把启动扫角当成 HFI 跟踪 |

推荐改动顺序：

1. Hall count 的 PLL 已在待上板版接入低速 `mech_rpm_fast`，须恢复硬件连接后对照实测；停转钉扇区中心。
2. 速度环 PI 先不动，等 PLL 速度稳了再看增益和 36 RPM 接管门槛。
3. 编码器接入后接到同一 PLL，不新开测速。
4. 无感继续 I/F → 观测器分阶段实机，与 Hall 72 分开。

## 6. 代码与文献落点

| 主题 | 本仓库 | 外部 |
|------|--------|------|
| VESC Hall 插值/限速 | `ThirtyLibrary/bldc/motor/foc_math.c` → `foc_correct_hall` | 默认 `MCCONF_FOC_HALL_INTERP_ERPM=500` |
| VESC 电角 PLL | 同文件 `foc_pll_run`；速度环 `foc_run_pid_control_speed` | |
| VESC 无感开环/HFI | `ThirtyLibrary/bldc/motor/mcpwm_foc.c` 传感器分支 | `mcconf_default.h` 的 `FOC_OPENLOOP_*`、`FOC_HFI_*` |
| 本机 Hall | `App/motor_sensor/motor_hall_angle.c` | 待上板版 PLL：`MOTOR_HALL_PLL_BW_HZ=10`；插值门限仍 150 RPM |
| 本机编码器 | `App/motor_sensor/motor_encoder.c` | Δcount/dt |
| 本机无感 | `App/motor/motor_sensorless.c`、`motor_hfi_start.c` | 默认 takeover 关 |
| ODrive PLL | `ThirtyLibrary/ODrive/Firmware/MotorControl/encoder.cpp` | 无感 `sensorless_estimator.cpp` |
| SimpleFOC Hall | `App/motor_simple_foc/` 仅开环拖动，不是官方 Hall 闭环 | `ThirtyLibrary/Arduino-FOC/src/sensors/HallSensor.cpp` |
| MESC Hall/HFI/观测器 | `ThirtyLibrary/MESC_Firmware/MESC_Common/Src/` | `MESCfoc.c`、`MESChfi.c`、`MESCfluxobs.c` |
| moteus 电流/位置环 | `ThirtyLibrary/moteus/fw/` | `bldc_servo.cc`、`foc.cc` |
| hoverboard Hall FOC | `ThirtyLibrary/hoverboard-firmware-hack-FOC/Src/` | `BLDC_controller.c`、`control.c` |

以上对照仓为 shallow clone，未编进 Keil，`.gitignore` 排除。ST MCSDK 无干净公开 git，未入库。已有 `ThirtyLibrary/bldc`（VESC）、`ThirtyLibrary/am32`（六步，非 FOC）。

Hall 复测脚本：`Scripts/hall_verify_serial.py`。`--profile all` 覆盖降速、停机直起 72、150→0 制动、反转、升速。
