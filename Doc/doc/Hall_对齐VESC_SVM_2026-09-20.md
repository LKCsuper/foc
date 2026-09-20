# Hall FOC 对齐 VESC `foc_svm`

> 2026-09-20。BM75 / STM32F405RG，24 V，空载，Hall + I/F，`spiq 0.8`，弱磁 0 A。COM20，SWD 100 kHz 烧录，不要芯片擦除。
>
> 目的：把 Hall 电流环的 PWM 从「`svm_hqd` + CCR 倍率」换成 VESC 那套恒定线性 SVPWM。速度环仍然只出 Iq；弱磁仍然只在 PWM 真正顶满、且 Id 跟得上时才开。
>
> 配套：[CCR / 母线 / 弱磁](./弱磁.md) · [提速调试经验](./Hall_FOC提速调试经验_2026-09-17.md) · [1500 RPM 原因](./Hall_1500RPM达标原因与限制_2026-09-16.md) · [偶发 Id 毛刺](./Hall偶发Id毛刺_2026-09-19.md)

口头说的 **oldriver** 即 ODrive。本机 2 对极、铭牌 24 VDC / 3000 RPM。

## 1. 结论

Hall + I/F 输出已经按 VESC `mcpwm_foc` 走：

```text
逆 Park(Vd,Vq) → mod = 1.5 × v / Vbus → 限幅到 (√3/2)×0.95 → foc_svm → A-C-B
```

零电压 CCR 在 **ARR/2 = 2800**（ARR=5600）。`MOTOR_HALL_PWM_GAIN` 已删除。电流环默认 **`olkp 0.12` / `olki 18`**。ADC / PWM / TIM / DMA 未改。

空载 COM20 验收：

| 目标 | 实测 | `|V|` / Vlim | `Speed Iq` | 日志 |
|------|------|--------------|------------|------|
| 150 | 153 | 0.59 / 13.8 | 0.33 A | [150](../verify/hall_verify_ed43d8a7-dirty_20260920-162155_150RPM.log) |
| 2000 | 2009 | 6.8 / 13.8 | 0.41 A | [2000](../verify/hall_verify_ed43d8a7-dirty_20260920-162355_2000RPM.log) |
| 2500 | **2500** | 8.6 / 13.8 | 0.49 A | [2500](../verify/hall_verify_ed43d8a7-dirty_20260920-162613_2500RPM.log) |
| 3000 | **2961**（±1.3%） | 10.3 / 13.7 | 0.73 A | [3000](../verify/hall_verify_ed43d8a7-dirty_20260920-162848_3000RPM.log) |

`PWM ideal` 已经和 `|Vd,Vq|` 对上，不再是旧 `svm_hqd` 的大约 1/3。2500 是真到目标，不是脚本 ±10% 踩线。3000 空载电压还没顶满；带载另说。弱磁仍关。1800 以上 `|Id|` 毛刺变多，转速能跟、电流还不能当准。

## 2. 为什么不继续加 CCR

旧 `svm_hqd` 时间合成多除了 2，零电压大约在 **ARR/4**。电流环请求 10 V，电机端只有大约 3 V。Hall 上乘常数 `MOTOR_HALL_PWM_GAIN` 把差模抬上去：

| 倍率 | 150 起转 | 2000 | 2500 |
|------|----------|------|------|
| 1.50 | 过 | 过 | 约 2009，失败 |
| 1.70 | 过 | 过 | 脚本过、实速 **2250** |
| 1.80 | **失败**（滤波 234） | 未测 | 未测 |

1.70 是还能从 150 起来的最高档。再加常数会在低速过压；按转速切换倍率会把速度环调参绑在调制上。VESC 和 ODrive 都没有这个旋钮：SVM 公式恒定，速度 PI 只出 Iq，占空比顶满才额外加负 Id。

完整倍率扫描见[弱磁文档第 2 节](./弱磁.md)。

## 3. VESC 实际怎么做

锁定源码 `Doc/VESC/vesc_foc.c`：

```text
voltage_normalize = 1.5 / v_bus          // 即 1 / ((2/3)×Vbus)
mod_d = vd × voltage_normalize
mod_q = vq × voltage_normalize
逆 Park → mod_α, mod_β
限幅到 √3/2 × l_max_duty（默认 0.95）
foc_svm(mod_α, mod_β, ARR) → CCR 中点 ARR/2
```

`|V|` 线性上限 `Vbus/√3` 时，`|mod|` 正好是 √3/2 ≈ 0.866，不会把 `foc_svm` 里的 `uint32_t` 作用时间算成负数。`l_max_duty≈0.95` 再留约 2.5% 零矢量给低边分流。

ODrive 也是速度 PI → Iq，duty 0～1 × ARR；弱磁是实验项。不要抄它的 Clarke `Iα=Ia`。

## 4. 本仓库落点

| 文件 | 改什么 |
|------|--------|
| [`App/motor/motor_foc.c`](../../App/motor/motor_foc.c) | 新增 `foc_svm_from_voltage()`：1.5/Vbus 归一、限幅、再调已有 `foc_svm` |
| [`App/motor/inc/motor_foc.h`](../../App/motor/inc/motor_foc.h) | 声明 |
| [`App/motor/motor_it.c`](../../App/motor/motor_it.c) | Hall + I/F 走上述函数；`MOTOR_HALL_USE_VESC_SVM` 可回退；相序仍 A-C-B |
| [`App/motor_param/motor_param.c`](../../App/motor_param/motor_param.c) | 默认 `ol_current_kp=0.12`、`ol_current_ki=18` |
| [`Scripts/hall_verify_serial.py`](../../Scripts/hall_verify_serial.py) | 验证脚本同步 `olkp 0.12` / `olki 18` |

其它模式（编码器、V/F、开环）仍用 `svm_hqd`。这是一次只改 Hall 电流环。

回退：把 `MOTOR_HALL_USE_VESC_SVM` 置 0，Hall I/F 回到 `svm_hqd`。不要把 CCR 倍率加回去。

## 5. 两次失败，一次才过

2026-09-16 曾经换过「标准线性 SVPWM」，150 RPM 只到约 58 RPM，当时撤回。CCR 已在 ARR/2，`|V|≈0.95 V`，但 `Id=0.625`（目标 0）跟不上。那次更像扇区/极性没对齐，不是采样窗口被 ARR/2 吃掉：低速调制很小，顶部零矢量仍然很长。

2026-09-20 按 VESC 公式重来，三次结果：

| 固件 | 150 RPM | 原因 |
|------|---------|------|
| `foc_svm(+vα,+vβ)` + A-C-B | 过流锁存。`Id≈−12.1 A`，`Ipeak=12.1 A`，母线 24→22.6 V，转速 0 | 标准 `foc_svm` 再叠本板 A-C-B 后，相对已验证的 `svm_hqd`/Park **差 180°**。电流环正反馈。 |
| 输入取反，仍 `olkp 0.2` | 无故障，滤波 210（上限 187.5），CCR 几乎 2800/2800/2800 | 电压增益从约 1.70/3 拉到 1，电流环带宽大约 1.76 倍。空载 I/F 的 0.6 A 把转速冲过去，速度环不允许制动，掉不回来。 |
| 取反 + **`olkp 0.12` / `olki 18`** | **通过** | 按 1.70 倍率时的电流带宽折算：`0.2×1.70/3`、`30×1.70/3`，Ki/Kp 仍是 150。 |

过流日志：[无取反](../verify/hall_verify_ed43d8a7-dirty_20260920-155602_150RPM.log)。过冲日志：[olkp 0.2](../verify/hall_verify_ed43d8a7-dirty_20260920-155835_150RPM.log)。

取反写在 Hall 输出调用处，`foc_svm_from_voltage()` 本身仍是 VESC 极性。不要改 A-C-B，那是 HD75 实机相序。

## 6. 和旧路径差在哪

150 RPM、同一套速度环：

| | 旧 `svm_hqd`×1.70 | 现在 `foc_svm` |
|--|-------------------|----------------|
| CCR 中点 | 约 ARR/4，再乘 1.70 | **2800（ARR/2）** |
| 150 RPM `|V|` | 请求约 1.35 V，PWM 理想约 1.07 V | 请求约 0.59 V，PWM 理想约 0.59 V |
| 2500 RPM | 请求顶满 13.8 V，PWM 理想约 **7.8 V**，实速 **2250** | 请求 8.6 V，PWM 理想 **8.6 V**，实速 **2500** |
| 3000 RPM | 未到 | 2961，请求 10.3 / 13.7 V |

软件限幅还是 `Vlim = min(ol_vq_limit, Vbus/√3) ≈ 13.8 V`。以前顶满是调制没送出去；现在空载 3000 还没用满，带载或更高转速才轮到弱磁。

铭牌 3000 RPM 的 `ωeλ≈16.2 V` 大于 13.8 V。空载实际 `|V|` 只有 10.3 V，说明当时 Iq 小、角度也不是理想 q 轴，不能拿铭牌反电势直接说「3000 必须弱磁」。带载时 IR 和 `ωLI` 上去，电压会先顶满。

## 7. 还没做的

- **弱磁仍关。** 空载 3000 `|V|≈10.3/13.7 V`。打开的最低条件仍是：PWM 理想接近请求且接近 13.8 V，并且 2000 上空载 Id 大体跟目标。现在 1800 以上 `|Id error|>1 A` 的样本明显变多（3000 档约 40%），负 Id 跟不上。
- **电流环默认增益只对 Hall + 新 SVM。** 其它模式还在 `svm_hqd`，若用同一套 `olkp 0.12` 会偏软。
- **速度环还没按新对象重调。** 150～3000 用的仍是 `spkp 0.006` / `spki 0.003` / `spbrake 0`。空载能跟，带载、加减速另测。
- **Id 毛刺不是这次修的。** 诊断门槛、最长零矢量重构见[偶发 Id 毛刺](./Hall偶发Id毛刺_2026-09-19.md)。3000 时峰值 CCR 4828，距 `ARR−900` 已近，重构有可能开始触发，还没单独核对。
- **不要改 VESC 来源的 ADC/PWM/TIM/DMA。** Hall 仍然只采 TIM1 下降 / 顶部零矢量。ARR/2 之后低速零矢量仍然够用；高速窗口变短，是以后要示波器核对的事，不是这次改外设的理由。
- **不要抬母线。** 电机铭牌 24 VDC。

## 8. 怎么复测

```text
Scripts/hall_build_flash_verify.ps1 -Port COM20 -SwdSpeedKhz 100 -BuildAction rebuild -Profile up -MaxRpm 150 -Samples 1
```

先 150，再 `--max-rpm 2000 / 2500 / 3000`。脚本会发 `olkp 0.12`、`olki 18`。看 `olsnap` 时同时看：

```text
Theta/CCR     中点应在 2800 附近，不要再回到 ~1400
PWM ideal     应接近 |Vd,Vq out|
Fault         CLEAR
Speed Iq      空载 150～2500 大约 0.3～0.5 A；3000 大约 0.73 A，未顶 0.8 A
```

UART 无响应：J-Link 复位停机，不要芯片擦除。
