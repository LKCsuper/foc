# GLM 版 TIM / ADC / DMA 外设总结

> 与 DeepSeek / Codex 版文档的定位不同：本文以 **信号链路 + 寄存器级** 视角组织，侧重于「数据怎么流、时序怎么对、坑在哪里」，而不是罗列初始化参数。  
> 源码引用格式：`文件:行号`，可直接跳转。

---

## 0. 一张图看全貌

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        FOC 电流环闭环数据流                             │
│                                                                         │
│  ┌─────┐   TRGO    ┌─────┐  CC2↓    ┌───────────┐  CDR   ┌─────┐     │
│  │TIM1 │──────────→│TIM2 │────────→ │ADC1/2/3   │──────→│DMA2 │     │
│  │PWM  │  Update   │Trigger│        │三重同步    │        │S4   │     │
│  └──┬───┘  Event   └──────┘         └───────────┘        └──┬──┘     │
│     │                                                   TC/HT│       │
│     │    ┌──────────────────────────────────────────────┐  │       │
│     │    │              Motor_DmaInt()                  │←─┘       │
│     │    │  1. 读 ADC_Value[] → ia/ib/ic + v_bus       │          │
│     │    │  2. Clarke → Iα/Iβ                          │          │
│     │    │  3. Park(θ) → Id/Iq                        │          │
│     │    │  4. PID → Vd/Vq                            │          │
│     │    │  5. 逆Park → Vα/Vβ → SVPWM                 │          │
│     │    │  6. TIMER_UPDATE_DUTY_M1(ccr1,ccr2,ccr3)   │──┘       │
│     │    │  7. TIM_GenerateEvent(COM) → 影子寄存器同步  │          │
│     │    └──────────────────────────────────────────────┘          │
│     │                  ↑ 新占空比在下个 PWM 周期生效                 │
│     └──────────────────┘                                            │
└─────────────────────────────────────────────────────────────────────────┘
```

**一句话**：TIM1 定节拍 → TIM2 定时刻 → ADC 三机同拍 → DMA 自动搬 → 中断喊 FOC 开算 → 算完写回 TIM1。

---

## 1. 两条初始化路径：FOC vs BLDC

本工程存在 **两套初始化代码**，对应两种电机驱动架构：

| 对比项 | FOC 路径（当前主力） | BLDC 路径（预留/参考） |
|--------|---------------------|----------------------|
| TIM 初始化 | `bsp_tim.c: Bsp_Tim_Init()` | `bsp_bldc_tim.c: Bsp_Tim_Bldc_Init()` |
| ADC 初始化 | `bsp_adc.c: Bsp_Adc_Init()` | `bsp_bldc_adc.c: Bsp_Adc_Bldc_Init()` |
| DMA 初始化 | `bsp_dma.c: Bsp_Dma_Init()` | `bsp_bldc_dma.c: Bsp_Dma_Bldc_Init()` |
| TIM1 计数模式 | **中央对齐**（CenterAligned1） | **向上计数**（Up） |
| ADC 触发源 | TIM2_CC2 下降沿 | TIM8_CC1 下降沿 |
| 注入组触发 | 未配置（FOC路径） | TIM1_CC4 / TIM8_CC2 / TIM8_CC3 |
| 注入组中断 | 未使用 | ADC1 JEOC → `ADC_IRQn` |
| DMA 中断 | HT/TC 编译期选择 | 固定 TC |
| PWM 频率 | 15kHz（f_zv=30000） | 由 `switching_frequency_now` 动态决定 |

> **当前 `Bsp_Init()` 走的是 FOC 路径**。BLDC 路径的代码存在但未在主初始化流程中被调用。

---

## 2. 信号链路逐级拆解

### 2.1 TIM1 — 节拍器

```
Bsp_Tim_Init()
  └─ Bsp_Tim_Pwm1(30000)
       ├─ Bsp_Tim1_Base(f_zv)        // 时基
       ├─ Bsp_Tim1_OC()              // 输出比较（4通道）
       ├─ Bsp_Tim1_BDTR()            // 刹车/死区/预装载
       ├─ TIM_Cmd(ENABLE)            // 启动
       ├─ Bsp_Tim_StopPwm(MOTOR1)    // ★ 安全关断
       ├─ TIM_CtrlPWMOutputs(ENABLE) // MOE 使能
       └─ TIMER_UPDATE_SAMP(2)       // 设置 ADC 触发偏移
```

**核心参数**：

| 参数 | 值 | 推导 |
|------|-----|------|
| 预分频 | 0 | 定时器时钟 = 168MHz |
| ARR | 5600 | 168M / 30000 |
| PWM 频率 | 15kHz | 168M / 5600 / 2（中央对齐除2） |
| PWM 周期 | 66.7µs | 1/15kHz |
| 死区 | ~350ns | `calculate_deadtime(360ns, 168MHz)` |
| 初始占空比 | 50% | CCR = ARR/2 = 2800 |

**为什么上电先 StopPwm 再 Enable MOE？**

这是典型的"安全启动"模式：
1. `TIM_Cmd(ENABLE)` → 计数器开始跑
2. `Bsp_Tim_StopPwm()` → 三相强制输出无效电平，MOS 全关
3. `TIM_CtrlPWMOutputs(ENABLE)` → MOE 置1，但因为通道已经是 ForcedInactive，引脚不会驱动
4. 等 FOC 算法就绪后调 `Bsp_Tim_StartPwm()` → 切回 PWM1 模式

**COM 事件机制**（容易忽略的关键点）：

```c
// BDTR 配置中：
TIM_CCPreloadControl(TIM1, ENABLE);  // CCR → 影子寄存器
TIM_ARRPreloadConfig(TIM1, ENABLE);  // ARR → 影子寄存器

// FOC 中断末尾：
TIM_GenerateEvent(TIM1, TIM_EventSource_COM);  // 影子 → 实际
```

这意味着：FOC 写入的新 CCR 值 **不会立即生效**，必须等 COM 事件才同步到输出。好处是占空比更新与 PWM 周期对齐，不会出现半周期切换导致的毛刺。

### 2.2 TIM2 — 采样闹钟

```
Bsp_Tim1_TrigAdc()
  └─ Bsp_Tim2_Pwm()
       ├─ Bsp_Tim2_Base()    // 时基：PSC=0, ARR=0xFFFF, 向上计数
       ├─ Bsp_Tim2_OC()      // 3通道 PWM1，CCR=250
       └─ Bsp_Tim2_NVIC()    // 中断（当前 #if 0 禁用）
```

**TIM2 为什么存在？不能直接用 TIM1 触发 ADC 吗？**

可以，但 TIM2 提供了 **采样时刻的精确可调能力**：

- TIM1 的 Update 事件固定在 ARR（顶端）处，无法灵活偏移
- TIM2 被 TIM1 复位后从 0 开始计数，CCR2 决定"复位后多久触发 ADC"
- 修改 `TIM2->CCR2` 即可在 PWM 周期内任意位置采样

**主从触发链**（`Bsp_Tim1_Trig`）：

```
单电机模式：
TIM1 TRGO = Update Event
     │
     │ (ITR0 内部连接)
     ▼
TIM2 SlaveMode = Reset  →  每次 TIM1 Update 都让 TIM2 归零

双电机模式：
TIM1 TRGO = Enable → TIM8 同步启动
TIM8 TRGO = Update → TIM2 Reset（取代 TIM1 直接触发）
```

**采样点计算**：

```c
#define MCPWM_FOC_CURRENT_SAMP_OFFSET  (2)
#define TIMER_UPDATE_SAMP(samp)         TIM2->CCR2 = (samp / 2)
// → TIM2->CCR2 = 1
```

TIM2 时钟 = APB1 定时器时钟 = 84MHz，1 个 tick ≈ 11.9ns。  
所以采样点在 TIM1 Update 后约 **12ns** 就触发 ADC —— 这非常靠近开关瞬间。

> ⚠️ **潜在问题**：CCR2=1 意味着采样几乎紧贴 TIM1 Update 时刻，此时开关噪声可能尚未衰减。实际是否安全取决于硬件布局，建议示波器验证。

### 2.3 ADC1/2/3 — 三机同拍

**三重规则同步模式的工作原理**：

```
TIM2_CC2 下降沿
     │
     ▼
ADC1 收到外部触发 ──→ 启动规则组扫描
     │                    │
     │  同时（硬件同步）    │
     ├──→ ADC2 跟随启动 ──┤
     ├──→ ADC3 跟随启动 ──┤
     │                    │
     │               5个rank依次转换
     │                    │
     │         每个rank: ADC1+ADC2+ADC3 同时转换
     │         结果写入 ADC->CDR（32位，低16=ADC1，高16=ADC2）
     │         ADC3 结果写入 ADC->CDR 高16位（DMA模式1交替）
     │                    │
     ▼                    ▼
     DMA 请求 ←───── 全部5 rank转换完成
```

**DMAAccessMode_1 的 CDR 排列**：

这是最容易混淆的地方。`ADC_DMAAccessMode_1` 的含义是"半字逐个交替"：

```
Rank 1: ADC1_CH0 → ADC_Value[0]
        ADC2_CH1 → ADC_Value[1]
        ADC3_CH2 → ADC_Value[2]
Rank 2: ADC1_CH10 → ADC_Value[3]
        ADC2_CH11 → ADC_Value[4]
        ADC3_CH12 → ADC_Value[5]
Rank 3: ADC1_CH5  → ADC_Value[6]
        ADC2_CH6  → ADC_Value[7]
        ADC3_CH3  → ADC_Value[8]
Rank 4: ADC1_CH14 → ADC_Value[9]
        ADC2_CH15 → ADC_Value[10]
        ADC3_CH13 → ADC_Value[11]
Rank 5: ADC1_Vref → ADC_Value[12]
        ADC2_CH0  → ADC_Value[13]
        ADC3_CH1  → ADC_Value[14]
```

**ADC_Value[] 物理含义速查**（来源：[hw_hd75.h](App/Motor/inc/hw_hd75.h)）：

| 索引 | ADC | 通道 | 宏名 | 含义 |
|------|-----|------|------|------|
| 0 | ADC1 | CH0 | `ADC_IND_SENS1` | A相相电压 |
| 1 | ADC2 | CH1 | `ADC_IND_SENS2` | B相相电压 |
| 2 | ADC3 | CH2 | `ADC_IND_SENS3` | C相相电压 |
| 3 | ADC1 | CH10 | `ADC_IND_CURR1` | A相电流 |
| 4 | ADC2 | CH11 | `ADC_IND_CURR2` | B相电流 |
| 5 | ADC3 | CH12 | `ADC_IND_CURR3` | C相电流 |
| 6 | ADC1 | CH5 | `ADC_IND_EXT` | 外部ADC1 |
| 7 | ADC2 | CH6 | `ADC_IND_EXT2` | 外部ADC2 |
| 8 | ADC3 | CH3 | `ADC_IND_TEMP_MOS` | MOS温度 |
| 9 | ADC1 | CH14 | `ADC_IND_TEMP_MOTOR` | 电机温度 |
| 10 | ADC2 | CH15 | `ADC_IND_SHUTDOWN` | 关机检测 |
| 11 | ADC3 | CH13 | `ADC_IND_VIN_SENS` | 母线电压 |
| 12 | ADC1 | Vref | `ADC_IND_VREFINT` | 内部参考电压 |
| 13 | ADC2 | CH0 | — | SENS1重复 |
| 14 | ADC3 | CH1 | — | SENS2重复 |

**一个 rank 的转换耗时估算**：

```
采样时间 = 15 ADC cycles
转换时间 = 12 ADC cycles（12bit）
三重间隔 = 5 ADC cycles

每个 rank ≈ 15 + 12 + 5 = 32 ADC cycles
5 个 rank ≈ 160 ADC cycles

ADC 时钟 = 42MHz
总转换时间 ≈ 160 / 42MHz ≈ 3.8µs
```

占 PWM 周期 66.7µs 的 ~5.7%，留给 FOC 计算的时间窗口约 62.9µs。

### 2.4 DMA2 Stream4 — 搬运工

**初始化防误触机制**（Codex 优化的关键改进）：

```c
// 1. 先 DeInit + 等待完全关闭
DMA_DeInit(DMA2_Stream4);
while (DMA_GetCmdStatus(DMA2_Stream4) != DISABLE && --dma_timeout) {}

// 2. 清所有旧标志
DMA_ClearFlag(DMA2_Stream4, TC|HT|TE|DME|FE);

// 3. 写配置（不使能）
DMA_Init(DMA2_Stream4, &DMA_InitStructure);

// 4. 配中断源（HT或TC）
DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);

// 5. 再清一次标志（防 pending）
DMA_ClearFlag(DMA2_Stream4, ...);

// 6. 最后才使能 DMA
DMA_Cmd(DMA2_Stream4, ENABLE);
```

为什么要这么繁琐？因为 DMA 标志是 **硬件置位、软件清除**，如果在配置过程中不清干净，一使能就立刻进中断，FOC 还没准备好就乱算了。

**HT vs TC 中断选择**：

```c
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
    DMA_IT_HT   // 半传输中断：搬运 7~8 个数据就触发
#else
    DMA_IT_TC   // 传输完成中断：15 个全搬完才触发
#endif
```

当前电流在索引 3/4/5，走 **TC**。如果想走 HT 获得更低延迟，需要把电流通道调整到前 3 个位置（即 ADC_Value[0..2]），这需要修改规则组 rank 排列。

### 2.5 中断回调链

```
DMA2_Stream4_IRQn (硬件)
    │
    ▼
DMA2_Stream4_IRQHandler()          [stm32f4xx_it.c:132]
    │
    ▼
Bsp_Dma2Stream4_Handler()          [bsp_it.c:81]
    │  检查 TC/HT 标志 → 清除 → 调用 FOC
    │
    ▼
Motor_DmaInt(NULL, NULL)           [motor_it.c:80]
    │
    ├─ 分频判断 (FOC_CONTROL_LOOP_FREQ_DIVIDER)
    │     skip++ == DIVIDER ? 继续 : return
    │
    ├─ Motor_It_ParamCal()          [motor_it.c:36]
    │     ├─ GET_CURRENT1/2/3()     → 读取 ADC_Value[3/4/5]
    │     ├─ ADC_VOLTS(SENS1/2/3)   → 读取相电压
    │     ├─ 电流零偏: ADC - 2045   → × FAC_CURRENT → 安培
    │     ├─ 相电压: ADC - offset   → × 分压比 → 伏特
    │     └─ v_bus: 低通滤波        → 母线电压
    │
    ├─ Simulink FOC (当前 #if 1)
    │     ├─ rtU 输入赋值 (Ia/Ib/Ic/Udc/Theta/we...)
    │     ├─ foc_simulink0_step()
    │     └─ TIMER_UPDATE_DUTY_M1(rtY.Tcm1/2/3)
    │
    ├─ 手写 FOC (当前 #if 0，备用)
    │     ├─ Clarke → Park → PID → 逆Park → SVPWM
    │     └─ TIMER_UPDATE_DUTY_M1(duty1/2/3)
    │
    └─ TIM_GenerateEvent(TIM1, COM)  ← 影子寄存器同步
```

---

## 3. 时序细节：一个 PWM 周期里发生了什么

```
          ┌─── TIM1 CNT 向上计数 ──────────────────────┐
          │                                             │
    CNT=0 │                                       CNT=ARR=5600
      │   │                                             │
      │   │   上桥导通                    上桥关断       │
      │   │   (CNT < CCR)               (CNT ≥ CCR)    │
      │   │                                             │
      ▼   ▼                                             ▼
  ────╥═══╥════════════════════════════════════╥═══════╥────
      ║   ║                                      ║       ║
      ║   ║  ← 死区 350ns →                      ║       ║
      ║   ║                                      ║       ║
  ────╨═══╨════════════════════════════════════╨═══════╨────
      下桥关断  下桥导通                     下桥关断  下桥导通
      
                                                          │
                                                    Update Event
                                                          │
                                                    TIM1 TRGO
                                                          │
                                                    TIM2 Reset (CNT=0)
                                                          │
                                                    TIM2 CNT=1 = CCR2
                                                          │
                                                    TIM2 CC2 ↓ → 触发 ADC
                                                          │
                                                    ADC 5×rank 转换 (~3.8µs)
                                                          │
                                                    DMA 搬运 15 个值
                                                          │
                                                    DMA TC 中断
                                                          │
                                                    Motor_DmaInt()
                                                          │
                                                    FOC 计算 + 更新 CCR
                                                          │
                                                    COM 事件 → 影子同步
                                                          │
                                          ┌─── TIM1 CNT 向下计数 ───┐
                                          │  新占空比在下半周期生效   │
                                          └──────────────────────────┘
```

**关键时间预算**（15kHz PWM = 66.7µs 周期）：

| 阶段 | 耗时 | 说明 |
|------|------|------|
| TIM2 延迟 → ADC 触发 | ~12ns | CCR2=1, 1个 TIM2 tick |
| ADC 转换 5 rank | ~3.8µs | 15+12+5 cycles × 5 rank / 42MHz |
| DMA 搬运 | ~0.5µs | 15个半字，DMA 传输极快 |
| FOC 计算 | ~10-20µs | 取决于算法复杂度 |
| **合计** | ~15-25µs | 占周期 22-37% |

剩余 ~40-50µs 可用于其他中断和 FreeRTOS 任务。

---

## 4. 两套代码的关键差异详解

### 4.1 BLDC 路径的注入组——FOC 路径缺失的环节

BLDC 路径在 `Bsp_Adc_Bldc_Init()` 中配置了注入组触发：

```c
// BLDC 路径:
ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T8_CC3);
ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);  // 注入组完成中断
```

而 FOC 路径的 `Bsp_Adc_Init()` **没有配置注入组触发源和中断**，注入通道虽然设置了但没有实际使用。

注入组的优势：优先级高于规则组，可以在规则组扫描过程中"插队"采样电流，确保电流采样时刻精确。如果后续 FOC 路径也需要更精确的电流采样，可以参考 BLDC 路径的注入组配置。

### 4.2 中央对齐 vs 向上计数

FOC 路径用中央对齐，BLDC 路径用向上计数。这不是随意选择：

- **中央对齐** → PWM 波形对称 → 电流纹波更小 → 适合 FOC 电流环
- **向上计数** → 实现更简单 → 适合六步换相 BLDC

### 4.3 DMA 初始化的健壮性差异

| 项目 | FOC 路径 (`bsp_dma.c`) | BLDC 路径 (`bsp_bldc_dma.c`) |
|------|------------------------|------------------------------|
| DeInit + 等待关闭 | ✅ 有 | ❌ 无 |
| DMA_StructInit 默认值 | ✅ 有 | ❌ 无 |
| 清旧标志 | ✅ 两次清除 | ❌ 无 |
| 中断标志精确匹配 | ✅ HT/TC 条件编译 | ❌ 固定 TC |
| DMA 使能时机 | ✅ 中断配好后再使能 | ❌ 配完立即使能 |

FOC 路径经过 Codex 优化后明显更健壮。BLDC 路径的 DMA 初始化存在残留标志误触发 FOC 的风险。

---

## 5. 已知问题与改进建议

### 5.1 🔴 必须关注

| # | 问题 | 位置 | 说明 |
|---|------|------|------|
| 1 | **ADC 时钟超频** | `bsp_adc.c:37` | ADC 时钟 42MHz，手册上限 36MHz。量产前建议改 `ADC_Prescaler_Div4` 或做高低温验证 |
| 2 | **电流零偏硬编码** | `motor_it.c:58-60` | `2045` 是写死的，应该上电校准。不同板子、不同温度下零偏会漂移 |
| 3 | **TIM5 时间换算错误** | `bsp_tim.c:745+806` | `TIMER_DT_HZ = 14MHz` 但实际 Prescaler=83 → 1MHz。`Bsp_Tim_DtSecondElapsedSince()` 返回值偏小 14 倍 |
| 4 | **ADC 采样点过近** | `bsp_tim.c:241` | CCR2=1 → 采样距 Update 仅 ~12ns，可能处于开关噪声区 |

### 5.2 🟡 建议改进

| # | 问题 | 位置 | 建议 |
|---|------|------|------|
| 5 | `Bsp_Adc_Init()` 重复调用 `SetChannels` | `bsp_adc.c:158-159` | `Bsp_Adc_Sample()` 已调用，再次调用多余 |
| 6 | `rtU.Tpwm = 5600` 硬编码 | `motor_it.c:141` | 应改为 `TIM1->ARR`，改 PWM 频率时不用手动同步 |
| 7 | 注入组未利用 | FOC 路径 | 可参考 BLDC 路径配置注入组触发，获取更精确的电流采样 |
| 8 | BLDC 路径 DMA 初始化缺防误触 | `bsp_bldc_dma.c` | 参考 FOC 路径的 DeInit + 清标志 + 延迟使能模式 |
| 9 | TIM2 CC 中断禁用 | `bsp_tim.c:609-624` | `#if 0` 包裹，如果需要 V0/V7 矢量检测可启用 |

### 5.3 🟢 可选优化

| # | 建议 | 说明 |
|---|------|------|
| 10 | 调整 ADC 通道排列使电流在前3位 | 编译期自动切 HT 中断，FOC 提前 ~2µs 开始计算 |
| 11 | 启用 TIM1 CH4 触发注入组 | 精确控制电流采样时刻在 V0/V7 矢量中点 |
| 12 | `Bsp_Tim_StopPwm` 中互补通道 Disable 而非 InActive | 当前下桥 Disable（高阻），如果需要刹车时下桥导通放电需改 |

---

## 6. 调试检查清单

上电后按以下顺序验证外设链路：

### 第一步：确认 TIM1 有 PWM

```c
// 在 main 循环中打印
PRINTF("ARR=%d CCR1=%d CCR2=%d CCR3=%d\r\n", TIM1->ARR, TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
PRINTF("CNT=%d\r\n", TIM1->CNT);
```

预期：ARR=5600, CCR=2800, CNT 在 0~5600 之间变化。  
用示波器看 PA8/PA9/PA10 应有互补 PWM。

### 第二步：确认 TIM2 被触发

```c
PRINTF("TIM2->CNT=%d TIM2->CCR2=%d\r\n", TIM2->CNT, TIM2->CCR2);
```

预期：CCR2=1, CNT 会在 0 附近跳动（被 TIM1 持续复位）。

### 第三步：确认 ADC 有数据

```c
PRINTF("ADC_Value: ");
for (int i = 0; i < 15; i++) PRINTF("%d ", ADC_Value[i]);
PRINTF("\r\n");
```

预期：各通道有非零非全满的值，Vrefint 应在 ~1200 左右（1.2V / 3.3V × 4095）。

### 第四步：确认 DMA 中断在跑

```c
// 在 Motor_DmaInt 入口计数
static uint32_t dma_int_count = 0;
dma_int_count++;
// 定时打印
PRINTF("DMA ints: %lu\r\n", dma_int_count);
```

预期：15kHz × (1/DIVIDER) 次/秒，即 DIVIDER=1 时约 15000 次/秒。

### 第五步：确认 ADC_Value[] 顺序正确

给几个通道加固定电压，检查对应索引的值是否与预期一致。重点关注 `ADC_IND_CURR1/2/3` 和 `ADC_IND_VIN_SENS`。

---

## 7. 关键宏定义速查

来源：[hw_hd75.h](App/Motor/inc/hw_hd75.h), [bsp_tim.h](Library/Bsp/inc/bsp_tim.h), [hw.h](App/Motor/inc/hw.h)

| 宏 | 值 | 说明 |
|----|-----|------|
| `SYSTEM_CORE_CLOCK` | 168000000 | 系统主频 |
| `HW_PWM1_TIM` | TIM1 | 电机1 PWM |
| `HW_TRIG_TIM` | TIM2 | ADC 触发 |
| `HW_COUNT_TIM` | TIM3 | 运行统计 |
| `HW_ADC_CHANNELS` | 15 | ADC 采样总路数 |
| `HW_ADC_NBR_CONV` | 5 | 每个 ADC 规则组 rank 数 |
| `HW_DEAD_TIME_NSEC` | 360.0f | 死区 360ns |
| `MCPWM_FOC_CURRENT_SAMP_OFFSET` | 2 | ADC 采样偏移 |
| `FOC_CONTROL_LOOP_FREQ_DIVIDER` | 1 | FOC 分频（1=每周期都算） |
| `V_REG` | 3.3f | ADC 参考电压 |
| `CURRENT_SHUNT_RES` | 0.0005f | 采样电阻 0.5mΩ |
| `CURRENT_AMP_GAIN` | 20.0f | 运放增益 |
| `FAC_CURRENT` | 0.0806 A/count | ADC→安培换算系数 |

---

## 8. 与其他两份文档的对比

| 维度 | DeepSeek 版 | Codex 版 | 本文（GLM 版） |
|------|------------|----------|---------------|
| 组织方式 | 按模块逐一展开 | 按信号流展开 | 按信号链路 + 寄存器级展开 |
| FOC vs BLDC 区分 | 未提及 | 提到 TIM8/注入组未启用 | **完整对比两条路径** |
| 时序分析 | 有完整时间线 | 有时间线 | **增加转换耗时估算 + 时间预算** |
| DMA 防误触 | 提到清标志 | 提到清标志 | **分析 6 步使能时序及原因** |
| 调试指南 | 无 | 有建议清单 | **增加分步验证代码** |
| 已知问题 | 7 条 | 4 条 | **12 条分级（红/黄/绿）** |
| 注入组分析 | 提到未使用 | 提到未使用 | **分析 BLDC 注入组配置作为参考** |

---

## 9. 源码索引

| 功能 | 文件 | 关键函数 |
|------|------|----------|
| TIM1 PWM | [bsp_tim.c](../../Library/Bsp/bsp_tim.c) | `Bsp_Tim_Pwm1()`, `Bsp_Tim1_Base()`, `Bsp_Tim1_OC()`, `Bsp_Tim1_BDTR()` |
| TIM2 触发 | [bsp_tim.c](../../Library/Bsp/bsp_tim.c) | `Bsp_Tim2_Pwm()`, `Bsp_Tim1_TrigAdc()`, `Bsp_Tim1_Trig()` |
| TIM8 PWM | [bsp_tim.c](../../Library/Bsp/bsp_tim.c) | `Bsp_Tim_Pwm2()` |
| TIM4 编码器 | [bsp_tim.c](../../Library/Bsp/bsp_tim.c) | `Bsp_Tim4_EncInit()` |
| TIM5 计时 | [bsp_tim.c](../../Library/Bsp/bsp_tim.c) | `Bsp_Tim5_Dt()`, `Bsp_Tim_DtNow()`, `Bsp_Tim_DtSecondElapsedSince()` |
| ADC 配置 | [bsp_adc.c](../../Library/Bsp/bsp_adc.c) | `Bsp_Adc_Sample()`, `Bsp_Adc123_Common()`, `Bsp_Adc123_Init()`, `Bsp_Adc_SetChannels()` |
| DMA 配置 | [bsp_dma.c](../../Library/Bsp/bsp_dma.c) | `Bsp_Dma2_Init()`, `Bsp_Dma2_NVIC()`, `Bsp_Dma_AdcSample()` |
| 中断处理 | [bsp_it.c](../../Library/Bsp/bsp_it.c) | `Bsp_Dma2Stream4_Handler()`, `Bsp_Usart_Handler()` |
| FOC 中断 | [motor_it.c](../../App/Motor/motor_it.c) | `Motor_DmaInt()`, `Motor_It_ParamCal()` |
| 硬件定义 | [hw_hd75.h](../../App/Motor/inc/hw_hd75.h) | `ADC_IND_*`, `HW_ADC_CHANNELS`, `GET_CURRENT*()` |
| BLDC TIM | [bsp_bldc_tim.c](../../Library/Bsp/bsp_bldc_tim.c) | `Bsp_Tim1_Bldc_Init()`, `Bsp_Tim8_Bldc_Init()` |
| BLDC ADC | [bsp_bldc_adc.c](../../Library/Bsp/bsp_bldc_adc.c) | `Bsp_Adc_Bldc_Init()`, `Bsp_Adc_Bldc_SetChannels()` |
| BLDC DMA | [bsp_bldc_dma.c](../../Library/Bsp/bsp_bldc_dma.c) | `Bsp_Dma_Bldc_Init()` |
