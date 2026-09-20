# TIM / ADC / DMA 初始化总结

对应源码：

| 模块 | 文件 |
|------|------|
| 定时器 | [Library/Bsp/bsp_tim.c](../../Library/Bsp/bsp_tim.c) |
| ADC | [Library/Bsp/bsp_adc.c](../../Library/Bsp/bsp_adc.c) |
| DMA | [Library/Bsp/bsp_dma.c](../../Library/Bsp/bsp_dma.c) |
| DMA 中断回调 | [Library/Bsp/bsp_it.c](../../Library/Bsp/bsp_it.c) |
| ISR 入口 | [App/user/stm32f4xx_it.c](../../App/user/stm32f4xx_it.c) |
| FOC 中断处理 | [App/Motor/motor_it.c](../../App/Motor/motor_it.c) |

---

## 1. 整体架构：电流采样的数据链路

```
 TIM1 更新事件          TIM2 输出比较            ADC1/2/3              DMA2 Stream4
 ────────────          ────────────            ─────────              ────────────

 每个PWM周期结束         收到触发后               收到触发后             自动搬运
 产生"更新事件"         计数器归零                三重同步转换            ADC->CDR
       │                 重新计数                    │                 → ADC_Value[]
       │                    │                       │                      │
       ├──ITR0触发────────→│                       │                      │
       │                    ├──CC2下降沿触发────────→│                      │
       │                    │                       ├──转换完成──────────→│
       │                    │                       │  (DMA自动搬运)       │
       │                    │                       │                      │
       │                    │                       │              DMA TC/HT中断
       │                    │                       │              → FOC电流环计算
```

**核心思路**：TIM1 的 PWM 更新事件作为节拍器 → 复位 TIM2 → TIM2 CC2 精确控制 ADC 在 PWM 周期中的采样时刻 → ADC 三重同步采样 → DMA 自动搬运到内存 → DMA 中断触发 FOC 计算。

---

## 2. 定时器初始化

### 2.1 定时器总览

| 定时器 | 类型 | 功能 | 初始化函数 |
|--------|------|------|-----------|
| TIM1 | 高级 | 电机1 PWM输出（三相互补） | `Bsp_Tim_Pwm1()` |
| TIM2 | 通用 | ADC采样触发（PWM输出模式） | `Bsp_Tim2_Pwm()` |
| TIM3 | 通用 | CPU运行时间统计 | `Bsp_Tim_RunCount()` |
| TIM4 | 通用 | 编码器接口（AB脉冲4倍频） | `Bsp_Tim4_EncInit()` |
| TIM5 | 通用 | 高精度计时器（µs级） | `Bsp_Tim5_Dt()` |
| TIM8 | 高级 | 电机2 PWM输出 | `Bsp_Tim_Pwm2()` |

### 2.2 TIM1 — 电机1 PWM 输出

**初始化流程**：`Bsp_Tim_Pwm1(f_zv)` → `Bsp_Tim1_Base()` + `Bsp_Tim1_OC()` + `Bsp_Tim1_BDTR()`

**时基参数（`Bsp_Tim1_Base`）**：
- **对齐模式**：中央对齐模式1（CenterAligned1），计数器先递增再递减
- **PWM 频率**：`f_pwm = f_zv / 2`，当前 `f_zv = 30000`，实际 PWM 频率 = 15kHz
- **预分频器**：0（不分频），定时器时钟 = 168MHz
- **自动重载值 ARR**：`168000000 / 30000 = 5600`
- **周期计算**：`168MHz / 5600 / 2 = 15kHz`（中央对齐模式下除2）

**输出比较（`Bsp_Tim1_OC`）**：
- PWM模式1：CNT < CCR 时输出有效电平
- 四个通道：CH1/CH2/CH3 → 三相电机 UVW 驱动（主输出 + 互补输出），CH4 → 预留
- 初始占空比 50%（Pulse = ARR / 2），上电默认半占空比避免电流冲击
- 输出极性通过 `INVERTED_TOP_DRIVER_INPUT` / `INVERTED_BOTTOM_DRIVER_INPUT` 适配驱动芯片
- 空闲状态（刹车/停机时）：输出 IdleState_Set（高电平关断 MOS）
- 预装载使能：CCR 写入影子寄存器，COM 事件后同步到实际输出

**保护功能（`Bsp_Tim1_BDTR`）**：
- **死区时间**：`calculate_deadtime(360ns, 168MHz)`，防止上下桥臂直通短路
- **刹车功能**：可选的硬件故障保护（`HW_USE_BRK`），外部低电平触发异步关闭全部 PWM
- **预装载控制**：CCR 预装载（COM事件更新）+ ARR 预装载（UEV更新）

**启动时序**：
1. 时基 + OC + BDTR 配置
2. CNT 清零，使能定时器
3. 立即调用 `Bsp_Tim_StopPwm(MOTOR1)` → 强制三相输出无效电平，MOS 全关
4. 使能 MOE 主输出（`TIM_CtrlPWMOutputs`）
5. 设置 ADC 采样偏移 `TIMER_UPDATE_SAMP(2)` → TIM2 CCR2 = 1

### 2.3 TIM2 — ADC 采样触发

**初始化**：`Bsp_Tim2_Pwm()` → `Bsp_Tim2_Base()` + `Bsp_Tim2_OC()` + `Bsp_Tim2_NVIC()`

**时基参数**：
- **计数模式**：向上计数
- **预分频器**：0，定时器时钟 = 84MHz（APB1 定时器时钟）
- **周期值**：0xFFFF（65535），满量程自由运行
- 注：TIM2 工作在 **Slave Mode Reset**，由 TIM1 更新事件触发复位，本身不会计满溢出

**主从触发链（`Bsp_Tim1_Trig`）**：
```c
// 单电机模式
TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);          // TIM1 更新事件 → TRGO
TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);                     // TIM2 输入 = TIM1 的 TRGO
TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);                // 收到触发 → 计数器归零
```

**关键宏**：
```c
#define TIMER_UPDATE_SAMP(samp)       TIM2->CCR2 = (samp / 2)
#define MCPWM_FOC_CURRENT_SAMP_OFFSET (2)
```
- `TIMER_UPDATE_SAMP(2)` → TIM2 CCR2 = 1，即 TIM1 更新事件后 1 个 TIM2 时钟周期触发 ADC
- 采样点位于 PWM 周期顶端（V7 矢量中点），避开开关噪声

### 2.4 TIM4 — 编码器接口

- 编码器模式 TI12（AB 两相4倍频）
- 周期 4095（1024×4-1），对应 1024 线编码器
- GPIOB6/7 复用为 TIM4 CH1/CH2

### 2.5 TIM5 — 高精度计时器

- 32位定时器，自由运行
- 计数频率：84MHz / 84 = 1MHz（每 tick = 1µs）
- API：`Bsp_Tim_DtNow()` 读当前值，`Bsp_Tim_DtSecondElapsedSince()` 计算差值

### 2.6 TIM8 — 电机2 PWM 输出

- 配置与 TIM1 基本相同
- 双电机模式（`HW_HAS_DUAL_MOTORS`）：TIM8 CNT 初始化为 TIM1 ARR，实现 180° 错相（interleaving），降低母线电容纹波
- 双电机并联模式（`HW_HAS_DUAL_PARALLEL`）：TIM8 跟随 TIM1 同步输出

---

## 3. ADC 初始化

### 3.1 总体配置

**入口**：`Bsp_Adc_Init()` → `Bsp_Adc_Sample()` + `Bsp_Adc_SetChannels()`

**工作模式**：三重规则同步模式（`ADC_TripleMode_RegSimult`）
- ADC1 作为主 ADC，接收外部触发（TIM2 CC2 下降沿）
- ADC2、ADC3 不单独接外部触发，跟随 ADC1 同步转换

**公共参数（`Bsp_Adc123_Common`）**：

| 参数 | 值 | 说明 |
|------|-----|------|
| ADC_Mode | `ADC_TripleMode_RegSimult` | 三重规则同步模式 |
| ADC_Prescaler | `ADC_Prescaler_Div2` | ADC 时钟 = APB2/2 = 42MHz（超频，手册上限36MHz） |
| ADC_DMAAccessMode | `ADC_DMAAccessMode_1` | 多 ADC 数据从 CDR 寄存器读取 |
| ADC_TwoSamplingDelay | `ADC_TwoSamplingDelay_5Cycles` | 采样间隔 5 个 ADC 周期 |

**单体参数（`Bsp_Adc123_Init`）**：

| 参数 | ADC1 | ADC2 | ADC3 |
|------|------|------|------|
| 分辨率 | 12bit | 12bit | 12bit |
| 扫描模式 | Enable | Enable | Enable |
| 连续转换 | Disable | Disable | Disable |
| 外部触发边沿 | 下降沿 | 无 | 无 |
| 触发源 | TIM2_CC2 | - | - |
| 数据对齐 | 右对齐 | 右对齐 | 右对齐 |
| 转换个数 | 5 | 5 | 5 |

### 3.2 规则通道配置

每个 ADC 配置 5 个规则转换序号，采样时间均为 `ADC_SampleTime_15Cycles`：

| 转换序号 | ADC1 | ADC2 | ADC3 |
|----------|------|------|------|
| 1 | CH0 | CH1 | CH2 |
| 2 | CH10 | CH11 | CH12 |
| 3 | CH5 | CH6 | CH3 |
| 4 | CH14 | CH15 | CH13 |
| 5 | Vrefint | CH0 | CH1 |

三重同步模式下，一次触发 ADC1/2/3 同时转换同一序号的通道，5 个序号共得到 **15 路**采样数据。

### 3.3 ADC_Value 索引定义

```c
ADC_Value[HW_ADC_CHANNELS]    // 共 15 个（三重同步 5 序号 × 3 ADC）

// 关键索引：
ADC_IND_SENS1     = 0    // ADC1 CH0  传感器1
ADC_IND_SENS2     = 1    // ADC2 CH1  传感器2
ADC_IND_SENS3     = 2    // ADC3 CH2  传感器3
ADC_IND_CURR1     = 3    // ADC1 CH10 相电流1
ADC_IND_CURR2     = 4    // ADC2 CH11 相电流2
ADC_IND_CURR3     = 5    // ADC3 CH12 相电流3
ADC_IND_VIN_SENS  = 11   // ADC2 CH15 输入电压检测
ADC_IND_VREFINT   = 12   // ADC1 Vrefint 内部参考电压
```

数据顺序由三重同步 DMA 排列规则决定：序号1 ADC1→ADC2→ADC3, 序号2 ADC1→ADC2→ADC3, ...

### 3.4 注入通道

ADC1/2/3 各配置 3 个注入通道（序号 1-3），通道与规则通道 CH10/CH11/CH12 相同。
注入通道可抢占规则通道转换，用于高优先级采样（如过流保护），当前 ISR 仅清除 JEOC 标志，未处理数据。

---

## 4. DMA 初始化

### 4.1 DMA2 Stream4 — ADC 采样数据搬运

**配置参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| 通道 | DMA_Channel_0 | ADC1 的 DMA 请求 |
| 外设地址 | `&ADC->CDR` | ADC 公共规则数据寄存器（固定） |
| 内存地址 | `&ADC_Value` | 采样结果数组（递增） |
| 方向 | 外设→内存 | ADC 读到数组 |
| 缓冲区大小 | 15（HW_ADC_CHANNELS） | 每轮搬运全部 15 路数据 |
| 数据宽度 | 半字（16bit） | ADC 12bit 转换结果 |
| 模式 | 循环（Circular） | 持续搬运，无需软件重配 |
| 优先级 | 高 | 确保 ADC 数据及时搬运 |
| FIFO | 禁用 | 直接模式 |

### 4.2 DMA2 Stream4 中断配置（`Bsp_Dma2_NVIC`）

```
NVIC:
  IRQ通道:    DMA2_Stream4_IRQn (中断号 60)
  抢占优先级: IRQ_DMA2_STREAM4 = 1
  子优先级:   0（最高）
```

**中断使能策略（编译期选择）**：

```c
// 条件：三个电流通道是否都在 ADC 转换序列的前 3 个位置
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
    DMA_ITConfig(DMA2_Stream4, DMA_IT_HT, ENABLE);  // ★ 半传输中断（HT）
#else
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);  // ★ 传输完成中断（TC）
#endif
```

当前三个电流通道索引为 3/4/5，**不满足** `< 3` 条件，使用 **传输完成中断（TC）**。

两种策略对比：
- **HT（半传输）**：DMA 搬运到第 7-8 个数据时（电流通道已到齐，共 15 路的一半），提前触发中断 → 更多 CPU 时间完成 FOC 计算再等 COM 事件更新占空比
- **TC（传输完成）**：等全部 15 路数据到齐再触发中断 → 延迟稍大但实现简单

### 4.3 DMA 中断回调链（完整 ISR 调用路径）

```
硬件中断 DMA2_Stream4_IRQn (中断号 60)
    │
    ▼
DMA2_Stream4_IRQHandler()          ← [App/user/stm32f4xx_it.c:132]
    │  NVIC 向量表 → 进入 ISR
    │
    ▼
Bsp_Dma2Stream4_Handler()          ← [Library/Bsp/bsp_it.c:81]
    │
    │  根据编译期条件，检查并清除对应中断标志：
    │
    ├── HT 模式 (ADC_IND_CURRx 全 < 3):
    │   if (DMA_GetITStatus(DMA2_Stream4, DMA_IT_HTIF4))
    │       DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_HTIF4)
    │       → Motor_DmaInt(NULL, NULL)
    │
    └── TC 模式 (当前):
        if (DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4))
            DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4)
            → Motor_DmaInt(NULL, NULL)
    │
    ▼
Motor_DmaInt(NULL, NULL)           ← [App/Motor/motor_it.c:80]
    │
    │  1. FOC 分频控制：
    │     static int skip = 0;
    │     if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) skip = 0;
    │     else return;  // 跳过一次 ADC 完成中断，FOC 运行频率 = PWM频率 / DIVIDER
    │
    │  2. Motor_It_ParamCal() → 从 ADC_Value[] 读取三相电流、母线电压
    │
    │  3. Clarke 变换：ia, ib, ic → Iα, Iβ
    │
    │  4. Park 变换：Iα, Iβ → Id, Iq（需要当前转子角度 θ）
    │
    │  5. PID 控制环：Id/Iq 误差 → Vd/Vq
    │
    │  6. 逆 Park 变换：Vd, Vq → Vα, Vβ
    │
    │  7. SVPWM 计算 → 三相占空比 TIM1 CCR1/CCR2/CCR3
    │
    │  8. TIM_GenerateEvent(TIM1, TIM_EventSource_COM)
    │     → COM 事件将影子寄存器中的新 CCR 值同步到实际 PWM 输出
    │
    └── FOC 计算完成，等待下一次 DMA 中断
```

### 4.4 DMA1 Stream1 — 串口接收

| 参数 | 值 | 说明 |
|------|-----|------|
| 通道 | DMA_Channel_4 | USART1_RX |
| 外设地址 | `&DEBUG_USART->DR` | 串口数据寄存器（固定） |
| 内存地址 | `gucURxBuf` | 接收缓冲区 |
| 缓冲区大小 | USART_RX_BUF_SIZE | 32 字节 |
| 数据宽度 | 字节（8bit） | |
| 模式 | 循环 | 必须循环模式，否则接收一包后停止 |
| 优先级 | 高 | |

**串口接收中断**：使用 USART IDLE 中断检测帧尾（而非 DMA 中断），在 `Bsp_Usart_Handler()` 中：[bsp_it.c:46](Library/Bsp/bsp_it.c#L46)
1. 检测 USART IDLE 标志（一帧接收完毕，总线空闲）
2. 关闭 DMA，计算本帧长度 = 缓冲区大小 - DMA 剩余计数
3. 通过 FreeRTOS 队列发送长度给命令行解析任务
4. 清除 DMA 标志，重置计数器，重新使能 DMA

---

## 5. 初始化顺序

```c
// bsp.c -> Bsp_Init()
Bsp_Gpio_Init();              // 1. GPIO（ADC引脚配置为模拟输入）
Bsp_Usart_DebugInit(115200);  // 2. 串口调试输出
Bsp_Adc_Init();               // 3. ADC1/2/3 配置（含通道）
Bsp_Dma_Init();               // 4. DMA2 Stream4 配置 + 中断使能
Bsp_Tim_Init();               // 5. TIM1→TIM2 启动触发链 → 整个采样链路开始运行
PRINTF("Hardware Init Success\r\n");
```

**顺序约束**：
- GPIO 必须最先，ADC 引脚需先配置为模拟输入
- ADC 在 DMA 之前：ADC 外设先配好，DMA 才能正确响应 ADC 请求
- DMA 在 TIM 之前：DMA 中断配置就绪后再启动定时器触发链，避免 TIM 触发 ADC 时 DMA 未就绪导致数据丢失
- `PRINTF` 放在最后：实测如果在 TIM 和 DMA 初始化之间不加打印，串口有时会死锁

---

## 6. 完整的数据流时序

```
每个 PWM 周期（15kHz, ~66.7µs）：

Timeline:
├─ TIM1 CNT=0：计数器开始向上计数
├─ TIM1 CNT=CCR：上下桥臂切换（死区插入）
├─ TIM1 CNT=ARR（顶端）：TIM1 产生更新事件 UEV
│   ├─ TIM1 更新事件 → TRGO → ITR0 → TIM2
│   └─ TIM2 收到触发 → 计数器归零（SlaveMode_Reset）
│
├─ TIM2 CNT=1（= CCR2）：TIM2 CC2 下降沿 → 触发 ADC1
│   ├─ ADC1/2/3 同步采样 序号1：CH0/CH1/CH2
│   ├─ ADC1/2/3 同步采样 序号2：CH10/CH11/CH12  ← 含三相电流
│   ├─ ADC1/2/3 同步采样 序号3：CH5/CH6/CH3
│   ├─ ADC1/2/3 同步采样 序号4：CH14/CH15/CH13
│   └─ ADC1/2/3 同步采样 序号5：Vrefint/CH0/CH1
│
├─ 全部 5 序号 × 3 ADC = 15 路转换完成
│   └─ DMA2 Stream4 搬运 ADC->CDR → ADC_Value[0..14]
│       └─ TC 中断 → Motor_DmaInt()
│           ├─ FOC 分频判断（skip 计数器）
│           ├─ Clarke/Park 变换
│           ├─ PID 控制
│           ├─ 逆 Park / SVPWM → 更新 TIM1 CCR
│           └─ TIM_GenerateEvent(COM) → 影子寄存器同步
│
├─ TIM1 CNT=CCR：PWM 切换（新占空比生效，已从影子寄存器同步）
└─ TIM1 CNT=0：计数器归零，下一周期开始
```

---

## 7. 关键宏速查

| 宏 | 值 | 说明 |
|----|-----|------|
| `SYSTEM_CORE_CLOCK` | 168000000 | 系统时钟 168MHz |
| `HW_PWM1_TIM` | TIM1 | 电机1 PWM 定时器 |
| `HW_TRIG_TIM` | TIM2 | ADC 触发定时器 |
| `HW_COUNT_TIM` | TIM3 | 运行时间统计定时器 |
| `HW_DEAD_TIME_NSEC` | 360.0f | 死区时间 360ns |
| `HW_ADC_CHANNELS` | 15 | ADC 通道总数 |
| `HW_ADC_NBR_CONV` | 5 | 每个 ADC 转换序号数 |
| `HW_ADC_CHANNELS_EXTRA` | 0 | 额外通道 |
| `MCPWM_FOC_CURRENT_SAMP_OFFSET` | 2 | 采样点偏移（TIM2 ticks） |
| `CURRENT_AMP_GAIN` | 20.0f | 电流采样运放增益 |
| `CURRENT_SHUNT_RES` | 0.0005f | 采样电阻 0.5mΩ |
| `V_REG` | 3.3f | ADC 参考电压 3.3V |
| `FAC_CURRENT` | `(3.3/4095)/(0.0005*20)` | ADC 读数→安培转换系数 |

---

## 8. 注意事项

1. **ADC 时钟超频**：ADC 时钟 = APB2/2 = 42MHz，手册上限 36MHz，当前运行可行但不合规。
2. **`Bsp_Adc_Init()` 重复配置**：末尾又调用了一次 `Bsp_Adc_SetChannels()`，功能无影响但建议清理。
3. **TIM2 CC2 初始值被覆盖**：OC 初始化时 `Pulse = 250`，但 `TIMER_UPDATE_SAMP(2)` 在 TIM1 初始化末尾将其改写为 1。
4. **DMA 中断类型**：当前三路电流通道索引为 3/4/5，使用 TC 中断。如果将电流通道调整到前 3 个位置，编译器会自动切换到 HT 中断以提前 FOC 计算。
5. **FOC 分频**：`Motor_DmaInt()` 内部通过 `FOC_CONTROL_LOOP_FREQ_DIVIDER` 跳帧，实际 FOC 控制频率 = PWM 频率 / DIVIDER。
6. **刹车后恢复**：硬件刹车触发后 PWM 输出保持关闭直到 MCU 复位，无法软件恢复。
7. **串口初始化敏感**：TIM 和 DMA 初始化之间若不加打印，串口可能死锁（已在 `Bsp_Init` 末尾加 `PRINTF` 规避）。
