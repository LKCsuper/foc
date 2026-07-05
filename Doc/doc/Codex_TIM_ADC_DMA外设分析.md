# Codex 版 TIM / ADC / DMA 外设运行图

这份文档不只讲“初始化了什么”，而是把当前工程里的外设当成一条流水线来看：

```text
TIM1 像节拍器，决定 PWM 周期
TIM2 像闹钟，在 PWM 周期里的指定位置响一下
ADC1/2/3 像三台同步相机，同时拍下电流、电压、温度
DMA 像搬运工，把照片自动搬到 ADC_Value[]
DMA 中断像门铃，提醒 FOC 可以开始算下一拍占空比
```

当前工程真正跑起来的主链路是：

```text
TIM1 Update
    |
    | TRGO
    v
TIM2 Reset
    |
    | CNT 到 CCR2
    v
TIM2_CC2 falling edge
    |
    v
ADC1 外部触发
    |
    v
ADC1 / ADC2 / ADC3 三重规则同步采样
    |
    v
ADC->CDR
    |
    v
DMA2 Stream4 -> ADC_Value[0..14]
    |
    v
DMA TC 中断
    |
    v
Motor_DmaInt()
    |
    v
foc_simulink0_step()
    |
    v
TIM1 CCR1/CCR2/CCR3 更新三相 PWM
```

---

## 1. 一眼看懂当前外设分工

| 外设 | 当前角色 | 是否在主链路里 |
| --- | --- | --- |
| TIM1 | 三相互补 PWM 主定时器 | 是 |
| TIM2 | ADC 采样触发定时器 | 是 |
| ADC1 | 主 ADC，接收 TIM2_CC2 外部触发 | 是 |
| ADC2 / ADC3 | 跟随 ADC1 同步采样 | 是 |
| DMA2 Stream4 | 搬运 ADC->CDR 到 ADC_Value[] | 是 |
| USART3 + DMA1 Stream1 | 串口命令接收 | 是，但不属于 FOC 主链路 |
| TIM5 | 运行时间/代码耗时统计 | 已初始化，但时间换算有问题 |
| TIM4 | 编码器接口代码存在 | 当前 `Bsp_Tim_Init()` 未启用 |
| TIM8 | 第二电机 PWM 代码存在 | 当前 `Bsp_Tim_Init()` 未启用 |
| ADC 注入组 | 配了电流通道 | 当前没有真正参与 FOC |

这个表是我和 DeepSeek 文档最大的区别：它把很多“代码存在的能力”也写进了运行图里；我这里按你当前工程初始化路径区分“正在跑”和“预留/未启用”。

---

## 2. 上电初始化顺序

入口在 `Bsp_Init()`：

```c
Bsp_Gpio_Init();
Bsp_Usart_DebugInit(115200);
Bsp_Adc_Init();
Bsp_Dma_Init();
Bsp_Tim_Init();
PRINTF("Hardware Init Success\r\n");
```

这条顺序像是在搭舞台：

1. **GPIO 先铺线**：PWM 引脚复用，ADC 引脚设为模拟输入，USART 引脚复用。
2. **USART 先开口**：方便后面打印和命令交互。
3. **ADC 先架好相机**：配置三 ADC 同步采样序列。
4. **DMA 先安排搬运工**：准备好从 `ADC->CDR` 搬到 `ADC_Value[]`。
5. **TIM 最后敲节拍**：一旦 TIM1/TIM2 跑起来，采样链路就开始工作。

这个顺序很关键。TIM 如果太早启动，ADC 可能已经开始转换，而 DMA 还没准备好。

---

## 3. TIM1：FOC 的节拍器

`Bsp_Tim_Init()` 里调用：

```c
Bsp_Tim_Pwm1(30000);
```

TIM1 配置：

```c
TIM_Prescaler = 0;
TIM_CounterMode = TIM_CounterMode_CenterAligned1;
TIM_Period = SYSTEM_CORE_CLOCK / f_zv;
```

当前数值：

```text
SYSTEM_CORE_CLOCK = 168 MHz
f_zv = 30000
TIM1->ARR = 168000000 / 30000 = 5600
中心对齐 PWM 频率 = 168 MHz / 5600 / 2 = 15 kHz
```

为什么要除以 2？

因为中心对齐模式下，计数器是：

```text
0 -> ARR -> 0
```

一次完整 PWM 周期要走上坡和下坡两段。

TIM1 输出的是三相互补 PWM：

| 功能 | GPIO | TIM 通道 |
| --- | --- | --- |
| U/V/W 上桥 | PA8 / PA9 / PA10 | TIM1 CH1/CH2/CH3 |
| U/V/W 下桥 | PB13 / PB14 / PB15 | TIM1 CH1N/CH2N/CH3N |

同时还配置了：

- 死区：`HW_DEAD_TIME_NSEC = 360 ns`
- 主输出 MOE
- CCR 预装载
- COM 事件同步更新

初始化后代码立刻调用 `Bsp_Tim_StopPwm(MOTOR1)`，所以不是一上电就驱动电机，而是先强制三相输出无效状态。后面算法准备好后，再通过 `Bsp_Tim_StartPwm()` 恢复 PWM1 模式。

---

## 4. TIM2：藏在 TIM1 后面的采样闹钟

ADC 没有直接被 TIM1 触发，而是中间夹了 TIM2。

这很有意思：TIM1 负责大节拍，TIM2 负责在这个节拍里精确找采样点。

单电机模式下触发关系是：

```c
TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
```

于是每个 TIM1 Update 事件都会让 TIM2 归零：

```text
TIM1 到达 Update 点
    |
    v
TIM2 CNT = 0
    |
    v
TIM2 从 0 开始数
    |
    v
TIM2 CNT == CCR2 时触发 ADC
```

当前采样偏移：

```c
#define MCPWM_FOC_CURRENT_SAMP_OFFSET (2)
#define TIMER_UPDATE_SAMP(samp) TIM2->CCR2 = (samp / 2)
```

所以：

```text
TIM2->CCR2 = 1
```

也就是说：**TIM1 Update 后非常短的时间内就触发 ADC**。

这是不是最佳采样点？代码层面只能说“它是当前采样点”。真正是否避开开关噪声，要看功率板波形。建议后续用示波器同时看：

- TIM1 PWM
- TIM2 CC2 或 ADC 触发时刻
- 三相电流采样运放输出

---

## 5. ADC1/2/3：三台同步相机

ADC 公共配置：

```c
ADC_Mode = ADC_TripleMode_RegSimult;
ADC_Prescaler = ADC_Prescaler_Div2;
ADC_DMAAccessMode = ADC_DMAAccessMode_1;
ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
```

这表示：

- ADC1 是主 ADC。
- ADC1 接收 `TIM2_CC2` 下降沿触发。
- ADC2/ADC3 不接外部触发，只跟随 ADC1。
- 三个 ADC 按同一个 rank 同步转换。
- 多 ADC 数据从公共寄存器 `ADC->CDR` 交给 DMA。

ADC1 的触发配置：

```c
ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
```

ADC2/ADC3：

```c
ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
ADC_ExternalTrigConv = 0;
```

这说明当前触发入口只有 ADC1。

---

## 6. ADC_Value[] 地图

当前每个 ADC 有 5 个规则转换 rank。三重同步后，一轮触发得到 15 个采样值。

```text
一次 TIM2_CC2 触发
    |
    +-- Rank 1: ADC1 CH0    ADC2 CH1    ADC3 CH2
    +-- Rank 2: ADC1 CH10   ADC2 CH11   ADC3 CH12
    +-- Rank 3: ADC1 CH5    ADC2 CH6    ADC3 CH3
    +-- Rank 4: ADC1 CH14   ADC2 CH15   ADC3 CH13
    +-- Rank 5: ADC1 Vref   ADC2 CH0    ADC3 CH1
```

对应 `ADC_Value[]`：

| 索引 | 来源 | 宏 | 含义 |
| --- | --- | --- | --- |
| 0 | ADC1 CH0 | `ADC_IND_SENS1` | 相电压 SENS1 |
| 1 | ADC2 CH1 | `ADC_IND_SENS2` | 相电压 SENS2 |
| 2 | ADC3 CH2 | `ADC_IND_SENS3` | 相电压 SENS3 |
| 3 | ADC1 CH10 | `ADC_IND_CURR1` | 相电流 1 |
| 4 | ADC2 CH11 | `ADC_IND_CURR2` | 相电流 2 |
| 5 | ADC3 CH12 | `ADC_IND_CURR3` | 相电流 3 |
| 6 | ADC1 CH5 | `ADC_IND_EXT` | 外部 ADC1 |
| 7 | ADC2 CH6 | `ADC_IND_EXT2` | 外部 ADC2 |
| 8 | ADC3 CH3 | `ADC_IND_TEMP_MOS` | MOS/PCB 温度 |
| 9 | ADC1 CH14 | `ADC_IND_TEMP_MOTOR` | 电机温度 |
| 10 | ADC2 CH15 | `ADC_IND_SHUTDOWN` | Shutdown |
| 11 | ADC3 CH13 | `ADC_IND_VIN_SENS` | 母线电压采样 |
| 12 | ADC1 Vrefint | `ADC_IND_VREFINT` | 内部参考电压 |
| 13 | ADC2 CH0 | - | SENS1 重复采样 |
| 14 | ADC3 CH1 | - | SENS2 重复采样 |

这里有个小心点：`ADC_DMAAccessMode_1` 的库注释是 “half-words one by one - 1 then 2 then 3”，当前索引理解是合理的。但多 ADC + CDR 容易被误读，建议调试时用固定电压实测确认一次 `ADC_Value[]` 排列。

---

## 7. DMA2 Stream4：自动搬运工

DMA 配置：

```c
DMA2_Stream4
DMA_Channel_0
PeripheralBaseAddr = &ADC->CDR
Memory0BaseAddr = &ADC_Value
DMA_BufferSize = HW_ADC_CHANNELS = 15
DMA_PeripheralDataSize = HalfWord
DMA_MemoryDataSize = HalfWord
DMA_Mode = Circular
```

画成数据流：

```text
ADC->CDR
   |
   | DMA2 Stream4, Channel 0
   v
ADC_Value[0]
ADC_Value[1]
ADC_Value[2]
...
ADC_Value[14]
```

循环模式意味着：每次 ADC 触发完成一轮 15 个值，DMA 就把数组刷新一遍，不需要 CPU 手动读 ADC 数据寄存器。

---

## 8. DMA 中断：FOC 的开算信号

DMA 中断选择逻辑：

```c
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
    DMA_IT_HT
#else
    DMA_IT_TC
#endif
```

当前：

```text
ADC_IND_CURR1 = 3
ADC_IND_CURR2 = 4
ADC_IND_CURR3 = 5
```

所以走的是 **TC 传输完成中断**。

也就是：

```text
等 ADC_Value[0..14] 全部搬完
    |
    v
DMA2_Stream4_IRQHandler()
    |
    v
Bsp_Dma2Stream4_Handler()
    |
    v
Motor_DmaInt()
```

如果以后把三相电流排到 `ADC_Value[0..2]`，代码会自动切到 HT 半传输中断，让 FOC 更早开始算，给本周期更新 PWM 留更多时间。

---

## 9. FOC 中断里发生的事

`Motor_DmaInt()` 的第一层是分频：

```c
static int skip = 0;
if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) {
    skip = 0;
} else {
    return;
}
```

然后读取采样值：

```c
currentAdc[0] = GET_CURRENT1();
currentAdc[1] = GET_CURRENT2();
currentAdc[2] = GET_CURRENT3();
```

电流换算：

```c
FAC_CURRENT = (V_REG / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)
```

当前硬件参数：

```text
V_REG = 3.3 V
CURRENT_SHUNT_RES = 0.0005 ohm
CURRENT_AMP_GAIN = 20
FAC_CURRENT ~= 0.0806 A/count
```

当前代码的电流零点是硬编码：

```c
ia = (GET_CURRENT1() - 2045) * FAC_CURRENT;
ib = (GET_CURRENT2() - 2045) * FAC_CURRENT;
ic = (GET_CURRENT3() - 2045) * FAC_CURRENT;
```

这适合先跑通，但不适合长期依赖。更稳的方式是上电时采集零电流偏置，写入 `currentOffset[]`。

当前主算法路径是 Simulink 生成代码：

```c
rtU.Tpwm = 5600;
rtU.Udc = stMotorNow->v_bus;
rtU.Theta = rtY.Angle;
rtU.we = rtY.Speed;
rtU.Iq_ref = 0.45;
rtU.Id_ref = 0;
rtU.Ia = ia;
rtU.Ib = ib;
rtU.Ic = ic;

foc_simulink0_step();
TIMER_UPDATE_DUTY_M1(rtY.Tcm1, rtY.Tcm2, rtY.Tcm3);
```

注意：`rtU.Tpwm = 5600` 是写死的，刚好等于当前 TIM1 ARR。如果以后改 PWM 频率，这里也要同步改，或者改成读取 `TIM1->ARR`。

---

## 10. 一个 PWM 周期里的时间线

按当前配置，整体节拍大概是：

```text
TIM1 中心对齐 PWM, ARR = 5600, 频率约 15 kHz

        TIM1 CNT = ARR 附近
              |
              | Update event
              v
        TIM1 TRGO 输出
              |
              v
        TIM2 被 Reset, CNT = 0
              |
              | TIM2 CNT 到 1
              v
        TIM2 CC2 触发 ADC1
              |
              v
        ADC1/2/3 开始同步扫描 5 个 rank
              |
              v
        DMA 搬运 15 个 half-word 到 ADC_Value[]
              |
              v
        DMA TC 中断
              |
              v
        FOC 计算
              |
              v
        更新 TIM1 CCR1/2/3
              |
              v
        等待下一次 PWM 同步生效
```

这条时间线就是你电流环的“心跳”。

---

## 11. DeepSeek 文档里需要修正或补充的地方

### 11.1 TIM8 当前不是主链路

代码里确实有 `Bsp_Tim_Pwm2()`，但当前 `Bsp_Tim_Init()` 没有调用它。

所以 TIM8 是“预留/代码存在”，不是“当前正在驱动第二电机”。

### 11.2 注入 ADC 当前没有真正用

注入通道配置了三相电流，但 `ADC_IRQHandler()` 只是清 `JEOC`，没有读取注入数据，也没有参与 FOC。

当前 FOC 数据来自规则组 + DMA。

### 11.3 TIM5 时间换算不一致

TIM5 实际：

```text
Prescaler = 84 - 1
计数频率 = 1 MHz
```

但函数用：

```c
#define TIMER_DT_HZ 1.4e7
```

所以时间会算小 14 倍。

### 11.4 ADC 时钟偏高

`ADC_Prescaler_Div2` 在 APB2 = 84 MHz 时得到 42 MHz ADC 时钟。这个频率偏激进，建议后续评估是否改为 `ADC_Prescaler_Div4`。

---

## 12. 建议优先调试清单

1. **确认 ADC_Value[] 顺序**

   给几个 ADC 通道固定电压，看数组索引是否和 `hw_hd75.h` 一致。

2. **确认 ADC 采样点**

   用示波器看 TIM1 PWM、TIM2 CC2、运放输出，确认 `CCR2 = 1` 是否太靠近开关瞬间。

3. **做电流零偏校准**

   不要长期用 `2045`，建议 PWM 未输出时多次平均三相电流 ADC。

4. **修 TIM5 时间基准**

   如果 TIM5 是 1 us/tick，就把 `TIMER_DT_HZ` 改成 `1000000.0f`。

5. **决定 ADC 时钟是否降频**

   当前 42 MHz 能跑不代表稳，电机控制这种强噪声环境里保守一点通常更舒服。

---

## 13. 最后用一句话记住

当前工程的 FOC 外设闭环是：

```text
TIM1 定 PWM 节拍，
TIM2 定采样时刻，
ADC1/2/3 同时拍三组模拟量，
DMA 自动把数据塞进 ADC_Value[]，
DMA 中断叫醒 FOC，
FOC 算完再写回 TIM1 占空比。
```

这条链路跑通了，电机控制才有稳定的“采样-计算-输出”闭环。
