# ADC 初始化笔记

对应源码：`Library/Bsp/bsp_adc.c`

## 1. 初始化入口

ADC 的对外初始化入口是：

```c
VOID Bsp_Adc_Init(VOID)
{
    Bsp_Adc_Sample();
    Bsp_Adc_SetChannels();
}
```

实际主要工作在 `Bsp_Adc_Sample()` 中完成：

1. 配置 ADC1/ADC2/ADC3 公共参数。
2. 分别初始化 ADC1、ADC2、ADC3。
3. 使能内部温度传感器和 Vrefint。
4. 使能多 ADC 模式下 DMA 请求。
5. 配置规则通道和注入通道。
6. 使能 ADC1、ADC2、ADC3。

`Bsp_Adc_Init()` 末尾又调用了一次 `Bsp_Adc_SetChannels()`，目前看属于重复配置，功能上影响不大，但不是必须步骤。

## 2. 公共配置

公共配置函数是 `Bsp_Adc123_Common()`。

```c
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |
                       RCC_APB2Periph_ADC2 |
                       RCC_APB2Periph_ADC3, ENABLE);
```

先打开 ADC1、ADC2、ADC3 外设时钟。

随后通过 `ADC_CommonInit()` 配置三个 ADC 的公共工作方式：

```c
ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
```

含义：

- `ADC_TripleMode_RegSimult`：三重规则同步模式，ADC1、ADC2、ADC3 同步转换规则通道。
- `ADC_Prescaler_Div2`：ADC 时钟为 APB2 时钟 2 分频。
- `ADC_DMAAccessMode_1`：多 ADC 模式 DMA 访问方式 1，转换结果从公共数据寄存器 `ADC->CDR` 取出。
- `ADC_TwoSamplingDelay_5Cycles`：两个采样阶段之间延迟 5 个 ADC 周期。

## 3. ADC1/2/3 单体配置

单体初始化函数是 `Bsp_Adc123_Init()`。

三个 ADC 大部分参数一致：

```c
ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
ADC_InitStructure.ADC_ScanConvMode = ENABLE;
ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;
```

含义：

- 12 位分辨率。
- 开启扫描模式，用于多通道转换。
- 关闭连续转换，由外部事件触发。
- 数据右对齐。
- 每个 ADC 的规则序列长度为 `HW_ADC_NBR_CONV`，当前硬件配置中是 5。

ADC1 额外配置了外部触发：

```c
ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
ADC_Init(ADC1, &ADC_InitStructure);
```

也就是 ADC1 由 `TIM2_CC2` 下降沿触发。由于当前是三重规则同步模式，ADC1 作为主 ADC 触发后，ADC2、ADC3 同步跟随。

ADC2、ADC3 不直接配置外部触发：

```c
ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
ADC_InitStructure.ADC_ExternalTrigConv = 0;
ADC_Init(ADC2, &ADC_InitStructure);
ADC_Init(ADC3, &ADC_InitStructure);
```

## 4. 规则通道配置

规则通道在 `Bsp_Adc_SetChannels()` 中配置。

每个 ADC 配置 5 个规则转换序号，采样时间均为 `ADC_SampleTime_15Cycles`。

| 转换序号 | ADC1 | ADC2 | ADC3 |
| --- | --- | --- | --- |
| 1 | CH0 | CH1 | CH2 |
| 2 | CH10 | CH11 | CH12 |
| 3 | CH5 | CH6 | CH3 |
| 4 | CH14 | CH15 | CH13 |
| 5 | Vrefint | CH0 | CH1 |

三重同步模式下，一次触发会让 ADC1/2/3 同时转换同一序号的通道。因此 5 个转换序号一共得到 15 路规则采样数据，对应：

```c
volatile uint16_t ADC_Value[HW_ADC_CHANNELS + HW_ADC_CHANNELS_EXTRA];
```

当前 `HW_ADC_CHANNELS` 为 15。

## 5. 注入通道配置

文件里还配置了注入通道：

```c
ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
```

后面序号 2、3 也重复配置了相同通道：

- ADC1 注入通道：CH10
- ADC2 注入通道：CH11
- ADC3 注入通道：CH12

注入通道通常用于需要更高优先级、更确定时刻的采样，例如电机相电流采样。当前 `ADC_IRQHandler()` 只清除了 `ADC1` 的 `ADC_IT_JEOC` 标志，未在中断里处理采样数据。

## 6. DMA 配合关系

ADC 初始化本身没有配置 DMA，DMA 在 `Library/Bsp/bsp_dma.c` 中完成。

ADC DMA 使用：

```c
DMA2_Stream4
DMA_Channel_0
DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR
DMA_Memory0BaseAddr = (uint32_t)&ADC_Value
DMA_BufferSize = HW_ADC_CHANNELS
DMA_Mode = DMA_Mode_Circular
```

关键点：

- 多 ADC 规则同步转换结果从公共数据寄存器 `ADC->CDR` 读取。
- DMA 目标地址是 `ADC_Value`。
- 循环模式持续搬运采样数据。
- 当前 DMA buffer 长度是 15，对应三路 ADC 各 5 个规则通道。

ADC 侧通过下面这句允许多 ADC 模式 DMA 在最后一次传输后继续请求：

```c
ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
```

## 7. TIM 触发关系

ADC1 的规则转换触发源是：

```c
ADC_ExternalTrigConv_T2_CC2
ADC_ExternalTrigConvEdge_Falling
```

定时器相关初始化在 `Library/Bsp/bsp_tim.c`：

1. `Bsp_Tim_Pwm1(30000)` 初始化 TIM1 PWM。
2. `Bsp_Tim1_TrigAdc()` 初始化 TIM2 PWM 输出，用 TIM2 的比较事件触发 ADC。
3. `Bsp_Tim1_Trig()` 将 TIM1 和 TIM2 串起来。

普通单电机配置下：

```c
TIM_SelectOutputTrigger(HW_PWM1_TIM, TIM_TRGOSource_Update);
TIM_SelectMasterSlaveMode(HW_PWM1_TIM, TIM_MasterSlaveMode_Enable);
TIM_SelectInputTrigger(HW_TRIG_TIM, TIM_TS_ITR0);
TIM_SelectSlaveMode(HW_TRIG_TIM, TIM_SlaveMode_Reset);
```

也就是 TIM1 更新事件作为主触发源，复位 TIM2 计数；TIM2 的 CC2 事件再触发 ADC1，ADC1 带动 ADC2/ADC3 同步采样。

## 8. 整体初始化顺序

在 `Bsp_Init()` 中当前顺序是：

```c
Bsp_Gpio_Init();
Bsp_Usart_DebugInit(115200);
Bsp_Adc_Init();
Bsp_Dma_Init();
Bsp_Tim_Init();
```

ADC 相关链路可以理解为：

```text
GPIO 模拟输入准备
    -> ADC1/2/3 配置
    -> DMA2_Stream4 配置到 ADC_Value
    -> TIM1/TIM2 配置
    -> TIM2_CC2 触发 ADC1
    -> ADC1/2/3 三重同步采样
    -> ADC->CDR 经 DMA 搬运到 ADC_Value
    -> DMA 半传输或传输完成中断进入 FOC 处理
```

## 9. 注意点

- ADC1 是触发主 ADC，ADC2/ADC3 不单独接外部触发。
- 当前使用三重规则同步模式，通道表必须按 ADC1/2/3 同一转换序号横向理解。
- `ADC_Value` 的顺序和硬件宏 `ADC_IND_*` 强相关，改通道时要同时检查 `App/Motor/inc/hw_hd75.h`。
- ADC 初始化依赖 GPIO 已经设置为模拟输入模式，相关配置不在 `bsp_adc.c` 内。
- ADC 采样真正跑起来依赖 DMA 和 TIM 初始化，单独调用 `Bsp_Adc_Init()` 只会完成 ADC 外设配置。
- `Bsp_Adc_Init()` 中重复调用 `Bsp_Adc_SetChannels()`，后续可考虑去掉一次，避免误导阅读。
