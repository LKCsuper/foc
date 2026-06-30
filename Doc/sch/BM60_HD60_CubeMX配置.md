# BM60/HD60 STM32CubeMX 配置建议

原理图文件：`BM60 HD60.pdf`

MCU：`STM32F405RGT6`，封装 `LQFP64`

## Clock

- RCC：启用 `HSE Crystal/Ceramic Resonator`
- 外部晶振：`8 MHz`
- 建议系统时钟：`168 MHz`
- 常用 PLL：`PLLM=8, PLLN=336, PLLP=2, PLLQ=7`
- APB1：`42 MHz`
- APB2：`84 MHz`
- USB 48 MHz：由 `PLLQ=7` 提供

## Motor PWM

使用 `TIM1`，三相互补 PWM：

| 信号 | MCU 引脚 | CubeMX 功能 |
| --- | --- | --- |
| INHA | PA8 | TIM1_CH1 |
| INHB | PA9 | TIM1_CH2 |
| INHC | PA10 | TIM1_CH3 |
| INLA | PB13 | TIM1_CH1N |
| INLB | PB14 | TIM1_CH2N |
| INLC | PB15 | TIM1_CH3N |

TIM1 建议：

- Mode：`PWM Generation CH1 CH1N CH2 CH2N CH3 CH3N`
- Counter Mode：`Center Aligned mode 1`
- Prescaler：`0`
- PWM 频率：FOC 常用 `20 kHz` 左右
- 计数周期参考：TIM1 时钟约 `168 MHz`，中心对齐时 `ARR = 168 MHz / (2 * PWM_FREQ)`，20 kHz 时约 `4200`
- Dead Time：先按功率管和驱动器保守设置，例如 `500 ns - 1000 ns`，再结合波形调整
- Break：如果暂时不用硬件刹车，可先不启用；软件用 `PB7 FAULT` 读取故障

## ADC

建议启用 `ADC1/ADC2/ADC3`，电流采样用注入转换并由 `TIM1` 触发；普通电压/温度采样用规则转换 + DMA。

| 信号 | MCU 引脚 | ADC 通道 | 用途 |
| --- | --- | --- | --- |
| SENS1 | PA0 | ADC123_IN0 | 相电压/反电动势 |
| SENS2 | PA1 | ADC123_IN1 | 相电压/反电动势 |
| SENS3 | PA2 | ADC123_IN2 | 相电压/反电动势 |
| TEMP | PA3 | ADC123_IN3 | 板载温度 |
| SCK_ADC_EXT | PA5 | ADC12_IN5 | 外部 ADC1 / 可复用 SPI1_SCK |
| MISO_ADC_EXT2 | PA6 | ADC12_IN6 | 外部 ADC2 / 可复用 SPI1_MISO |
| SOA | PC0 | ADC123_IN10 | A 相电流 INA240 输出 |
| SOB | PC1 | ADC123_IN11 | B 相电流 INA240 输出 |
| SOC | PC2 | ADC123_IN12 | C 相电流 INA240 输出 |
| SENS_V | PC3 | ADC123_IN13 | 母线电压 |
| M_TEMP | PC4 | ADC12_IN14 | 电机温度 |
| SHUTDOWN | PC5 | ADC12_IN15 | 关机/按键采样，也可作 GPIO 保持 |

注入电流采样建议：

- `ADC1 Injected`：`PC0 / IN10 / SOA`
- `ADC2 Injected`：`PC1 / IN11 / SOB`
- `ADC3 Injected`：`PC2 / IN12 / SOC`
- External Trigger：选择 `TIM1_TRGO` 或 `TIM1_CC4`，具体取决于后续代码采样点设计
- 采样时间：先用 `15 cycles` 或 `28 cycles` 起步，噪声大再加长

## GPIO

| 信号 | MCU 引脚 | CubeMX 配置 |
| --- | --- | --- |
| LED_GREEN | PB0 | GPIO_Output |
| LED_RED | PB1 | GPIO_Output |
| PB2 | PB2 | GPIO_Output/OpenDrain 或软件 I2C SDA，连接 BMI160 SDX |
| PA15 | PA15 | GPIO_Output/OpenDrain 或软件 I2C SCL，连接 BMI160 SCX |
| SERVO | PB6 | TIM4_CH1 input capture，或普通 GPIO |
| FAULT | PB7 | GPIO_Input，建议 Pull-Up |
| SHUTDOWN | PC5 | GPIO_Output 维持上电；若要采样按键则也接 ADC_IN15 |
| SENS_FILTER | PC13 | GPIO_Output |
| CURR_FILTER | PD2 | GPIO_Output |
| HALL_1 | PC6 | TIM3_CH1 或 GPIO_EXTI |
| HALL_2 | PC7 | TIM3_CH2 或 GPIO_EXTI |
| HALL_3 | PC8 | TIM3_CH3 或 GPIO_EXTI |
| BOOT0 | BOOT0 | 原理图下拉到 GND |
| NRST | NRST | 默认 |

## Communication

### USB FS

| 信号 | MCU 引脚 | CubeMX 功能 |
| --- | --- | --- |
| USB_DM | PA11 | USB_OTG_FS_DM |
| USB_DP | PA12 | USB_OTG_FS_DP |

启用 `USB_OTG_FS`，Device Only。需要 USB 通信时再启用 Middleware `USB_DEVICE`。

### CAN1

| 信号 | MCU 引脚 | CubeMX 功能 |
| --- | --- | --- |
| CAN_RX | PB8 | CAN1_RX |
| CAN_TX | PB9 | CAN1_TX |

CAN 管脚选择 remap 到 PB8/PB9。收发器是 TJA1042，板上有 120R 终端电阻。

### 串口 / 外设口

板子把 PB10/PB11 和 PC10/PC11 都引出来：

| 信号 | MCU 引脚 | 可选功能 |
| --- | --- | --- |
| TX_SCL_MOSI | PB10 | USART3_TX 或 I2C2_SCL |
| RX_SDA_NSS | PB11 | USART3_RX 或 I2C2_SDA / SPI1_NSS 软件片选 |
| PC10_TX | PC10 | UART4_TX 或 USART3_TX |
| PC11_RX | PC11 | UART4_RX 或 USART3_RX |

按现有工程习惯：

- 主通信口可用 `USART3`：PB10/PB11
- 永久调试/无线模块口可用 `UART4`：PC10/PC11

## BMI160

原理图中 BMI160 只接出两根控制线：

| BMI160 信号 | MCU 引脚 |
| --- | --- |
| SDX | PB2 |
| SCX | PA15 |

这两个脚不是 STM32F405 的硬件 I2C 标准组合，建议作为软件 I2C/GPIO bit-bang 使用。CubeMX 里先配成 `GPIO_Output Open-Drain + Pull-Up`，或普通 GPIO，后续驱动里手动翻转。

## SWD

| 信号 | MCU 引脚 |
| --- | --- |
| SWDIO | PA13 |
| SWCLK | PA14 |
| NRST | NRST |

CubeMX：`SYS -> Debug -> Serial Wire`。

## 注意项

- PA5/PA6/PA7 同时承担外部 ADC 口和 SPI1 口语义，不能一边当 ADC 一边当硬件 SPI。
- PB10/PB11 同时可作为 USART3、I2C2、外设口信号，CubeMX 里只能选择一种硬件复用；VESC 常见做法是按运行模式重配。
- PA15 是 JTAG 引脚，必须关闭 JTAG，仅保留 SWD，否则 BMI160 的 SCX 会被调试功能占用。
- PC5 既画成 `SHUTDOWN`，也在现有代码里作为上电保持脚；实际固件启动早期要尽快拉高，否则板子可能掉电。
