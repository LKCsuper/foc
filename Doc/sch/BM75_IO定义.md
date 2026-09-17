# BM75 / HD75 IO 定义

> 更新日期：2026-09-08  
> 原稿：`Doc/sch/BM75_Core.pdf`（控制/采样/通信）、`Doc/sch/BM75_MOS.pdf`（桥臂与分流器）  
> 固件板级宏：`App/motor/inc/hw_hd75.h`（`HW_NAME "HD75"`）  
> 本文以原理图网络名为准。改引脚或采样通道前先对本文和 PDF，不要从 BM60 文档或未使用的 DRV8323 宏推断。

当前功率板是 **BM75**（工程里也叫 HD75）：STM32F405RGT6（LQFP64）+ **FD6288Q** 三相门极驱动 + 三颗 **INA240A1** 电流放大器，**没有**可编程 DRV8323S。

## 1. 板级关键参数

| 项 | 原理图 |
| --- | --- |
| MCU | STM32F405RGT6，LQFP64 |
| 晶振 | 8 MHz（`OSC_IN`/`OSC_OUT`，PH0/PH1） |
| 输入电压 | 丝印约 4.5 V～85 V；固件限制见 `hw_hd75.h`（约 6～74 V） |
| 门极驱动 | FD6288Q，六路独立 PWM（HIN/LIN） |
| 功率管 | 六颗 N 沟 MOSFET（MOS 页：NCEP018N85LL） |
| 电流采样 | 低边三相分流 **0.5 mΩ** + INA240A1 **20 V/V**，输出偏置约 VCC/2 |
| 相电压 / 母线分压 | **56 kΩ / 2.2 kΩ**（SENS1/2/3、SENS_V） |
| 相电压滤波 | `SENS_FILTER`（PC13）控制三路模拟开关，接入约 100 nF |
| 电流滤波 | `CURR_FILTER`（PD2）控制三路模拟开关，接在 INA240 输出 |
| CAN | TJA1042，`CAN_H`/`CAN_L` |
| IMU | BMI160，软件 I2C：PB2=SDX，PA15=SCX |

原理图上的 **U/V/W、INHA/B/C、SENS1/2/3、SOA/B/C** 是 PCB 网络名，对应 TIM1 通道 1/2/3 和 ADC 的 CURR1/2/3、SENS1/2/3。它们**不是**电机线标的绝对保证。当前固件对 TIM1 CH2/CH3 做了 B/C 对调（ACB），见第 8 节。

## 2. MCU 引脚（按端口）

外设栏写原理图主要用途；同一脚的复用在连接器上可能并存，硬件一次只能当一种功能用。

### 2.1 GPIOA

| 引脚 | 封装脚 | 网络名 | 外设 | 作用 |
| --- | --- | --- | --- | --- |
| PA0 | 14 | SENS1 | ADC123_IN0 | A 相端电压分压（反电动势 / 相电压） |
| PA1 | 15 | SENS2 | ADC123_IN1 | B 相端电压分压 |
| PA2 | 16 | SENS3 | ADC123_IN2 | C 相端电压分压 |
| PA3 | 17 | TEMP | ADC123_IN3 | PCB NTC 温度 |
| PA4 | 20 | NRF_SWCLK | GPIO | 板上 NRF/无线模块 SWCLK；本工程未用 |
| PA5 | 21 | SCK_ADC_EXT | ADC12_IN5 / SPI1_SCK | 外设口模拟输入 1，或 COMM SPI 时钟 |
| PA6 | 22 | MISO_ADC_EXT2 | ADC12_IN6 / SPI1_MISO | 外设口模拟输入 2，或 COMM SPI MISO |
| PA7 | 23 | MOSI（COMM） | SPI1_MOSI | 外设口 SPI MOSI，与 PA5/PA6/PB11 一组；现工程未开 SPI |
| PA8 | 41 | INHA | TIM1_CH1 | FD6288 高边输入 A |
| PA9 | 42 | INHB | TIM1_CH2 | FD6288 高边输入 B |
| PA10 | 43 | INHC | TIM1_CH3 | FD6288 高边输入 C |
| PA11 | 44 | USB_DM | USB_OTG_FS_DM | Type-C D− |
| PA12 | 45 | USB_DP | USB_OTG_FS_DP | Type-C D+ |
| PA13 | 46 | SWDIO | SYS_SWDIO | MCU 调试 SWDIO |
| PA14 | 49 | SWCLK | SYS_SWCLK | MCU 调试 SWCLK |
| PA15 | 50 | SCX | GPIO（JTDI） | BMI160 SCL；须关 JTAG 只留 SWD |

### 2.2 GPIOB

| 引脚 | 封装脚 | 网络名 | 外设 | 作用 |
| --- | --- | --- | --- | --- |
| PB0 | 26 | LED_GREEN | GPIO 输出 | 绿灯 |
| PB1 | 27 | LED_RED | GPIO 输出 | 红灯 |
| PB2 | 28 | SDX / BOOT1 | GPIO | BMI160 SDA |
| PB3 | 55 | SWO / JTDO | TRACE / GPIO | 调试 SWO；本板无 DRV SPI，不当 MISO 用 |
| PB4 | 56 | NJTRST | GPIO | JTAG 复位脚；本板无 DRV SPI，不当 SCK 用 |
| PB5 | 57 | （门极使能） | GPIO 输出 | FD6288 EN：高=允许输出，低=关桥。固件 `ENABLE_GATE()` / `DISABLE_GATE()` |
| PB6 | 58 | SERVO | TIM4_CH1 / GPIO | 舵机/PPM 输入，引出到连接器 |
| PB7 | 59 | FAULT | GPIO 输入 | FD6288 故障，低有效。固件 `IS_DRV_FAULT()` |
| PB8 | 61 | CAN_RX | CAN1_RX | CAN 收 |
| PB9 | 62 | CAN_TX | CAN1_TX | CAN 发 |
| PB10 | 29 | TX_SCL_MOSI | USART3_TX / I2C2_SCL | 外设口 TX；现工程调试串口不在这组 |
| PB11 | 30 | RX_SDA_NSS | USART3_RX / I2C2_SDA / SPI NSS | 外设口 RX |
| PB12 | 33 | NRF_SWDIO | GPIO | NRF 模块 SWDIO；本工程未用 |
| PB13 | 34 | INLA | TIM1_CH1N | FD6288 低边输入 A |
| PB14 | 35 | INLB | TIM1_CH2N | FD6288 低边输入 B |
| PB15 | 36 | INLC | TIM1_CH3N | FD6288 低边输入 C |

### 2.3 GPIOC

| 引脚 | 封装脚 | 网络名 | 外设 | 作用 |
| --- | --- | --- | --- | --- |
| PC0 | 8 | SOA | ADC123_IN10 | A 相电流，INA240 输出（固件 CURR1） |
| PC1 | 9 | SOB | ADC123_IN11 | B 相电流（CURR2） |
| PC2 | 10 | SOC | ADC123_IN12 | C 相电流（CURR3） |
| PC3 | 11 | SENS_V | ADC123_IN13 | 母线电压分压（固件 AN_IN / VIN） |
| PC4 | 24 | M_TEMP | ADC12_IN14 | 电机温度 NTC（MOS 页 TEMP1） |
| PC5 | 25 | SHUTDOWN | ADC12_IN15 / GPIO | 开关机采样与保持。VESC 习惯上电拉高维持供电 |
| PC6 | 37 | HALL_1 | TIM3_CH1 / GPIO | 霍尔 / 编码器 A |
| PC7 | 38 | HALL_2 | TIM3_CH2 / GPIO | 霍尔 / 编码器 B |
| PC8 | 39 | HALL_3 | TIM3_CH3 / GPIO | 霍尔 / 编码器 Z 或第三霍尔 |
| PC9 | 40 | PC9 | GPIO | 已引出 MCU 脚；本板无 DRV8323 片选，主功能未用 |
| PC10 | 51 | PC10_TX | UART4_TX | **当前调试串口 TX** |
| PC11 | 52 | PC11_RX | UART4_RX | **当前调试串口 RX** |
| PC12 | 53 | PC12 | GPIO | 已引出 MCU 脚；本板无 DRV MOSI，主功能未用 |
| PC13 | 2 | SENS_FILTER | GPIO 输出 | 相电压 RC 滤波开关控制。高=接入滤波电容 |
| PC14 | 3 | OSC32_IN | — | 未接 32.768 kHz 晶振 |
| PC15 | 4 | OSC32_OUT | — | 未接 32.768 kHz 晶振 |

### 2.4 GPIOD 与专用脚

| 引脚 | 封装脚 | 网络名 | 外设 | 作用 |
| --- | --- | --- | --- | --- |
| PD2 | 54 | CURR_FILTER | GPIO 输出 | 电流采样滤波开关。高=接入滤波。`bsp_gpio.h` 里曾叫 DCCAL |
| PH0 | 5 | OSC_IN | RCC_OSC_IN | 8 MHz 晶振 |
| PH1 | 6 | OSC_OUT | RCC_OSC_OUT | 8 MHz 晶振 |
| BOOT0 | 60 | BOOT0 | — | 原理图下拉，从 Flash 启动 |
| NRST | 7 | NRST | — | 复位，SWD 插头引出 |
| VBAT / VDD / VSS / VDDA / VSSA | 1,12,13,18,19,31,32,47,48,63,64 | 电源 | — | 3.3 V / 模拟地，按数据手册去耦 |

## 3. ADC 通道（与 `hw_hd75.h` 一致）

规则组三重同步，一次触发采 5 个 rank。当前 FOC 电流环通过 `ADC_Value[3..5]` 读取规则组 rank 2 的 CURR1/2/3，由 DMA 传输完成中断驱动；注入组虽配置了相同电流通道，当前这条电流环路径没有读取其结果。

| 软件下标 | ADC 通道 | MCU 脚 | 网络名 | 用途 |
| --- | --- | --- | --- | --- |
| 0 | IN0 | PA0 | SENS1 | 相电压 A |
| 1 | IN1 | PA1 | SENS2 | 相电压 B |
| 2 | IN2 | PA2 | SENS3 | 相电压 C |
| 3 | IN10 | PC0 | SOA | 电流 A（CURR1） |
| 4 | IN11 | PC1 | SOB | 电流 B（CURR2） |
| 5 | IN12 | PC2 | SOC | 电流 C（CURR3） |
| 6 | IN5 | PA5 | SCK_ADC_EXT | 外部模拟 1 |
| 7 | IN6 | PA6 | MISO_ADC_EXT2 | 外部模拟 2 |
| 8 | IN3 | PA3 | TEMP | PCB 温度 |
| 9 | IN14 | PC4 | M_TEMP | 电机温度 |
| 10 | IN15 | PC5 | SHUTDOWN | 开关机采样 |
| 11 | IN13 | PC3 | SENS_V | 母线电压 |
| 12 | Vrefint | 内部 | — | 内部参考 |
| 13 | IN0 | PA0 | SENS1 | 相电压重复采样 |
| 14 | IN1 | PA1 | SENS2 | 相电压重复采样 |

换算（`hw_hd75.h`）：

- 电流：`I = (Vadc − Voffset) / (0.0005 Ω × 20 V/V)`，INA240 零流约 1.65 V（ADC 中点附近）
- 相电压 / 母线：`V = Vadc × (56k+2.2k)/2.2k`

## 4. PWM、门极与功率网

```text
TIM1 CH1/CH1N  PA8 / PB13  →  INHA / INLA  →  FD6288  →  GHA / GLA  →  A 桥  →  SHA
TIM1 CH2/CH2N  PA9 / PB14  →  INHB / INLB  →  FD6288  →  GHB / GLB  →  B 桥  →  SHB
TIM1 CH3/CH3N  PA10/ PB15  →  INHC / INLC  →  FD6288  →  GHC / GLC  →  C 桥  →  SHC
```

MOS 页：

| 网络 | 作用 |
| --- | --- |
| B+ / B− | 母线正 / 负 |
| U / V / W | 电机三相输出端子 |
| SHA / SHB / SHC | 半桥中点（相节点），经分压到 SENS1/2/3 |
| SPA–SNA、SPB–SNB、SPC–SNC | 三相 0.5 mΩ 分流器开尔文取样，进 INA240 |
| GHA/GHB/GHC、GLA/GLB/GLC | 高/低边门极，串约 5.1 Ω |
| TEMP1 | 功率区 NTC，到 MCU `M_TEMP` |

FD6288：`EN`←PB5，故障←PB7（低有效）。无 SPI 寄存器。`hw_hd75.h` 里 `DRV8323S_*` 宏是历史残留，**不要**按那些脚去配本板。

## 5. 滤波开关

三路 `MC74VHC1G66` 模拟开关：

| 控制脚 | 网络 | 作用 | 当前固件 |
| --- | --- | --- | --- |
| PC13 | SENS_FILTER | 把约 100 nF 接到 SENS1/2/3 | 推挽输出；上电默认拉低断开，FOC 启动接通，BLDC 启动后断开 |
| PD2 | CURR_FILTER | 把滤波接到 SOA/SOB/SOC | `CURRENT_FILTER_ON` 为空；PD2 初始化被注释 |

需要滤波时必须把对应脚配成推挽输出再拉高。只改 ODR、脚仍是输入，开关不会动作。详见 `Doc/sch/滤波.md`。

## 6. 连接器与板上接口（按网络）

精确孔位以 PDF 丝印为准。下面列出原理图出现的插头及其网络。

| 接口 | 主要网络 | 作用 |
| --- | --- | --- |
| SWD 排针 P1 | SWDIO、SWCLK、NRST、3.3 V、GND | 下载调试 |
| 调试 UART | PC10_TX、PC11_RX | 当前 Shell，460800 8N1 |
| 外设口 COMM | TX_SCL_MOSI、RX_SDA_NSS、SCK_ADC_EXT、MISO_ADC_EXT2、3.3 V/5 V/GND | USART3 / I2C2 / SPI1 / 外部 ADC，与 PA5/PA6 互斥 |
| 霍尔/编码器 | HALL_1/2/3、M_TEMP、5 V、GND | PC6/PC7/PC8、PC4 |
| CAN | CAN_H、CAN_L | TJA1042，板上有 120 Ω |
| USB Type-C | USB_DM、USB_DP、VBUS、CC | 现工程未启用 USB（PLL48 未配 48 MHz） |
| SERVO | SERVO | PB6 |
| NRF 调试 | NRF_SWDIO、NRF_SWCLK | 无线模块 SWD，本工程未用 |
| 电源开关 SW1 | SHUTDOWN | 与 PC5 相关 |
| 电机端子 | U、V、W | 功率输出 |
| 母线 | B+、B− | 直流输入 |

## 7. 当前固件实际用到的 IO

以 `Library/Bsp/bsp_gpio.c`、`bsp_usart`、`hw_hd75.h` 为准。

| 用途 | 引脚 | 状态 |
| --- | --- | --- |
| 三相 PWM | PA8/PA9/PA10、PB13/PB14/PB15 | 已配 TIM1 复用 |
| 相电压 ADC | PA0/PA1/PA2 | 模拟输入 |
| 电流 ADC | PC0/PC1/PC2 | 模拟输入 |
| 母线 / PCB 温度 / 电机温度 / 外部 ADC | PC3、PA3、PC4、PA5、PA6 | 模拟输入 |
| 调试串口 | PC10、PC11 | UART4 |
| 霍尔 | PC6、PC7、PC8 | 输入 |
| 门极 EN / nFAULT | PB5、PB7 | 宏已定义；GPIO 初始化里 EN 曾被注释，以运行时 `ENABLE_GATE` 为准 |
| LED | PB0、PB1 | 宏已定义，`Bsp_Gpio_Init` 里 LED 初始化被注释 |
| SENS_FILTER / CURR_FILTER | PC13、PD2 | 原理图有，固件目前不驱动 |
| CAN、USB、BMI160、USART3、SPI、NRF SWD | 见上表 | 原理图有，当前应用未初始化 |

## 8. 软件相序（ACB）

原理图 **INHA/SENS1/SOA** 接 TIM1 CH1 / ADC IN0 / IN10，**不要**把软件 A 理解成电机线 U。

当前控制代码为了让旋转磁场与本板走线一致，对 **通道 2 和 3** 对调：

- PWM：`Hal_Motor_PwmSetDuty(duty1, duty3, duty2)` → 引脚 PA8←A、PA9←C、PA10←B
- 电流：`ia←CURR1(SOA)`，`ib←CURR3(SOC)`，`ic←CURR2(SOB)`
- 相电压：`va←SENS1`，`vb←SENS3`，`vc←SENS2`

改相序只改这一处映射，不要改 Park 公式正负号。

## 9. 不要和这些搞混

| 来源 | 说明 |
| --- | --- |
| `Doc/sch/BM60 HD60.pdf` 与 `BM60_HD60_CubeMX配置.md` | 另一块板，仅作 HD 系列引脚习惯参考 |
| `hw_hd75.h` 的 `DRV8323S_*` | BM75 没有这颗芯片 |
| VESC 75/300 的 DRV SPI（PC9/PC12/PB3/PB4） | 本板这些脚不是驱动器 SPI |
