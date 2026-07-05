# VESC (ChibiOS) 任务调度与 Tick 分配分析

> 源码路径：`ThirtyLibrary/bldc/`  
> RTOS：ChibiOS 3.0.5  
> 硬件平台：STM32F407，SYSCLK = 168MHz

---

## 1. 系统时钟基础

### 1.1 ChibiOS Tick 配置

```c
// chconf.h
#define CH_CFG_ST_FREQUENCY     10000   // 系统 tick 频率：10kHz
#define CH_CFG_ST_RESOLUTION    32      // 系统计数器 32 位
#define CH_CFG_TIME_QUANTUM     4       // 同优先级轮转时间片：4 ticks = 400µs
#define CH_CFG_ST_TIMEDELTA     0       // 经典周期 tick 模式（非 tickless）
```

**核心换算**：

| 单位 | 值 | 说明 |
|------|-----|------|
| 1 tick | **100µs** | 1 / 10000 = 100µs |
| `MS2ST(1)` | 10 ticks | 1ms = 10 ticks |
| `MS2ST(5)` | 50 ticks | 5ms = 50 ticks |
| `S2ST(1)` | 10000 ticks | 1秒 = 10000 ticks |
| 时间片 | 400µs | 4 ticks 轮转 |
| 计数器溢出 | ~429500秒 ≈ 4.97天 | 2³² / 10000 / 86400 |

> 与你工程的 FreeRTOS 对比：FreeRTOS tick = 1ms（1000Hz），ChibiOS tick = 100µs（10kHz），**精度高 10 倍**。这是 VESC 能做到 500µs 级线程调度的前提。

### 1.2 SysTick 与 ChibiOS 时基的关系

```
STM32 SysTick (168MHz / 16800 = 10kHz)
     │
     │  每 100µs 触发一次
     ▼
chSysTickHandler()               ← ChibiOS 内核入口
     │
     ├─ 递增系统计数器 chVTGetSystemTimeX()
     ├─ 检查线程超时/睡眠唤醒
     └─ 同优先级轮转调度（每 4 ticks）
```

---

## 2. 全局任务清单

### 2.1 总览图

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          VESC 任务调度全景                                    │
│                                                                              │
│  ┌─── 硬件中断级（最高优先级，不可抢占） ──────────────────────────────────┐  │
│  │                                                                        │  │
│  │  DMA2 Stream4 TC/HT  ──→ mcpwm_foc_adc_int_handler()   ~15kHz        │  │
│  │  TIM2 CC2            ──→ mcpwm_foc_tim_sample_int_handler() ~15kHz   │  │
│  │  ADC1_2_3 JEOC       ──→ mc_interface_adc_inj_int_handler()  ~15kHz  │  │
│  │  EXTI (编码器)        ──→ encoder_pin_isr()                  异步    │  │
│  │                                                                        │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌─── ChibiOS 线程级（NORMALPRIO，可被中断抢占，同优先级轮转） ─────────┐  │
│  │                                                                        │  │
│  │  foc timer    ──→ timer_update()                  1ms    (1kHz)       │  │
│  │  foc hfi      ──→ hfi_update()                   500µs  (2kHz)       │  │
│  │  foc pid      ──→ foc_run_pid_control_*()        可配置 25Hz~10kHz   │  │
│  │  Main LED     ──→ LED 状态显示                    ~10ms              │  │
│  │  Main periodic ──→ 位置显示/HSI 修调              10ms              │  │
│  │  app_adc      ──→ ADC 输入采样                    可配置 1~1000Hz   │  │
│  │  app_ppm      ──→ PPM 脉冲捕获                    ~2ms              │  │
│  │  app_nunchuk  ──→ Nunchuk 控制                    3~10ms            │  │
│  │  app_uartcomm ──→ 串口通信包处理                  ~10ms             │  │
│  │  comm_can     ──→ CAN 状态广播                    可配置            │  │
│  │  encoder_routine ──→ 编码器定期校准               1~10kHz           │  │
│  │                                                                        │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌─── 低优先级线程（LOWPRIO） ────────────────────────────────────────┐    │
│  │                                                                        │  │
│  │  Flash check  ──→ Flash 完整性校验                  6ms              │  │
│  │                                                                        │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌─── 虚拟定时器回调（非线程，在中断上下文执行） ──────────────────────┐  │
│  │                                                                        │  │
│  │  output_vt    ──→ 输出控制定时回调                   可配置 ms 级    │  │
│  │  dpv_vt       ──→ DPV 模式定时回调                  1~2ms           │  │
│  │  timeout_vt   ──→ 超时保护回调                      1ms            │  │
│  │                                                                        │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 硬件中断级详解

### 3.1 DMA2 Stream4 — FOC 电流环（最核心）

```
触发源：ADC 转换完成 → DMA 搬运完成 → TC/HT 中断
回调：  mcpwm_foc_adc_int_handler()
频率：  foc_f_zv / 2（中央对齐模式上下半周各触发一次）
优先级：nvicEnableVector DMA2_Stream4 = 5（ChibiOS 优先级，数值越低越高）
```

| 硬件配置 | `foc_f_zv` | PWM 频率 | 中断频率 | 周期 |
|----------|-----------|---------|---------|------|
| 默认（HD75） | 30000 | 15kHz | ~15kHz | ~66.7µs |
| 低频模式 | 10000 | 5kHz | ~5kHz | 200µs |
| 高频模式 | 40000 | 20kHz | ~20kHz | 50µs |
| 硬件上限 | — | — | — | >38kHz RTOS 崩溃（axiom 注释） |

**中断内执行内容**（`mcpwm_foc_adc_int_handler`）：

```
1. 判断 V0/V7 零矢量（TIM1->CR1 DIR 位）
2. 双电机模式下，上下半周分别服务不同电机
3. 读取三相电流 ADC → 减零偏 → × FAC_CURRENT → 安培
4. 读取母线电压
5. Clarke 变换 → Iα, Iβ
6. 观测器/编码器/HFI 角度更新
7. Park 变换 → Id, Iq
8. 电流环 PID → Vd, Vq
9. 逆 Park → Vα, Vβ
10. SVPWM → 三相占空比 CCR1/CCR2/CCR3
11. TIMER_UPDATE_DUTY_M1() 写入（UDIS 保护）
```

> ⚠️ 这个中断是整个系统**优先级最高**的代码，执行时间直接决定了 `foc_f_zv` 能设到多高。VESC 注释指出：在 axiom 硬件上，FOC ISR 耗时约 26µs，超过 38kHz 就会压垮 RTOS。

### 3.2 TIM2 CC2 — COM 事件同步

```
触发源：TIM2 计数器到达 CCR2
回调：  mcpwm_foc_tim_sample_int_handler()
频率：  ~15kHz（与 DMA 中断同频）
优先级：nvicEnableVector TIM2_IRQn = 6
```

中断内只做一件事：

```c
void mcpwm_foc_tim_sample_int_handler(void) {
    if (m_init_done) {
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);   // 影子→实际
        TIM_GenerateEvent(TIM8, TIM_EventSource_COM);   // 双电机
    }
}
```

> **与你的工程的区别**：你把 COM 事件放在 DMA 中断末尾，VESC 放在独立的 TIM2 中断中。VESC 的做法让占空比更新与采样时刻更精确同步，但也多用了一个中断源。

### 3.3 ADC 注入组中断

```
触发源：ADC1 注入组转换完成（JEOC）
回调：  mc_interface_adc_inj_int_handler()
频率：  ~15kHz
优先级：nvicEnableVector ADC_IRQn = 6
```

### 3.4 中断优先级映射

ChibiOS 使用 **4 位优先级**（NVIC_PriorityGroup_4），数值越低优先级越高：

| 中断源 | ChibiOS 优先级 | 含义 |
|--------|--------------|------|
| DMA2 Stream4 (FOC 电流环) | 5 | **最高电机控制优先级** |
| TIM2 CC2 (COM 同步) | 6 | 次高 |
| ADC1 JEOC (注入组) | 6 | 同级 |
| 编码器 EXTI | 0 | **最高**（硬件零位检测） |
| SysTick | ChibiOS 内核管理 | 抢占阈值以上 |

> ChibiOS 的优先级数值和 STM32 NVIC 是**反方向**的：ChibiOS 0 = 最高，STM32 NVIC 0 = 最高。`nvicEnableVector(IRQn, 5)` 中的 5 经 ChibiOS 转换后写入 NVIC 寄存器。

---

## 4. FOC 三个线程详解

### 4.1 timer_thread — FOC 定时器线程

```c
// mcpwm_foc.c:4168
static THD_FUNCTION(timer_thread, arg) {
    chRegSetThreadName("foc timer");
    for(;;) {
        const float dt = 0.001;                 // ← 硬编码 1ms
        timer_update((motor_all_state_t*)&m_motor_1, dt);
        chThdSleepMilliseconds(1);              // ← 1ms 周期
    }
}
```

| 参数 | 值 |
|------|-----|
| 线程名 | `foc timer` |
| 优先级 | NORMALPRIO |
| 栈大小 | 512 字节 |
| 执行周期 | **1ms（1kHz）** |
| dt | 硬编码 0.001s |

**`timer_update()` 做什么**：

- 速度/位置相关的慢速更新
- 电流零偏自动校准（上电后累积采样取平均）
- 温度监测与降额
- 观测器慢速更新

### 4.2 hfi_thread — HFI 高频注入线程

```c
// mcpwm_foc.c:4460
static THD_FUNCTION(hfi_thread, arg) {
    chRegSetThreadName("foc hfi");
    for(;;) {
        float dt = timer_seconds_elapsed_since(t_last);
        hfi_update(&m_motor_1, dt);
        chThdSleepMicroseconds(500);            // ← 500µs 周期
    }
}
```

| 参数 | 值 |
|------|-----|
| 线程名 | `foc hfi` |
| 优先级 | NORMALPRIO |
| 栈大小 | 512 字节 |
| 执行周期 | **500µs（2kHz）** |
| dt | 动态计算（`timer_seconds_elapsed_since`） |

**`hfi_update()` 做什么**：

- HFI（High Frequency Injection）无感角度估计
- 对采样缓冲区做 DFT 提取角度信息
- 低速时（低于 `foc_sl_erpm_hfi`）切换到 HFI 角度
- 高速时回退到滑模观测器

### 4.3 pid_thread — 速度/位置 PID 线程

```c
// mcpwm_foc.c:4485
static THD_FUNCTION(pid_thread, arg) {
    chRegSetThreadName("foc pid");
    for(;;) {
        switch (m_motor_1.m_conf->sp_pid_loop_rate) {
            case PID_RATE_25_HZ:    sleep 40ms;   break;
            case PID_RATE_50_HZ:    sleep 20ms;   break;
            case PID_RATE_100_HZ:   sleep 10ms;   break;
            case PID_RATE_250_HZ:   sleep 4ms;    break;
            case PID_RATE_500_HZ:   sleep 2ms;    break;
            case PID_RATE_1000_HZ:  sleep 1ms;    break;   ← 默认
            case PID_RATE_2500_HZ:  sleep 400µs;  break;
            case PID_RATE_5000_HZ:  sleep 200µs;  break;
            case PID_RATE_10000_HZ: sleep 100µs;  break;
        }
        float dt = timer_seconds_elapsed_since(last_time);
        foc_run_pid_control_pos(index_found, dt, motor);
        foc_run_pid_control_speed(index_found, dt, motor);
    }
}
```

| 参数 | 值 |
|------|-----|
| 线程名 | `foc pid` |
| 优先级 | NORMALPRIO |
| 栈大小 | 256 字节 |
| 执行周期 | **可配置 25Hz ~ 10kHz** |
| 默认频率 | **1kHz**（`MCCONF_SP_PID_LOOP_RATE = PID_RATE_1000_HZ`） |
| dt | 动态计算 |

**PID 线程频率选择指南**：

| 频率 | 适用场景 |
|------|---------|
| 25~100Hz | 位置伺服，低速精确控制 |
| 250~500Hz | 一般速度控制 |
| 1kHz（默认） | 平衡速度/性能 |
| 2.5~5kHz | 高动态响应（无人机、平衡车） |
| 10kHz | 极端响应需求，但 CPU 占用高 |

---

## 5. 主线程创建的系统线程

```c
// main.c:386-388
chThdCreateStatic(led_thread_wa,     256, NORMALPRIO, led_thread,     NULL);
chThdCreateStatic(periodic_thread_wa, 256, NORMALPRIO, periodic_thread, NULL);
chThdCreateStatic(flash_integrity_check_thread_wa, 256, LOWPRIO, flash_integrity_check_thread, NULL);
```

| 线程名 | 优先级 | 栈 | 周期 | 功能 |
|--------|--------|-----|------|------|
| `Main LED` | NORMALPRIO | 256B | ~10ms | 运行/故障 LED 指示 |
| `Main periodic` | NORMALPRIO | 256B | 10ms | 位置显示 + HSI 温度修调 |
| `Flash check` | LOWPRIO | 256B | 6ms | Flash 完整性增量校验 |

---

## 6. 应用层线程

各应用模块在启动时自行创建线程：

| 应用 | 线程名 | 优先级 | 周期 | 功能 |
|------|--------|--------|------|------|
| `app_adc` | `App ADC` | NORMALPRIO | 可配置 1~1000Hz | ADC 输入映射到电机控制量 |
| `app_ppm` | `App PPM` | NORMALPRIO | ~2ms | PPM/PWM 脉冲捕获 |
| `app_nunchuk` | `App Chuk` / `App Chuk Out` | NORMALPRIO | 3~10ms | Wii Nunchuk 控制 |
| `app_dpv` | `App DPV` | NORMALPRIO | 1~2ms | 潜水推进器控制 |
| `app_pas` | `App PAS` | NORMALPRIO | 可配置 | 踏频助力传感器 |
| `app_sten` | `App STEN` | NORMALPRIO-1 | ~1ms | STEER/THROTTLE |
| `app_skypuff` | `App Skypuff` | NORMALPRIO | ~1ms | 自动绞盘控制 |
| `app_uartcomm` | `Packet Process` | NORMALPRIO | ~10ms | 串口命令包处理 |
| `comm_can` | `CAN status` | NORMALPRIO | 可配置 | CAN 状态广播 |

---

## 7. 虚拟定时器（非线程）

ChibiOS 的虚拟定时器回调在 **中断上下文** 中执行，不是独立线程，用于轻量级定时任务：

| 定时器 | 周期 | 功能 |
|--------|------|------|
| `output_vt` | 可配置（ms级） | 输出控制超时回调 |
| `dpv_vt` | 1~2ms | DPV 模式更新 |
| `timeout_vt` | 1ms | 安全超时保护（关电机） |
| `servo_dec` | 1ms | 伺服脉冲解码 |

---

## 8. CPU 时间预算分析

以 `foc_f_zv = 30000`（默认配置）为例：

### 8.1 中断级 CPU 占用

```
PWM 周期 = 1 / 15000 = 66.7µs

每个周期内中断：
├─ DMA 中断（FOC 电流环）：~20µs 执行
├─ TIM2 CC2 中断（COM 同步）：~1µs 执行
└─ ADC JEOC 中断：~1µs 执行

中断总占用 ≈ (20 + 1 + 1)µs / 66.7µs ≈ 33%
```

### 8.2 线程级 CPU 占用（剩余 ~67%）

```
可用时间 ≈ 66.7µs × 67% ≈ 44.7µs/周期

各线程每秒 CPU 需求估算：
├─ foc timer (1kHz)：~5µs/次 × 1000 = 5ms/s
├─ foc hfi (2kHz)：  ~10µs/次 × 2000 = 20ms/s
├─ foc pid (1kHz)：  ~8µs/次 × 1000 = 8ms/s
├─ Main LED (100Hz)： ~2µs/次 × 100 = 0.2ms/s
├─ Main periodic(100Hz)：~5µs/次 × 100 = 0.5ms/s
├─ Flash check(167Hz)：~3µs/次 × 167 = 0.5ms/s
├─ App ADC (可配)：   ~5µs/次
└─ comm/其他：       ~2ms/s

线程总占用 ≈ 36ms/s ≈ 3.6% CPU
```

**结论**：中断占用是大头（~33%），线程只占 ~4%，整体 CPU 利用率 ~37%，还有 ~63% 空闲。这也是 VESC 能同时跑 CAN、UART、LED 等任务的原因。

### 8.3 高频模式下的压力

| `foc_f_zv` | PWM 频率 | 中断频率 | FOC ISR 耗时 | 中断 CPU 占用 | 剩余 |
|-----------|---------|---------|-------------|-------------|------|
| 10000 | 5kHz | ~5kHz | ~20µs | ~10% | 90% |
| 30000 | 15kHz | ~15kHz | ~20µs | ~30% | 70% |
| 40000 | 20kHz | ~20kHz | ~20µs | ~40% | 60% |
| 50000 | 25kHz | ~25kHz | ~20µs | ~50% | 50% ⚠️ |
| 60000 | 30kHz | ~30kHz | ~20µs | ~60% | 40% 🔴 |

> 注释中提到 axiom 硬件在 ~38kHz 时 RTOS 开始崩溃（ISR 占用 ~50%+）。你的工程 `FOC_CONTROL_LOOP_FREQ_DIVIDER=1`（每周期都跑 FOC），与 VESC 默认行为一致。

---

## 9. 与你工程（FreeRTOS）的架构对比

| 维度 | VESC (ChibiOS) | 你的工程 (FreeRTOS) |
|------|---------------|-------------------|
| **系统 tick** | 10kHz (100µs) | 1kHz (1ms) |
| **电流环** | DMA 中断 ~15kHz | DMA 中断 15kHz |
| **速度/位置 PID** | 独立线程，可配 25Hz~10kHz | 在 DMA 中断中一起跑 |
| **HFI 角度估计** | 独立线程 2kHz | 未实现 |
| **COM 事件** | TIM2 CC2 独立中断 | DMA 中断末尾 |
| **电流零偏** | 上电自动校准 + 运行中持续更新 | 硬编码 2045 |
| **调度精度** | 100µs（ChibiOS tick） | 1ms（FreeRTOS tick） |
| **同优先级轮转** | 4 ticks = 400µs | 由 configTICK_RATE_HZ 决定 |
| **线程间通信** | ChibiOS 信号量/邮箱/事件 | FreeRTOS 队列/信号量 |
| **CPU 占用** | ~37%（15kHz PWM） | 未知（建议测量） |

### 9.1 你工程可以参考的改进点

1. **拆分电流环和速度环**：电流环留在 DMA 中断（15kHz），速度/位置 PID 移到 FreeRTOS 任务（1kHz），降低中断执行时间
2. **提高 FreeRTOS tick 频率**：从 1kHz 提高到 10kHz（`Bsp_Sys_Init(10000)`），获得与 VESC 同等的调度精度
3. **加入 HFI 线程**：500µs~1ms 周期运行，低速无感启动时角度估计更准确
4. **电流零偏校准**：参考 VESC 的 `mcpwm_foc_dc_cal()`，上电时在 PWM 关闭状态下采样取平均
5. **COM 事件独立中断**：用 TIM2 CC2 中断触发 COM 同步，让占空比更新与采样时刻更精确对齐
6. **foc_f_zv 运行时可调**：VESC 通过 `timer_reinit()` 支持运行时改 PWM 频率，你的工程目前固定 30000

---

## 10. 关键源码索引

| 功能 | 文件 | 行号 |
|------|------|------|
| ChibiOS tick 配置 | [chconf.h](../../ThirtyLibrary/bldc/chconf.h) | 54 |
| FOC 电流环中断 | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 2860 |
| TIM2 COM 同步中断 | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 2848 |
| DMA 分配 + 回调注册 | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 408 |
| timer_thread | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 4168 |
| hfi_thread | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 4460 |
| pid_thread | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 4485 |
| timer_reinit (PWM频率动态切换) | [mcpwm_foc.c](../../ThirtyLibrary/bldc/motor/mcpwm_foc.c) | 169 |
| IRQ 处理入口 | [irq_handlers.c](../../ThirtyLibrary/bldc/irq_handlers.c) | 30-63 |
| main 线程创建 | [main.c](../../ThirtyLibrary/bldc/main.c) | 386-388 |
| PID 频率枚举 | [datatypes.h](../../ThirtyLibrary/bldc/datatypes.h) | 358-366 |
| 采样模式枚举 | [datatypes.h](../../ThirtyLibrary/bldc/datatypes.h) | 71-75 |
| 默认 PID 频率 | [mcconf_default.h](../../ThirtyLibrary/bldc/motor/mcconf_default.h) | 137 |
| foc_f_zv 默认值 | [mcconf_default.h](../../ThirtyLibrary/bldc/motor/mcconf_default.h) | 266 |
