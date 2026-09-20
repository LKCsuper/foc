# 项目修改历史

---

## 2026-09-20

### Hall 电流环对齐 VESC `foc_svm`

去掉 Hall CCR 倍率，Hall + I/F 改为 `1.5/Vbus` 归一 + `foc_svm`（零电压 ARR/2）。HD75 A-C-B 上标准 `foc_svm` 差 180°，输入取反；电流环默认 `olkp 0.12` / `olki 18`。空载 150～2500 真到目标，3000 到 2961。弱磁仍关。ADC/PWM/TIM/DMA 未改。

记录：`Doc/doc/Hall_对齐VESC_SVM_2026-09-20.md`。

### Hall 空载 3000 电流原理与使用边界

速度给定改成 VESC 式 ERPM 斜坡，避免 150 运转中 `olspeed 3000` 误锁 `MOTOR_SPEED_LOSS`。重构门槛收到 ARR−500，空载 3000 不再误触发。`olcurdiag` 增加每拍 mean/RMS：空载 3000 的 1～3 A 是交流 Id（mean Id≈0，rmsI≈1.2 A），不是持续直轴电流。不用观测器，空载 Hall 速度闭环可以从 150 用到 3000。

记录：`Doc/doc/Hall空载3000电流与使用边界_2026-09-20.md`。

---

## 2026-09-09

### 修复 I/F 300 RPM Id/Iq 偏离

300 RPM 下 21 点 `olsnap` 显示 Id/Iq 平均值接近给定，但存在拟合度 94%～99% 的二倍电角纹波。根因为旧 `svm_hqd` 的 β 轴调制增益只有 α 轴一半。

- 将 `X/Y/Z` 中 β 分量系数统一乘 2，保留已验证的扇区、整体极性和 A-C-B 映射。
- 纯开环时跳过未选用的编码器/霍尔每拍计算，避免约 90% ISR 负载饿死串口命令。
- 复测 300 RPM：Id/Iq=−0.028/0.609 A；500/800 RPM 扩展测试通过，全程 Fault=CLEAR。

记录：`Doc/doc/开环稳定性验证.md`。

---

### UART 在 300 RPM 下发不完 olsnap

`--only V5,V9`：72 RPM 快照完整，Id/Iq≈0.12/0.61 A；300 RPM 快照卡在 `V_bus :`，随后 `olstop` 也无回显。原因是第一段 DMA 发出后完成中断进不来，队列又没满，命令任务堵在收串口上，回绕后的下一段永不启动。

- 任务空闲每 1 ms 轮询 TX DMA；`Flush` 同样轮询，不再空等中断。
- `olsnap` / `olstatus` 打完后 `Flush`。
- 验证脚本必须看到完整页尾和 `Id, Iq`，不再把半截快当成功。

记录：`Doc/doc/开环稳定性验证.md`。

---

## 2026-09-08

### 开环验证问题与处理

空载验证（`f024406`）上 I/F 能转但不稳：`olstatus` 的 Theta 不刷新、300 RPM 电流跟踪差、运行中过零反转会顿。固件侧：

- `Motor_Control_Step` 把本拍 Park/PWM 电角写回 `motor.theta`，状态页 `Theta/CCR` 一致。
- `olstart` 改为先打印并 `Flush` 再使能，避免 FOC 一起转就把 “started” 卡在 UART 里。
- 运行中 `olspeed` 改符号须先 `olstop`。
- `olalign 0` 时跳过定角灌 Id，直接拖转，避免转子被吸进齿槽后低速拖不动。

记录：`Doc/doc/开环稳定性验证.md`、`Doc/doc/开环电流环调试说明.md`；串口原文 `Doc/verify/`。

---

## 2026-07-05

### FreeRTOS Tick 频率提升
- **[FreeRTOSConfig.h](../App/configs/inc/FreeRTOSConfig.h)**: `configTICK_RATE_HZ` 从 **1000 → 10000**（10kHz tick），提高系统实时精度
- **[bsp.c](../Library/Bsp/bsp.c)**: `Bsp_Sys_Init(1000)` → `Bsp_Sys_Init(10000)`，SysTick 与 FreeRTOS tick 保持一致

### `osDelay()` 全部改用 `pdMS_TO_TICKS()`
- 所有任务中的 `osDelay(N)` 统一替换为 `osDelay(pdMS_TO_TICKS(N))`，确保不同 tick 频率下延时时间正确（`osDelay` 参数为 tick 数而非毫秒）
- 影响文件：
  - [main.c](../App/user/main.c)
  - [cmd_duty.c](../App/cmd/cmd_duty.c)
  - [motor_task.c](../App/motor/motor_task.c)
  - [motor_detect.c](../App/motor_param/motor_detect.c)
  - [motor_hall.c](../App/motor_sensor/motor_hall.c)

### 初始化顺序调整
- **[main.c](../App/user/main.c)**: 任务初始化流程优化：
  - `MOTOR_Param_Init()`（EasyFlash 加载）前置到 `Detect_Init()` 之前，确保 Flash 参数先加载
  - `Command_Init()`（Shell）移到 `MOTOR_ShowParam()` 之后，避免 Shell 抢 UART 导致参数打印乱序
  - 添加注释说明步骤目的

### DMA 中断精简（停止 FOC 闭环）
- **[motor_it.c](../App/motor/motor_it.c)**: 注释掉 DMA 中断中的所有 FOC 闭环控制代码（保留纯电流电压采集），电机暂时处于开环状态，用于调试或过渡阶段
  - 注释掉的代码包括：Clarke/Park 变换、PI 电流环、SVM 调制、Simulink 模型调用

### 电流/电压采样方式优化
- **[motor_detect.c](../App/motor_param/motor_detect.c)**: 偏移量检测函数中的采样方式从 `motor.currentAdc[]`/`motor.voltAdc[]` 改为宏 `GET_CURRENTx()`/`ADC_VOLTS(ADC_IND_SENSx)`，更清晰且减少中间变量依赖
- 恢复偏移量调试打印（之前因任务栈不足被注释，现已解决）

### USART 宏配置修正
- **[bsp_gpio.h](../Library/Bsp/inc/bsp_gpio.h)**: `USART_NUM` 从 **3 → 4**（切换串口号）
- **[bsp_gpio.c](../Library/Bsp/bsp_gpio.c)**: USART3 GPIO 复用配置中 `GPIO_AF_USART3` 改为自定义宏 `DEBUG_USART_TX_AF`/`DEBUG_USART_RX_AF`，支持串口号切换

### BSP 时钟文档化
- **[bsp.c](../Library/Bsp/bsp.c)**: `Bsp_RTOS_Init()` 函数补充完整的中文时钟树 ASCII 图解，包括 HSE/HSI 两种模式说明、各总线频率汇总表、ADC 时钟超限警告

### EasyFlash 日志输出修正
- **[ef_port.c](../ThirtyLibrary/easyflash/port/ef_port.c)**: `ef_log_debug` 中日志输出从 `WARNING()` 改为 `PRINTF()`（`WARNING` 宏可能依赖串口初始化，在早期 boot 阶段可能卡死）

### 工程配置更新（IDE 自动生成）
- [foc.uvguix.15403](../App/Project/foc.uvguix.15403)
- [foc.uvoptx](../App/Project/foc.uvoptx)
- [Master_Project.uvmpw.uvgui.15403](../Project/Master_Project.uvmpw.uvgui.15403)
- [JLinkLog.txt](../App/Project/JLinkLog.txt)
