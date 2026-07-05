# 项目修改历史

---

## 2026-07-05

### FreeRTOS Tick 频率提升
- **[FreeRTOSConfig.h](../App/Configs/inc/FreeRTOSConfig.h)**: `configTICK_RATE_HZ` 从 **1000 → 10000**（10kHz tick），提高系统实时精度
- **[bsp.c](../Library/Bsp/bsp.c)**: `Bsp_Sys_Init(1000)` → `Bsp_Sys_Init(10000)`，SysTick 与 FreeRTOS tick 保持一致

### `osDelay()` 全部改用 `pdMS_TO_TICKS()`
- 所有任务中的 `osDelay(N)` 统一替换为 `osDelay(pdMS_TO_TICKS(N))`，确保不同 tick 频率下延时时间正确（`osDelay` 参数为 tick 数而非毫秒）
- 影响文件：
  - [main.c](../App/User/main.c)
  - [cmd_duty.c](../App/cmd/cmd_duty.c)
  - [motor_task.c](../App/motor/motor_task.c)
  - [motor_detect.c](../App/motor_param/motor_detect.c)
  - [motor_hall.c](../App/motor_sensor/motor_hall.c)

### 初始化顺序调整
- **[main.c](../App/User/main.c)**: 任务初始化流程优化：
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
