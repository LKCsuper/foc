# FOC 学习框架

面向自己学习与逐步跨芯片移植的 C 语言 FOC 工程。当前工程目标为 STM32F405RGTx，板级宏来自 `App/motor/inc/hw_hd75.h`；实际功率板接线与宏是否一致需要自行核对。

## 构建入口

- 应用工程：`App/Project/foc.uvprojx`，目标 `Foc`，使用 Keil MDK 的 ARM 工具链。
- 工作区：`Project/Master_Project.uvmpw`；Boot 是独立工程，不是核心算法依赖。
- 当前应用输出目录在仓库外 `../build/`（相对应用工程为 `../../../build/`）。本次没有修改该路径，也没有运行烧录。
- `App/Project/Foc.BAT` 含原开发机的绝对路径，不能当作通用构建脚本。
- 主机测试入口：`Tests/host/run.ps1`，用 Zig 的 C 编译器编译生产代码，不需要 MCU SDK/RTOS；详情见 `Tests/host/README.md`。

## 主控制链与阅读顺序

```text
App/User/main.c：main → Task_Init
  Bsp_Init → Motor_Encoder_Init → MOTOR_Param_Init → Detect_Init
  → Command_Init → Hal_Motor_PwmStart

DMA2_Stream4 → Bsp_Dma2Stream4_Handler → Motor_DmaInt
  → Motor_Sense_Update → Motor_Protect_Run
  → 按分频执行启动/电流环/无感/速度环 → 逆 Park/SVPWM → HAL PWM
```

先读 `motor_foc.c` / `motor_pid.c`，再读 `motor_it.c` 主链和 `motor_settings.h` 配置边界，最后看硬件适配。

- 当前配置推算：PWM 约 15 kHz，ADC 回调约 30 kHz，FOC 分频 2 后约 15 kHz；这些是代码配置值，本次未用示波器测量。
- `motor_simple_foc` 为学习实现，其初始化仍由 main 调用；未接入主 FOC 中断，暂不删除。
- `motor_ctrl.c` 开环函数和 `Motor_PidControl` 未发现外部 C 调用，保留兼容/占位身份。
- 已有 `motor_encoder.c` 提供编码器读数与零点接口，正常有感反馈接入主电流环仍待实现，不能把“有传感器文件”当作“已有感闭环”。

## 启动与调试

上电执行电流/电压偏移校准，完成后运行命令仍为禁用。`Hal_Motor_PwmStart` 恢复 PWM 模式，因此“禁用运行命令”不等同于所有门极持续关闭。

通过 Shell 的 `olstatus` 查看模式、参数、故障与耗时。`olstart` / `olspeed` 会发起实际运行；先完成接线、方向和电流采样检查，再按自己的板子选择参数。`olstop` 停止运行命令；现有 ISR 会输出中点占空比，该行为本次未改变。

当前默认 I/F、电流 PI 0.2/30、Iq 0.3 A、对齐 0.5 A/0.3 s；HFI 与无感观测开启，无感接管与速度环关闭。默认值针对当前调试工程，不是任意电机通用参数。

## 参数保存（格式 v2）

- `motor_config_t` 与 `motor_calibration_t` 已独立；现有 `MOTOR_PARAM_S` 暂作控制代码兼容对象。
- `saveparam` 只保存配置与校准的固定 216-byte 小端 payload，不保存运行命令、采样值、PI 积分或已停用的 R/L 自动检测结果。
- v1 格式不迁移。升级后旧槽无法加载时使用代码默认值，上电重新执行偏移校准，霍尔表需重新标定；不会自动擦除旧槽，显式 `saveparam` 才写入新格式。
- 保留开发阶段既有策略：重启后控制增益等由代码默认值覆盖，只恢复校准和霍尔额外采样配置。保存成功不代表运行配置会全部在重启后生效。
- 保存必须在运行命令禁用、偏移/霍尔校准结束后执行。当前 Flash 驱动仍绑定 STM32F4，完整擦写互斥和输出仲裁属于后续工作。

## 实施进度

见 [AI_CODE.md](AI_CODE.md)、[本轮实施记录](Doc/doc/架构实施记录_2026-09-05.md) 和 [代码阅读指南](Doc/doc/代码阅读指南.md)。

本轮完成步骤 0 的静态梳理和步骤 1 的配置/校准存储边界。命令一致提交、完整 runtime 拆分、主控制抽取与第二芯片适配仍待执行；主机通过不能代替完整固件构建和上板验证。
