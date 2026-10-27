# FOC 学习框架

面向自己学习与逐步跨芯片移植的 C 语言 FOC 工程。当前工程目标为 STM32F405RGTx，板级宏来自 `App/motor/inc/hw_hd75.h`；实际功率板接线与宏是否一致需要自行核对。

## 构建入口

- 应用工程：`App/Project/foc.uvprojx`，目标 `Foc`，使用 Keil MDK 的 ARM 工具链。
- 应用目标默认采用 `-O3 -Otime --fpmode=std`，各文件已有的局部优化指令仍保留；源码单步可能跳行、部分变量可能无法观察。软件周期仿真的夹具固定为 O1。
- 工作区：`Project/Master_Project.uvmpw`；Boot 是独立工程，不是核心算法依赖。
- Keil 批编译：仓库根目录 `.\build_keil.ps1`（可选 `-Action rebuild|clean`、`-Project Boot`、`-Uv4 <UV4.exe>`）；实现在 `Scripts/build_keil.ps1`，日志在 `.build/keil/`。旧的 `App/Project/Foc.BAT` 含本机绝对路径，勿当通用脚本。
- 当前应用输出目录在仓库外 `../build/`（相对应用工程为 `../../../build/`）。
- 主机测试入口：`Tests/host/run.ps1`，用 Zig 的 C 编译器编译生产代码，不需要 MCU SDK/RTOS；详情见 `Tests/host/README.md`。
- 最小学习示例：`Examples/`（变换 → 校准 → 开环 → 有感电流 → 有感速度）；**主机通过与上板通过分开陈述**。

## 主控制链与阅读顺序

```text
App/user/main.c：main → Task_Init
  Bsp_Init → Motor_Encoder_Init → MOTOR_Param_Init → Detect_Init
  → Command_Init → Hal_Motor_PwmStart

DMA2_Stream4 → Bsp_Dma2Stream4_Handler → Motor_DmaInt
  → Motor_Sense_Update → Motor_Protect_Run
  → 按分频执行启动/电流环/无感/速度环 → 逆 Park/SVPWM → HAL PWM
```

先读 `motor_foc.c` / `motor_pid.c`，再读 `motor_it.c` 主链和 `motor_settings.h` 配置边界，最后看硬件适配。有感路径：`olfb encoder|hall` + 可选 `spenable`（见 `Examples/04`/`05`）。

- 当前配置推算：PWM 约 15 kHz，ADC 回调约 30 kHz，FOC 分频 2 后约 15 kHz；这些是代码配置值，本次未用示波器测量。
- 早期学习实现 `motor_simple_foc` 已移除；学习路径见 `Examples/`，勿再复制第二套核心算法。
- `motor_ctrl.c` 开环函数和 `Motor_PidControl` 未发现外部 C 调用，保留兼容/占位身份。

## 启动与调试

串口命令见 [命令行使用手册](Doc/doc/命令行使用手册.md)（UART4 460800 8N1，提示符 `$ `）。

上电从 Flash 恢复有效电流零点则跳过偏移检测，否则校准约 5 s 并自动写入 Flash。完成后运行命令仍为禁用。`Hal_Motor_PwmStart` 恢复 PWM 模式，因此“禁用运行命令”不等同于所有门极持续关闭。

通过 Shell 的 `olstatus` 查看模式、参数、故障与耗时。`olstart` / `olspeed` 会发起实际运行；先完成接线、方向和电流采样检查，再按自己的板子选择参数。`olstop` 停止运行命令；现有 ISR 会输出中点占空比，该行为本次未改变。

当前默认 I/F、电流 PI 0.2/30、Iq 0.6 A；对齐、HFI、无感观测与速度环关闭。默认值针对当前调试工程，不是任意电机通用参数。

## 参数保存（格式 v2）

- `motor_config_t` 与 `motor_calibration_t` 已独立；现有 `MOTOR_PARAM_S` 暂作控制代码兼容对象。
- `saveparam` 只保存配置与校准的固定 216-byte 小端 payload，不保存运行命令、采样值、PI 积分或已停用的 R/L 自动检测结果。
- v1 格式不迁移。升级后旧槽无法加载时使用代码默认值，上电重新执行偏移校准，霍尔表需重新标定；不会自动擦除旧槽，显式 `saveparam` 才写入新格式。
- 保留开发阶段既有策略：重启后控制增益等由代码默认值覆盖，只恢复校准和霍尔额外采样配置。保存成功不代表运行配置会全部在重启后生效。
- 保存必须在运行命令禁用、偏移/霍尔校准结束后执行。Flash 经 `Hal_Flash_*`；完整上板擦写与输出仲裁仍以实机为准。

## 实施进度

实施进度见 [AI_CODE.md](AI_CODE.md)、[本轮实施记录](Doc/doc/架构实施记录_2026-09-05.md)、[基线电气行为（静态）](Doc/doc/基线电气行为_静态.md) 和 [代码阅读指南](Doc/doc/代码阅读指南.md)。

步骤 0～6 主机/文档侧已通（步骤 0 上板末项未勾）；主机通过不能代替完整固件构建和上板验证。
