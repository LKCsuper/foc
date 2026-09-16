# 主机测试

在仓库根目录运行 `Tests/host/run.ps1 -Compiler <zig.exe路径>`。脚本面向 Windows；Flash 测试使用 VirtualAlloc 模拟地址范围，不访问实际板子。

可选本地工具安装：`python -m pip install --target .build/tools ziglang==0.16.0`，随后运行 `Tests/host/run.ps1`。工具不属于固件依赖，`.build/` 不进入版本库。

## 主机通过 vs 上板通过

| | 主机通过 | 上板通过 |
| --- | --- | --- |
| 含义 | PC 编译并断言生产核心逻辑 | Keil 烧录后按 `Examples/` 观测量确认 |
| 依赖 | Zig C 编译器 | 功率板、电机、示波器/电流探头 |
| 不覆盖 | 真实 ADC/PWM 时序、力矩方向、擦写延时 | — |
| 状态 | `run.ps1` 全绿即可声称 | 本仓库步骤 0～5 **尚未**标为上板验收完成 |

学习递进与 Shell 步骤见 [Examples/README.md](../../Examples/README.md)。

## 当前用例一览

| 可执行文件 | 针对性验证 |
| --- | --- |
| `test_motor_settings` | 配置/校准编码、状态隔离、非法输入 |
| `test_flash_storage` | 双槽校验与中断恢复（Flash 为 host 替身） |
| `test_motor_api` | 命令整体提交、PWM Start/Safe 桥 |
| `test_motor_speed` | 速度环状态步进 |
| `test_motor_protect` | 过流/欠压计时与锁存 |
| `test_motor_startup` | 分频、对齐、I/F 双实例 |
| `test_motor_sensorless` | 无感 runtime / effects 隔离 |
| `test_motor_current` | 电流目标选择、PI 实例隔离 |
| `test_motor_control` | 开环编排、仲裁、有感角与速度 Iq |
| `test_motor_sense` | 采样帧换算（假 ADC） |
| `test_motor_detect_offset` | 偏移校准非阻塞核心 |
| `test_motor_feedback` | 反馈源选角与失效策略 |
| `test_motor_encoder` | 编码器回绕速度、零点有效 |
| `test_motor_hall_angle` | 霍尔表/扇区/Hall-PLL 72 与 200 RPM |
| `test_motor_bldc_six_step` | 独立 BLDC 六步换相、正反转、非法霍尔与超时停机 |
| `test_motor_bldc_sensorless` | BLDC 开环启动、BEMF 过零接管、30°延迟换相与丢步保护 |
| `example_01_transform` | Clarke/Park/逆 Park/SVM 往返（学习示例） |

## 编码与存储注意

测试一编译真实 settings 与初始化模块，存储入口为替身；测试二编译真实 settings 与 Flash 模块，只有硬件 Flash 操作和 CRC 服务被模拟。断言检查状态隔离、错误输入与双槽中断恢复，不验证真实擦写延时、PWM、ADC 或电机行为。

字段顺序冻结在 `motor_settings.c` Encode/Decode 中；改变字段顺序或类型必须升级 `FLASH_STORAGE_FORMAT_VERSION` 并更新测试和迁移说明。
