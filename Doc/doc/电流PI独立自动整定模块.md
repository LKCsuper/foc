# 电流 PI 独立自动整定模块

## 职责与范围

`App/motor/calibration/motor_autotune.c` 和 `App/motor/calibration/inc/motor_autotune.h` 提供无硬件、RTOS、全局电机依赖的纯计算 API：`Motor_Autotune_CurrentPi`。

第一版为基于已辨识 RL 的模型初值计算器，不包含自动扫频、闭环试验状态机、速度环整定或在线自适应。模块自身不调用参数检测、不驱动电机、不写运行参数或 Flash；由上层决定何时检测、何时请求候选值及如何验证。

## 原理

在电流单轴模型 `G(s)=1/(Ls+R)`、PI `C(s)=Kp+Ki/s` 下，令 `ωc=2π*bandwidth_hz`，取：

- `Kp_d=Ld*ωc`、`Kp_q=Lq*ωc`，单位 V/A。
- `Ki=R*ωc`，单位 V/(A*s)。

PI 零点抵消 RL 极点后，理想开环近似 `ωc/s`，理想闭环为 `ωc/(s+ωc)`。现有 `Motor_Current_PI_Run` 在每次调用中执行 `integral += error*Ki*dt`，因此候选 Ki 不得预乘 dt。

实际 PI 为离散实现，模型消零并不精确，采样延时、滤波、交叉耦合和电压饱和均会改变响应。

## 输入、输出与拒绝条件

输入包括每相等效 R、工作点增量 Ld/Lq、实际 PI 控制周期、目标带宽、总有效延时及目标最小相位裕量。

- 总延时包含保持、PWM 更新和测量等效延时；调用方须核实，至少填半个控制周期。这个下限只防止漏填，不代表已知硬件延时。
- 采样检查采用 `ωc*dt<=0.2` 的保守设计策略，而非硬件物理极限。
- 延时检查采用理想消零模型 `PM_model≈90°-ωc*Td*180/π`；这是预算检查，不能当成实测相位裕量。
- 目标超出预算直接返回具体拒绝状态，不静默降低带宽；失败输出清零。
- 同时拒绝空指针、非有限数、非正 R/L/dt/带宽、不合理裕量和结果溢出。

输出 d/q 理论增益，以及当前共享增益结构可用的 `shared` 候选值。shared 采用 `min(Ld,Lq)` 计算 Kp，Ki 不变，意在避免较小电感轴的比例增益按较大电感配置；另一轴不能保证达到同一目标带宽，也不能保证达到分轴消零模型的裕量。两轴都需要实际验证。

## 上层调用示例

```c
#include "motor_autotune.h"

/* 以下仅为仿真示例，不是实机建议参数。 */
motor_autotune_config_t config = {
    .resistance_ohm = 0.5f,
    .ld_h = 0.001f,
    .lq_h = 0.002f,
    .control_dt_s = 1.0f / 15000.0f,
    .bandwidth_hz = 150.0f,
    .total_delay_s = 1.5f / 15000.0f,
    .min_phase_margin_deg = 60.0f,
};
motor_autotune_result_t candidate;
motor_autotune_status_t status = Motor_Autotune_CurrentPi(&config, &candidate);
/* status==MOTOR_AUTOTUNE_OK 时仅获得候选值，后续验证/应用由上层明确处理。 */
```

实际调用方必须先确认 `motor_detect_result_t` 对应检测成功且 R/L 有效；不得把无效结果、默认零值或另一电机的旧结果作为输入。检测电感是有限脉冲工作点估计，不是精确全工况增量电感，且当前检测保留经验缩放，整定不能因此被视为已校准。

控制周期应从实际控制频率获得，不能将 ADC 基准 30 kHz 直接当成分频后的 PI 频率。未核实总延时前，模块不能替上层给出可靠硬件带宽。

## 板上只读命令入口

`App/cmd/cmd_autotune.c` 为独立应用包装，通过命令表注册：

```text
pitune <bandwidth_Hz> <total_delay_us> [min_margin_deg=60]
```

使用顺序：先对当前连接电机执行 `detect rl [test_current_A]`，确认检测成功，再输入目标带宽和已核实的总有效延时。`pitune` 本身不启动检测，也不会让电机转动。

命令要求运行命令与状态均停机、无故障、无检测占用；结果版本、R/L 有效标志、检测错误码及故障码都须通过。频率取 HAL 的 ADC 采样频率，再按现有 FOC 分频计算 PI 周期；HAL 无有效频率时直接拒绝，不使用配置回退值冒充实测频率。

总延时必须显式填写。输出分轴 Kp/Ki、共享候选值、周期、来源 R/L、采样与延时带宽预算。理想相位裕量会注明并非实测，Ki 未预乘 dt。命令不支持 apply/save，也不写入运行增益。

RAM 检测结果当前没有电机序列号/配置代次绑定，命令无法自动识别是否换过电机；换电机或检测条件发生变化后必须重新检测。不能仅凭 valid 标志把旧结果用于新电机。

## 验证与后续

- `Tests/host/run_motor_autotune.ps1`：验证正常增益、采样/延时超限、NaN/Inf、负参数、输出溢出、失败清零和空指针。
- 用工程真实 `Motor_Current_PI_Run`、连续 RL 的解析 ZOH 步进和一拍输出延迟验证 d/q 及共享候选值的小电流阶跃，检查稳态跟踪与过冲。
- 同一测试入口编译真实 `cmd_autotune.c`，用主机替身验证成功输出、缺失结果、检测占用、运行、故障、HAL 频率无效、检测中止、非法字符串和超限拒绝；比较前后完整电机对象，确认命令不修改状态或增益。
- 模块及命令接入后 Keil 编译链接通过，0 错误、0 警告；未烧录、未做板上命令验证。
- Keil 工程新增本模块源文件引用；不改变现有控制环调用和参数。
- 未上板测频响或进行自动应用。后续可在独立应用编排中增加小阶跃验收，再扩展闭环频响修正；不得把当前模型预算写成实测性能。
