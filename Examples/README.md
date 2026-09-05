# 最小学习示例

按下面顺序阅读；**复用同一套 `App/` 核心算法**，不另写第二套 Clarke/Park/PI。

| 顺序 | 文档 / 代码 | 主机可跑？ | 上板？ |
| --- | --- | --- | --- |
| 1 | [01_transform](01_transform/) | 是（本目录可执行） | 否 |
| 2 | [02_offset_cal.md](02_offset_cal.md) | 部分（`test_motor_detect_offset`） | 是（偏移校准） |
| 3 | [03_voltage_openloop.md](03_voltage_openloop.md) | 部分（control/startup host） | 是（电压开环） |
| 4 | [04_sensored_current.md](04_sensored_current.md) | 部分（feedback/encoder/hall/control） | 是（有感电流） |
| 5 | [05_sensored_speed.md](05_sensored_speed.md) | 部分（control 速度 Iq） | 是（有感速度） |

## 主机通过 vs 上板通过

- **主机通过**：在 PC 上 `Tests/host/run.ps1`（及示例 01）编译并断言通过。不证明 PWM、采样时序、方向与实机力矩正确。
- **上板通过**：Keil 烧录后按各示例「观测量 / 预期」在电机与示波器上确认。本仓库当前**未**把步骤 0～5 标为上板验收完成。

一键主机回归：仓库根目录执行 `Tests/host/run.ps1`（末尾会编示例 01）。详见 [Tests/host/README.md](../Tests/host/README.md)。

## 与旧学习代码的关系

- `App/motor_simple_foc/`：早期独立学习实现，**未接入** `Motor_DmaInt` 主链；保留作对照，**不要**再复制一套算法进 Examples。
- 主学习路径以本目录文档 + 生产模块为准。

## 推荐阅读

根目录 [README.md](../README.md) → [AI_CODE.md](../AI_CODE.md) §11 → [代码阅读指南](../Doc/doc/代码阅读指南.md) → 本目录 01→05。
