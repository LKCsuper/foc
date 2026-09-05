# 示例 05：有感速度闭环

## 接线与参数前提

- 示例 04 已能有感稳转；`speed_valid` 与 `angle_valid` 同时为真才会发布有感转速。
- 速度 PI / max RPM 等经配置 API；改停机类字段须先停机。

## 调用顺序（上板）

1. `olfb encoder|hall` 且角、速度有效。
2. `olstart` 后再 `spenable 1`（允许有感源，不再强制 `slenable`）。
3. `olspeed` 改目标；观察转速跟踪与 Iq 由速度环给出。
4. `spenable 0` / `olstop` 退出。

## 主机对应

- `test_motor_control`：`speed_enable` 时电流环吃到速度 Iq（如 1.25）；角失效则速度反馈关闭。
- `test_motor_speed`：速度环状态机本身。

## 观测量 / 预期

- `PublishSensoredSpeed` 仅在 `angle_valid && speed_valid` 时发布。
- 反馈失效不会被当成「有效零速」继续强拧。

## 常见失败

- 低速霍尔换向稀疏 → `speed_valid` 抖动，Iq 乱跳；先提高最低转速或滤波。
- 编码器线数/极对数填错 → RPM 差倍数。
- 主机 Iq 断言通过 ≠ 实机带宽与负载合适。
