# 示例 03：电压开环（I/F / V/F）

## 接线与参数前提

- 完成示例 02 偏移；相序与编码器/霍尔（若已接）方向稍后核对。
- 默认 `olmode` 与 I/F 电流、对齐电流见根 README；先用保守 `olspeed` / `oliq`。
- 反馈默认开环角；本示例不要求 `olfb`。

## 调用顺序（上板）

1. `olstatus` 确认 enable=0、无故障。
2. 按需 `olmode` / `oliq` / `olspeed`。
3. `olstart` → 观察电流与转速爬升。
4. `olstop`；故障时 `olfault clear` 前先查原因。

## 主机对应

- `test_motor_startup`：对齐 / I/F 分频与状态隔离。
- `test_motor_control`：斜坡、限幅、双实例、仲裁 SAFE。
- `test_motor_api`：命令提交与 PWM Start/Safe 桥同步。

## 观测量 / 预期

- 电机按命令方向旋转；`olstatus` 中模式与目标一致。
- 故障锁存后输出保持 SAFE（中点占空比，≠硬件关桥）；`cmd_duty` 在 enable/故障时拒绝。

## 常见失败

- 相序反：振动大或反转；先降 Iq/速度再查接线。
- 把 SAFE 中点当成「桥已关断」→ 需另走 `PwmStop` 语义（见 HAL 注释）。
