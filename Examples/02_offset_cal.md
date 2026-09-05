# 示例 02：电流/电压偏移校准

## 接线与参数前提

- 功率级已供电；相电流采样偏置接近中点（默认 ADC 中点约 2045，见 `Motor_Sense`）。
- 校准期间电机应静止、无外部拖转。
- 上电路径：`Detect_Init` → `DETECT_Offset`（包装）→ 核心 `Motor_Detect_Offset_*`。

## 调用顺序（上板）

1. 上电完成偏移校准（约数秒，有日志）。
2. Shell：`olstatus` 确认未故障、偏移已写入校准侧。
3. 需要保存时：停机后 `saveparam`（仅配置/校准 payload，见根 README）。

## 主机对应

- `Tests/host/test_motor_detect_offset.c`：FeedDriven / Undriven / Finalize / Apply，无 RTOS、无真实 ADC。
- 断言：驱动/未驱动窗口均值写入校准字段；不验证真实噪声与 5 s 延时。

## 观测量 / 预期

- 校准后静止相电流读数接近 0 A（相对偏移）。
- 母线/相电压偏移合理，不出现持续饱和。

## 常见失败

- 校准时已有相电流或 PWM 非安全态 → 偏移偏大，闭环起步易抖。
- 把主机 Finalize 通过当成「板级偏置已正确」。
- 校准中过流/欠压锁存：ISR 会 `PwmStop`，任务返回 0；须 `olfault clear` 后重跑 `detect offset`，勿在故障下强行起转。
