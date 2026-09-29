# 示例 04：有感电流闭环

## 接线与参数前提

- 编码器：AB（及 Z 若用）；或霍尔三相，表已标定（6 有效扇区）。
- 先完成开环能转（示例 03），再切有感。
- 编码器：`Motor_Encoder_Detect` 置零点；霍尔：表 ready 且解码有效。
- MT6701 使用 AB 输出时，将芯片设为 1024 PPR；PB6/PB7 经 TIM4 正交 4 倍频后为 4096 count/rev。
- `encoder show` 可查看滤波/原始 RPM、最近边沿时间、零点有效、信号跳变故障和累计拒绝样本。

## 调用顺序（上板）

1. `olmode 1`（或当前工程约定的有感相关模式，以 Shell 帮助为准）。
2. 编码器：完成 Detect；霍尔：确认标定表。
3. `olfb encoder` 或 `olfb hall`。
4. `oliq` / `olstart`；**先不要** `spenable`（本示例只验电流环用有感角）。
5. `olstatus` / 观测 Id/Iq 与电角是否跟随传感器。

## 主机对应

- `test_motor_encoder` / `test_motor_hall_angle` / `test_motor_feedback`。
- `test_motor_control`：encoder 源下 Step 使用有感 theta，且跳过无感 Update。

## 观测量 / 预期

- 角有效时电流环 θ 来自编码器电角或霍尔扇区角。
- 角/零点无效时**不得**当有效零速；回退策略见 `motor_feedback`。

## 常见失败

- 未 Detect 零点就 `olfb encoder` → 角无效。
- MT6701 若使用 SSI/I²C 绝对角，需另接串行接口并校验磁场状态/CRC；当前 AB 路径无法读取芯片诊断位。
- 霍尔表未 ready 或非法扇区 → 角无效、易失步。
- 极对数与机械角换算错误 → 电流环角漂。
