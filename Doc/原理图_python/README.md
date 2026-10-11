# 当前代码控制原理图（Python 绘制）

**先看物理与控制原理：[原理图阅读.md](原理图阅读.md)。**

principles/ 包含磁场、转矩、负反馈、电路、时序和检测关系共 12 组原理图；draw_principles.py 用于重新生成。下面 images/ 的图件是源码结构与流程辅助说明。

依据：2026-10-09 工作区实际源码，包含已有未提交修改。只新增此目录，未修改固件或外设基线。

## 阅读与重新生成

安装 Python、matplotlib、numpy、Pillow 后，在本目录运行：

```powershell
python generate_diagrams.py
```

images/ 同时包含 PNG 和可放大编辑的 SVG；source_hashes.json 记录源码 SHA-256；module_inventory.json 列出项目电机实现文件与直接对应图件。

## 范围和约定

覆盖电机控制、启动、采样、传感器、外环、无感、BLDC、校准检测、保护、补偿及存储。第三方库内部、串口驱动、RTOS 内核及历史 motor_simple_foc 示例不作为另一套主控制实现展开。流程图表达模块原理，不逐个绘制打印/包装函数。

流程页按 1→2→3→4→5→6 阅读；明确标注的条件分支与独立补偿不是无条件串联。曲线图是给定参数的教学示意，不是仿真验收或硬件实测。

特别注意：正常 FOC 先筛选控制拍再采样/保护；速度 PI 在任务运行；快速 V/F 是恒 boost 电压诊断；逆 Park 整体负号、线性 SVM 二次取负和 A-C-B 提交必须整体理解。SAFE 是中点占空比，不能推断关桥。

独立 flux/Luenberger/SMO/EKF/HFI 模块的存在不代表默认主路径正在运行；主 sensorless 当前直接实现 Ortega/PLL。

## 01_main 中断分派与正常 FOC 控制拍

![中断分派与正常 FOC 控制拍](images/01_main.png)

速度/位置/关节外环在 foc pid 任务，电流环在 ISR；筛选发生在正常路径采样与软件保护之前。

源码：[App/motor/motor_it.c](../../App/motor/motor_it.c)、[App/motor/motor_foc_isr.c](../../App/motor/motor_foc_isr.c)、[App/motor/motor_foc_divider.c](../../App/motor/motor_foc_divider.c)

## 02_sampling ADC 物理量、板级相序与电流重构

![ADC 物理量、板级相序与电流重构](images/02_sampling.png)

最大 duty 并列时不重构；BM75 约 0.0806 A/count，0.1～0.2 A 的精确跟踪无法由单拍读数证明。

源码：[App/motor/motor_sense.c](../../App/motor/motor_sense.c)、[App/motor/motor_current.c](../../App/motor/motor_current.c)、[Library/Hal/port/stm32f4/hal_motor_stm32f4.c](../../Library/Hal/port/stm32f4/hal_motor_stm32f4.c)

## 03_current d/q 电流 PI 与两级抗饱和

![d/q 电流 PI 与两级抗饱和](images/03_current.png)

电流目标单位 A、电压 V、dt 秒。对齐定角 Id 建立磁场，正常 Iq 提供转矩。

源码：[App/motor/motor_current.c](../../App/motor/motor_current.c)、[App/motor/motor_pid.c](../../App/motor/motor_pid.c)、[App/motor/motor_control.c](../../App/motor/motor_control.c)

## 04_output 逆 Park 极性、调制选择与 PWM 提交

![逆 Park 极性、调制选择与 PWM 提交](images/04_output.png)

图内两个调制节点是条件分支；SAFE 三相中点不等于关桥。无感记录实际调制约定下的电压。

源码：[App/motor/motor_output.c](../../App/motor/motor_output.c)、[App/motor/motor_foc.c](../../App/motor/motor_foc.c)

## 05_task 任务、编码器采集与快环之间的数据流

![任务、编码器采集与快环之间的数据流](images/05_task.png)

PID dt 按真实经过时间取得并钳位；编码器总线事务不进入 ADC ISR。

源码：[App/motor/motor_task.c](../../App/motor/motor_task.c)、[App/motor_sensor/motor_mt6701.c](../../App/motor_sensor/motor_mt6701.c)、[App/motor/motor_control.c](../../App/motor/motor_control.c)

## 06_vf 当前快速 V/F 诊断：恒 Vq 旋转磁场

![当前快速 V/F 诊断：恒 Vq 旋转磁场](images/06_vf.png)

此实际诊断路径不是经典 V=kf 曲线；最低转速地板是此历史路径行为，不能套到正常 I/F。

源码：[App/motor/diagnostic/motor_fast_vf.c](../../App/motor/diagnostic/motor_fast_vf.c)、[App/motor/diagnostic/motor_fast_vf_isr.c](../../App/motor/diagnostic/motor_fast_vf_isr.c)

## 07_start I/F 启动：电流建立、保持与拖转

![I/F 启动：电流建立、保持与拖转](images/07_start.png)

ol_align_enable=0 跳过前两阶段。正常 I/F 不把真实角速度瞬间抬到 ol_min_rpm。

源码：[App/motor/motor_control.c](../../App/motor/motor_control.c)、[App/motor/start/motor_if_start.c](../../App/motor/start/motor_if_start.c)、[App/motor/start/motor_alignment.c](../../App/motor/start/motor_alignment.c)

## 08_encoder MT6701 编码器：回绕、PLL 与电角

![MT6701 编码器：回绕、PLL 与电角](images/08_encoder.png)

近零速度吸附前的 raw RPM 单独保留；电角零点有效性与速度有效性分开处理。

源码：[App/motor_sensor/motor_encoder.c](../../App/motor_sensor/motor_encoder.c)、[App/motor_sensor/motor_encoder_detect.c](../../App/motor_sensor/motor_encoder_detect.c)

## 09_hall 霍尔角度与速度反馈

![霍尔角度与速度反馈](images/09_hall.png)

表值 raw/200×2π 得电角；霍尔的角度量化与低速边沿稀疏决定反馈精度。

源码：[App/motor_sensor/motor_hall_angle.c](../../App/motor_sensor/motor_hall_angle.c)、[App/motor_sensor/motor_hall.c](../../App/motor_sensor/motor_hall.c)

## 10_speed 速度 PI、制动语义与饱和泄放

![速度 PI、制动语义与饱和泄放](images/10_speed.png)

禁止制动时零速命令自然滑停；允许制动时检测过零后释放；位置保持具有独立语义。

源码：[App/motor/motor_speed.c](../../App/motor/motor_speed.c)、[App/motor/motor_control.c](../../App/motor/motor_control.c)

## 11_position 多圈位置控制：大误差移动与电压保持

![多圈位置控制：大误差移动与电压保持](images/11_position.png)

纯位置控制模块输出电压；应用层根据误差滞回选择移动/保持。不能简化成所有工况都三环串联。

源码：[App/motor/motor_position.c](../../App/motor/motor_position.c)、[App/motor/motor_position_control.c](../../App/motor/motor_position_control.c)

## 12_joint 关节阻抗与低电流混合后端

![关节阻抗与低电流混合后端](images/12_joint.png)

纯算法 output 单位由配置决定；应用层映射为 V 或 A。切换是命令需求阈值，不能等同实测电流精度。

源码：[App/motor/joint/motor_joint_control.c](../../App/motor/joint/motor_joint_control.c)、[App/motor/joint/motor_joint.c](../../App/motor/joint/motor_joint.c)

## 13_trajectory 梯形轨迹：按剩余制动距离提前减速

![梯形轨迹：按剩余制动距离提前减速](images/13_trajectory.png)

轨迹生成器给外环参考，不直接写 PWM；反向运动先按减速规则处理。

源码：[App/motor/motor_trajectory.c](../../App/motor/motor_trajectory.c)

## 14_flux Ortega 磁链观测与主路径 PLL

![Ortega 磁链观测与主路径 PLL](images/14_flux.png)

重要差异：主 sensorless 实现将 ε>0 截为 0；独立 motor_flux 保留双向误差。

源码：[App/motor/observer/motor_sensorless.c](../../App/motor/observer/motor_sensorless.c)、[App/motor/observer/motor_flux.c](../../App/motor/observer/motor_flux.c)

## 15_lbg 独立龙伯格反电势观测器

![独立龙伯格反电势观测器](images/15_lbg.png)

独立算法模块不等于默认主控制入口；图示为此文件的具体实现。

源码：[App/motor/observer/motor_luenberger.c](../../App/motor/observer/motor_luenberger.c)

## 16_smo 独立 SMO：残差注入到反电势预测

![独立 SMO：残差注入到反电势预测](images/16_smo.png)

当前代码没有在电流方程再减满幅 z，不能套用常见 textbook SMO 框图。

源码：[App/motor/observer/motor_smo.c](../../App/motor/observer/motor_smo.c)

## 17_ekf 独立 EKF：四状态预测与电流观测

![独立 EKF：四状态预测与电流观测](images/17_ekf.png)

代码每拍直接加配置 Q，图未把 Q 改写为 Q×dt；矩阵异常时复位并声明输出不可用。

源码：[App/motor/observer/motor_ekf.c](../../App/motor/observer/motor_ekf.c)

## 18_hfi HFI 扫描起步与低速凸极跟踪

![HFI 扫描起步与低速凸极跟踪](images/18_hfi.png)

扫描与跟踪是两个模块；依赖凸极/饱和响应，存在性不代表任意电机上已验收。

源码：[App/motor/start/motor_hfi_start.c](../../App/motor/start/motor_hfi_start.c)、[App/motor/observer/motor_hfi.c](../../App/motor/observer/motor_hfi.c)

## 19_bldc BLDC 六步与 BEMF 面积换相

![BLDC 六步与 BEMF 面积换相](images/19_bldc.png)

采用 BEMF 积分面积，不能画成单纯过零延时 30°；前半扇区避免钳位区，后半扇区允许积分。

源码：[App/motor/bldc/motor_bldc_sensorless.c](../../App/motor/bldc/motor_bldc_sensorless.c)、[App/motor/bldc/motor_bldc_six_step.c](../../App/motor/bldc/motor_bldc_six_step.c)、[Library/Hal/port/stm32f4/hal_motor_bldc_stm32f4.c](../../Library/Hal/port/stm32f4/hal_motor_bldc_stm32f4.c)

## 20_detect 偏移、R/L/磁链检测

![偏移、R/L/磁链检测](images/20_detect.png)

检测过程有保护、超时及退出恢复；源码文件头存在旧说明，本图以实际函数体为依据。 偏移/R/L/磁链为按命令分别执行的检测分支，不是必经串联。

源码：[App/motor_param/motor_detect.c](../../App/motor_param/motor_detect.c)、[App/motor_param/motor_detect_offset.c](../../App/motor_param/motor_detect_offset.c)、[App/motor/calibration/motor_detect_isr.c](../../App/motor/calibration/motor_detect_isr.c)

## 21_protection 保护去抖、首因锁存与最终仲裁

![保护去抖、首因锁存与最终仲裁](images/21_protection.png)

无可靠速度反馈时不进行堵转/失速判定；软件 SAFE 为中点电压语义，硬件短路保护不由此图验收。

源码：[App/motor/motor_protect.c](../../App/motor/motor_protect.c)、[App/motor/motor_foc_isr.c](../../App/motor/motor_foc_isr.c)、[App/motor/motor_control.c](../../App/motor/motor_control.c)

## 22_storage 双槽 Flash：最后写提交标记

![双槽 Flash：最后写提交标记](images/22_storage.png)

不保存运行 enable、命令目标、PI 积分或指针；版本兼容以当前解码函数为准。

源码：[App/motor_param/flash_storage.c](../../App/motor_param/flash_storage.c)、[App/motor_param/motor_settings.c](../../App/motor_param/motor_settings.c)

## 23_compensation 弱磁、速度前馈拟合与抗齿槽

![弱磁、速度前馈拟合与抗齿槽](images/23_compensation.png)

三个节点组代表三种独立补偿；不表示它们必须串联。抗齿槽补偿当前以电压表表示。

源码：[App/motor/motor_fw.c](../../App/motor/motor_fw.c)、[App/motor/motor_speed_ff_detect.c](../../App/motor/motor_speed_ff_detect.c)、[App/motor/motor_anticogging.c](../../App/motor/motor_anticogging.c)

## 24_commands 命令、状态快照与关节固定帧

![命令、状态快照与关节固定帧](images/24_commands.png)

命令一致性不依靠 volatile 保证；状态快照与控制内部积分分离。

源码：[App/motor/motor.c](../../App/motor/motor.c)、[App/motor/motor_snapshot.c](../../App/motor/motor_snapshot.c)、[App/motor/joint/motor_joint_protocol.c](../../App/motor/joint/motor_joint_protocol.c)、[App/motor/joint/motor_joint.c](../../App/motor/joint/motor_joint.c)

## Clarke/Park 波形

![Clarke/Park 波形](images/25_transform_waveforms.png)

## 电压矢量圆限幅

![电压矢量圆限幅](images/26_voltage_circle.png)

## 六步桥臂表

![六步桥臂表](images/27_bldc_table.png)

## BEMF 积分面积

![BEMF 积分面积](images/28_bemf_area.png)

## 编码器回绕与低电流量化

![编码器回绕与低电流量化](images/29_wrap_quantization.png)

## 静止坐标与旋转坐标几何投影

![静止坐标与旋转坐标几何投影](images/30_coordinate_geometry.png)

## SVPWM 空间矢量六边形与线性区

![SVPWM 空间矢量六边形与线性区](images/31_svpwm_hexagon.png)

