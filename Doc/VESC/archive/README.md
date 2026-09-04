# VESC 归档

本目录存放已从主工程剥离的 VESC 遗留头文件，**不参与编译**。

| 文件 | 说明 |
|------|------|
| `datatypes_full.h` | 原 `App/motor/inc/datatypes.h`（~1400 行类型） |
| `mcconf_default.h` | 原 `App/motor/inc/mcconf_default.h` |

当前工程公共类型见 `App/motor/inc/motor_types.h`，运行时状态见 `MOTOR_PARAM_S`。

参考实现：`vesc_foc.c`（上级目录）。
