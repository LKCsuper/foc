# 主机测试

在仓库根目录运行 `Tests/host/run.ps1 -Compiler <zig.exe路径>`。脚本面向 Windows；Flash 测试使用 VirtualAlloc 模拟地址范围，不访问实际板子。

可选本地工具安装：`python -m pip install --target .build/tools ziglang==0.16.0`，随后运行 `Tests/host/run.ps1`。工具不属于固件依赖，`.build/` 不进入版本库。

测试一编译真实 settings 与初始化模块，存储入口为替身；测试二编译真实 settings 与 Flash 模块，只有硬件 Flash 操作和 CRC 服务被模拟。断言检查状态隔离、错误输入与双槽中断恢复，不验证真实擦写延时、PWM、ADC 或电机行为。

新编码的字段順序冻结在 `motor_settings.c` Encode/Decode 中；改变字段顺序或类型必须升级 `FLASH_STORAGE_FORMAT_VERSION` 并更新测试和迁移说明。
