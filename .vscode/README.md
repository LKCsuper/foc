# VS Code 代码跳转

用 VS Code 打开仓库根目录，使用 Microsoft C/C++ 扩展。

导航配置来自 `App/Project/foc.uvprojx` 的 `Foc` 目标：同步宏、按顺序列出的头文件路径，以及工程源文件所在目录。索引目录不递归，避免把参考仓库和其他 FreeRTOS 移植一起收进来。工作区另行排除 Boot、测试、示例及 VESC/AM32 等参考实现的 C/C++ 索引；文件仍可正常浏览。

首次应用后执行命令面板中的 `C/C++: Reset IntelliSense Database`，再执行 `Developer: Reload Window`，清除原有全仓索引。正常使用 F12 / Ctrl+单击跳转。

Keil 增删文件、包含目录或宏后，运行任务“同步 Keil 代码跳转配置”，或在根目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Scripts/update_vscode.ps1
```

`c_cpp_properties.json` 为生成文件，优先调整 Keil 工程后同步。Keil 已不存在的包含目录会提示并跳过。

这里使用 ARM 语义解析，并保留 Keil 宏；未配置本机编译器查询，编辑器诊断不等同于 Keil 编译结果。该配置针对主固件，开发 Boot 或主机测试时需使用对应目标的独立配置。如果仍出现多组候选，可通过 `C/C++: Log Diagnostics` 检查当前配置是否为 `STM32F405 - Foc`，并检查是否同时启用了其他 C/C++ 跳转扩展。
