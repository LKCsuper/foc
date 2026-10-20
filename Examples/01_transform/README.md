# 示例 01：纯坐标变换

## 前提

- 无需接线、无需 MCU。
- 使用生产代码：`App/motor/algo/motor_foc.c` + `motor_utils.c`（`sin_f32`/`cos_f32`）。

## 调用顺序

1. 给定平衡三相电流 `Ia,Ib,Ic`。
2. `clarke_transform` → `Iα,Iβ`。
3. `park_transform(..., θ)` → `Id,Iq`。
4. `inverse_park` 还原 `αβ`，与 Clarke 结果对照。
5. 可选：`svm` 检查小幅 `αβ` 占空比落在 `[0,1]`。

## 观测量 / 预期

- θ=0、`Ia=1,Ib=Ic=-0.5`：`Id≈1`，`Iq≈0`（容差约 1e-3）。
- 逆 Park 后 `αβ` 与 Clarke 一致。
- `svm` 对小矢量返回 0。

## 常见失败

- 链接缺 `motor_utils.c`：`sin_f32` 未定义。
- 三相不平衡时 Clarke 与「两电阻假设」路径不一致（本示例用三电阻宏默认）。

## 构建

由 `Tests/host/run.ps1` 自动编译运行，或：

```powershell
# 在仓库根目录；zig.exe 路径按本机调整
zig cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" -IApp/motor/inc `
  App/motor/algo/motor_foc.c App/motor/algo/motor_utils.c Examples/01_transform/main.c `
  -o .build/host/example_01_transform.exe
.\.build\host\example_01_transform.exe
```
