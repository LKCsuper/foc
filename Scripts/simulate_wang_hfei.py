"""论文 HFEI 原理级仿真：连续 RL 精确步进、ZOH、延时与同步 DFT。"""
import cmath
import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'Doc/verify/wang_hfei_sim'
R, L, FS, U = 0.5, 0.001, 30000, 1.0
DELAY = 2  # 仿真设定，不代表硬件实测延时


def simulate(freq, delay=0, voltage_gain=1.0):
    # 每个采样时刻记录当前电流；指令经 delay 个周期后作用于下一段区间。
    # RL 在区间内保持电压不变，使用解析解，避免欧拉积分数值误差。
    period = round(FS / freq)
    assert abs(FS / period - freq) < 1e-9
    settle = max(period * 40, round(0.08 * FS))
    count = settle + period * 40
    a = math.exp(-R / L / FS)
    history = []
    current = 0.0
    vc = ic = 0j
    for k in range(count):
        command = U * math.cos(2 * math.pi * freq * k / FS)
        history.append(command)
        if k >= settle:
            basis = cmath.exp(-2j * math.pi * freq * k / FS)
            vc += command * basis
            ic += current * basis
        applied = history[k-delay] * voltage_gain if k >= delay else 0.0
        current = a * current + (1-a) / R * applied
    raw = vc / ic
    # 精确离散 RL 的稳态传递函数；用它核对模拟波形的 DFT。
    omega = 2 * math.pi * freq
    step = cmath.exp(1j * omega / FS)
    predicted = (step-a) / ((1-a)/R) * step**delay / voltage_gain
    assert abs(raw-predicted) / abs(predicted) < 1e-7
    # 方案1：按连续模型只补偿相位；ZOH 等效还约有半采样周期延时。
    phase_only = raw * cmath.exp(-1j * omega * (delay + 0.5) / FS)
    # 方案2：在已知 voltage_gain 的条件下，使用精确离散模型反求 R/L。
    physical_z = raw * voltage_gain / step**delay
    imag = physical_z.imag
    assert imag > 0
    a_fit = step.real - step.imag * physical_z.real / imag
    b_fit = step.imag / imag
    r_fit = (1-a_fit) / b_fit
    l_fit = -r_fit / FS / math.log(a_fit)
    return raw, phase_only, r_fit, l_fit


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for freq in [300, 500, 1000, 1500, 3000]:
        for name, delay, gain in [('ideal_ZOH', 0, 1), ('delay_2samples', DELAY, 1),
                                  ('delay_and_gain_0.8', DELAY, .8)]:
            raw, phase, rf, lf = simulate(freq, delay, gain)
            rows.append([name, freq, raw.real, raw.imag/(2*math.pi*freq),
                         phase.real, phase.imag/(2*math.pi*freq), rf, lf])
            assert abs(rf/R-1) < 1e-6 and abs(lf/L-1) < 1e-6
    with (OUT/'results.csv').open('w', newline='', encoding='utf-8-sig') as file:
        writer = csv.writer(file)
        writer.writerow(['scenario', 'frequency_Hz', 'raw_R_ohm', 'raw_L_H',
                         'phase_corrected_R_ohm', 'phase_corrected_L_H',
                         'known_gain_discrete_corrected_R_ohm', 'known_gain_discrete_corrected_L_H'])
        writer.writerows(rows)
    # SVG 为可独立查看的标准矢量图，无第三方绘图库依赖。
    colors = ['#2463eb', '#e35d30', '#15946b']
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="620" viewBox="0 0 1000 620">',
           '<rect width="1000" height="620" fill="white"/>',
           '<g font-family="Arial" font-size="16" fill="#263449">',
           '<text x="65" y="35" font-size="24">HFEI: inductance error vs injection frequency</text>',
           '<text x="65" y="62">R = 0.5 ohm, L = 1 mH, sampling = 30 kHz; exact RL + ZOH</text>']
    def point(freq, error):
        return 80+(freq-300)/2700*840, 480-(error+110)/150*380
    for err in [-100, -75, -50, -25, 0, 25]:
        y = point(300, err)[1]
        svg += [f'<path d="M80 {y} H920" stroke="#e2e8f0"/>',
                f'<text x="30" y="{y+5}">{err}%</text>']
    for freq in [300, 500, 1000, 1500, 3000]:
        x = point(freq, 0)[0]
        svg.append(f'<text x="{x-15}" y="515">{freq}</text>')
    for index, name in enumerate(['ideal_ZOH', 'delay_2samples', 'delay_and_gain_0.8']):
        selected = [row for row in rows if row[0] == name]
        for col, dashed in [(3, False), (5, True)]:
            points = ' '.join(f'{x:.2f},{y:.2f}' for x, y in
                              [point(row[1], (row[col]/L-1)*100) for row in selected])
            style = ' stroke-dasharray="7 5"' if dashed else ''
            svg.append(f'<polyline points="{points}" fill="none" stroke="{colors[index]}" stroke-width="3"{style}/>')
        svg.append(f'<text x="{65+index*310}" y="560" fill="{colors[index]}">{name}</text>')
    svg += ['<text x="65" y="595">Solid: raw. Dashed: phase-only correction. Exact calibrated correction: ~0% (CSV).</text>', '</g></svg>']
    (OUT/'inductance_error.svg').write_text('\n'.join(svg), encoding='utf-8')
    report = ['# 王奇维论文 HFEI 核心原理仿真', '',
              '本仿真为方法级复现，不是论文整套算法或当前硬件的数字孪生。', '',
              '## 模型与计算', '',
              '- 连续 RL：R=0.5 Ω、L=1 mH，正弦电压指令峰值 1 V。',
              '- 30 kHz 采样和零阶保持，按 RL 解析解逐步演化电流。',
              '- 延时人为设为 2 个采样周期；幅值误差场景实际电压为指令的 80%。',
              '- 每种工况先稳定至少 80 ms 且不少于 40 个周期，再用 40 个整周期同步 DFT。',
              '- 原始估计：Z=U/I，R=Re(Z)、L=Im(Z)/(2πf)。',
              '- 相位补偿：扣除延时和 ZOH 近似半周期相位，仍存在离散幅相误差。',
              '- 精确离散补偿：已知实际电压比例与整数延时后，按离散 RL 反求参数。', '',
              '## 结果', '',
              '| 工况 | 频率 Hz | 原始 R Ω | 原始 L mH | 相位补偿 L mH | 已知增益后精确离散补偿 L mH |',
              '| --- | --- | --- | --- | --- | --- |']
    for row in rows:
        report.append(f'| {row[0]} | {row[1]} | {row[2]:.5f} | {row[3]*1000:.5f} | {row[5]*1000:.5f} | {row[7]*1000:.5f} |')
    report += ['', '![电感误差](inductance_error.svg)', '', '## 结论与边界', '',
               '1. 即使 RL 参数完全不变，数字保持和延时也能造成辨识结果随频率变化。',
               '2. 只纠正相位不能消除电压幅值比例误差；实际端电压必须被正确估计。',
               '3. 高频下阻抗实部对相位偏差敏感，原始估计甚至可能出现负电阻；这不是电机真实负电阻。',
               '4. 精确补偿恢复真值依赖于已知延时、已知电压增益、无噪声及准确 RL 模型，属于数学一致性验证，不是硬件精度保证。',
               '5. 尚未加入磁饱和、交叉耦合、转子运动、PWM 开关纹波、死区换向非线性及电流环作用。',
               '6. 下一步可逐项加入非线性与测量噪声，再考虑完整 PMSM 多工作点仿真。', '',
               '## 运行', '',
               '从仓库根目录运行 `python Scripts/simulate_wang_hfei.py`。仅依赖 Python 标准库。',
               '脚本对时域 DFT 与精确离散频响作交叉核验，并检查已知条件补偿后的 R/L 恢复误差小于 1e-6（相对值）。', '',
               '输出：results.csv、inductance_error.svg 与本文。不会修改或应用固件参数。']
    (OUT/'README.md').write_text('\n'.join(report)+'\n', encoding='utf-8')
    print('Simulation and analytic cross-checks passed:', OUT)


if __name__ == '__main__':
    main()
