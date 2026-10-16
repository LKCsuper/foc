"""生成电机控制知识补充图；教学模型，不读写固件或连接硬件。"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "knowledge"
OUT.mkdir(exist_ok=True)
plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                     "axes.unicode_minus": False, "svg.fonttype": "none"})
gallery = []


def page(name, title):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))
    fig.suptitle(title, fontsize=19, weight="bold", color="#245b8a")
    for ax in axes:
        ax.grid(alpha=.2)
    return fig, axes


def save(fig, name):
    fig.text(.025, .025, "教学示意：采用明确简化模型；不是硬件实测，也不代表固件全部已实现。", fontsize=10)
    fig.tight_layout(rect=(0, .07, 1, .90))
    fig.savefig(OUT / f"{name}.png", dpi=150)
    fig.savefig(OUT / f"{name}.svg")
    plt.close(fig)
    gallery.append(name)


fig, (a, b) = page("K01", "K01：机械速度、电气速度与量纲")
rpm = np.linspace(0, 3000, 301)
for p in (1, 7, 14):
    a.plot(rpm, p * rpm / 60, label=f"{p} 极对")
a.set(xlabel="机械转速 RPM", ylabel="电气频率 Hz")
a.legend()
b.axis("off")
b.text(.03, .9, "7 极对、2600 RPM\n\n电气转速 = 18200 eRPM\n电气频率 = 303.3 Hz\n电角速度 = 1905.9 rad/s\n\n1266 μH = 1.266 mH = 0.001266 H\n3.103 mWb = 0.003103 Wb\n\n角度、速度、参数必须先统一单位", va="top", fontsize=15, linespacing=1.5)
save(fig, "K01_units")

fig, (a, b) = page("K02", "K02：电流限制与电压限制是两件事")
r, l, psi, p = 3.808156, .001144148, .0031033, 7
omega = p * rpm * 2 * np.pi / 60
for iq in (.15, .30):
    vd = -omega * l * iq
    vq = r * iq + omega * psi
    a.plot(rpm, np.hypot(vd, vq), label=f"Id=0，Iq={iq:.2f} A")
a.axhline(8, color="red", linestyle="--", label="8 V 测试限制")
a.set(xlabel="机械转速 RPM", ylabel="稳态 dq 电压幅值 V")
a.legend()
required = .04 + .000000035 * rpm**2
b.plot(rpm, required, label="假设空载损耗需要的 Iq")
b.axhline(.15, linestyle="--", label="旧限流 0.15 A")
b.axhline(.30, linestyle="--", label="高速限流 0.30 A")
b.set(xlabel="机械转速 RPM", ylabel="转矩电流需求 A", ylim=(0, .4))
b.legend()
save(fig, "K02_limits")

fig, (a, b) = page("K03", "K03：角度估错，电流方向也会错")
degrees = np.linspace(-90, 90, 361)
delta = np.deg2rad(degrees)
a.plot(degrees, np.cos(delta), label="真实 Iq / 指令 Iq")
a.plot(degrees, -np.sin(delta), label="真实 Id / 指令 Iq")
a.set(xlabel="估计电角度减真实电角度 °", ylabel="电流分量比例")
a.legend()
theta = np.linspace(0, 2 * np.pi, 501)
for bias in (0, .35, .8):
    observed = np.unwrap(np.arctan2(np.sin(theta), np.cos(theta) + bias))
    b.plot(np.rad2deg(theta), np.rad2deg(observed - theta), label=f"磁链中心偏移 / 半径 = {bias}")
b.set(xlabel="真实磁链电角度 °", ylabel="观测角度误差 °")
b.legend(fontsize=9)
save(fig, "K03_angle")

fig, (a, b) = page("K04", "K04：延迟的影响随速度上升")
for delay in (1 / 15000, .0002, .001):
    a.plot(rpm, np.rad2deg(omega * delay), label=f"延迟 {delay * 1e6:.0f} μs")
a.set(xlabel="机械转速 RPM（7 极对）", ylabel="等效电角度滞后 °")
a.legend()
current = np.linspace(-.3, .3, 501)
b.plot(current, current, label="理想电流")
b.step(current, np.round(current / .0806) * .0806, where="mid", label="0.0806 A/count 示意")
b.set(xlabel="真实电流 A", ylabel="量化后的单拍电流 A")
b.legend()
save(fig, "K04_sampling")

fig, (a, b) = page("K05", "K05：参数检测需要分离斜率与偏置")
i = np.array([.5, .75, 1.0])
voltage = r * i - .35
a.plot(i, voltage, "o-", label="V = R × I − 0.35 V")
a.set(xlabel="电流 A", ylabel="电压 V", ylim=(1.4, 3.85))
for xx, yy in zip(i, voltage):
    a.annotate(f"V/I={yy / xx:.2f} Ω", (xx, yy), xytext=(0, 12), textcoords="offset points", ha="center")
a.legend(loc="lower right")
speed = np.linspace(0, 2000, 301)
b.plot(speed, psi * speed, label="理想 E=ψω")
b.plot(speed, psi * speed + .05, label="带 0.05 V 偏置")
b.set(xlabel="电角速度 rad/s", ylabel="反电势幅值 V")
b.legend()
save(fig, "K05_detection")

fig, (a, b) = page("K06", "K06：机械惯量、负载与制动能量")
time = np.linspace(0, 1, 401)
for inertia in (1e-5, 3e-5):
    kt = 1.5 * p * psi
    acceleration = (kt * .2 - .004) / inertia
    a.plot(time, acceleration * time * 60 / (2 * np.pi), label=f"J={inertia:.0e} kg·m²")
a.set(xlabel="时间 s", ylabel="简化恒转矩加速 RPM")
a.legend()
energy = .5 * 3e-5 * (rpm * 2 * np.pi / 60)**2
b.plot(rpm, energy, label="J=3e-5 kg·m² 的转动能量")
b.set(xlabel="机械转速 RPM", ylabel="转动能量 J")
b.legend()
save(fig, "K06_mechanics")

sheet = Image.new("RGB", (1200, 3 * 320), "#f4f6f8")
for index, name in enumerate(gallery):
    with Image.open(OUT / f"{name}.png") as source:
        thumb = ImageOps.contain(source.convert("RGB"), (590, 300))
        x, y = (index % 2) * 600 + 5, (index // 2) * 320 + 5
        sheet.paste(thumb, (x, y))
sheet.save(ROOT / "knowledge_contact_sheet.jpg", quality=92)
print(f"已生成 {len(gallery)} 组知识图（PNG/SVG）及总览")
