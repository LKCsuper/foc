"""Headless PMSM simulation for the HFI-to-flux-observer handoff.

This model uses the same parameters and handoff timing as sl_hfi_flux.m, but
does not require MATLAB or Simulink.  It writes numerical samples and a PNG so
the result can be inspected in CI or from a terminal.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


DT = 1.0 / 15_000.0
STOP_TIME = 3.0
POLE_PAIRS = 2.0
VBUS = 24.0
MAX_DUTY = 0.95
R_PHASE = 0.20
LD = 0.0005
LQ = 0.0008
FLUX = 0.02573
J = 210.0e-7
B_VISC = 5.25e-5
CURRENT_KP = 0.12
CURRENT_KI = 18.0
# The original Simulink draft used 0.0012/0.06 and is under-damped for this
# mechanical plant.  These gains are the verified offline-simulation baseline.
SPEED_KP = 0.003
SPEED_KI = 0.01
IQ_LIMIT = 1.2
IQ_SLEW = 8.0
ALIGN_TIME = 0.20
TARGET_RPM = 500.0
RAMP_RPM_S = 800.0
CRUISE_TIME = 0.25
BLEND_TIME = 0.20
HFI_VOLTAGE = 2.0


def wrap_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


def current_pi(ref: float, value: float, integral: float, kp: float, ki: float, limit: float):
    error = ref - value
    candidate = clamp(integral + ki * error * DT, -limit, limit)
    output = clamp(kp * error + candidate, -limit, limit)
    # Conditional integration prevents winding farther into saturation.
    if output == kp * error + candidate or output * error <= 0.0:
        integral = candidate
    return output, integral


@dataclass
class FluxObserver:
    seeded: bool = False
    x_a: float = 0.0
    x_b: float = 0.0
    pll_theta: float = 0.0
    pll_omega: float = 0.0
    pll_omega_f: float = 0.0

    def step(self, va: float, vb: float, ia: float, ib: float, seed_rpm: float, seed_theta: float):
        gamma = 1.2e7
        if not self.seeded and abs(seed_rpm) > 150.0:
            self.x_a = LQ * ia + FLUX * math.cos(seed_theta)
            self.x_b = LQ * ib + FLUX * math.sin(seed_theta)
            self.pll_theta = seed_theta
            self.pll_omega = seed_rpm * 2.0 * math.pi / 60.0 * POLE_PAIRS
            self.pll_omega_f = self.pll_omega
            self.seeded = True

        if not self.seeded:
            return 0.0, 0.0, False

        psi_a = self.x_a - LQ * ia
        psi_b = self.x_b - LQ * ib
        magnitude_error = min(FLUX * FLUX - psi_a * psi_a - psi_b * psi_b, 0.0)
        self.x_a += (va - R_PHASE * ia + 0.5 * gamma * psi_a * magnitude_error) * DT
        self.x_b += (vb - R_PHASE * ib + 0.5 * gamma * psi_b * magnitude_error) * DT

        psi_a = self.x_a - LQ * ia
        psi_b = self.x_b - LQ * ib
        magnitude = math.hypot(psi_a, psi_b)
        angle = math.atan2(psi_b, psi_a)
        delta = wrap_pi(angle - self.pll_theta)
        self.pll_theta = wrap_pi(self.pll_theta + (self.pll_omega + 2000.0 * delta) * DT)
        self.pll_omega += 30000.0 * delta * DT
        omega_max = 3000.0 * 2.0 * math.pi / 60.0 * POLE_PAIRS * 1.5
        self.pll_omega = clamp(self.pll_omega, -omega_max, omega_max)
        alpha = DT * 2.0 * math.pi * 10.0
        self.pll_omega_f += alpha * (self.pll_omega - self.pll_omega_f)
        rpm = self.pll_omega_f / POLE_PAIRS * 60.0 / (2.0 * math.pi)
        valid = 0.5 * FLUX < magnitude < 1.5 * FLUX
        return self.pll_theta, rpm, valid


def run_simulation():
    count = int(STOP_TIME / DT) + 1
    t = np.arange(count) * DT
    names = [
        "rpm_cmd", "rpm_motor", "rpm_used", "theta_motor_deg", "theta_hfi_deg",
        "theta_flux_deg", "theta_used_deg", "blend", "id", "iq", "iq_ref",
        "torque", "v_injection",
    ]
    data = {name: np.zeros(count) for name in names}

    # Plant state, expressed in the true rotor dq frame.
    id_r = iq_r = omega_m = theta_e = 0.0
    # Controller state.
    speed_int = vd_int = vq_int = iq_ref = rpm_cmd = 0.0
    rpm_feedback = 0.0
    cruise = 0.0
    observer = FluxObserver()
    theta_hfi = 0.0
    theta_flux_unwrapped = theta_used_unwrapped = theta_motor_unwrapped = 0.0
    theta_flux_prev = theta_used_prev = theta_motor_prev = 0.0

    for k, now in enumerate(t):
        c, s = math.cos(theta_e), math.sin(theta_e)
        # Rotor dq currents -> stationary alpha-beta currents.
        i_alpha = c * id_r - s * iq_r
        i_beta = s * id_r + c * iq_r

        # The offline HFI estimate includes a small, decaying initial angle error.
        # Its ripple represents the demodulated 7.5 kHz tracking residual.
        if now < ALIGN_TIME:
            theta_hfi = 0.0
        else:
            tracking_error = 0.035 * math.exp(-(now - ALIGN_TIME) / 0.12)
            ripple = math.radians(0.45) * math.sin(2.0 * math.pi * 7500.0 * now)
            theta_hfi = theta_e + tracking_error + ripple
        rpm_hfi = omega_m * 60.0 / (2.0 * math.pi)

        seed_rpm = rpm_hfi if 150.0 < abs(rpm_hfi) < 700.0 else 0.0
        theta_flux, rpm_flux, flux_ok = observer.step(
            0.0 if k == 0 else v_alpha,
            0.0 if k == 0 else v_beta,
            i_alpha,
            i_beta,
            seed_rpm,
            theta_hfi,
        )

        if now < ALIGN_TIME:
            rpm_cmd = 0.0
            speed_int = 0.0
            id_ref = 0.6
            blend = 0.0
            theta_used = 0.0
            cruise = 0.0
        else:
            rpm_cmd = min(TARGET_RPM, rpm_cmd + RAMP_RPM_S * DT)
            id_ref = 0.0
            if rpm_cmd >= TARGET_RPM and flux_ok:
                cruise += DT
            elif rpm_cmd < TARGET_RPM or not flux_ok:
                cruise = 0.0
            blend = clamp((cruise - CRUISE_TIME) / BLEND_TIME, 0.0, 1.0)
            theta_used = wrap_pi(theta_hfi + blend * wrap_pi(theta_flux - theta_hfi))

        rpm_raw = (1.0 - blend) * rpm_hfi + blend * rpm_flux
        rpm_feedback += min(DT * 2.0 * math.pi * 25.0, 1.0) * (rpm_raw - rpm_feedback)
        iq_cmd, speed_int = current_pi(rpm_cmd, rpm_feedback, speed_int, SPEED_KP, SPEED_KI, IQ_LIMIT)
        max_iq_step = IQ_SLEW * DT
        iq_ref += clamp(iq_cmd - iq_ref, -max_iq_step, max_iq_step)

        # Measured current in the estimated Park frame.
        ce, se = math.cos(theta_used), math.sin(theta_used)
        id_est = ce * i_alpha + se * i_beta
        iq_est = -se * i_alpha + ce * i_beta
        voltage_limit = VBUS / math.sqrt(3.0) * MAX_DUTY
        vd, vd_int = current_pi(id_ref, id_est, vd_int, CURRENT_KP, CURRENT_KI, voltage_limit)
        vq, vq_int = current_pi(iq_ref, iq_est, vq_int, CURRENT_KP, CURRENT_KI, voltage_limit)
        scale = min(1.0, voltage_limit / max(math.hypot(vd, vq), 1e-12))
        vd *= scale
        vq *= scale
        v_alpha = ce * vd - se * vq
        v_beta = se * vd + ce * vq
        injection = (1.0 - blend) * HFI_VOLTAGE * (1.0 if k % 2 == 0 else -1.0)
        v_alpha += injection * math.cos(theta_hfi)
        v_beta += injection * math.sin(theta_hfi)

        # Stationary voltage -> true rotor dq voltage; integrate IPMSM plant.
        vd_r = c * v_alpha + s * v_beta
        vq_r = -s * v_alpha + c * v_beta
        omega_e = omega_m * POLE_PAIRS
        did = (vd_r - R_PHASE * id_r + omega_e * LQ * iq_r) / LD
        diq = (vq_r - R_PHASE * iq_r - omega_e * (LD * id_r + FLUX)) / LQ
        id_r += did * DT
        iq_r += diq * DT
        torque = 1.5 * POLE_PAIRS * (FLUX * iq_r + (LD - LQ) * id_r * iq_r)
        omega_m += ((torque - B_VISC * omega_m) / J) * DT
        theta_e = wrap_pi(theta_e + omega_m * POLE_PAIRS * DT)

        d_motor = wrap_pi(theta_e - theta_motor_prev)
        theta_motor_unwrapped += math.degrees(d_motor)
        theta_motor_prev = theta_e
        d_flux = wrap_pi(theta_flux - theta_flux_prev)
        theta_flux_unwrapped += math.degrees(d_flux)
        theta_flux_prev = theta_flux
        d_used = wrap_pi(theta_used - theta_used_prev)
        theta_used_unwrapped += math.degrees(d_used)
        theta_used_prev = theta_used

        data["rpm_cmd"][k] = rpm_cmd
        data["rpm_motor"][k] = omega_m * 60.0 / (2.0 * math.pi)
        data["rpm_used"][k] = rpm_feedback
        data["theta_motor_deg"][k] = theta_motor_unwrapped
        data["theta_hfi_deg"][k] = math.degrees(theta_hfi)
        data["theta_flux_deg"][k] = theta_flux_unwrapped
        data["theta_used_deg"][k] = theta_used_unwrapped
        data["blend"][k] = blend
        data["id"][k] = id_est
        data["iq"][k] = iq_est
        data["iq_ref"][k] = iq_ref
        data["torque"][k] = torque
        data["v_injection"][k] = abs(injection)

    return t, data


def save_outputs(t, data, output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "hfi_flux_waveforms.csv"
    png_path = output_dir / "hfi_flux_waveforms.png"
    json_path = output_dir / "hfi_flux_metrics.json"

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        keys = list(data)
        writer.writerow(["time_s", *keys])
        writer.writerows(zip(t, *(data[key] for key in keys)))

    tail = t >= 2.50
    angle_error = (data["theta_used_deg"] - data["theta_motor_deg"] + 180.0) % 360.0 - 180.0
    metrics = {
        "final_rpm_command": float(data["rpm_cmd"][-1]),
        "final_rpm_motor": float(data["rpm_motor"][-1]),
        "final_rpm_feedback": float(data["rpm_used"][-1]),
        "tail_speed_error_mean_rpm": float(np.mean(data["rpm_motor"][tail] - data["rpm_cmd"][tail])),
        "tail_speed_error_max_abs_rpm": float(np.max(np.abs(data["rpm_motor"][tail] - data["rpm_cmd"][tail]))),
        "tail_angle_error_mean_abs_deg": float(np.mean(np.abs(angle_error[tail]))),
        "tail_angle_error_max_abs_deg": float(np.max(np.abs(angle_error[tail]))),
        "tail_id_mean_abs_a": float(np.mean(np.abs(data["id"][tail]))),
        "tail_iq_mean_a": float(np.mean(data["iq"][tail])),
        "blend_start_s": float(t[np.flatnonzero(data["blend"] > 0.0)[0]]),
        "blend_complete_s": float(t[np.flatnonzero(data["blend"] >= 1.0)[0]]),
        "finite": bool(all(np.isfinite(values).all() for values in data.values())),
    }
    metrics["pass"] = bool(
        metrics["finite"]
        and metrics["tail_speed_error_max_abs_rpm"] <= 50.0
        and metrics["tail_angle_error_mean_abs_deg"] <= 20.0
    )
    json_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")

    render_plot(t, data, png_path)
    return metrics, csv_path, png_path, json_path


def render_plot(t, data, png_path: Path):
    width, height = 1600, 1200
    image = Image.new("RGB", (width, height), "#0d1117")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=20)
    small = ImageFont.load_default(size=16)
    draw.text((40, 20), "HFI to Ortega Flux Observer Handoff - Offline PMSM Simulation", fill="#f0f6fc", font=font)

    panels = [
        ("Speed (RPM)", [("command", data["rpm_cmd"], "#e3b341"), ("motor", data["rpm_motor"], "#2f81f7"), ("feedback", data["rpm_used"], "#f0883e")]),
        ("Electrical angle (deg)", [("motor", data["theta_motor_deg"], "#2f81f7"), ("flux", data["theta_flux_deg"], "#f0883e"), ("Park used", data["theta_used_deg"], "#e3b341")]),
        ("Current (A)", [("Id", data["id"], "#2f81f7"), ("Iq", data["iq"], "#f0883e"), ("Iq ref", data["iq_ref"], "#3fb950")]),
        ("Handoff ratio", [("flux blend", data["blend"], "#a371f7"), ("HFI injection / 2 V", data["v_injection"] / HFI_VOLTAGE, "#3fb950")]),
    ]
    left, right = 120, width - 35
    top0, panel_height, gap = 75, 245, 35
    sample = max(1, len(t) // (right - left))
    xs = t[::sample]

    for panel_index, (title, series) in enumerate(panels):
        top = top0 + panel_index * (panel_height + gap)
        bottom = top + panel_height
        values = np.concatenate([line[1] for line in series])
        ymin, ymax = float(np.min(values)), float(np.max(values))
        padding = max((ymax - ymin) * 0.08, 0.05)
        ymin -= padding
        ymax += padding
        draw.rectangle((left, top, right, bottom), outline="#30363d", width=2)
        for grid in range(1, 5):
            gy = top + grid * panel_height / 5.0
            draw.line((left, gy, right, gy), fill="#21262d", width=1)
        draw.text((20, top + 5), title, fill="#f0f6fc", font=small)
        draw.text((30, top + 32), f"{ymax:.1f}", fill="#8b949e", font=small)
        draw.text((30, bottom - 20), f"{ymin:.1f}", fill="#8b949e", font=small)
        legend_x = left + 12
        for label, values, color in series:
            draw.line((legend_x, top + 16, legend_x + 28, top + 16), fill=color, width=4)
            draw.text((legend_x + 34, top + 6), label, fill=color, font=small)
            legend_x += 180
            ys = values[::sample]
            points = [
                (
                    left + float(x / STOP_TIME) * (right - left),
                    bottom - float((y - ymin) / (ymax - ymin)) * panel_height,
                )
                for x, y in zip(xs, ys)
            ]
            if len(points) >= 2:
                draw.line(points, fill=color, width=2)
        if panel_index == len(panels) - 1:
            for tick in np.linspace(0.0, STOP_TIME, 7):
                x = left + float(tick / STOP_TIME) * (right - left)
                draw.text((x - 12, bottom + 7), f"{tick:.2f}", fill="#8b949e", font=small)
            draw.text(((left + right) // 2 - 35, bottom + 30), "Time (s)", fill="#f0f6fc", font=small)
    image.save(png_path)


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[2]
    times, waveforms = run_simulation()
    result, csv_file, png_file, json_file = save_outputs(times, waveforms, root / "Doc" / "matlab" / "out")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(csv_file)
    print(png_file)
    print(json_file)
