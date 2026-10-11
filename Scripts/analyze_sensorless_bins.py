"""停机分箱日志分析；I/F相位不代表独立真实转子角。"""
import argparse
import json
import math
import re
from pathlib import Path


def analyze(path, resistance=0.474):
    raw = Path(path).read_text(encoding="utf-8")
    bins = []
    pattern = (r"bin(\d+) n=(\d+) P=(\d+) ph=([-\d.]+) ph2=([-\d.]+) "
               r"flux=([-\d.]+) ia=([-\d.]+) ib=([-\d.]+) va=([-\d.]+) vb=([-\d.]+)"
               r"(?: fa=([-\d.]+) fb=([-\d.]+))?")
    for match in re.finditer(pattern, raw):
        index, count, fail = map(int, match.groups()[:3])
        values = list(map(float, match.groups()[3:10]))
        bins.append(dict(index=index, count=count, fail=fail,
                         phase=values[0], phase2=values[1], flux=values[2],
                         ia=values[3], ib=values[4], va=values[5], vb=values[6]))
        if match[11] is not None:
            bins[-1]["fa"] = float(match[11])
            bins[-1]["fb"] = float(match[12])
    if len(bins) != 8 or sorted(b["index"] for b in bins) != list(range(8)):
        raise ValueError("需要单轮完整的8个电角分段")
    count = sum(b["count"] for b in bins)

    def mean(field):
        return sum(b["count"] * b[field] for b in bins) / count

    # 不依赖额外库的3变量加权最小二乘：phase = C+A*cos(theta)+B*sin(theta)。
    matrix = [[0.0] * 4 for _ in range(3)]
    for b in bins:
        theta = -math.pi + (b["index"] + 0.5) * 2 * math.pi / 8
        row = [1.0, math.cos(theta), math.sin(theta)]
        for i in range(3):
            for j in range(3):
                matrix[i][j] += b["count"] * row[i] * row[j]
            matrix[i][3] += b["count"] * row[i] * b["phase"]
    for i in range(3):
        pivot = max(range(i, 3), key=lambda j: abs(matrix[j][i]))
        matrix[i], matrix[pivot] = matrix[pivot], matrix[i]
        scale = matrix[i][i]
        matrix[i] = [x / scale for x in matrix[i]]
        for j in range(3):
            if j != i:
                factor = matrix[j][i]
                matrix[j] = [a - factor * c for a, c in zip(matrix[j], matrix[i])]
    constant, cosine, sine = (r[3] for r in matrix)
    between = sum(b["count"] * (b["phase"] - mean("phase")) ** 2 for b in bins)
    residual = 0.0
    for b in bins:
        theta = -math.pi + (b["index"] + 0.5) * 2 * math.pi / 8
        estimate = constant + cosine * math.cos(theta) + sine * math.sin(theta)
        residual += b["count"] * (b["phase"] - estimate) ** 2
    error = re.search(r"PWM reconstruction error max=([-\d.]+)", raw)
    totals = re.search(r"slstats n=(\d+).*?P=(\d+)", raw)
    result = {
        "samples": int(totals[1]) if totals else count,
        "binned_samples": count,
        "phase_fail_percent": 100 * int(totals[2]) / int(totals[1]) if totals else None,
        "sampled_phase_fail_percent": 100 * sum(b["fail"] for b in bins) / count,
        "phase_mean_rad": mean("phase"),
        "phase_std_rad": math.sqrt(max(0, mean("phase2") - mean("phase") ** 2)),
        "phase_first_harmonic_amplitude_rad": math.hypot(cosine, sine),
        "between_bins_variance_explained": 1 - residual / between if between else 0,
        "measured_current_dc_alpha_beta_A": [mean("ia"), mean("ib")],
        "command_voltage_dc_alpha_beta_V": [mean("va"), mean("vb")],
        "command_V_minus_RI_dc_V": [mean("va") - resistance * mean("ia"),
                                    mean("vb") - resistance * mean("ib")],
        "pwm_reconstruction_error_max_V": float(error[1]) if error else None,
        "bins": bins,
    }
    if all("fa" in b for b in bins):
        cx, cy = mean("fa"), mean("fb")
        centered_angles, radii = [], []
        for b in bins:
            theta = -math.pi + (b["index"] + 0.5) * 2 * math.pi / 8
            centered_angles.append(math.remainder(math.atan2(b["fb"] - cy, b["fa"] - cx) - theta,
                                                 2 * math.pi))
            # 均匀角度分箱使旋转矢量均值半径缩小sinc(pi/8)；这里只作近似诊断。
            radii.append(math.hypot(b["fa"] - cx, b["fb"] - cy) / (math.sin(math.pi / 8) / (math.pi / 8)))
        result["flux_center_alpha_beta_Wb"] = [cx, cy]
        result["approx_centered_phase_span_rad"] = max(centered_angles) - min(centered_angles)
        result["approx_centered_radius_range_Wb"] = [min(radii), max(radii)]
        result["note"] = "去中心仅用于诊断，不应用到控制；均匀分箱近似，不是独立转子角测量"
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--resistance", type=float, default=0.474)
    args = parser.parse_args()
    print(json.dumps(analyze(args.log, args.resistance), ensure_ascii=False, indent=2))
