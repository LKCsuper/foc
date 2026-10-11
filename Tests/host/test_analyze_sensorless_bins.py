import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "Scripts"))
from analyze_sensorless_bins import analyze

lines = ["slstats n=6400 time=0.4s fail F=0 S=0 D=0 P=123",
         "PWM reconstruction error max=0.004000"]
radius = 0.002 * math.sin(math.pi / 8) / (math.pi / 8)
for index in range(8):
    theta = -math.pi + (index + 0.5) * 2 * math.pi / 8
    fa = 0.0004 + radius * math.cos(theta + 1.2)
    fb = -0.0001 + radius * math.sin(theta + 1.2)
    phase = math.remainder(math.atan2(fb, fa) - theta, 2 * math.pi)
    lines.append(f"bin{index} n=100 P=1 ph={phase:.8f} ph2={phase**2:.8f} "
                 f"flux={math.hypot(fa, fb):.9f} ia=0.02 ib=-0.01 va=0.01 vb=-0.02 "
                 f"fa={fa:.9f} fb={fb:.9f}")
with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
    path = Path(directory) / "fixture.log"
    path.write_text("\n".join(lines), encoding="utf-8")
    result = analyze(path)
    assert result["samples"] == 6400 and result["binned_samples"] == 800
    assert abs(result["phase_fail_percent"] - 123 / 64) < 1e-8
    assert abs(result["sampled_phase_fail_percent"] - 1.0) < 1e-8
    assert result["approx_centered_phase_span_rad"] < 1e-5
    assert abs(result["approx_centered_radius_range_Wb"][0] - 0.002) < 1e-8
    assert result["between_bins_variance_explained"] > 0.98
    path.write_text("\n".join(lines[:-1]), encoding="utf-8")
    try:
        analyze(path)
        raise AssertionError("missing bin accepted")
    except ValueError:
        pass
print("PASS: independent offset circle, subsampled/global counts and incomplete-data rejection")
