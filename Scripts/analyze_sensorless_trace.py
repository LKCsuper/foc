"""离线分析完整/轻量RAM记录；不把观测角或RPM当作独立转子真值。"""
import argparse
import json
import math
import re
import statistics
from pathlib import Path


def elapsed_us(later, earlier):
    """32位微秒计数器的有符号间隔，支持单次回绕。"""
    return (int(later) - int(earlier) + 2**31) % 2**32 - 2**31


def describe(values):
    if not values:
        return None
    return {"min": min(values), "median": statistics.median(values),
            "max": max(values), "mean": statistics.mean(values)}


def flux_phase_relation(rows, pole_pairs):
    if not math.isfinite(pole_pairs) or pole_pairs <= 0:
        raise ValueError('pole_pairs must be finite and positive')
    rates, fluxes, inverse_fluxes, reported = [], [], [], []
    for previous, current in zip(rows, rows[1:]):
        if previous['blend'] < 1 or current['blend'] < 1:
            continue
        interval = elapsed_us(current['time_us'], previous['time_us'])
        flux = (previous['flux'] + current['flux']) / 2
        if interval <= 0 or flux <= 0:
            continue
        flux_squared = flux * flux
        if not math.isfinite(flux_squared) or flux_squared == 0:
            continue
        inverse_flux = 1 / flux_squared
        if not math.isfinite(inverse_flux):
            continue
        angle = (current['pll'] - previous['pll'] + math.pi) % (2 * math.pi) - math.pi
        rates.append(angle * 60e6 / (2 * math.pi * pole_pairs * interval))
        fluxes.append(flux)
        inverse_fluxes.append(inverse_flux)
        reported.append((previous['rpm'] + current['rpm']) / 2)

    def correlation(first, second):
        if len(first) < 2:
            return None
        first_mean, second_mean = statistics.mean(first), statistics.mean(second)
        numerator = sum((left - first_mean) * (right - second_mean) for left, right in zip(first, second))
        denominator = math.sqrt(sum((value - first_mean)**2 for value in first) *
                                sum((value - second_mean)**2 for value in second))
        return numerator / denominator if denominator else None

    absolute_rates = [abs(value) for value in rates]
    return {'intervals': len(rates), 'pll_angle_rate_equivalent_rpm': describe(rates),
            'reported_filtered_rpm': describe(reported), 'midpoint_flux_Wb': describe(fluxes),
            'angle_rate_minus_reported_rpm': describe([rate - rpm for rate, rpm in zip(rates, reported)]),
            'correlation_abs_angle_rate_vs_flux': correlation(absolute_rates, fluxes),
            'correlation_abs_angle_rate_vs_inverse_flux_squared': correlation(absolute_rates, inverse_fluxes),
            'note': 'Post-blend estimated quantities only; correlation is not causation or an independent rotor reference. Shortest-angle differencing assumes less than pi per interval.'}


def analyze(path, nominal_dt_us=1e6 / 15000, pole_pairs=None):
    if not math.isfinite(nominal_dt_us) or nominal_dt_us <= 0:
        raise ValueError("nominal_dt_us must be finite and positive")
    header = None
    rows = []
    malformed = 0
    flight_counts = None
    required = {"time_us", "flags", "theta", "pll", "flux", "rpm", "id", "iq", "blend", "iq_cmd"}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        match = re.match(r"trace flight entry_rows=(\d+) rolling_rows=(\d+)", line)
        if match:
            flight_counts = tuple(map(int, match.groups()))
        if line.startswith("time_us,flags,"):
            current = line.split(",")
            if header is not None and current != header:
                raise ValueError("Multiple trace formats: split logs before analysis")
            if not required.issubset(current):
                raise ValueError("Missing required trace fields")
            header = current
            continue
        if header is None or not line or not line[0].isdigit() or "," not in line:
            continue
        try:
            values = list(map(float, line.split(",")))
            if len(values) != len(header) or not all(map(math.isfinite, values)):
                raise ValueError()
            row = dict(zip(header, values))
            if (row["time_us"] != int(row["time_us"]) or not 0 <= row["time_us"] < 2**32 or
                    row["flags"] != int(row["flags"]) or not 0 <= row["flags"] <= 511):
                raise ValueError()
            if rows and elapsed_us(row["time_us"], rows[-1]["time_us"]) <= 0:
                raise ValueError()
        except ValueError:
            malformed += 1
            continue
        rows.append(row)
    if header is None:
        raise ValueError("No trace CSV header found")
    entry = next((r["time_us"] for r in rows if int(r["flags"]) & 256), None)
    post = [r for r in rows if int(r["flags"]) & 256]
    intervals = [elapsed_us(b["time_us"], a["time_us"]) for a, b in zip(post, post[1:])]
    summary = {
        "format": "full" if "enc" in header else "lite",
        "rows": len(rows), "malformed": malformed, "entry_us": entry,
        "entry_rows": len(post), "nominal_dt_us": nominal_dt_us,
        "record_interval_us": describe(intervals),
        "record_stride": "Not inferred: record intervals are not individual control intervals",
        "loss_marked_rows": sum(bool(int(r["flags"]) & 128) for r in rows),
        "loss_events": sum(bool(int(r["flags"]) & 128) and (i == 0 or not int(rows[i-1]["flags"]) & 128)
                           for i, r in enumerate(rows)),
        "independent_rotor_reference": False,
        "note": "Observer RPM and PLL-control angle difference do not measure true rotor speed/phase error",
    }
    if post:
        summary["entry_span_ms"] = elapsed_us(post[-1]["time_us"], post[0]["time_us"]) / 1000
        summary["entry_metrics"] = {key: describe([r[key] for r in post])
                                    for key in ("rpm", "flux", "id", "iq", "blend")}
        summary["iq_tracking_error_A"] = describe([r["iq"] - r["iq_cmd"] for r in post])
        if "id_cmd" in header:
            summary["id_tracking_error_A"] = describe([r["id"] - r["id_cmd"] for r in post])
        summary["pll_control_difference_rad"] = describe([
            (r["pll"] - r["theta"] + math.pi) % (2 * math.pi) - math.pi for r in post])
        for key in ("gap_max_us", "full_max_us"):
            if key in header:
                summary[key] = describe([r[key] for r in post])
        if "task_age_us" in header:
            summary["speed_task"] = {"age_us": describe([r["task_age_us"] for r in post]),
                "distinct_counts": len({int(r["task_count"]) for r in post}),
                "gate_off_rows": sum(r["task_gate"] == 0 for r in post),
                "feedback_error_RPM": describe([r["speed_target"] - r["speed_feedback"] for r in post]),
                "raw_iq_A": describe([r["speed_iq"] for r in post])}
    if flight_counts is not None:
        initial_count, rolling_count = flight_counts
        if initial_count > 192 or rolling_count > 320 or initial_count + rolling_count != len(rows):
            raise ValueError("Flight window counts mismatch: damaged or incomplete log")
        summary["flight_windows"] = {}
        for name, window in (("initial", rows[:initial_count]), ("rolling", rows[initial_count:])):
            summary["flight_windows"][name] = {
                "rows": len(window),
                "span_ms": elapsed_us(window[-1]["time_us"], window[0]["time_us"]) / 1000 if window else None,
                "metrics": {key: describe([r[key] for r in window]) for key in ("rpm", "flux", "id", "iq")},
                "last": window[-1] if window else None,
            }
        summary["flight_windows"]["gap_us"] = elapsed_us(rows[initial_count]["time_us"],
            rows[initial_count - 1]["time_us"]) if initial_count and rolling_count else None
        summary["note"] += "; Flight initial and rolling windows may be separated by unrecorded time"
    if pole_pairs is not None:
        summary['flux_phase_relation'] = flux_phase_relation(rows, pole_pairs)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--nominal-dt-us", type=float, default=1e6 / 15000)
    parser.add_argument('--pole-pairs', type=float)
    args = parser.parse_args()
    print(json.dumps(analyze(args.log, args.nominal_dt_us, args.pole_pairs), ensure_ascii=False, indent=2))
