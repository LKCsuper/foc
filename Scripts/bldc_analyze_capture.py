#!/usr/bin/env python3
"""Summarize one bounded BLDC CSV capture for the next tuning iteration."""
from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze a BLDC capture CSV")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--mode", choices=("auto", "open"), default="auto")
    parser.add_argument("--period-ms", type=float)
    args = parser.parse_args()

    with args.csv_path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("analysis failed: CSV has no status rows")

    def print_bemf_waveform(row: dict[str, str]) -> None:
        if "step0_bemf_bin0_v" not in row:
            return
        all_values = [float(row[f"step{step}_bemf_bin{index}_v"])
                      for step in range(6) for index in range(8)]
        if not any(abs(value) > 0.0005 for value in all_values):
            return
        print("  settled BEMF waveform by step (sector bins 0..7):")
        for step in range(6):
            values = [float(row[f"step{step}_bemf_bin{index}_v"]) for index in range(8)]
            print(f"    step {step}: " + ", ".join(f"{value:.3f}" for value in values) + " V")

    faults = [row for row in rows if row["fault"] != "NONE" or int(row["motor_fault"]) != 0]
    print("\nBLDC automatic analysis")
    if args.mode == "open":
        fixed = [row for row in rows if row["mode"] == "OPEN_FIXED"]
        print(f"  samples: {len(rows)}, fixed-open samples: {len(fixed)}, faults: {len(faults)}")
        if faults:
            print("  result: FAIL - a motor or BLDC fault was reported")
            return 2
        if not fixed:
            print("  result: FAIL - controller did not enter OPEN_FIXED")
            return 2
        if args.period_ms is not None:
            settled = [row for row in fixed
                       if abs(float(row["period_ms"]) - args.period_ms) <= 0.2]
        else:
            settled = fixed[len(fixed) // 3:]
        if not settled:
            print("  result: FAIL - fixed-open period did not reach the requested value")
            return 2
        periods = [float(row["period_ms"]) for row in settled]
        mean_period = statistics.fmean(periods)
        spread = max(periods) - min(periods)
        print(f"  settled period: {mean_period:.3f} ms, sample spread: {spread:.3f} ms")
        print_bemf_waveform(settled[-1])
        print("  result: TIMING_ONLY - fixed-open timing completed; rotor synchronism is not measured")
        return 0

    closed = [row for row in rows if row["mode"] == "CLOSED_LOOP"]
    print(f"  samples: {len(rows)}, closed-loop samples: {len(closed)}, faults: {len(faults)}")
    if not closed:
        print("  result: FAIL - controller did not enter CLOSED_LOOP")
        return 2

    last = closed[-1]
    periods = [float(last[f"step{index}_period_ms"]) for index in range(6)]
    valid_periods = [value for value in periods if value > 0.0]
    mean_period = statistics.fmean(valid_periods)
    spread = max(valid_periods) - min(valid_periods)
    spread_ratio = spread / mean_period if mean_period > 0.0 else 0.0
    print(f"  final filtered period: {float(last['period_ms']):.3f} ms")
    if "duty" in last and "target_duty" in last:
        print(
            f"  duty: applied={float(last['duty']):.3f}, "
            f"target={float(last['target_duty']):.3f}"
        )
    print("  last period/step: " + ", ".join(f"{value:.3f}" for value in periods) + " ms")
    print(f"  step spread: {spread:.3f} ms ({spread_ratio * 100.0:.1f}%)")
    startup_forward = [float(last[f"step{index}_startup_forward_vs"])
                       for index in range(6)]
    startup_reverse = [float(last[f"step{index}_startup_reverse_vs"])
                       for index in range(6)]
    print(
        f"  startup: sync={last['startup_sync']}/6, valid={last['startup_valid']}, "
        f"invalid={last['startup_invalid']}, resets={last['startup_resets']}, "
        f"takeover={float(last['startup_takeover_ms']):.3f} ms "
        f"at {float(last['startup_takeover_period_ms']):.3f} ms"
    )
    print("  startup forward/step: " +
          ", ".join(f"{value:.6f}" for value in startup_forward) + " V*s")
    print("  startup reverse/step: " +
          ", ".join(f"{value:.6f}" for value in startup_reverse) + " V*s")
    print_bemf_waveform(last)
    if "step0_integral_count" in last:
        integral_counts = [int(last[f"step{index}_integral_count"]) for index in range(6)]
        forced_counts = [int(last[f"step{index}_forced_count"]) for index in range(6)]
        last_integrals = [float(last[f"step{index}_last_integral_vs"])
                          for index in range(6)]
        print("  integral commutation/step: " +
              ", ".join(str(value) for value in integral_counts))
        print("  forced commutation/step: " +
              ", ".join(str(value) for value in forced_counts))
        print("  last integral/step: " +
              ", ".join(f"{value:.6f}" for value in last_integrals) + " V*s")
    integral_commutations = int(last["commutations"])
    forced_commutations = int(last.get("forced", "0"))
    print(
        f"  commutations: {integral_commutations}, forced: {forced_commutations}, "
        f"threshold: {float(last['threshold_vs']):.6f} V*s"
    )

    if faults:
        print("  result: FAIL - a motor or BLDC fault was reported")
        return 2
    if int(last["startup_sync"]) < 6:
        print("  result: FAIL - closed loop was entered without complete startup synchronization")
        return 2
    if any(forward <= reverse
           for forward, reverse in zip(startup_forward, startup_reverse)):
        print("  result: FAIL - startup BEMF polarity is not dominant on every step")
        return 2
    if integral_commutations == 0 and forced_commutations > 0:
        print("  result: FAIL - closed-loop state is running only on forced commutation")
        return 2
    if spread_ratio > 0.25:
        slow_step = periods.index(max(valid_periods))
        print(f"  result: NEEDS_TUNING - step {slow_step} is substantially slower than the others")
        return 1
    if forced_commutations > 0:
        print("  result: NEEDS_TUNING - synchronized startup succeeded, but forced commutation remains")
        return 1
    if int(last["startup_resets"]) > 0:
        print("  result: PASS_WITH_STARTUP_RETRY - synchronization succeeded after a reset")
        return 1
    print("  result: PASS_SYNCHRONIZED - startup and integral commutation are both valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
