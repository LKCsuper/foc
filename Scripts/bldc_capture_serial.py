#!/usr/bin/env python3
"""Capture BM75 sensorless BLDC status over UART.

The script runs one bounded test, polls ``bldc status``, writes the original
UART transcript and a CSV summary, and always requests BLDC stop/exit.
It does not flash firmware or change the VESC-derived peripheral setup.
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from pathlib import Path

from ol_verify_serial import REPO_ROOT, Shell, decode, repo_short_hash


STATUS_HEADER = "BLDC selected="
FAULT_PATTERN = re.compile(r"mode=(\S+)\s+fault=(\S+)\s+motor_fault=(\d+)")


def match_float(pattern: str, text: str) -> float:
    match = re.search(pattern, text)
    if match is None:
        raise ValueError(pattern)
    return float(match.group(1))


def match_int(pattern: str, text: str) -> int:
    match = re.search(pattern, text)
    if match is None:
        raise ValueError(pattern)
    return int(match.group(1))


def parse_status(text: str, elapsed_s: float) -> dict[str, float | int | str]:
    state = FAULT_PATTERN.search(text)
    periods = re.search(r"last_period/step=([0-9.,-]+)ms", text)
    integral_steps = re.search(r"integral_comm/step=([0-9,]+)", text)
    forced_steps = re.search(r"forced_comm/step=([0-9,]+)", text)
    last_integrals = re.search(r"last_integral/step=([0-9.,-]+)V\*s", text)
    startup = re.search(
        r"startup valid=(\d+) invalid=(\d+) resets=(\d+) "
        r"takeover=([-+0-9.]+)ms@([-+0-9.]+)ms", text
    )
    startup_forward = re.search(r"startup_forward/step=([0-9.,-]+)V\*s", text)
    startup_reverse = re.search(r"startup_reverse/step=([0-9.,-]+)V\*s", text)
    waveforms = [re.search(rf"bemf_wave/{index}=([0-9.,-]+)V", text) for index in range(6)]
    period_range = re.search(r"period_range=\[([-+0-9.]+),([-+0-9.]+)\]ms", text)
    if (STATUS_HEADER not in text or state is None or periods is None or
            period_range is None or
            integral_steps is None or forced_steps is None or last_integrals is None or
            startup is None or
            startup_forward is None or startup_reverse is None or
            any(waveform is None for waveform in waveforms)):
        raise ValueError("incomplete BLDC status; flash the latest firmware")

    values = [float(value) for value in periods.group(1).split(",")]
    integral_counts = [int(value) for value in integral_steps.group(1).split(",")]
    forced_counts = [int(value) for value in forced_steps.group(1).split(",")]
    last_integral_values = [float(value) for value in last_integrals.group(1).split(",")]
    startup_forward_values = [float(value) for value in startup_forward.group(1).split(",")]
    startup_reverse_values = [float(value) for value in startup_reverse.group(1).split(",")]
    waveform_values = [[float(value) for value in waveform.group(1).split(",")]
                       for waveform in waveforms if waveform is not None]
    if (len(values) != 6 or len(integral_counts) != 6 or len(forced_counts) != 6 or
            len(last_integral_values) != 6 or
            len(startup_forward_values) != 6 or len(startup_reverse_values) != 6 or
            any(len(waveform) != 8 for waveform in waveform_values)):
        raise ValueError("invalid BLDC per-step diagnostics")
    return {
        "elapsed_s": round(elapsed_s, 3),
        "mode": state.group(1),
        "fault": state.group(2),
        "motor_fault": int(state.group(3)),
        "duty": match_float(r"\bduty=([-+0-9.]+)", text),
        "target_duty": match_float(r"\btarget=([-+0-9.]+)", text),
        "step": match_int(r"\bstep=(\d+)", text),
        "commutations": match_int(r"commutations=(\d+)", text),
        "forced": match_int(r"forced=(\d+)", text),
        "consecutive_forced": match_int(r"consecutive=(\d+)", text),
        "startup_sync": match_int(r"sync=(\d+)/6", text),
        "startup_valid": int(startup.group(1)),
        "startup_invalid": int(startup.group(2)),
        "startup_resets": int(startup.group(3)),
        "startup_takeover_ms": float(startup.group(4)),
        "startup_takeover_period_ms": float(startup.group(5)),
        "bemf_v": match_float(r"\bbemf=([-+0-9.]+)V", text),
        "sector_ms": match_float(r"sector=([-+0-9.]+)ms", text),
        "period_ms": match_float(r"\bperiod=([-+0-9.]+)ms", text),
        "period_min_ms": float(period_range.group(1)),
        "period_max_ms": float(period_range.group(2)),
        "integral_vs": match_float(r"integral=([-+0-9.]+)/", text),
        "threshold_vs": match_float(r"integral=[-+0-9.]+/([-+0-9.]+)V\*s", text),
        "vbus_v": match_float(r"vbus=([-+0-9.]+)V", text),
        **{f"step{index}_period_ms": value for index, value in enumerate(values)},
        **{f"step{index}_integral_count": value
           for index, value in enumerate(integral_counts)},
        **{f"step{index}_forced_count": value for index, value in enumerate(forced_counts)},
        **{f"step{index}_last_integral_vs": value
           for index, value in enumerate(last_integral_values)},
        **{f"step{index}_startup_forward_vs": value
           for index, value in enumerate(startup_forward_values)},
        **{f"step{index}_startup_reverse_vs": value
           for index, value in enumerate(startup_reverse_values)},
        **{f"step{step}_bemf_bin{bin_index}_v": value
           for step, waveform in enumerate(waveform_values)
           for bin_index, value in enumerate(waveform)},
    }


def workspace_id() -> str:
    return repo_short_hash()


def default_output_prefix(mode: str) -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return REPO_ROOT / "Doc" / "verify" / f"bldc_{mode}_{workspace_id()}_{stamp}"


class Capture:
    def __init__(self, shell: Shell) -> None:
        self.shell = shell
        self.sections: list[tuple[str, str]] = []
        self.rows: list[dict[str, float | int | str]] = []

    def record(self, name: str, text: str) -> None:
        self.sections.append((name, text))
        print(f"\n######## {name} ########\n{text}", flush=True)

    def command(self, name: str, line: str, timeout: float = 20.0) -> str:
        text = self.shell.run(line, timeout=timeout, progress=True)
        self.record(name, text)
        return text

    def stop_and_exit(self) -> None:
        try:
            self.command("stop", "bldc stop")
        finally:
            self.command("exit", "bldc exit")


def validate_args(args: argparse.Namespace, parser: argparse.ArgumentParser) -> None:
    if not 0.0 < args.duty <= 0.25:
        parser.error("--duty must be in (0, 0.25]")
    if args.mode == "auto" and not 0.08 <= args.duty <= 0.10:
        parser.error("automatic mode duty must be in the verified range [0.08, 0.10]")
    if not 1.0 <= args.duration <= 300.0:
        parser.error("--duration must be in [1, 300] seconds")
    if not 0.5 <= args.interval <= 30.0:
        parser.error("--interval must be in [0.5, 30] seconds")
    if args.period_ms is not None and not 0.5 <= args.period_ms <= 500.0:
        parser.error("--period-ms must be in [0.5, 500]")
    if args.integral_mvs is not None and not 0.05 <= args.integral_mvs <= 10.0:
        parser.error("--integral-mvs must be in [0.05, 10]")
    if args.mode == "auto" and ((args.period_ms is None) != (args.integral_mvs is None)):
        parser.error("auto mode requires --period-ms and --integral-mvs together")


def write_results(capture: Capture, args: argparse.Namespace, prefix: Path) -> None:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    log_path = prefix.with_suffix(".log")
    csv_path = prefix.with_suffix(".csv")
    header = (
        "# bldc_capture\n"
        f"# workspace: {workspace_id()}\n"
        f"# time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
        f"# mode: {args.mode}\n"
        f"# duty: {args.duty}\n"
        f"# period_ms: {args.period_ms}\n"
        f"# integral_mvs: {args.integral_mvs}\n"
        f"# direction: {args.direction}\n"
        f"# note: {args.note}\n"
    )
    body = [header]
    for name, text in capture.sections:
        body.append(f"\n######## {name} ########\n{text}")
    log_path.write_text("".join(body), encoding="utf-8")

    if capture.rows:
        with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(capture.rows[0].keys()))
            writer.writeheader()
            writer.writerows(capture.rows)
        print(f"CSV written: {csv_path}", flush=True)
    print(f"Log written: {log_path}", flush=True)


def run_capture(capture: Capture, args: argparse.Namespace) -> int:
    capture.command("pre-stop", "bldc stop")
    capture.command("pre-exit", "bldc exit")
    if args.mode == "auto":
        if args.integral_mvs is not None and args.period_ms is not None:
            tune = capture.command(
                "tune", f"bldc tune {args.integral_mvs:g} {args.period_ms:g}"
            )
            if "BLDC tune:" not in tune:
                raise RuntimeError("BLDC tune was rejected")
        start_line = f"bldc start {args.duty:g} {args.direction}"
    else:
        period_ms = 13.0 if args.period_ms is None else args.period_ms
        start_line = f"bldc open {args.duty:g} {period_ms:g} {args.direction}"

    started = capture.command("start", start_line)
    expected = "BLDC start:" if args.mode == "auto" else "BLDC open:"
    if expected not in started:
        raise RuntimeError("BLDC start was rejected")

    begin = time.monotonic()
    next_sample = begin
    index = 1
    while True:
        now = time.monotonic()
        if now - begin >= args.duration:
            break
        if now < next_sample:
            capture.shell.pause(min(next_sample - now, 1.0))
            continue
        text = capture.command(f"status-{index:03d}", "bldc status")
        row = parse_status(text, time.monotonic() - begin)
        capture.rows.append(row)
        if row["mode"] == "FAULT" or row["fault"] != "NONE" or row["motor_fault"] != 0:
            raise RuntimeError(
                f"fault detected: mode={row['mode']} fault={row['fault']} "
                f"motor_fault={row['motor_fault']}"
            )
        index += 1
        next_sample += args.interval
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="BM75 BLDC bounded UART capture")
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--mode", choices=("auto", "open"), default="auto")
    parser.add_argument("--duty", type=float, default=0.10)
    parser.add_argument("--period-ms", type=float)
    parser.add_argument("--integral-mvs", type=float)
    parser.add_argument("--direction", choices=("cw", "ccw"), default="cw")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--note", default="", help="写入日志的听感或工况备注")
    parser.add_argument("--output", default="", help="输出文件前缀，不含扩展名")
    args = parser.parse_args()
    validate_args(args, parser)

    shell = Shell(args.port, args.baud)
    capture = Capture(shell)
    rc = 1
    prefix = Path(args.output) if args.output else default_output_prefix(args.mode)
    if not prefix.is_absolute():
        prefix = REPO_ROOT / prefix
    try:
        wake = decode(shell.read_until_prompt(0.5))
        if not wake.strip():
            wake = shell.kick()
        capture.record("wake", wake)
        rc = run_capture(capture, args)
    except KeyboardInterrupt:
        capture.record("ABORT", "用户按下 Ctrl+C\n")
        rc = 130
    except (RuntimeError, ValueError) as exc:
        capture.record("ABORT", str(exc) + "\n")
        rc = 2
    finally:
        try:
            capture.stop_and_exit()
        except Exception as exc:
            capture.record("STOP_ERROR", str(exc) + "\n")
            rc = 3
        shell.close()
        write_results(capture, args, prefix)
    return rc


if __name__ == "__main__":
    sys.exit(main())
