#!/usr/bin/env python3
"""BM75 Hall feedback stability verification over UART.

Uses the saved Hall calibration table, starts at 150 RPM, then ramps through
300/500/800 RPM. Every stage records multiple coherent ``olsnap`` pages and
stops immediately when feedback is invalid, stale, outside tolerance, or faulted.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

from ol_verify_serial import Abort, REPO_ROOT, Shell, decode, has_fault, repo_short_hash


STAGES = (
    (150, 10.0, 0.25),
    (300, 7.0, 0.15),
    (500, 8.0, 0.15),
    (800, 11.0, 0.15),
)
SNAP_END = "============================="


def field(pattern: str, text: str) -> float:
    match = re.search(pattern, text)
    if match is None:
        raise ValueError(pattern)
    return float(match.group(1))


def parse_snap(text: str) -> dict[str, float | int | bool]:
    if "Open-Loop Snap" not in text or SNAP_END not in text:
        raise ValueError("incomplete olsnap")
    if has_fault(text) or "Fault      : 0 NONE" not in text:
        raise ValueError("motor fault")

    hall = re.search(
        r"Hall RPM\s*:\s*raw=([-+0-9.]+)\s+filtered=([-+0-9.]+)\s+"
        r"(?:fast=([-+0-9.]+)\s+)?age=([-+0-9.]+)ms\s+state=(-?\d+)",
        text,
    )
    measured = re.search(r"Meas RPM\s*:\s*([-+0-9.]+)\s+\((valid|invalid)\)", text)
    if hall is None or measured is None:
        raise ValueError("new Hall snapshot fields missing; flash latest firmware")

    return {
        "target": field(r"Target RPM\s*:\s*([-+0-9.]+)", text),
        "ramp": field(r"Ramp RPM\s*:\s*([-+0-9.]+)", text),
        "measured": float(measured.group(1)),
        "valid": measured.group(2) == "valid",
        "raw": float(hall.group(1)),
        "filtered": float(hall.group(2)),
        "fast": float(hall.group(3)) if hall.group(3) is not None else float(hall.group(2)),
        "age_ms": float(hall.group(4)),
        "state": int(hall.group(5)),
        "id": field(r"Id, Iq\s*:\s*([-+0-9.]+)\s+[-+0-9.]+", text),
        "iq": field(r"Id, Iq\s*:\s*[-+0-9.]+\s+([-+0-9.]+)", text),
    }


class HallRunner:
    def __init__(self, shell: Shell, samples: int) -> None:
        self.shell = shell
        self.samples = samples
        self.sections: list[tuple[str, str]] = []
        self.results: list[tuple[int, dict[str, float | int | bool]]] = []

    def rec(self, name: str, text: str) -> None:
        self.sections.append((name, text))
        print(f"\n######## {name} ########\n{text}", flush=True)

    def commands(self, name: str, commands: tuple[str, ...]) -> None:
        output = ""
        for command in commands:
            output += self.shell.run(command, timeout=20.0, progress=command == "olstart")
        self.rec(name, output)

    def pause(self, seconds: float, reason: str) -> None:
        self.rec(f"wait {seconds:.0f}s", reason + "\n")
        self.shell.pause(seconds)

    def snapshot(self, rpm: int, index: int, tolerance: float) -> None:
        text = self.shell.run("olsnap", timeout=15.0, progress=True)
        name = f"H{rpm}-{index}"
        self.rec(name, text)
        try:
            snap = parse_snap(text)
        except ValueError as exc:
            raise Abort(f"{name}: {exc}") from exc

        measured = float(snap["filtered"])
        if not bool(snap["valid"]):
            raise Abort(f"{name}: Hall speed invalid")
        if int(snap["state"]) not in range(1, 7):
            raise Abort(f"{name}: invalid Hall state {snap['state']}")
        if float(snap["age_ms"]) >= 200.0:
            raise Abort(f"{name}: Hall edge stale {snap['age_ms']} ms")
        if abs(measured - rpm) > rpm * tolerance:
            raise Abort(f"{name}: filtered RPM {measured:.1f} outside ±{tolerance * 100:.0f}%")
        self.results.append((rpm, snap))

    def run(self, max_rpm: int) -> int:
        self.commands(
            "H0-config",
            (
                "olstop", "olfault clear", "olmode 1", "olfb hall", "slenable 0",
                "spkp 0.006", "spki 0.003", "spiq 0.8", "spbrake 0",
                "oliq 0.6", "olid 0", "olkp 0.2", "olki 30", "olramp 50",
                "olspeed 150", "spenable 1", "olstart",
            ),
        )
        try:
            for rpm, wait_s, tolerance in STAGES:
                if rpm > max_rpm:
                    break
                if rpm != 150:
                    self.rec(f"H{rpm}-set", self.shell.run(f"olspeed {rpm}", timeout=20.0))
                self.pause(wait_s, f"Hall 闭环升速并稳定到 {rpm} RPM")
                for index in range(1, self.samples + 1):
                    self.snapshot(rpm, index, tolerance)
                    if index != self.samples:
                        self.shell.pause(1.0)
            self.rec("PASS", "Hall stability verification passed\n")
            return 0
        except Abort as exc:
            self.rec("ABORT", str(exc) + "\n")
            return 2
        finally:
            self.rec("STOP", self.shell.stop())


def default_log_path(max_rpm: int) -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return REPO_ROOT / "Doc" / "verify" / f"hall_verify_{workspace_id()}_{stamp}_{max_rpm}RPM.log"


def workspace_id() -> str:
    revision = repo_short_hash()
    try:
        dirty = subprocess.run(
            ["git", "diff", "--quiet"], cwd=REPO_ROOT, check=False,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        ).returncode != 0
    except OSError:
        dirty = False
    return revision + ("-dirty" if dirty else "")


def main() -> int:
    parser = argparse.ArgumentParser(description="Hall feedback stability verification")
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--max-rpm", type=int, choices=(150, 300, 500, 800), default=800)
    parser.add_argument("--samples", type=int, default=3, help="每档快照数，默认 3")
    parser.add_argument("--log", default="", help="日志路径；传 - 不写文件")
    args = parser.parse_args()
    if args.samples < 1 or args.samples > 10:
        parser.error("--samples must be 1..10")

    shell = Shell(args.port, args.baud)
    runner = HallRunner(shell, args.samples)
    rc = 1
    try:
        wake = decode(shell.read_until_prompt(0.5))
        if not wake.strip():
            wake = shell.kick()
        runner.rec("wake", wake)
        rc = runner.run(args.max_rpm)
        return rc
    finally:
        try:
            shell.stop()
        except Exception:
            pass
        shell.close()
        if args.log.strip() != "-":
            path = Path(args.log) if args.log.strip() else default_log_path(args.max_rpm)
            if not path.is_absolute():
                path = REPO_ROOT / path
            path.parent.mkdir(parents=True, exist_ok=True)
            header = (
                "# hall_verify\n"
                f"# workspace: {workspace_id()}\n"
                f"# time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"# max_rpm: {args.max_rpm}\n"
                f"# samples: {args.samples}\n"
                "# script: Scripts/hall_verify_serial.py\n"
            )
            body = [header]
            for name, text in runner.sections:
                body.append(f"\n######## {name} ########\n{text}")
            path.write_text("".join(body), encoding="utf-8")
            print(f"\nlog written: {path}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
