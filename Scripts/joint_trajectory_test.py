#!/usr/bin/env python3
"""Run one bounded 2804 joint trajectory and reject residual oscillation."""
from __future__ import annotations

import argparse
import re
import sys
import time

from ol_verify_serial import REPO_ROOT, Shell, repo_short_hash


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--target", type=float, default=9.0)
    parser.add_argument("--settle", type=float, default=6.0)
    args = parser.parse_args()

    shell = Shell(args.port, args.baud)
    sections: list[str] = []

    def run(command: str, timeout: float = 8.0) -> str:
        reply = shell.run(command, timeout=timeout, progress=timeout >= 15.0, tries=2)
        block = f"\n$ {command}\n{reply}"
        sections.append(block)
        print(block, flush=True)
        return reply

    try:
        sections.append(shell.kick(timeout=5.0))
        for command in (
            "olstop", "pos off", "joint off", "olfault clear", "olmode 1",
            "olfb encoder", "slenable 0", "olalign 0", "joint zero",
            "joint kp 0.10", "joint kd 0.02", "joint ki 0.02", "joint lim 1.0",
            "joint on", "olstart", f"joint set {args.target:g}",
        ):
            run(command)
        shell.pause(args.settle)
        run("joint status")
        watch = run("encoder watch 3", timeout=10.0)
        status = run("joint status")
        fault = run("olfault")
    finally:
        try:
            run("olstop", timeout=5.0)
            run("joint off", timeout=5.0)
        finally:
            shell.close()

    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = REPO_ROOT / "Doc" / "verify" / f"joint_trajectory_{repo_short_hash()}-dirty_{stamp}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(sections), encoding="utf-8")
    print(f"Log: {path}", flush=True)

    state = re.search(r"pos=([-+0-9.]+) target=([-+0-9.]+) err=([-+0-9.]+).*?traj=(\d)", status)
    speeds = [abs(float(value)) for value in re.findall(r"raw=([-+0-9.]+)", watch)]
    if state is None or not speeds or "Fault = 0 NONE" not in fault:
        print("FAIL: incomplete joint trajectory evidence", file=sys.stderr)
        return 1
    error = float(state.group(3))
    active = int(state.group(4))
    peak_raw_rpm = max(speeds)
    print(f"error={error:.2f} deg trajectory_active={active} hold_peak_raw_rpm={peak_raw_rpm:.2f}")
    if abs(error) > 0.75 or active != 0 or peak_raw_rpm > 2.0:
        print("FAIL: target error or residual oscillation exceeds limit", file=sys.stderr)
        return 1
    print("PASS: joint trajectory settled without residual oscillation", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
