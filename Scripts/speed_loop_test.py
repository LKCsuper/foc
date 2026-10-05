#!/usr/bin/env python3
"""Run the firmware's bounded encoder speed-loop sequence and save its UART log."""
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
    parser.add_argument("--dwell", type=int, default=2000)
    args = parser.parse_args()

    shell = Shell(args.port, args.baud)
    sections: list[str] = []
    commands = (
        "olstop", "pos off", "joint off", "olfault clear", "olmode 1",
        "olfb encoder", "slenable 0", "olalign 0", "olkp 0.12", "olki 18",
        "spkp 0.003", "spki 0.01", "spiq 0.30", "spbrake 1", "oliq 0.30",
        "olid 0", "olspeed 150", "spenable 1", "olstart",
    )
    try:
        sections.append(shell.kick(timeout=5.0))
        for command in commands:
            text = shell.run(command, timeout=8.0, tries=2)
            sections.append(f"\n$ {command}\n{text}")
            print(sections[-1], flush=True)
        command = f"speed test {args.dwell}"
        text = shell.run(command, timeout=max(35.0, args.dwell * 12 / 1000.0), progress=True, tries=1)
        sections.append(f"\n$ {command}\n{text}")
        print(sections[-1], flush=True)
        fault_text = shell.run("olfault", timeout=5.0, tries=1)
        sections.append("\n$ olfault\n" + fault_text)
        print(sections[-1], flush=True)
    finally:
        try:
            sections.append("\n$ olstop\n" + shell.run("olstop", timeout=5.0, tries=1))
        finally:
            shell.close()

    output = "".join(sections)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = REPO_ROOT / "Doc" / "verify" / f"speed_loop_{repo_short_hash()}-dirty_{stamp}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(output, encoding="utf-8")
    print(f"Log: {path}", flush=True)

    rows = re.findall(r"target=([-+0-9]+) RPM.*?meas=([-+0-9.]+) err=([-+0-9.]+) RPM", output, re.S)
    if len(rows) != 10 or "speed test done" not in output or "Fault = 0 NONE" not in output:
        print(f"FAIL: incomplete speed test ({len(rows)}/10 rows)", file=sys.stderr)
        return 1
    failed = []
    for target_text, measured_text, error_text in rows:
        target = float(target_text)
        error = float(error_text)
        tolerance = 8.0 if target == 0.0 else max(15.0, abs(target) * 0.12)
        if abs(error) > tolerance:
            failed.append(f"target {target:.0f}: error {error:.1f} > {tolerance:.1f} RPM")
        print(f"target={target:>5.0f} measured={float(measured_text):>7.1f} error={error:>7.1f}")
    if failed:
        print("FAIL: " + "; ".join(failed), file=sys.stderr)
        return 1
    print("PASS: encoder speed-loop sequence completed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
