#!/usr/bin/env python3
"""Run a conservative bounded encoder position-loop test and save the UART log."""
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
    parser.add_argument("--settle", type=float, default=4.0)
    args = parser.parse_args()

    shell = Shell(args.port, args.baud)
    sections: list[str] = []
    rows: list[tuple[float, float, float]] = []

    def run(command: str, timeout: float = 8.0) -> str:
        text = shell.run(command, timeout=timeout, progress=timeout >= 15.0, tries=2)
        block = f"\n$ {command}\n{text}"
        sections.append(block)
        print(block, flush=True)
        return text

    try:
        sections.append(shell.kick(timeout=5.0))
        for command in (
            "olstop", "pos off", "joint off", "olfault clear", "olmode 1",
            "olfb encoder", "slenable 0", "olalign 0", "olkp 0.12", "olki 18",
            "spkp 0.003", "spki 0.01", "spiq 0.30", "spbrake 1",
            "pos zero", "pos max 2", "pos kp 1", "pos set 0", "olstart",
        ):
            run(command)

        for target in (2.0, 5.0, -2.0, 0.0):
            run(f"pos set {target:g}")
            shell.pause(args.settle)
            text = run("pos status")
            match = re.search(
                r"pos=([-+0-9.]+)deg target=([-+0-9.]+)deg err=([-+0-9.]+)deg", text
            )
            if match:
                rows.append(tuple(float(match.group(i)) for i in range(1, 4)))
        fault_text = run("olfault")
    finally:
        try:
            run("olstop", timeout=5.0)
            run("pos off", timeout=5.0)
        finally:
            shell.close()

    output = "".join(sections)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = REPO_ROOT / "Doc" / "verify" / f"position_loop_{repo_short_hash()}-dirty_{stamp}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(output, encoding="utf-8")
    print(f"Log: {path}", flush=True)

    if len(rows) != 4 or "Fault = 0 NONE" not in fault_text:
        print(f"FAIL: incomplete position test ({len(rows)}/4 rows)", file=sys.stderr)
        return 1
    failures = []
    for pos, target, error in rows:
        print(f"target={target:>5.1f} position={pos:>6.2f} error={error:>6.2f}")
        if abs(error) > 1.0:
            failures.append(f"target {target:.1f}: error {error:.2f} deg")
    if failures:
        print("FAIL: " + "; ".join(failures), file=sys.stderr)
        return 1
    print("PASS: conservative encoder position-loop test completed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
