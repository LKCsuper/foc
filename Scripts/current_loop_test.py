#!/usr/bin/env python3
"""Run the bounded static D-axis current-loop test and save the UART log."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from ol_verify_serial import REPO_ROOT, Shell, repo_short_hash


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM20")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--current", type=float, default=0.30)
    parser.add_argument("--duration", type=int, default=2500)
    args = parser.parse_args()

    stamp = time.strftime("%Y%m%d-%H%M%S")
    log_path = REPO_ROOT / "Doc" / "verify" / (
        f"current_loop_{repo_short_hash()}-dirty_{stamp}.log"
    )
    sections: list[str] = []
    shell = Shell(args.port, args.baud)
    try:
        sections.append(shell.kick(timeout=5.0))
        for command, timeout in (
            ("olstop", 5.0),
            ("pos off", 5.0),
            ("joint off", 5.0),
            ("olfault clear", 5.0),
            (f"curtest {args.current:.2f} {args.duration}", 12.0),
            ("olfault", 5.0),
        ):
            text = shell.run(command, timeout=timeout, progress=timeout >= 10.0, tries=2)
            sections.append(f"\n$ {command}\n{text}")
            print(sections[-1], flush=True)
    finally:
        try:
            sections.append("\n$ olstop\n" + shell.run("olstop", timeout=3.0, tries=1))
        finally:
            shell.close()

    log_path.parent.mkdir(parents=True, exist_ok=True)
    output = "".join(sections)
    log_path.write_text(output, encoding="utf-8")
    print(f"Log: {log_path}", flush=True)

    required = (
        "Current test result:",
        "fault=NONE",
        "auto-stopped=yes",
        "config restored=yes",
        "Fault = 0 NONE",
    )
    missing = [item for item in required if item not in output]
    if missing:
        print("FAIL: missing " + ", ".join(missing), file=sys.stderr)
        return 1
    print("PASS: bounded D-axis current-loop test completed", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
