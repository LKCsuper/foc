#!/usr/bin/env python3
"""Send bounded shell commands to the controller and print/save the replies."""
from __future__ import annotations

import argparse
import time

from ol_verify_serial import REPO_ROOT, Shell


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("commands", nargs="+")
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--log", default="")
    args = parser.parse_args()

    shell = Shell(args.port, args.baud)
    output: list[str] = []
    try:
        output.append(shell.kick(timeout=5.0))
        for command in args.commands:
            text = shell.run(command, timeout=args.timeout, progress=args.timeout >= 15.0, tries=2)
            block = f"\n$ {command}\n{text}"
            output.append(block)
            print(block, flush=True)
    finally:
        shell.close()
    if args.log:
        path = REPO_ROOT / args.log
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(output), encoding="utf-8")
        print(f"Log: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
