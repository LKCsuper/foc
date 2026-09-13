#!/usr/bin/env python3
"""Stop BLDC output before building or programming the controller."""
from __future__ import annotations

import argparse

from ol_verify_serial import Shell, decode


def main() -> int:
    parser = argparse.ArgumentParser(description="Stop BM75 BLDC output over UART")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=460800)
    args = parser.parse_args()

    shell = Shell(args.port, args.baud)
    try:
        decode(shell.read_until_prompt(0.5))
        for command in ("bldc stop", "bldc exit"):
            text = shell.run(command, timeout=5.0, progress=True)
            print(text, flush=True)
    finally:
        shell.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
