#!/usr/bin/env python3
"""只读采集停机时的电流快照，与 Hall 运行统计作对照。"""

import argparse
import re
from pathlib import Path

from hall_verify_serial import parse_snap
from ol_verify_serial import Shell


def raw_counts(text: str) -> tuple[int, int, int]:
    match = re.search(r"Iadc raw\s*:\s*(\d+)\s+(\d+)\s+(\d+)", text)
    if match is None:
        raise ValueError("ADC raw counts missing")
    return tuple(int(match.group(i)) for i in range(1, 4))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM19")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--samples", type=int, default=80)
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 1 or args.samples > 200:
        parser.error("--samples must be 1..200")

    shell = Shell(args.port, args.baud)
    lines = ["index,phase_sum_a,abs_phase_sum_a,curr1,curr2,curr3\n"]
    maximum = 0.0
    exceeded = 0
    try:
        for index in range(1, args.samples + 1):
            response = shell.run("olsnap", timeout=15.0)
            if "Enable     : OFF" not in response or "Target RPM : 0" not in response:
                raise RuntimeError("motor is not stopped; idle probe aborted")
            snap = parse_snap(response)
            total = float(snap["abc_sum"])
            raw = raw_counts(response)
            maximum = max(maximum, abs(total))
            exceeded += abs(total) > 0.5
            lines.append(f"{index},{total:.3f},{abs(total):.3f},{raw[0]},{raw[1]},{raw[2]}\n")
    finally:
        shell.close()
        args.log.parent.mkdir(parents=True, exist_ok=True)
        args.log.write_text("".join(lines), encoding="utf-8")
    print(f"stopped samples={len(lines) - 1}, |sum|>0.5 A={exceeded}, max={maximum:.3f} A")
    print(f"log written: {args.log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
