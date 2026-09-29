#!/usr/bin/env python3
"""Capture one encoder speed-loop run over UART and always stop the motor.

Default sequence is the C2208 150 RPM bench: encoder feedback, current gains
1.06/2120, speed Kp 0.0005, Ki 0.00015, Iq limit 0.08 A. Raw text goes to Doc/verify/.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

from ol_verify_serial import REPO_ROOT, Shell, decode, has_fault, repo_short_hash

VERIFY_LOG_DIR = REPO_ROOT / "Doc" / "verify"
SNAP_END = "============================="
def bench_commands(speed_kp: float, speed_ki: float, speed_iq: float, current_kp: float, current_ki: float) -> tuple[str, ...]:
    return (
    "olstop",
    "olfault clear",
    "olalign 0",
    "olmode 1",
    "olfb encoder",
    "slenable 0",
    f"olkp {current_kp:g}",
    f"olki {current_ki:g}",
    f"spkp {speed_kp:g}",
    f"spki {speed_ki:g}",
    f"spiq {speed_iq:g}",
    "spbrake 1",
    "oliq 0.25",
    "olid 0",
    "olspeed 150",
    "spenable 1",
    "olstart",
)
MAX_PHASE_CURRENT = 2.0
MIN_VBUS = 22.0


def workspace_id() -> str:
    revision = repo_short_hash()
    try:
        dirty = subprocess.run(
            ["git", "diff", "--quiet"],
            cwd=REPO_ROOT,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode != 0
    except OSError:
        dirty = False
    return revision + ("-dirty" if dirty else "")


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return VERIFY_LOG_DIR / f"encoder_speed_{workspace_id()}_{stamp}_150RPM.log"


def field(pattern: str, text: str) -> float | None:
    match = re.search(pattern, text)
    if match is None:
        return None
    return float(match.group(1))


def parse_encoder_snap(text: str) -> dict[str, float | int | bool]:
    if "Open-Loop Snap" not in text or SNAP_END not in text:
        raise ValueError("incomplete olsnap")
    measured = re.search(r"Meas RPM\s*:\s*([-+0-9.]+)\s+\((valid|invalid)\)", text)
    sample = re.search(r"FOC sample\s*:\s*up=(\d+)\s+down=(\d+)", text)
    vq = re.search(r"Vd, Vq out\s*:\s*([-+0-9.]+)\s+([-+0-9.]+)", text)
    iq = re.search(
        r"Id, Iq\s*:\s*([-+0-9.]+)\s+([-+0-9.]+)\s+A(?:\s+\(target\s+([-+0-9.]+)\s+([-+0-9.]+)\))?",
        text,
    )
    if measured is None or vq is None or iq is None:
        raise ValueError("encoder snapshot fields missing")
    pwm = field(r"PWM ideal\s*:\s*\|Valphabeta\|=([-+0-9.]+)", text)
    speed_iq = field(r"Speed Iq\s*:\s*([-+0-9.]+)", text)
    vbus = field(r"V_bus\s*:\s*([-+0-9.]+)", text)
    return {
        "measured": float(measured.group(1)),
        "valid": measured.group(2) == "valid",
        "fault": has_fault(text) or "Fault      : 0 NONE" not in text,
        "up": int(sample.group(1)) if sample else -1,
        "down": int(sample.group(2)) if sample else -1,
        "id": float(iq.group(1)),
        "iq": float(iq.group(2)),
        "iq_target": float(iq.group(4)) if iq.group(4) is not None else float("nan"),
        "speed_iq": speed_iq if speed_iq is not None else float("nan"),
        "vq": float(vq.group(2)),
        "pwm": pwm if pwm is not None else float("nan"),
        "vbus": vbus if vbus is not None else float("nan"),
    }


def format_row(index: int, snap: dict[str, float | int | bool]) -> str:
    return (
        f"{index}: rpm={float(snap['measured']):.1f} valid={int(bool(snap['valid']))} "
        f"SpeedIq={float(snap['speed_iq']):.3f} Iq={float(snap['iq']):.3f}/{float(snap['iq_target']):.3f} "
        f"Id={float(snap['id']):.3f} Vq={float(snap['vq']):.3f} PWM={float(snap['pwm']):.3f} "
        f"up={int(snap['up'])} down={int(snap['down'])} Vbus={float(snap['vbus']):.1f}"
    )


def unsafe(snap: dict[str, float | int | bool]) -> str:
    if bool(snap["fault"]):
        return "fault latched"
    if abs(float(snap["id"])) > MAX_PHASE_CURRENT or abs(float(snap["iq"])) > MAX_PHASE_CURRENT:
        return f"phase current Id={float(snap['id']):.2f} Iq={float(snap['iq']):.2f} A"
    if float(snap["vbus"]) < MIN_VBUS:
        return f"Vbus {float(snap['vbus']):.1f} V"
    return ""


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture encoder 150 RPM speed-loop snaps")
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--settle", type=float, default=8.0)
    parser.add_argument("--spkp", type=float, default=0.0005)
    parser.add_argument("--spki", type=float, default=0.00015)
    parser.add_argument("--spiq", type=float, default=0.08)
    parser.add_argument("--olkp", type=float, default=1.06)
    parser.add_argument("--olki", type=float, default=2120.0)
    parser.add_argument("--log", default="")
    args = parser.parse_args()
    if args.samples < 1 or args.samples > 12:
        parser.error("--samples must be 1..12")

    sections: list[tuple[str, str]] = []
    rows: list[str] = []
    shell = Shell(args.port, args.baud)
    rc = 1
    try:
        wake = decode(shell.read_until_prompt(2.0))
        if not wake.strip():
            wake = shell.kick()
        sections.append(("wake", wake))
        print(f"\n######## wake ########\n{wake}", flush=True)

        blob = ""
        for command in bench_commands(args.spkp, args.spki, args.spiq, args.olkp, args.olki):
            blob += shell.run(command, timeout=20.0, progress=command == "olstart")
        sections.append(("config", blob))
        print(f"\n######## config ########\n{blob}", flush=True)
        if "Speed loop enabled" not in blob:
            sections.append(("FAIL", "speed loop was not enabled\n"))
            print("speed loop was not enabled", flush=True)
            rc = 2
        elif has_fault(blob):
            sections.append(("FAIL", "fault during start\n"))
            print("fault during start", flush=True)
            rc = 2
        else:
            shell.pause(args.settle)
            for index in range(1, args.samples + 1):
                text = shell.run("olsnap", timeout=45.0, progress=True)
                sections.append((f"snap-{index}", text))
                print(f"\n######## snap-{index} ########\n{text}", flush=True)
                try:
                    snap = parse_encoder_snap(text)
                except ValueError as exc:
                    rows.append(f"{index}: parse failed: {exc}")
                    rc = 2
                    break
                rows.append(format_row(index, snap))
                reason = unsafe(snap)
                if reason:
                    rows.append(f"STOP: {reason}")
                    rc = 2
                    break
                if index != args.samples:
                    shell.pause(0.5)
            if rc == 1:
                rc = 0
    finally:
        try:
            stopped = shell.stop()
            sections.append(("STOP", stopped))
            print(f"\n######## STOP ########\n{stopped}", flush=True)
        except Exception as exc:
            sections.append(("STOP-ERROR", f"{exc}\n"))
            rc = 2
        shell.close()

    summary = "encoder 150 RPM capture\n" + "\n".join(rows) + "\n"
    sections.append(("SUMMARY", summary))
    print(f"\n######## SUMMARY ########\n{summary}", flush=True)
    if args.log.strip() != "-":
        path = Path(args.log) if args.log.strip() else default_log_path()
        if not path.is_absolute():
            path = REPO_ROOT / path
        path.parent.mkdir(parents=True, exist_ok=True)
        header = (
            "# encoder_speed_capture\n"
            f"# workspace: {workspace_id()}\n"
            f"# time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"# port: {args.port}\n"
            f"# samples: {args.samples}\n"
            "# script: Scripts/encoder_speed_capture.py\n"
        )
        body = [header]
        for name, text in sections:
            body.append(f"\n######## {name} ########\n{text}")
        path.write_text("".join(body), encoding="utf-8")
        print(f"log: {path}", flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
