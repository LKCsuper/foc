#!/usr/bin/env python3
"""Bounded UART acceptance: encoder -> current -> speed -> position -> joint watchdog."""
from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

from ol_verify_serial import REPO_ROOT, Shell, decode, has_fault, repo_short_hash


def log_path() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return REPO_ROOT / "Doc" / "verify" / f"encoder_full_{repo_short_hash()}-dirty_{stamp}.log"


def number(pattern: str, text: str) -> float | None:
    match = re.search(pattern, text)
    return float(match.group(1)) if match else None


class Verify:
    def __init__(self, shell: Shell) -> None:
        self.shell = shell
        self.sections: list[tuple[str, str]] = []
        self.results: list[str] = []

    def run(self, name: str, command: str, timeout: float = 20.0) -> str:
        text = self.shell.run(command, timeout=timeout, progress=timeout >= 20.0, tries=2)
        self.sections.append((name, text))
        print(f"\n######## {name}: {command} ########\n{text}", flush=True)
        return text

    def pass_stage(self, name: str, detail: str) -> None:
        self.results.append(f"PASS {name}: {detail}")

    def fail(self, name: str, detail: str) -> None:
        self.results.append(f"FAIL {name}: {detail}")
        raise RuntimeError(f"{name}: {detail}")

    def ensure_safe(self, name: str, text: str) -> None:
        if has_fault(text) or "Fault      : 0 NONE" not in text:
            self.fail(name, "fault present or snapshot incomplete")
        vbus = number(r"V_bus\s*:\s*([-+0-9.]+)", text)
        if vbus is None or vbus < 8.0:
            self.fail(name, f"invalid Vbus {vbus}")
        id_a = number(r"Id, Iq\s*:\s*([-+0-9.]+)", text)
        iq_a = number(r"Id, Iq\s*:\s*[-+0-9.]+\s+([-+0-9.]+)", text)
        if id_a is None or iq_a is None or max(abs(id_a), abs(iq_a)) > 1.5:
            self.fail(name, f"unsafe current Id={id_a} Iq={iq_a}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM20")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--log", default="")
    args = parser.parse_args()
    out = Path(args.log) if args.log else log_path()
    if not out.is_absolute():
        out = REPO_ROOT / out

    shell = Shell(args.port, args.baud)
    verify = Verify(shell)
    rc = 1
    try:
        wake = decode(shell.read_until_prompt(2.0)) or shell.kick()
        verify.sections.append(("wake", wake))

        baseline = ""
        for cmd in ("olstop", "joint off", "olfault clear", "encoder show", "olsnap"):
            baseline += verify.run("baseline", cmd, 30.0 if cmd == "olsnap" else 10.0)
        if "valid=1" not in baseline or "signal_fault=0" not in baseline:
            verify.fail("encoder", "MT6701 invalid or signal fault")
        verify.ensure_safe("baseline", baseline)
        verify.pass_stage("baseline", "UART, Vbus, fault and MT6701 valid")

        # BM75 上 0.15 A 只有约 1.9 个 ADC 计数，方向对齐容易受量化和齿槽影响。
        verify.run("encoder-detect", "oliq 0.30", 8.0)
        detect = verify.run("encoder-detect", "encoder detect", 35.0)
        if "rejected" in detect.lower() or "FAILED" in detect:
            verify.fail("encoder", "encoder detect was rejected or failed")
        show = verify.run("encoder-detect", "encoder show", 10.0)
        if "zero_valid=1" not in show or "signal_fault=0" not in show:
            verify.fail("encoder", "detect did not establish a valid zero")
        verify.pass_stage("encoder", "zero valid and signal healthy")

        for cmd in (
            "olstop", "olmode 1", "olfb encoder", "slenable 0", "spenable 0",
            "olalign 0", "hfistart 0 2 8", "olkp 0.12", "olki 18", "olid 0.30", "oliq 0", "olspeed 0", "olstart",
        ):
            verify.run("current-config", cmd, 20.0 if cmd == "olstart" else 8.0)
        shell.pause(2.0)
        current = verify.run("current", "olsnap", 35.0)
        verify.ensure_safe("current", current)
        id_a = number(r"Id, Iq\s*:\s*([-+0-9.]+)", current)
        iq_a = number(r"Id, Iq\s*:\s*[-+0-9.]+\s+([-+0-9.]+)", current)
        if id_a is None or iq_a is None or abs(id_a - 0.30) > 0.10 or abs(iq_a) > 0.15:
            verify.fail("current", f"static D-axis tracking failed: Id={id_a} Iq={iq_a}")
        verify.pass_stage("current", f"static target Id=0.300A, measured Id={id_a:.3f}A Iq={iq_a:.3f}A")

        for cmd in ("olstop", "olid 0", "oliq 0.15", "spiq 0.20", "spkp 0.0005", "spki 0.00015", "spbrake 0", "olspeed 60", "spenable 1", "olstart"):
            verify.run("speed-config", cmd, 20.0 if cmd == "olstart" else 8.0)
        shell.pause(5.0)
        speed = verify.run("speed", "olsnap", 35.0)
        verify.ensure_safe("speed", speed)
        rpm = number(r"Meas RPM\s*:\s*([-+0-9.]+)\s+\(valid\)", speed)
        if rpm is None or rpm < 30.0 or rpm > 90.0:
            verify.fail("speed", f"60 RPM target out of bounds: {rpm}")
        verify.pass_stage("speed", f"target 60 RPM, measured {rpm:.1f} RPM")

        for cmd in ("olstop", "pos zero", "pos max 20", "pos kp 1", "pos hold", "olstart"):
            verify.run("position-config", cmd, 20.0 if cmd == "olstart" else 8.0)
        for target in (5, 0, -5, 0):
            verify.run("position", f"pos set {target}", 8.0)
            shell.pause(2.0)
            status = verify.run("position", "pos", 8.0)
            err = number(r"err=([-+0-9.]+)deg", status)
            if err is None or abs(err) > 4.0:
                verify.fail("position", f"target {target} deg, error {err}")
        verify.pass_stage("position", "+/-5 degree sequence settled")

        for cmd in ("olstop", "joint zero", "joint lim 1", "joint kp 0.03", "joint kd 0.02", "joint ki 0", "joint on", "olstart"):
            verify.run("joint-config", cmd, 20.0 if cmd == "olstart" else 8.0)
        timeout = verify.run("joint-timeout", "joint timeouttest 0.05 180", 20.0)
        if not re.search(r"timeouttest result:\s*timeout=1\s+age=(1[0-9]{5}|[2-9][0-9]{5,})us", timeout):
            verify.fail("joint-timeout", "watchdog did not expire after the one-shot command")
        joint = verify.run("joint-timeout", "joint status", 8.0)
        if "timeout=0" not in joint or "watchdog=0" not in joint:
            verify.fail("joint-timeout", "cleanup did not restore local hold")
        verify.pass_stage("joint-timeout", "100 ms watchdog expired and cleanup restored hold")
        rc = 0
    except Exception as exc:
        verify.sections.append(("exception", f"{type(exc).__name__}: {exc}\n"))
        print(f"ABORT: {exc}", flush=True)
        rc = 2
    finally:
        try:
            stopped = shell.stop()
            verify.sections.append(("final-stop", stopped))
            print(f"\n######## final-stop ########\n{stopped}", flush=True)
        except Exception as exc:
            verify.sections.append(("final-stop-error", f"{exc}\n"))
            rc = 2
        shell.close()
        summary = "\n".join(verify.results) + "\n"
        verify.sections.append(("summary", summary))
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text("".join(f"\n######## {name} ########\n{text}" for name, text in verify.sections), encoding="utf-8")
        print(f"\n######## SUMMARY ########\n{summary}log: {out}", flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
