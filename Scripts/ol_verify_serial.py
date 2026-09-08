#!/usr/bin/env python3
"""Empty-shaft open-loop electrical check over UART (COM17, 460800).

串口原文默认写入仓库 `Doc/verify/ol_verify_<提交短哈希>_<YYYYMMDD-HHMMSS>.log`，该目录可提交。
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parents[1]
VERIFY_LOG_DIR = REPO_ROOT / "Doc" / "verify"

PROMPT_TAIL = b"] $ "


def decode(data: bytes) -> str:
    return data.decode("utf-8", errors="replace").replace("\r\n", "\n").replace("\r", "\n")


class Shell:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.15, write_timeout=2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        self.ser.close()

    def read_until_prompt(self, timeout: float) -> bytes:
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
                if PROMPT_TAIL in buf[-48:]:
                    time.sleep(0.04)
                    extra = self.ser.read(4096)
                    if extra:
                        buf += extra
                    return buf
            else:
                time.sleep(0.02)
        return buf

    def run(self, line: str, timeout: float = 4.0) -> str:
        payload = (line + "\r").encode("ascii")
        self.ser.write(payload)
        return decode(self.read_until_prompt(timeout))

    def stop(self) -> str:
        return self.run("olstop", timeout=3.0)


def has_fault(text: str) -> bool:
    return ("LATCHED" in text) or ("ABS_OVER_CURRENT" in text) or ("UNDER_VOLTAGE" in text)


def repo_short_hash() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=REPO_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return out.strip() or "unknown"
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return VERIFY_LOG_DIR / f"ol_verify_{repo_short_hash()}_{stamp}.log"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument(
        "--log",
        default="",
        help="默认 Doc/verify/ol_verify_<hash>_<时间>.log；传 - 则不写文件",
    )
    args = parser.parse_args()

    sections: list[tuple[str, str]] = []
    shell = Shell(args.port, args.baud)
    aborted = ""

    def rec(name: str, text: str) -> None:
        sections.append((name, text))
        print(f"\n######## {name} ########\n{text}", flush=True)

    try:
        wake = decode(shell.read_until_prompt(0.4))
        if PROMPT_TAIL.decode() not in wake:
            wake += shell.run("")
        rec("wake", wake)

        v0 = ""
        for cmd in ("olstop", "olalign 0", "hfistart 0", "olfb openloop", "spenable 0", "slenable 0"):
            v0 += shell.run(cmd)
        rec("V0", v0)

        v1 = shell.run("olstatus", timeout=5.0)
        rec("V1", v1)
        if has_fault(v1):
            aborted = "V1 fault"
            rec("ABORT", aborted)
            return 2

        v2_cmd = ""
        for cmd in ("olstop", "olmode 0", "olboost 5", "olkvf 0.057", "olramp 10", "olspeed 72"):
            v2_cmd += shell.run(cmd)
        v2_cmd += shell.run("olstart")
        rec("V2-start", v2_cmd)
        time.sleep(20.0)
        v2 = shell.run("olstatus", timeout=5.0)
        rec("V2", v2)
        if has_fault(v2):
            aborted = "V2 fault"
            rec("ABORT", aborted)
            rec("stop", shell.stop())
            return 2

        v3 = shell.stop() + shell.run("olstatus", timeout=5.0)
        rec("V3", v3)

        v4_cmd = ""
        for cmd in ("olstop", "olmode 1", "oliq 0.6", "olid 0", "olkp 0.2", "olki 30", "olramp 10", "olspeed 72"):
            v4_cmd += shell.run(cmd)
        rec("V4-cfg", v4_cmd)
        shell.ser.write(b"olstart\rolstatus\r")
        v4 = decode(shell.read_until_prompt(6.0))
        rec("V4", v4)
        if has_fault(v4):
            aborted = "V4 fault"
            rec("ABORT", aborted)
            rec("stop", shell.stop())
            return 2

        time.sleep(8.0)
        v5 = shell.run("olstatus", timeout=5.0)
        rec("V5", v5)
        if has_fault(v5):
            aborted = "V5 fault"
            rec("ABORT", aborted)
            rec("stop", shell.stop())
            return 2

        for label, wait_s in (("V6-15s", 7.0), ("V6-30s", 15.0), ("V6-45s", 15.0), ("V6-60s", 15.0)):
            time.sleep(wait_s)
            text = shell.run("olstatus", timeout=5.0)
            rec(label, text)
            if has_fault(text):
                aborted = f"{label} fault"
                rec("ABORT", aborted)
                rec("stop", shell.stop())
                return 2

        v7 = shell.stop() + shell.run("olstatus", timeout=5.0)
        rec("V7", v7)

        v8_cmd = ""
        for cmd in ("olmode 1", "oliq 0.6", "olramp 10", "olspeed 72"):
            v8_cmd += shell.run(cmd)
        v8_cmd += shell.run("olstart")
        rec("V8-start72", v8_cmd)
        time.sleep(18.0)
        rec("V8-72hold", shell.run("olstatus", timeout=5.0))
        rec("V8-set150", shell.run("olspeed 150"))
        time.sleep(28.0)
        v8 = shell.run("olstatus", timeout=5.0)
        rec("V8", v8)
        if has_fault(v8):
            aborted = "V8 fault"
            rec("ABORT", aborted)
            rec("stop", shell.stop())
            return 2

        rec("V9-set300", shell.run("olspeed 300"))
        time.sleep(36.0)
        v9 = shell.run("olstatus", timeout=5.0)
        rec("V9", v9)
        if has_fault(v9):
            aborted = "V9 fault"
            rec("ABORT", aborted)
            rec("stop", shell.stop())
            return 2

        rec("V10-set72", shell.run("olspeed 72"))
        time.sleep(34.0)
        v10 = shell.run("olstatus", timeout=5.0)
        rec("V10", v10)
        if has_fault(v10):
            aborted = "V10 fault"
            rec("ABORT", aborted)
            rec("stop", shell.stop())
            return 2

        rec("V11-stop", shell.stop())
        rec("V11-set", shell.run("olspeed -72"))
        rec("V11-start", shell.run("olstart"))
        time.sleep(28.0)
        v11 = shell.run("olstatus", timeout=5.0)
        rec("V11", v11)
        rec("V11-end", shell.stop() + shell.run("olstatus", timeout=5.0))
        if has_fault(v11):
            aborted = "V11 fault"
            rec("ABORT", aborted)
            return 2
        return 0
    finally:
        try:
            shell.stop()
        except Exception:
            pass
        shell.close()
        if args.log.strip() != "-":
            path = Path(args.log) if args.log.strip() else default_log_path()
            if not path.is_absolute():
                path = REPO_ROOT / path
            path.parent.mkdir(parents=True, exist_ok=True)
            header = (
                f"# ol_verify\n"
                f"# hash: {repo_short_hash()}\n"
                f"# time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"# script: Scripts/ol_verify_serial.py\n"
            )
            body = [header]
            for name, text in sections:
                body.append(f"\n######## {name} ########\n{text}")
            path.write_text("".join(body), encoding="utf-8")
            print(f"\nlog written: {path}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
