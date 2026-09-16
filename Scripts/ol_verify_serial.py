#!/usr/bin/env python3
"""Empty-shaft open-loop electrical check over UART (COM17, 460800).

默认跑完整 V0～V11。单测某一项：

    python Scripts/ol_verify_serial.py --only V9
    python Scripts/ol_verify_serial.py --only V4,V5
    python Scripts/ol_verify_serial.py --list

串口原文默认写入 `Doc/verify/ol_verify_<提交短哈希>_<YYYYMMDD-HHMMSS>[_项].log`。
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
STATUS_MARK = b"Open-Loop Status"
SNAP_MARK = b"Open-Loop Snap"
STATUS_END = b"============================="

# 与 Doc/doc/开环稳定性验证.md 条目对应；V6 含 15/30/45/60 s 四张快照。
ITEM_IDS = ("V0", "V1", "V2", "V3", "V4", "V5", "V6", "V7", "V8", "V9", "V10", "V11")
ITEM_HELP = {
    "V0": "关闭对齐/HFI/有感/速度环/无感",
    "V1": "停机 olstatus",
    "V2": "V/F 72 RPM 约 20 s",
    "V3": "V/F 停机快照",
    "V4": "I/F olstart 后立刻 olstatus（建流）",
    "V5": "I/F 72 RPM 拖转约 8 s",
    "V6": "I/F 72 RPM 保持 60 s（四张）",
    "V7": "I/F 停机快照",
    "V8": "I/F 升到 150 RPM",
    "V9": "I/F 升到 300 RPM",
    "V10": "I/F 降回 72 RPM",
    "V11": "先停再反 −72 RPM",
}


class Abort(Exception):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.name = name


def decode(data: bytes) -> str:
    return data.decode("utf-8", errors="replace").replace("\r\n", "\n").replace("\r", "\n")


class Shell:
    def __init__(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.15, write_timeout=5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        self.ser.close()

    def read_until_prompt(self, timeout: float, *, progress: bool = False) -> bytes:
        buf = b""
        t0 = time.time()
        deadline = t0 + timeout
        last_print = t0
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
                tail = buf[-120:] if len(buf) > 120 else buf
                if PROMPT_TAIL in tail or buf.rstrip().endswith(b"$"):
                    time.sleep(0.08)
                    extra = self.ser.read(4096)
                    if extra:
                        buf += extra
                    return buf
            else:
                time.sleep(0.02)
            now = time.time()
            if progress and now - last_print >= 3.0:
                print(f"  ... uart {len(buf)} bytes {now - t0:.0f}/{timeout:.0f}s", flush=True)
                last_print = now
        return buf

    def _got_prompt(self, raw: bytes) -> bool:
        if not raw:
            return False
        tail = raw[-120:] if len(raw) > 120 else raw
        if PROMPT_TAIL in tail or raw.rstrip().endswith(b"$"):
            return True
        return (STATUS_MARK in raw or SNAP_MARK in raw) and STATUS_END in raw

    def run(self, line: str, timeout: float = 8.0, *, progress: bool = False, tries: int = 3) -> str:
        last = b""
        for attempt in range(max(1, tries)):
            send = attempt == 0 or (not last.strip() and line != "olstart")
            if attempt and send:
                print(f"  ... retry {attempt + 1}/{tries} {line!r}", flush=True)
            elif attempt:
                print(f"  ... extra wait {line!r}", flush=True)
            if send:
                if progress:
                    print(f"  ... send {line} (up to {timeout:.0f}s)", flush=True)
                self.ser.write((line + "\r").encode("ascii"))
            last += self.read_until_prompt(timeout, progress=progress)
            if self._got_prompt(last):
                return decode(last)
            if line == "olstart" and b"Open-loop started" in last:
                return decode(last)
        return decode(last)

    def drain(self, seconds: float) -> bytes:
        """持续读串口，保持 USB 轮询。CH340 空闲 sleep 后再写，第一拍常丢。"""
        buf = b""
        end = time.time() + seconds
        while time.time() < end:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
        return buf

    def kick(self, timeout: float = 20.0) -> str:
        """回车同步提示符。电机转着时命令任务可能慢，超时放宽。"""
        waiting = getattr(self.ser, "in_waiting", 0) or 0
        if waiting:
            self.ser.read(waiting)
        return self.run("", timeout=timeout)

    def pause(self, seconds: float) -> None:
        """等待 seconds：持续读串口保持 USB 活跃，并打印进度。结束后不发空回车。"""
        t0 = time.time()
        end = t0 + seconds
        last_print = t0
        print(f"  ... waiting 0/{seconds:.0f}s", flush=True)
        while True:
            remain = end - time.time()
            if remain <= 0.0:
                break
            self.drain(min(remain, 1.0))
            now = time.time()
            if now - last_print >= 2.0:
                print(f"  ... waiting {now - t0:.0f}/{seconds:.0f}s", flush=True)
                last_print = now
        print(f"  ... waiting {seconds:.0f}/{seconds:.0f}s done", flush=True)

    def stop(self) -> str:
        self.drain(0.3)
        return self.run("olstop", timeout=20.0)


def has_fault(text: str) -> bool:
    return ("LATCHED" in text) or ("ABS_OVER_CURRENT" in text) or ("UNDER_VOLTAGE" in text)


def has_status_page(text: str) -> bool:
    if "=============================" not in text:
        return False
    if ("Open-Loop Status" not in text) and ("Open-Loop Snap" not in text):
        return False
    return "Id, Iq" in text or "Id / Iq" in text


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


def default_log_path(tag: str = "") -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    name = f"ol_verify_{repo_short_hash()}_{stamp}"
    if tag:
        name += f"_{tag}"
    return VERIFY_LOG_DIR / f"{name}.log"


def parse_only(raw: str) -> list[str]:
    if not raw.strip():
        return []
    items: list[str] = []
    for part in raw.replace(";", ",").split(","):
        token = part.strip().upper()
        if not token:
            continue
        if token.isdigit():
            token = f"V{token}"
        if token.startswith("V6-"):
            token = "V6"
        if token not in ITEM_IDS:
            raise ValueError(f"未知项 {part.strip()}，可用: {', '.join(ITEM_IDS)}")
        if token not in items:
            items.append(token)
    return items


class Runner:
    def __init__(self, shell: Shell) -> None:
        self.shell = shell
        self.sections: list[tuple[str, str]] = []
        self.if_running = False

    def rec(self, name: str, text: str) -> None:
        self.sections.append((name, text))
        print(f"\n######## {name} ########\n{text}", flush=True)

    def rec_status(self, name: str, text: str | None = None, *, timeout: float = 12.0) -> None:
        if text is None:
            print(f"\n######## {name} ########", flush=True)
            print(f"olsnap  (短快照，最多 {timeout:.0f}s)", flush=True)
            text = self.shell.run("olsnap", timeout=timeout, progress=True)
            self.sections.append((name, text))
            print(text, flush=True)
        else:
            self.rec(name, text)
        if not has_status_page(text):
            raise Abort(f"{name} empty")
        if has_fault(text):
            raise Abort(name)

    def rec_speed(self, name: str, text: str) -> None:
        self.rec(name, text)
        if "Target RPM" not in text:
            raise Abort(f"{name} empty")

    def wait(self, seconds: float, why: str) -> None:
        self.rec(f"wait {seconds:.0f}s", why + "\n")
        self.shell.pause(seconds)

    def cmds(self, name: str, lines: tuple[str, ...]) -> str:
        blob = ""
        for cmd in lines:
            blob += self.shell.run(cmd)
        self.rec(name, blob)
        return blob

    def wake(self) -> None:
        text = decode(self.shell.read_until_prompt(0.4))
        if PROMPT_TAIL.decode() not in text:
            text += self.shell.run("")
        self.rec("wake", text)

    def v0(self) -> None:
        self.cmds("V0", ("olstop", "olalign 0", "hfistart 0", "olfb openloop", "spenable 0", "slenable 0"))
        self.if_running = False

    def v1(self) -> None:
        self.rec_status("V1", timeout=8.0)

    def v2(self) -> None:
        self.cmds("V2-start", ("olstop", "olmode 0", "olboost 5", "olkvf 0.057", "olramp 10", "olspeed 72"))
        self.rec("V2-olstart", self.shell.run("olstart", timeout=20.0, progress=True, tries=1))
        self.if_running = False
        self.wait(20.0, "V/F 72 RPM 斜坡")
        self.rec_status("V2")

    def v3(self) -> None:
        self.rec("V3", self.shell.stop() + self.shell.run("olsnap", timeout=8.0))
        self.if_running = False

    def if_cfg(self, rpm: float) -> str:
        blob = ""
        for cmd in ("olstop", "olmode 1", "oliq 0.6", "olid 0", "olkp 0.2", "olki 30", "olramp 10", f"olspeed {rpm:g}"):
            blob += self.shell.run(cmd)
        self.if_running = False
        return blob

    def v4(self) -> None:
        self.rec("V4-cfg", self.if_cfg(72))
        self.shell.ser.write(b"olstart\rolsnap\r")
        self.rec_status("V4", decode(self.shell.read_until_prompt(12.0)))
        self.if_running = True

    def ensure_if_72(self) -> None:
        if self.if_running:
            return
        self.rec("IF-cfg72", self.if_cfg(72))
        start = self.shell.run("olstart", timeout=20.0, progress=True, tries=1)
        if not start.strip():
            start = "(no uart reply after olstart; assume running)\n"
        self.rec("IF-start72", start)
        self.if_running = True
        self.wait(10.0, "I/F 72 RPM：烧新固件后轴应马上开始转；脚本在读串口，不是卡死")

    def v5(self) -> None:
        self.ensure_if_72()
        self.wait(8.0, "I/F 72 RPM 再保持")
        self.rec_status("V5")

    def v6(self) -> None:
        self.ensure_if_72()
        for label, wait_s in (("V6-15s", 7.0), ("V6-30s", 15.0), ("V6-45s", 15.0), ("V6-60s", 15.0)):
            self.wait(wait_s, label)
            self.rec_status(label)

    def v7(self) -> None:
        self.rec("V7", self.shell.stop() + self.shell.run("olsnap", timeout=8.0))
        self.if_running = False

    def v8(self) -> None:
        if not self.if_running:
            self.rec("V8-cfg72", self.if_cfg(72))
            self.rec("V8-start72", self.shell.run("olstart", timeout=20.0, progress=True, tries=1))
            self.if_running = True
            self.wait(18.0, "I/F 72 RPM 斜坡")
            self.rec_status("V8-72hold")
        self.rec_speed("V8-set150", self.shell.run("olspeed 150", timeout=12.0))
        self.wait(28.0, "升到 150 RPM")
        self.rec_status("V8")

    def v9(self) -> None:
        self.ensure_if_72()
        self.rec_speed("V9-set300", self.shell.run("olspeed 300", timeout=12.0))
        self.wait(36.0, "升到 300 RPM")
        self.rec_status("V9")

    def v10(self) -> None:
        self.ensure_if_72()
        self.rec_speed("V10-set72", self.shell.run("olspeed 72", timeout=12.0))
        self.wait(34.0, "降回 72 RPM")
        self.rec_status("V10")

    def v11(self) -> None:
        self.rec("V11-stop", self.shell.stop())
        self.if_running = False
        self.rec_speed("V11-set", self.shell.run("olspeed -72", timeout=12.0))
        self.rec("V11-start", self.shell.run("olstart", timeout=20.0, progress=True, tries=1))
        self.if_running = True
        self.wait(28.0, "反转 −72 RPM 斜坡")
        self.rec_status("V11")
        self.rec("V11-end", self.shell.stop() + self.shell.run("olsnap", timeout=12.0))
        self.if_running = False

    def run_items(self, items: list[str], *, prep: bool) -> int:
        self.wake()
        skip_first_v0 = False
        if prep:
            self.v0()
            skip_first_v0 = True
        dispatch = {
            "V0": self.v0,
            "V1": self.v1,
            "V2": self.v2,
            "V3": self.v3,
            "V4": self.v4,
            "V5": self.v5,
            "V6": self.v6,
            "V7": self.v7,
            "V8": self.v8,
            "V9": self.v9,
            "V10": self.v10,
            "V11": self.v11,
        }
        try:
            for item in items:
                if item == "V0" and skip_first_v0:
                    skip_first_v0 = False
                    continue
                skip_first_v0 = False
                dispatch[item]()
            return 0
        except Abort as exc:
            self.rec("ABORT", str(exc))
            self.rec("stop", self.shell.stop())
            self.if_running = False
            return 2


def run_full_suite(runner: Runner) -> int:
    """完整 V0～V11，时序与原先脚本一致（V4 紧接 V5，不重复起步）。"""
    runner.wake()
    runner.v0()
    try:
        runner.v1()
        runner.v2()
        runner.v3()
        runner.v4()
        runner.wait(8.0, "I/F 72 RPM 再保持")
        runner.rec_status("V5")
        for label, wait_s in (("V6-15s", 7.0), ("V6-30s", 15.0), ("V6-45s", 15.0), ("V6-60s", 15.0)):
            runner.wait(wait_s, label)
            runner.rec_status(label)
        runner.v7()
        runner.v8()
        runner.v9()
        runner.v10()
        runner.v11()
        return 0
    except Abort as exc:
        runner.rec("ABORT", f"{exc.name} fault")
        runner.rec("stop", runner.shell.stop())
        return 2


def main() -> int:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python Scripts/ol_verify_serial.py
  python Scripts/ol_verify_serial.py --only V9
  python Scripts/ol_verify_serial.py --only V4,V5,V9
  python Scripts/ol_verify_serial.py --list
""".strip(),
    )
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument(
        "--log",
        default="",
        help="默认 Doc/verify/ol_verify_<hash>_<时间>[_项].log；传 - 则不写文件",
    )
    parser.add_argument(
        "--only",
        default="",
        help="只跑这些项，逗号分隔，如 V9 或 4,5,9。未指定则跑完整 V0～V11",
    )
    parser.add_argument("--list", action="store_true", help="列出可单测的项后退出")
    parser.add_argument(
        "--no-prep",
        action="store_true",
        help="单测时不先做 V0（关闭闭环）。完整套件忽略此开关",
    )
    args = parser.parse_args()

    if args.list:
        for item in ITEM_IDS:
            print(f"  {item:4}  {ITEM_HELP[item]}")
        return 0

    try:
        only = parse_only(args.only)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    sections: list[tuple[str, str]] = []
    shell = Shell(args.port, args.baud)
    runner = Runner(shell)
    rc = 1
    try:
        if only:
            rc = runner.run_items(only, prep=not args.no_prep)
        else:
            rc = run_full_suite(runner)
        return rc
    finally:
        try:
            shell.stop()
        except Exception:
            pass
        shell.close()
        sections = runner.sections
        if args.log.strip() != "-":
            tag = "-".join(only) if only else ""
            path = Path(args.log) if args.log.strip() else default_log_path(tag)
            if not path.is_absolute():
                path = REPO_ROOT / path
            path.parent.mkdir(parents=True, exist_ok=True)
            header = (
                f"# ol_verify\n"
                f"# hash: {repo_short_hash()}\n"
                f"# time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"# items: {','.join(only) if only else 'V0-V11'}\n"
                f"# script: Scripts/ol_verify_serial.py\n"
            )
            body = [header]
            for name, text in sections:
                body.append(f"\n######## {name} ########\n{text}")
            path.write_text("".join(body), encoding="utf-8")
            print(f"\nlog written: {path}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
