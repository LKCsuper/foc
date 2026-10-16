#!/usr/bin/env python3
"""读取3508参数检测原文；等待后台结束，超时中止，最终停机。"""
import argparse
import math
import time
from datetime import datetime
from ol_verify_serial import REPO_ROOT, Shell, decode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("command")
    args = parser.parse_args()
    fields = args.command.split()
    if len(fields) < 2 or fields[:2] not in (["detect", "rl"], ["detect", "motor"]):
        parser.error("仅支持 detect rl 或 detect motor")
    if len(fields) > (3 if fields[1] == "rl" else 5):
        parser.error("检测参数过多")
    try:
        if any(not math.isfinite(float(value)) for value in fields[2:]):
            raise ValueError
    except ValueError:
        parser.error("检测参数必须为有限数值")
    args.command = " ".join(fields)
    path = REPO_ROOT / "Doc/verify" / ("sensorless_detect_3508_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
    path.parent.mkdir(parents=True, exist_ok=True)
    shell = Shell(args.port, 2_000_000)
    with path.open("w", encoding="utf-8") as log:
        def emit(text):
            print(text, flush=True)
            log.write(text + "\n")
            log.flush()

        def run(command):
            reply = shell.run(command, timeout=4, tries=1)
            emit("$ " + command + "\n" + reply)
            if not shell._got_prompt(reply.encode("utf-8")):
                raise RuntimeError("串口未返回完整提示符")
            return reply

        try:
            for command in ("olstop", "pos off", "joint off", "spenable 0", "sltakeover 0"):
                run(command)
            reply = run(args.command)
            if "background task is running" not in reply:
                raise RuntimeError("后台检测未启动")
            captured = reply
            deadline = time.monotonic() + 35
            while time.monotonic() < deadline:
                part = decode(shell.ser.read(4096))
                if part:
                    emit(part)
                    captured += part
                if "Detect motor finished:" in captured:
                    break
            else:
                run("detect abort")
                raise RuntimeError("参数检测超时")
            run("detect show")
            if "Detect motor finished: SUCCESS" not in captured:
                raise RuntimeError("参数检测未通过质量检查；结果未应用")
        finally:
            try:
                run("olstop")
                run("olfault")
                run("olstatus")
            finally:
                shell.close()
                print("Log: " + str(path), flush=True)


if __name__ == "__main__":
    main()
