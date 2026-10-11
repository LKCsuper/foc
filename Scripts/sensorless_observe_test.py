#!/usr/bin/env python3
"""3508 无感观测短测：目标限制在800 RPM内，保持接管关闭，结束必停机。"""
import re
import argparse
import math
import time
from datetime import datetime

from ol_verify_serial import REPO_ROOT, Shell


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", nargs=3, type=float, metavar=("R_OHM", "L_MH", "FLUX_MWB"))
    parser.add_argument("--gain", type=float)
    parser.add_argument("--iq", type=float, default=1.0)
    parser.add_argument("--cruise-iq", type=float, help="首档完成后降低运行电流，起转仍用--iq")
    parser.add_argument("--rpms", nargs="+", type=int, default=[300, 600])
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--coherent", action="store_true", help="检查新版同拍相位/角度诊断一致性")
    args = parser.parse_args()
    if any(rpm <= 0 or rpm > 800 for rpm in args.rpms) or not 4 <= args.samples <= 20:
        parser.error("目标须为1～800 RPM，每档采样4～20次")
    if not math.isfinite(args.iq) or not 0.3 <= args.iq <= 1.0:
        parser.error("本观测脚本电流限制为0.3～1.0 A")
    if args.cruise_iq is not None and (
            not math.isfinite(args.cruise_iq) or not 0.3 <= args.cruise_iq <= args.iq):
        parser.error("运行电流须为0.3A至起转电流之间的有限值")
    if args.model and any(not math.isfinite(value) or value <= 0 for value in args.model):
        parser.error("电机模型参数必须为有限正数")
    if args.gain is not None and (not math.isfinite(args.gain) or args.gain < 0):
        parser.error("观测器增益必须为有限非负数")
    path = REPO_ROOT / "Doc" / "verify" / ("sensorless_3508_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".log")
    path.parent.mkdir(parents=True, exist_ok=True)
    shell = Shell("COM5", 2_000_000)
    with path.open("w", encoding="utf-8") as log:
        def run(command, timeout=3):
            reply = shell.run(command, timeout=timeout, tries=2 if command == "olrpm" else 1)
            block = f"\n$ {command}\n{reply}"
            log.write(block)
            log.flush()
            print(block, flush=True)
            if not shell._got_prompt(reply.encode("utf-8")):
                raise RuntimeError("串口没有返回完整提示符，停止测试")
            if re.search(r"(?im)^\s*(Invalid|Unknown|.* failed|.* requires)\b", reply):
                raise RuntimeError("命令未成功，停止测试")
            return reply

        try:
            run("olstop")
            for command in ("pos off", "joint off", "olfb sensorless", "olmode 1", "spenable 0",
                            "slenable 1", "sltakeover 0", "olalign 0", "olid 0", f"oliq {args.iq}",
                            "olramp 100", "slguard 800 1.4 0.2", f"olspeed {args.rpms[0]}"):
                run(command)
            if args.model:
                run("slmotor " + " ".join(str(value) for value in args.model))
            if args.gain is not None:
                run(f"slobs {args.gain} 2000 30000")
            run("olstart")
            for index, rpm in enumerate(args.rpms):
                if index:
                    if index == 1 and args.cruise_iq is not None:
                        run(f"oliq {args.cruise_iq}")
                    run(f"olspeed {rpm}")
                tier_start = time.monotonic()
                ramp_seconds = abs(rpm - (args.rpms[index - 1] if index else 0)) / 100.0
                tracking_failures = 0
                for _ in range(args.samples):
                    shell.drain(0.7)
                    reply = run("olrpm")
                    match = re.search(r"rpm obs=([-\d.]+).*?tgt=([-\d.]+).*?iq=([-\d.]+).*?tk=(\d).*?fault=(\d+)", reply)
                    if not match:
                        raise RuntimeError("观测回包不完整，停止测试")
                    obs, target, iq, takeover, fault = map(float, match.groups())
                    if target != rpm:
                        raise RuntimeError("目标配置发生变化或控制器重启，停止测试")
                    if fault or takeover or abs(iq) > 3.0 or abs(obs) > 1200:
                        raise RuntimeError("故障/意外接管/观测或电流越界，停止测试")
                    # 起转失败或反馈不可信时，不升到下一档。观察RPM不能代替独立轴速。
                    if time.monotonic() - tier_start >= ramp_seconds + 1.0:
                        tracking_failures = tracking_failures + 1 if obs < 0.5 * rpm else 0
                        if tracking_failures >= 2:
                            raise RuntimeError("观测未跟上/可能未起转，停止测试，不继续升速")
                    if args.coherent:
                        checks = re.search(r"chk .*?P=(\d).*?ph=([-\d.]+)", reply)
                        angles = re.search(r"ang if=([-\d.]+) raw=([-\d.]+) pll=([-\d.]+)", reply)
                        if not checks or not angles:
                            raise RuntimeError("缺少新版同拍诊断，停止测试")
                        phase = float(checks[2])
                        delta = math.remainder(float(angles[3]) - float(angles[1]), 2 * math.pi)
                        if abs(math.remainder(delta - phase, 2 * math.pi)) > 0.004:
                            raise RuntimeError("同拍角差与相位读数不一致")
                        lock_match = re.search(r"lockph=([-\d.]+)", reply)
                        lock_phase = float(lock_match[1]) if lock_match else phase
                        if abs(abs(lock_phase) - 1.4) > 0.002 and bool(int(checks[1])) != (abs(lock_phase) <= 1.4):
                            raise RuntimeError("同拍相位条件与误差读数不一致")
        finally:
            try:
                # 单条命令超时不能跳过后续停机尝试；仅停机命令允许重发。
                stopped = False
                for command in ("olstop", "olspeed 0", "olfault", "slstats", "olstatus"):
                    try:
                        reply = shell.run(command, timeout=3, tries=3 if command in ("olstop", "olspeed 0") else 1)
                        block = f"\n$ {command}\n{reply}"
                        log.write(block)
                        log.flush()
                        print(block, flush=True)
                        if command == "olstatus" and shell._got_prompt(reply.encode("utf-8")):
                            stopped = bool(re.search(r"Enable\s*:\s*OFF", reply) and
                                           re.search(r"Target RPM\s*:\s*0(?:\s|$)", reply))
                    except Exception as error:
                        message = f"Cleanup {command}: {error}\n"
                        log.write(message)
                        log.flush()
                        print(message, flush=True)
                if not stopped:
                    raise RuntimeError("未确认停机，请关闭功率电源；串口停机不能保证在失联时生效")
            finally:
                shell.close()
                print(f"Log: {path}", flush=True)


if __name__ == "__main__":
    main()
