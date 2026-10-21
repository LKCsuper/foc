#!/usr/bin/env python3
"""3508 分档观察短测：默认800RPM，上调需显式参数，保持接管关闭，结束必停机。"""
import re
import argparse
import math
import time
import shutil
import subprocess
import sys
from pathlib import Path
from datetime import datetime

from ol_verify_serial import REPO_ROOT, Shell
from fault_trace import export_fault_trace


def validate_plan(rpms, maximum, samples):
    if not 1 <= maximum <= 5000 or not 4 <= samples <= 20:
        raise ValueError("上限须为1～5000 RPM，每档采样4～20次")
    if not rpms or any(rpm <= 0 or rpm > maximum for rpm in rpms):
        raise ValueError("目标须为正数且不超过显式上限")
    if maximum > 800 and (rpms[0] > 300 or any(
            current <= previous or current - previous > 600 for previous, current in zip(rpms, rpms[1:]))):
        raise ValueError("高速观察须从不超过300RPM起步，逐档递增且每档不超过600RPM")


def export_stopped_fault(run, path, mode, jlink):
    if mode == 'jlink':
        run('sltrace debug')
        prefix = path.with_name(path.stem + '_fault_jlink')
        subprocess.run([sys.executable, str(REPO_ROOT / 'Scripts' / 'jlink_fault_trace.py'),
                        '--jlink', str(jlink), '--output', str(prefix)], check=True, timeout=180)
    else:
        verified_csv, report = export_fault_trace(run)
        export_path = path.with_name(path.stem + '_fault.csv')
        export_path.write_text(verified_csv, encoding='utf-8')
        print(f'Verified fault trace: {report}; {export_path}', flush=True)


def observation_collapsed(previous, current):
    return previous is not None and previous > 500 and current < previous * .25


def read_observation(run, command):
    for _ in range(2):
        reply = run(command)
        match = re.search(r"rpm obs=([-\d.]+).*?tgt=([-\d.]+).*?iq=([-\d.]+).*?tk=(\d).*?fault=(\d+)", reply)
        if match:
            return reply, match
    raise RuntimeError("连续两次观测回包不完整，停止测试")


def observation_command(brief, coherent, rpms, maximum):
    if brief and (coherent or rpms != [300] or maximum > 800):
        raise ValueError('精简安全查询仅用于单档300RPM对照，不能同时请求同拍角度校验')
    return 'olrpm brief' if brief else 'olrpm'


def voltage_near_limit(reply):
    fault = re.search(r'Fault\s*:\s*(\d+)\s', reply)
    if not fault or int(fault[1]) != 0:
        raise RuntimeError("档末故障或故障字段不完整，停止测试")
    match = re.search(r'\|V\|=([\d.]+) / ([\d.]+) V', reply)
    if not match or float(match[2]) <= 0:
        raise RuntimeError("缺少有效电压限幅快照")
    return float(match[1]) >= .98 * float(match[2])


def validate_voltage_limit(value):
    if value is not None and (not math.isfinite(value) or not 0 < value <= 4.5):
        raise ValueError("本对照脚本电压上限须为有限正数且不超过4.5V")


def validate_voltage_schedule(rpms, limits):
    if limits is None:
        return
    if len(limits) != len(rpms) or not limits:
        raise ValueError("电压档位数量必须与转速档位一致")
    if any(not math.isfinite(value) or not 0 < value <= 10 for value in limits):
        raise ValueError("分档软件电压限幅须为有限正数且不超过10V")
    if limits[0] > 4.5 or any(current < previous or current - previous > 1.5
                              for previous, current in zip(limits, limits[1:])):
        raise ValueError("起步电压限幅不超过4.5V，逐档非递减且每档最多提高1.5V")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--model", nargs=3, type=float, metavar=("R_OHM", "L_MH", "FLUX_MWB"))
    parser.add_argument("--gain", type=float)
    parser.add_argument("--iq", type=float, default=1.0)
    parser.add_argument("--cruise-iq", type=float, help="首档完成后降低运行电流，起转仍用--iq")
    parser.add_argument("--rpms", nargs="+", type=int, default=[300, 600])
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--max-rpm", type=int, default=800, help="经用户确认的本轮上限，最高5000RPM")
    parser.add_argument("--voltage-snapshots", action="store_true", help="每档末读取一次电压与清零式PI统计，接近限幅即结束")
    voltage_group = parser.add_mutually_exclusive_group()
    voltage_group.add_argument("--voltage-limit", type=float, help="经用户确认的临时软件电压上限，最高4.5V，停机后恢复")
    voltage_group.add_argument("--stage-voltage-limits", nargs="+", type=float, help="与转速逐档对应的限幅，起步≤4.5V，最高10V，停机后恢复")
    parser.add_argument("--coherent", action="store_true", help="检查新版同拍相位/角度诊断一致性")
    parser.add_argument('--brief', action='store_true', help='单档300RPM用单行安全查询替代完整诊断，检查次数不变')
    parser.add_argument("--fault-trace", action="store_true", help="武装故障前逐拍RAM记录，停机恢复限幅后导出")
    parser.add_argument('--fault-export', choices=('jlink', 'uart'), default='jlink',
                        help='故障RAM默认用J-Link只读导出，uart保留旧串口方式')
    parser.add_argument('--jlink', default=shutil.which('JLink.exe') or shutil.which('JLinkExe'),
                        help='J-Link Commander路径；默认故障导出需要，启动前检查')
    args = parser.parse_args()
    if args.fault_trace and args.fault_export == 'jlink' and (not args.jlink or not Path(args.jlink).is_file()):
        parser.error('J-Link故障导出需要有效--jlink路径；或显式选择--fault-export uart')
    try:
        validate_plan(args.rpms, args.max_rpm, args.samples)
        poll_command = observation_command(args.brief, args.coherent, args.rpms, args.max_rpm)
        validate_voltage_limit(args.voltage_limit)
        validate_voltage_schedule(args.rpms, args.stage_voltage_limits)
    except ValueError as error:
        parser.error(str(error))
    if args.stage_voltage_limits is not None and not args.voltage_snapshots:
        parser.error("分档电压测试必须开启--voltage-snapshots")
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
    shell = Shell(args.port, 2_000_000)
    with path.open("w", encoding="utf-8") as log:
        def run(command, timeout=3):
            reply = shell.run(command, timeout=timeout, tries=2 if command in ("olrpm", "olrpm brief") else 1)
            block = f"\n$ {command}\n{reply}"
            log.write(block)
            log.flush()
            print(block, flush=True)
            if not shell._got_prompt(reply.encode("utf-8")):
                raise RuntimeError("串口没有返回完整提示符，停止测试")
            if re.search(r"(?im)^\s*(Invalid|Unknown|.* failed|.* requires)\b", reply):
                raise RuntimeError("命令未成功，停止测试")
            return reply

        saved_guard = None
        saved_voltage = None
        fault_trace_armed = False
        def set_voltage(value):
            run(f"olvlimit {value}")
            applied = run("olvlimit")
            if not re.search(r'Vq_limit = ' + re.escape(f'{value:.1f}') + r' V', applied):
                raise RuntimeError("电压限幅回读不一致，不启动或继续升速")

        try:
            run("olstop")
            guard = run("slguard")
            guard_match = re.search(r"OLmax=([\d.]+)RPM phase=([\d.]+)rad blend=([\d.]+)s", guard)
            if not guard_match:
                raise RuntimeError("无法读取原保护配置，不启动")
            saved_guard = tuple(map(float, guard_match.groups()))
            if args.voltage_limit is not None or args.stage_voltage_limits is not None:
                voltage_reply = run("olvlimit")
                voltage_match = re.search(r'Vq_limit = ([\d.]+) V', voltage_reply)
                if not voltage_match:
                    raise RuntimeError("无法读取原电压限幅，不启动")
                saved_voltage = float(voltage_match[1])
                set_voltage(args.stage_voltage_limits[0] if args.stage_voltage_limits else args.voltage_limit)
            for command in ("pos off", "joint off", "olfb sensorless", "olmode 1", "spenable 0",
                            "slenable 1", "sltakeover 0", "olalign 0", "olid 0", f"oliq {args.iq}",
                            "olramp 100", f"slguard {args.max_rpm} 1.4 0.2", f"olspeed {args.rpms[0]}"):
                run(command)
            if args.model:
                run("slmotor " + " ".join(str(value) for value in args.model))
            if args.gain is not None:
                run(f"slobs {args.gain} 2000 30000")
            if args.voltage_snapshots:
                run("olsnap")
            if args.fault_trace:
                if "Fault RAM trace armed:" not in run("sltrace fault"):
                    raise RuntimeError("故障前记录未成功武装，不启动")
                fault_trace_armed = True
            run("olstart")
            previous_observation = None
            for index, rpm in enumerate(args.rpms):
                if index:
                    if args.stage_voltage_limits and args.stage_voltage_limits[index] != args.stage_voltage_limits[index - 1]:
                        set_voltage(args.stage_voltage_limits[index])
                    if index == 1 and args.cruise_iq is not None:
                        run(f"oliq {args.cruise_iq}")
                    run(f"olspeed {rpm}")
                tier_start = time.monotonic()
                ramp_seconds = abs(rpm - (args.rpms[index - 1] if index else 0)) / 100.0
                tracking_failures = 0
                tracking_good = 0
                for _ in range(args.samples):
                    shell.drain(0.7)
                    reply, match = read_observation(run, poll_command)
                    obs, target, iq, takeover, fault = map(float, match.groups())
                    if target != rpm:
                        raise RuntimeError("目标配置发生变化或控制器重启，停止测试")
                    if fault or takeover or abs(iq) > 3.0 or abs(obs) > max(1200, args.max_rpm * 1.1):
                        raise RuntimeError("故障/意外接管/观测或电流越界，停止测试")
                    if observation_collapsed(previous_observation, obs):
                        raise RuntimeError("观测转速突然坍塌，立即结束，不等待爬坡结束")
                    previous_observation = obs
                    # 起转失败或反馈不可信时，不升到下一档。观察RPM不能代替独立轴速。
                    if time.monotonic() - tier_start >= ramp_seconds + 1.0:
                        follows = abs(obs - rpm) <= max(30, 0.2 * rpm)
                        tracking_failures = tracking_failures + 1 if not follows else 0
                        tracking_good = tracking_good + 1 if follows else 0
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
                if args.max_rpm > 800 and tracking_good < 2:
                    raise RuntimeError("本档缺少两次连续稳定跟随记录，不升档")
                if args.voltage_snapshots and voltage_near_limit(run("olsnap")):
                    raise RuntimeError("电压已接近本轮限幅，结束观察，不继续升速")
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
                try:
                    if saved_voltage is not None:
                        run(f"olvlimit {saved_voltage}")
                finally:
                    if saved_guard is not None:
                        run("slguard " + " ".join(str(value) for value in saved_guard))
                if fault_trace_armed:
                    export_stopped_fault(run, path, args.fault_export, args.jlink)
            finally:
                shell.close()
                print(f"Log: {path}", flush=True)


if __name__ == "__main__":
    main()
