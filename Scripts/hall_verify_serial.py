#!/usr/bin/env python3
"""BM75 Hall closed-loop verification over UART.

Profiles:

    up      150→300→500→800→1200→1500→1800→2000→2500→3000（可用 --max-rpm 截断）
    down    150→120→100→72
    start72 停机后直接起 72 RPM
    brake   150→0 主动制动（spbrake 1）
    reverse 反转 −150→−72
    all     上述全部：正转降速、停机直起 72、制动、反转、正转升速

默认遇到转速误差或边沿超时立即停。``--keep-going`` 或 ``all`` 会在误差后继续
后续档位；反转和故障仍立即 ``olstop``。
"""
from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from ol_verify_serial import Abort, REPO_ROOT, Shell, decode, has_fault, repo_short_hash


UP_STAGES = (
    (150, 10.0, 0.25),
    (300, 7.0, 0.15),
    (500, 8.0, 0.15),
    (800, 11.0, 0.15),
    (1200, 12.0, 0.12),
    (1500, 10.0, 0.05),
    (1800, 12.0, 0.02),
    (2000, 14.0, 0.02),
    (2500, 14.0, 0.10),
    (3000, 14.0, 0.10),
)
MAX_RPM_CHOICES = tuple(stage[0] for stage in UP_STAGES)
DOWN_STAGES = (
    (150, 10.0, 0.25),
    (120, 8.0, 0.30),
    (100, 8.0, 0.35),
    (72, 10.0, 0.40),
)
PROFILES = ("up", "down", "start72", "brake", "reverse", "all")
SNAP_END = "============================="
REVERSE_RPM = 20.0
BRAKE_RPM_LIMIT = 15.0
BRAKE_IQ_LIMIT = 0.05
MAX_DQ_CURRENT = 2.5  # 仿 VESC：停机看 |Id,Iq| 幅值，不是单拍 Id 跟踪或三相和。
MAX_ID_ERROR = 1.0  # Id 跟踪观察门槛；偶发超限只记告警。
PHASE_SUM_WARN = 0.5  # 三相和观察门槛；VESC 把 unbalance 当遥测，不拿来跳闸。
MIN_VBUS = 22.0  # 24 V 台架：快照母线跌至此值以下时停止升速。


class UnsafeSample(Abort):
    """电流或母线异常，任何测试模式都必须立即停机。"""


@dataclass(frozen=True)
class Step:
    kind: str
    rpm: int = 0
    wait_s: float = 0.0
    tolerance: float = 0.0
    name: str = ""
    brake: int = 0


def field(pattern: str, text: str) -> float:
    match = re.search(pattern, text)
    if match is None:
        raise ValueError(pattern)
    return float(match.group(1))


def parse_snap(text: str) -> dict[str, float | int | bool]:
    if "Open-Loop Snap" not in text or SNAP_END not in text:
        raise ValueError("incomplete olsnap")
    if has_fault(text) or "Fault      : 0 NONE" not in text:
        raise ValueError("motor fault")

    hall = re.search(
        r"Hall RPM\s*:\s*raw=([-+0-9.]+)\s+filtered=([-+0-9.]+)\s+"
        r"(?:fast=([-+0-9.]+)\s+)?age=([-+0-9.]+)ms\s+state=(-?\d+)",
        text,
    )
    measured = re.search(r"Meas RPM\s*:\s*([-+0-9.]+)\s+\((valid|invalid)\)", text)
    if hall is None or measured is None:
        raise ValueError("new Hall snapshot fields missing; flash latest firmware")

    id_line = re.search(
        r"Id, Iq\s*:\s*([-+0-9.]+)\s+([-+0-9.]+)\s+A(?:\s+\(target\s+([-+0-9.]+)\s+([-+0-9.]+)\))?",
        text,
    )
    if id_line is None:
        raise ValueError("Id/Iq snapshot fields missing")
    abc_line = re.search(r"Ia,Ib,Ic\s*:\s*([-+0-9.]+)\s+([-+0-9.]+)\s+([-+0-9.]+)", text)
    if abc_line is None:
        raise ValueError("abc current snapshot fields missing")

    vd_line = re.search(r"Vd, Vq int\s*:\s*([-+0-9.]+)\s+([-+0-9.]+)", text)
    speed_iq = re.search(r"Speed Iq\s*:\s*([-+0-9.]+)", text)
    return {
        "target": field(r"Target RPM\s*:\s*([-+0-9.]+)", text),
        "ramp": field(r"Ramp RPM\s*:\s*([-+0-9.]+)", text),
        "measured": float(measured.group(1)),
        "valid": measured.group(2) == "valid",
        "raw": float(hall.group(1)),
        "filtered": float(hall.group(2)),
        "fast": float(hall.group(3)) if hall.group(3) is not None else float(hall.group(2)),
        "age_ms": float(hall.group(4)),
        "state": int(hall.group(5)),
        "id": float(id_line.group(1)),
        "iq": float(id_line.group(2)),
        "id_target": float(id_line.group(3)) if id_line.group(3) is not None else float("nan"),
        "iq_target": float(id_line.group(4)) if id_line.group(4) is not None else float("nan"),
        "abc_sum": sum(float(abc_line.group(i)) for i in range(1, 4)),
        "speed_iq": float(speed_iq.group(1)) if speed_iq is not None else float("nan"),
        "v_bus": field(r"V_bus\s*:\s*([-+0-9.]+)", text),
        "vd_int": float(vd_line.group(1)) if vd_line is not None else float("nan"),
        "vq_int": float(vd_line.group(2)) if vd_line is not None else float("nan"),
    }


def parse_current_diag(text: str) -> dict[int, tuple[int, int, float, int, float]]:
    if "Hall current diag" not in text or "total samples:" not in text:
        raise ValueError("incomplete olcurdiag; flash latest firmware")
    rows = re.findall(r"(?m)^([1-6])\s+(\d+)\s+(\d+)\s+([0-9.]+)\s+(\d+)\s+([0-9.]+)\s*$", text)
    if len(rows) != 6:
        raise ValueError("six Hall current diagnostic rows required")
    return {
        int(state): (int(samples), int(sum_over), float(sum_max), int(id_over), float(id_max))
        for state, samples, sum_over, sum_max, id_over, id_max in rows
    }


def format_snap(snap: dict[str, float | int | bool]) -> str:
    return (
        f"parsed: target={snap['target']:.0f} ramp={snap['ramp']:.0f} "
        f"meas={snap['measured']:.1f} raw={snap['raw']:.1f} "
        f"filtered={snap['filtered']:.1f} fast={snap['fast']:.1f} "
        f"age={snap['age_ms']:.1f}ms state={snap['state']} "
        f"Id/Iq={snap['id']:.3f}/{snap['iq']:.3f} "
        f"IabcSum={snap['abc_sum']:.3f} "
        f"target={snap['id_target']:.3f}/{snap['iq_target']:.3f} "
        f"SpeedIq={snap['speed_iq']:.3f} Vbus={snap['v_bus']:.1f}V "
        f"Vd/Vq={snap['vd_int']:.3f}/{snap['vq_int']:.3f}\n"
    )


def speed_chain(
    stages: tuple[tuple[int, float, float], ...],
    direction: int,
    max_rpm: int | None,
    prefix: str,
    brake: int,
) -> list[Step]:
    steps: list[Step] = []
    for rpm, wait_s, tolerance in stages:
        if max_rpm is not None and rpm > max_rpm:
            break
        target = rpm * direction
        kind = "start" if not steps else "set"
        steps.append(Step(kind, target, wait_s, tolerance, f"{prefix}{target}", brake))
    return steps


def build_steps(profile: str, direction: int, max_rpm: int) -> list[Step]:
    if profile == "up":
        return speed_chain(UP_STAGES, direction, max_rpm, "up", 0)
    if profile == "down":
        return speed_chain(DOWN_STAGES, direction, None, "down", 0)
    if profile == "start72":
        return [Step("start", 72 * direction, 12.0, 0.40, f"start{72 * direction}", 0)]
    if profile == "brake":
        start = 150 * direction
        return [
            Step("start", start, 10.0, 0.25, f"brake{start}", 1),
            Step("brake", 0, 5.0, 0.0, "brake0", 1),
        ]
    if profile == "reverse":
        return [
            Step("start", -150, 10.0, 0.25, "rev-150", 0),
            Step("set", -72, 10.0, 0.40, "rev-72", 0),
        ]
    if profile == "all":
        steps: list[Step] = []
        steps += speed_chain(DOWN_STAGES, 1, None, "down", 0)
        steps.append(Step("stop", name="stop-before-start72"))
        steps.append(Step("start", 72, 12.0, 0.40, "start72", 0))
        steps.append(Step("stop", name="stop-before-brake"))
        steps.append(Step("start", 150, 10.0, 0.25, "brake150", 1))
        steps.append(Step("brake", 0, 5.0, 0.0, "brake0", 1))
        steps.append(Step("stop", name="stop-before-rev"))
        steps.append(Step("start", -150, 10.0, 0.25, "rev-150", 0))
        steps.append(Step("set", -72, 10.0, 0.40, "rev-72", 0))
        steps.append(Step("stop", name="stop-before-up"))
        steps += speed_chain(UP_STAGES, 1, max_rpm, "up", 0)
        return steps
    raise ValueError(profile)


class HallRunner:
    def __init__(self, shell: Shell, samples: int, keep_going: bool) -> None:
        self.shell = shell
        self.samples = samples
        self.keep_going = keep_going
        self.sections: list[tuple[str, str]] = []
        self.results: list[tuple[str, dict[str, float | int | bool]]] = []
        self.failed = False

    def rec(self, name: str, text: str) -> None:
        self.sections.append((name, text))
        print(f"\n######## {name} ########\n{text}", flush=True)

    def commands(self, name: str, commands: tuple[str, ...]) -> None:
        output = ""
        for command in commands:
            output += self.shell.run(command, timeout=20.0, progress=command == "olstart")
        self.rec(name, output)

    def pause(self, seconds: float, reason: str) -> None:
        self.rec(f"wait {seconds:.0f}s", reason + "\n")
        self.shell.pause(seconds)

    def start_motor(self, rpm: int, brake: int, name: str) -> None:
        self.commands(
            name,
            (
                "olstop", "olfault clear", "olalign 0", "olmode 1", "olfb hall", "slenable 0",
                "spkp 0.006", "spki 0.003", "spiq 0.8", f"spbrake {brake}",
                "oliq 0.6", "olid 0", "olkp 0.12", "olki 18",
                f"olspeed {rpm}", "spenable 1", "olstart",
            ),
        )

    def handle_soft_fail(self, exc: Abort, tag: str) -> None:
        message = str(exc)
        if isinstance(exc, UnsafeSample) or not self.keep_going or "reversed" in message or "motor fault" in message:
            raise exc
        self.failed = True
        self.rec(f"{tag}-FAIL", message + "\n")

    def take_snap(self, name: str) -> dict[str, float | int | bool]:
        text = self.shell.run("olsnap", timeout=45.0, progress=True)
        self.rec(name, text)
        try:
            snap = parse_snap(text)
        except ValueError as exc:
            raise Abort(f"{name}: {exc}") from exc
        self.rec(f"{name}-parsed", format_snap(snap))
        return snap

    def check_speed(self, name: str, snap: dict[str, float | int | bool], rpm: int, tolerance: float) -> None:
        measured = float(snap["filtered"])
        current = math.hypot(float(snap["id"]), float(snap["iq"]))
        if not math.isfinite(current) or current > MAX_DQ_CURRENT:
            raise UnsafeSample(f"{name}: measured DQ current {current:.3f} A exceeds {MAX_DQ_CURRENT:.1f} A")
        abc_sum = abs(float(snap["abc_sum"]))
        if abc_sum > PHASE_SUM_WARN:
            self.rec(f"{name}-CURRENT-WARN", f"|Ia+Ib+Ic|={abc_sum:.3f} A exceeds {PHASE_SUM_WARN:.1f} A\n")
        id_error = abs(float(snap["id"]) - float(snap["id_target"]))
        if id_error > MAX_ID_ERROR:
            self.rec(
                f"{name}-CURRENT-WARN",
                f"|Id error|={id_error:.3f} A exceeds {MAX_ID_ERROR:.1f} A (telemetry, not trip)\n",
            )
        if not math.isfinite(float(snap["v_bus"])) or float(snap["v_bus"]) < MIN_VBUS:
            raise UnsafeSample(f"{name}: Vbus {float(snap['v_bus']):.1f} V below {MIN_VBUS:.1f} V")
        if not bool(snap["valid"]):
            raise Abort(f"{name}: Hall speed invalid")
        if int(snap["state"]) not in range(1, 7):
            raise Abort(f"{name}: invalid Hall state {snap['state']}")
        if abs(measured) >= REVERSE_RPM and measured * rpm < 0.0:
            raise Abort(f"{name}: reversed filtered RPM {measured:.1f}")
        if float(snap["age_ms"]) >= 200.0:
            raise Abort(f"{name}: Hall edge stale {snap['age_ms']} ms")
        if abs(measured - rpm) > abs(rpm) * tolerance:
            raise Abort(f"{name}: filtered RPM {measured:.1f} outside ±{tolerance * 100:.0f}%")
        self.results.append((name, snap))

    def check_brake(self, name: str, snap: dict[str, float | int | bool]) -> None:
        if not bool(snap["valid"]):
            raise Abort(f"{name}: Hall speed invalid")
        if abs(float(snap["target"])) > 1.0 or abs(float(snap["ramp"])) > 1.0:
            raise Abort(f"{name}: target/ramp not zero ({snap['target']:.1f}/{snap['ramp']:.1f})")
        if abs(float(snap["measured"])) > BRAKE_RPM_LIMIT or abs(float(snap["filtered"])) > BRAKE_RPM_LIMIT:
            raise Abort(f"{name}: not stopped meas={snap['measured']:.1f} filtered={snap['filtered']:.1f}")
        if abs(float(snap["speed_iq"])) > BRAKE_IQ_LIMIT:
            raise Abort(f"{name}: Speed Iq {snap['speed_iq']:.3f} A not released")
        if abs(float(snap["id_target"])) > BRAKE_IQ_LIMIT or abs(float(snap["iq_target"])) > BRAKE_IQ_LIMIT:
            raise Abort(
                f"{name}: Id/Iq target {snap['id_target']:.3f}/{snap['iq_target']:.3f} A not released"
            )
        self.results.append((name, snap))

    def sample_stage(self, step: Step) -> None:
        for index in range(1, self.samples + 1):
            name = f"{step.name}-{index}"
            snap = self.take_snap(name)
            if step.kind == "brake":
                self.check_brake(name, snap)
            else:
                self.check_speed(name, snap, step.rpm, step.tolerance)
            if index != self.samples:
                self.shell.pause(1.0)
        if step.kind != "brake":
            response = self.shell.run("olcurdiag", timeout=15.0)
            self.rec(f"{step.name}-CURRENT-DIAG", response)
            try:
                sectors = parse_current_diag(response)
            except ValueError as exc:
                raise UnsafeSample(f"{step.name}: {exc}") from exc
            sample_count = sum(row[0] for row in sectors.values())
            id_over_count = sum(row[3] for row in sectors.values())
            if sample_count == 0:
                raise UnsafeSample(f"{step.name}: no steady-state current diagnostic samples")
            max_phase_sum = max(row[2] for row in sectors.values())
            if max_phase_sum > PHASE_SUM_WARN:
                self.rec(
                    f"{step.name}-CURRENT-WARN",
                    f"aggregated |Ia+Ib+Ic| peak {max_phase_sum:.3f} A exceeds {PHASE_SUM_WARN:.1f} A\n",
                )
            if id_over_count != 0:
                self.rec(
                    f"{step.name}-CURRENT-WARN",
                    f"{id_over_count}/{sample_count} samples |Id error| >{MAX_ID_ERROR:.1f} A "
                    "(telemetry, not trip)\n",
                )

    def run(self, steps: list[Step]) -> int:
        result = 2
        try:
            for step in steps:
                if step.kind == "stop":
                    self.rec(step.name, self.shell.stop())
                    self.shell.pause(0.5)
                    continue
                if step.kind == "start":
                    self.start_motor(step.rpm, step.brake, f"{step.name}-config")
                    reason = f"Hall 闭环从停机起转到 {step.rpm} RPM"
                elif step.kind == "set":
                    self.rec(f"{step.name}-set", self.shell.run(f"olspeed {step.rpm}", timeout=20.0))
                    reason = f"Hall 闭环稳定到 {step.rpm} RPM"
                elif step.kind == "brake":
                    self.rec(f"{step.name}-set", self.shell.run("olspeed 0", timeout=20.0))
                    reason = "Hall 主动制动到 0 RPM"
                else:
                    raise Abort(f"unknown step {step.kind}")
                self.pause(step.wait_s, reason)
                try:
                    self.sample_stage(step)
                except Abort as exc:
                    self.handle_soft_fail(exc, step.name)
            if self.failed:
                self.rec("FAIL", "Hall verification finished with stage errors\n")
            else:
                result = 0
        except Abort as exc:
            self.rec("ABORT", str(exc) + "\n")
        finally:
            try:
                self.rec("STOP", self.shell.stop())
            except Exception as exc:
                result = 2
                self.rec("STOP-ERROR", f"{exc}\n")
            try:
                self.rec("HALL-CURRENT-DIAG", self.shell.run("olcurdiag", timeout=15.0))
            except Exception as exc:
                result = 2
                self.rec("HALL-CURRENT-DIAG-ERROR", f"{exc}\n")
            try:
                stopped = self.shell.run("olsnap", timeout=15.0)
                self.rec("FINAL-STOP-SNAP", stopped)
                if "Enable     : OFF" not in stopped or "Target RPM : 0" not in stopped or \
                        "Fault      : 0 NONE (CLEAR)" not in stopped:
                    result = 2
                    self.rec("FINAL-FAIL", "motor not safely stopped or fault latched after stop\n")
                    self.rec("FINAL-FAULT", self.shell.run("olfault", timeout=15.0))
            except Exception as exc:
                result = 2
                self.rec("FINAL-STOP-ERROR", f"{exc}\n")
        if result == 0:
            self.rec("PASS", "Hall verification passed\n")
        return result


def default_log_path(tag: str) -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return REPO_ROOT / "Doc" / "verify" / f"hall_verify_{workspace_id()}_{stamp}_{tag}.log"


def workspace_id() -> str:
    revision = repo_short_hash()
    try:
        dirty = subprocess.run(
            ["git", "diff", "--quiet"], cwd=REPO_ROOT, check=False,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        ).returncode != 0
    except OSError:
        dirty = False
    return revision + ("-dirty" if dirty else "")


def profile_tag(profile: str, max_rpm: int, direction: str) -> str:
    if profile == "up":
        return f"{max_rpm}RPM"
    if profile == "down":
        return "down72RPM"
    if profile == "all":
        return "allHall"
    if profile in ("start72", "brake") and direction == "ccw":
        return f"{profile}-ccw"
    return profile


def main() -> int:
    parser = argparse.ArgumentParser(description="Hall feedback stability verification")
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--profile", choices=PROFILES, default="up",
                        help="up/down/start72/brake/reverse/all")
    parser.add_argument("--max-rpm", type=int, choices=MAX_RPM_CHOICES, default=800,
                        help="仅 up/all 的升速段有效，截到该档；电机额定 3000 RPM")
    parser.add_argument("--direction", choices=("cw", "ccw"), default="cw",
                        help="up/down/start72/brake 的转向；all 与 reverse 固定含正反转")
    parser.add_argument("--samples", type=int, default=3, help="每档快照数，默认 3")
    parser.add_argument("--keep-going", action="store_true",
                        help="转速误差或边沿超时后继续下一档；反转和故障仍立即停机")
    parser.add_argument("--log", default="", help="日志路径；传 - 不写文件")
    args = parser.parse_args()
    if args.samples < 1 or args.samples > 10:
        parser.error("--samples must be 1..10")

    direction = 1 if args.direction == "cw" else -1
    steps = build_steps(args.profile, direction, args.max_rpm)
    keep_going = True if args.profile == "all" else args.keep_going
    log_tag = profile_tag(args.profile, args.max_rpm, args.direction)
    shell = Shell(args.port, args.baud)
    runner = HallRunner(shell, args.samples, keep_going)
    rc = 1
    try:
        wake = decode(shell.read_until_prompt(0.5))
        if not wake.strip():
            wake = shell.kick()
        runner.rec("wake", wake)
        rc = runner.run(steps)
        return rc
    finally:
        try:
            shell.stop()
        except Exception:
            pass
        shell.close()
        if args.log.strip() != "-":
            path = Path(args.log) if args.log.strip() else default_log_path(log_tag)
            if not path.is_absolute():
                path = REPO_ROOT / path
            path.parent.mkdir(parents=True, exist_ok=True)
            header = (
                "# hall_verify\n"
                f"# workspace: {workspace_id()}\n"
                f"# time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"# profile: {args.profile}\n"
                f"# max_rpm: {args.max_rpm}\n"
                f"# direction: {args.direction}\n"
                f"# samples: {args.samples}\n"
                f"# keep_going: {int(keep_going)}\n"
                "# script: Scripts/hall_verify_serial.py\n"
            )
            body = [header]
            for name, text in runner.sections:
                body.append(f"\n######## {name} ########\n{text}")
            path.write_text("".join(body), encoding="utf-8")
            print(f"\nlog written: {path}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
