/* 由 sync-catalog.cjs 从当前固件源码生成。 */
export default {
  "commands": [
    {
      "name": "studio",
      "title": "网页与周期控制",
      "description": "web: on|off|status|control [token]|start|release|params|set <id> <value>",
      "category": "系统与存储",
      "file": "App/cmd/cmd_studio.c",
      "line": 246,
      "subcommands": [
        "status",
        "on",
        "release",
        "off",
        "control",
        "start",
        "params",
        "set"
      ],
      "usage": [
        "studio on|off|status|control|release|params|set <field_id> <value>"
      ],
      "gate": "#endif"
    },
    {
      "name": "detect",
      "title": "参数检测与校准",
      "description": "detect offset/hall/phase or motor R/L",
      "category": "检测与校准",
      "file": "App/cmd/cmd_detect.c",
      "line": 165,
      "subcommands": [
        "1",
        "offset",
        "2",
        "hall",
        "3",
        "phase",
        "fluxscan",
        "motor",
        "rl",
        "show",
        "abort"
      ],
      "usage": [
        "Invalid offset arguments: detect offset [ram]",
        "  detect | detect offset [ram] | detect 1",
        "  detect offset ram set ADC0 ADC1 ADC2 (stopped, +/-1 count, RAM only)",
        "  detect hall | detect 2",
        "  detect phase | detect 3   — UVW order vs hall steps",
        "  detect rl [test_current_A]",
        "  detect motor [rl_current_A] [electrical_rpm] [flux_current_A]",
        "  detect fluxscan [rl_current_A] [max_eRPM 900..3000] [flux_current_A]",
        "  detect show",
        "  detect abort"
      ],
      "gate": "#endif"
    },
    {
      "name": "pitune",
      "title": "PI 自整定预览",
      "description": "preview PI: <bandwidth_Hz> <total_delay_us> [margin_deg]",
      "category": "检测与校准",
      "file": "App/cmd/cmd_autotune.c",
      "line": 21,
      "subcommands": [],
      "usage": [
        "Usage: pitune <bandwidth_Hz> <total_delay_us> [min_margin_deg=60]",
        "pitune rejected: stop motor, finish detection and clear faults first.",
        "pitune rejected: no successful R/L result; run detect rl on this motor first.",
        "pitune rejected: HAL sample frequency unavailable.",
        "pitune rejected: %s"
      ],
      "gate": "#if MOTOR_CFG_CMD_IDENTIFY"
    },
    {
      "name": "bldc",
      "title": "六步换相控制",
      "description": "BLDC: start/open | stop | exit | status",
      "category": "BLDC 六步",
      "file": "App/cmd/cmd_bldc.c",
      "line": 290,
      "subcommands": [
        "status",
        "stop",
        "exit",
        "sample",
        "tune",
        "start",
        "open"
      ],
      "usage": [
        "Usage: bldc sample <up|down>",
        "Usage: bldc tune <integral_mV_s> <minimum_period_ms>",
        "Usage: bldc start <duty> [cw|ccw] | open <duty> <sector_ms> [cw|ccw]",
        "       bldc tune <integral_mV_s> <minimum_period_ms>",
        "       bldc sample <up|down>",
        "       bldc stop | exit | status",
        "Usage: bldc open <duty> <sector_ms> [cw|ccw]"
      ],
      "gate": "#if MOTOR_CFG_CMD_BLDC"
    },
    {
      "name": "saveparam",
      "title": "保存参数至 Flash",
      "description": "save motor params to Flash",
      "category": "系统与存储",
      "file": "App/cmd/cmd_env.c",
      "line": 31,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "olspeed",
      "title": "目标转速",
      "description": "openloop V/F speed set <rpm>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 422,
      "subcommands": [],
      "usage": [
        "Usage: olspeed <rpm>  — set target speed, negative for reverse"
      ],
      "gate": "#endif"
    },
    {
      "name": "olstatus",
      "title": "控制状态",
      "description": "show openloop current and fault status",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 734,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "olrpm",
      "title": "转速与电流诊断",
      "description": "rpm/current diagnostics; brief for safety line only",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 784,
      "subcommands": [
        "brief"
      ],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "slstats",
      "title": "无感稳态统计",
      "description": "sensorless steady-window counters retained after stop",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 38,
      "subcommands": [],
      "usage": [
        "slstats n=%lu time=%.3fs fail F=%lu S=%lu D=%lu P=%lu"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "sltest",
      "title": "无感分档测试",
      "description": "[--3508] [--trace=off|lite|flight|entry|entryfast] [hold_s [rpm...]], stopped only",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 255,
      "subcommands": [],
      "usage": [
        "Invalid sltest trace: off|entry|entryfast|lite",
        "Usage: sltest [--3508] [--trace=off|entry|entryfast|lite|flight] [hold_s [rpm1 ... rpm6]]",
        "sltest hold: %lu..%lu seconds",
        "sltest RPM: %.0f..%.0f, positive only",
        "sltest requires stopped FOC, no detection, clear fault",
        "sltest: requires %.0f pole-pairs, %.1f..%.1fV, transition<=%.0f, fallback<%.0f",
        "sltest voltage limit=%.1f V; high speeds require CLOSED",
        "sltest limits: Iq=%.2f/%.2fA above %.0fRPM; tracking %.1f%% or %.0fRPM, %u samples @ %lums",
        "sltest %u/%u target=%.0f RPM",
        "sltest output: target=%.0f rpm=%.1f iq=%.4f/%.4f speedIq=%.4f limit=%.3f "
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "sltrace",
      "title": "运行与故障轨迹",
      "description": "RAM trace fault|debug|compact|lite|flight|arm|entry|entryfast|show (stopped only)",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_motor_diag.c",
      "line": 789,
      "subcommands": [
        "tracking",
        "tracking-clear",
        "tracking-arm",
        "tracking-off",
        "debug",
        "fault",
        "arm",
        "compact",
        "entry",
        "entryfast",
        "flight",
        "lite"
      ],
      "usage": [
        "sltrace requires stopped motor"
      ],
      "gate": "#if MOTOR_CFG_CMD_DIAG"
    },
    {
      "name": "slhandoff",
      "title": "交接与 PWM 时序诊断",
      "description": "arm|phase|phaselong|step|pair|half|pwm|pwmlong|pwmoff|phaseedge|phase3000|phaseoff|show; stopped only",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_motor_diag.c",
      "line": 608,
      "subcommands": [
        "pwm",
        "pwmlong",
        "phaseedge",
        "phase3000",
        "phaseoff",
        "pwmoff",
        "pair",
        "half",
        "arm",
        "phase",
        "phaselong",
        "step"
      ],
      "usage": [],
      "gate": "#if MOTOR_CFG_CMD_DIAG"
    },
    {
      "name": "slcadence",
      "title": "中断与任务节拍诊断",
      "description": "timing reset|split|lean|task|probe|stop|debug|show|currentbench|trigbench|irqtest on/off",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_motor_diag.c",
      "line": 258,
      "subcommands": [
        "trigbench",
        "currentbench",
        "irqtest",
        "debug",
        "blendbench",
        "faultbench",
        "bench",
        "reset",
        "detail",
        "lean",
        "task",
        "probe",
        "split",
        "stop"
      ],
      "usage": [
        "Invalid: slcadence irqtest on|off (5 seconds, stopped only)"
      ],
      "gate": "#if MOTOR_CFG_CMD_DIAG"
    },
    {
      "name": "olsnap",
      "title": "短窗口快照",
      "description": "short Id/Iq/RPM/fault snapshot while running",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_motor_diag.c",
      "line": 953,
      "subcommands": [],
      "usage": [],
      "gate": "#if MOTOR_CFG_CMD_DIAG"
    },
    {
      "name": "olcurdiag",
      "title": "分扇区电流诊断",
      "description": "Hall current errors by sector since last start",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_motor_diag.c",
      "line": 1248,
      "subcommands": [],
      "usage": [],
      "gate": "#if MOTOR_CFG_CMD_DIAG"
    },
    {
      "name": "curtest",
      "title": "静态 d 轴电流测试",
      "description": "static D-axis current-loop test [A] [ms]",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_motor_diag.c",
      "line": 1086,
      "subcommands": [],
      "usage": [
        "Usage: curtest [Id_A] [duration_ms]"
      ],
      "gate": "#if MOTOR_CFG_CMD_DIAG"
    },
    {
      "name": "olboost",
      "title": "V/F 启动补偿",
      "description": "openloop V/F boost voltage <V>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 464,
      "subcommands": [],
      "usage": [
        "V_boost = %.2f V  (usage: olboost <volts>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olkvf",
      "title": "V/F 电压斜率",
      "description": "openloop V/F slope <V/Hz>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 497,
      "subcommands": [],
      "usage": [
        "Kv_f = %.4f V/Hz  (usage: olkvf <V/Hz>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olramp",
      "title": "转速斜坡",
      "description": "openloop V/F ramp rate <rpm/s>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 530,
      "subcommands": [],
      "usage": [
        "Usage: olramp <rpm/s>"
      ],
      "gate": "#endif"
    },
    {
      "name": "olvlimit",
      "title": "电压限幅",
      "description": "openloop Vq limit <V>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 564,
      "subcommands": [],
      "usage": [
        "Vq_limit = %.1f V  (usage: olvlimit <volts>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olmax",
      "title": "最大转速",
      "description": "openloop max speed <rpm>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 597,
      "subcommands": [],
      "usage": [
        "Max RPM = %.0f  (usage: olmax <rpm>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olmin",
      "title": "默认起步转速",
      "description": "openloop min speed <rpm>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 637,
      "subcommands": [],
      "usage": [
        "Start RPM = %.0f  (usage: olmin <rpm>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olstart",
      "title": "启动电机",
      "description": "openloop start",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 674,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "olstop",
      "title": "停止电机",
      "description": "openloop stop",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 714,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "olfault",
      "title": "故障查看与清除",
      "description": "show or clear latched motor fault",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 851,
      "subcommands": [
        "clear"
      ],
      "usage": [
        "Usage: olfault clear"
      ],
      "gate": "#endif"
    },
    {
      "name": "olmode",
      "title": "V/F 与 I/F 模式",
      "description": "openloop mode 0=VF 1=current",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 919,
      "subcommands": [],
      "usage": [
        "Usage: olmode <0|1>"
      ],
      "gate": "#endif"
    },
    {
      "name": "olfb",
      "title": "反馈源",
      "description": "feedback openloop|sensorless|encoder|hall",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 959,
      "subcommands": [
        "openloop",
        "0",
        "sensorless",
        "1",
        "encoder",
        "2",
        "hall",
        "3"
      ],
      "usage": [
        "Usage: olfb <openloop|sensorless|encoder|hall>"
      ],
      "gate": "#endif"
    },
    {
      "name": "oliq",
      "title": "q 轴电流目标",
      "description": "openloop current Iq target <A>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1021,
      "subcommands": [],
      "usage": [
        "Iq set = %.2f A  (usage: oliq <amps>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olid",
      "title": "d 轴电流目标",
      "description": "openloop current Id target <A>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1048,
      "subcommands": [],
      "usage": [
        "Id set = %.2f A  (usage: olid <amps>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "fw",
      "title": "弱磁",
      "description": "field weakening: <max_A> [duty_start] [ramp_A/s]",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1073,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "olkp",
      "title": "电流环 Kp",
      "description": "openloop current Kp",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1122,
      "subcommands": [],
      "usage": [
        "Current Kp = %.4f  (usage: olkp <value>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olki",
      "title": "电流环 Ki",
      "description": "openloop current Ki",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1155,
      "subcommands": [],
      "usage": [
        "Current Ki = %.2f  (usage: olki <value>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olphase",
      "title": "旧启动相位偏置",
      "description": "openloop start phase boost <deg>",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1188,
      "subcommands": [],
      "usage": [
        "Legacy phase boost = %.2f rad (unused by smooth startup; usage: olphase <radians>)"
      ],
      "gate": "#endif"
    },
    {
      "name": "olalign",
      "title": "转子预对齐",
      "description": "startup rotor alignment config",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1220,
      "subcommands": [],
      "usage": [
        "Usage: olalign <0|1> <Id_A> <time_s> [angle_rad]"
      ],
      "gate": "#endif"
    },
    {
      "name": "hfistart",
      "title": "HFI 启动扫描",
      "description": "HFI startup: <0|1> <V> <cycles> [confidence]",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1276,
      "subcommands": [],
      "usage": [
        "Usage: hfistart <0|1> <voltage_V> <cycles_per_axis> [min_confidence]"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "speed",
      "title": "速度状态与测试",
      "description": "speed status/test, or ff detect [max_rpm]",
      "category": "速度与位置",
      "file": "App/cmd/cmd_speed.c",
      "line": 314,
      "subcommands": [
        "ff",
        "test"
      ],
      "usage": [
        "Usage: speed [test [ms] | ff detect [max_rpm]]",
        "Need speed feedback: slenable 1, or olfb encoder|hall",
        "speed ff detect requires stopped motor: olstop",
        "speed ff detect needs clear fault and calibrated encoder feedback",
        "speed ff max RPM must be 600..%.0f",
        "speed ff detect: REMOVE LOAD, %+.0f RPM, any key aborts",
        "speed ff detect: fit rejected",
        "speed ff detect: PASS points=%u",
        "speed ff detect: FAIL, motor auto-stopped",
        "speed test: pass through 0 before reverse",
        "speed test: invalid speed",
        "speed test ",
        "speed test stopped",
        "speed test done"
      ],
      "gate": "#if MOTOR_CFG_CMD_SPEED"
    },
    {
      "name": "spenable",
      "title": "速度环使能",
      "description": "enable speed PI loop <0/1>",
      "category": "速度与位置",
      "file": "App/cmd/cmd_speed.c",
      "line": 368,
      "subcommands": [],
      "usage": [
        "Speed loop = %s  (usage: spenable <0|1>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SPEED"
    },
    {
      "name": "spkp",
      "title": "速度环 Kp",
      "description": "set speed PI Kp",
      "category": "速度与位置",
      "file": "App/cmd/cmd_speed.c",
      "line": 424,
      "subcommands": [],
      "usage": [
        "Speed Kp = %.6f A/RPM  (usage: spkp <value>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SPEED"
    },
    {
      "name": "spki",
      "title": "速度环 Ki",
      "description": "set speed PI Ki",
      "category": "速度与位置",
      "file": "App/cmd/cmd_speed.c",
      "line": 458,
      "subcommands": [],
      "usage": [
        "Speed Ki = %.6f A/(RPM*s)  (usage: spki <value>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SPEED"
    },
    {
      "name": "spiq",
      "title": "速度环电流限幅",
      "description": "set speed PI Iq limit <A>",
      "category": "速度与位置",
      "file": "App/cmd/cmd_speed.c",
      "line": 492,
      "subcommands": [],
      "usage": [
        "Speed Iq limit = %.3f A  (usage: spiq <amps>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SPEED"
    },
    {
      "name": "spbrake",
      "title": "速度环制动",
      "description": "allow speed loop braking <0/1>",
      "category": "速度与位置",
      "file": "App/cmd/cmd_speed.c",
      "line": 523,
      "subcommands": [],
      "usage": [
        "Speed braking = %s  (usage: spbrake <0|1>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SPEED"
    },
    {
      "name": "pos",
      "title": "位置控制",
      "description": "encoder position loop: status|zero|set|move|test",
      "category": "速度与位置",
      "file": "App/cmd/cmd_position.c",
      "line": 241,
      "subcommands": [
        "status",
        "zero",
        "off",
        "hold",
        "set",
        "move",
        "test",
        "on",
        "kp",
        "max"
      ],
      "usage": [
        "usage: pos zero|hold|set <deg>|move <deg>|test [ms]|on|off|kp <rpm/deg>|max <rpm>",
        "pos test ",
        "pos test: bad target",
        "pos test stopped",
        "pos test: target not settled, motor stopped",
        "pos test done, motor auto-stopped"
      ],
      "gate": "#if MOTOR_CFG_CMD_POSITION"
    },
    {
      "name": "joint",
      "title": "关节控制",
      "description": "joint control: set|test|slow|hybridtest [rpm]|iqtest",
      "category": "速度与位置",
      "file": "App/cmd/cmd_joint.c",
      "line": 719,
      "subcommands": [
        "status",
        "zero",
        "off",
        "kp",
        "kd",
        "ki",
        "lim",
        "on",
        "set",
        "test",
        "slow",
        "iqtest",
        "timeouttest",
        "hybridtest"
      ],
      "usage": [
        "usage: joint zero|on|off|set <deg> [rpm] [A]|test [ms]|slow [deg/s]|hybridtest [rpm]|iqtest <A> [ms]|timeouttest <A> [ms]|kp <V/deg>|kd <V/RPM>|ki <V/deg/s>|lim <V>",
        "joint slow speed must be %.1f..%.1f deg/s",
        "joint iqtest current must be -%.2f..-0.001 or 0.001..%.2f A",
        "joint iqtest duration must be %lu..%lu ms",
        "joint timeouttest current must be -%.2f..-0.001 or 0.001..%.2f A",
        "joint timeouttest wait must be 120..1000 ms",
        "joint hybridtest rpm must be > 0",
        "joint test ",
        "joint test: bad target",
        "joint test stopped",
        "joint test done",
        "joint slow ",
        "joint slow: bad target",
        "joint slow stopped, holding current position",
        "joint slow done",
        "joint iqtest %.3fA %lums, any key stops",
        "joint iqtest: command rejected",
        "joint iqtest %s: pos=%.2fdeg vel=%.2fRPM iq_cmd=%.3fA iq_avg=%.3fA ",
        "joint timeouttest submit %.3fA once, wait %lums",
        "joint timeouttest: command rejected",
        "joint timeouttest stopped by key",
        "joint timeouttest result: timeout=%d age=%luus backend=%d iq_cmd=%.3fA vel=%.2fRPM",
        "joint timeouttest cleanup: motor auto-stopped",
        "joint hybridtest: cannot capture motor config",
        "joint hybridtest: cannot set temporary Iq limit",
        "joint hybridtest ",
        "joint hybridtest motor auto-stopped"
      ],
      "gate": "#if MOTOR_CFG_CMD_JOINT"
    },
    {
      "name": "slenable",
      "title": "观测器使能",
      "description": "enable sensorless observer <0/1>",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 412,
      "subcommands": [],
      "usage": [
        "Sensorless observer = %s  (usage: slenable <0|1>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "sltakeover",
      "title": "观测器角度接管",
      "description": "allow observer angle takeover <0/1>",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 81,
      "subcommands": [],
      "usage": [
        "Sensorless takeover = %s  (usage: sltakeover <0|1>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "slrpm",
      "title": "接管与回退转速",
      "description": "set sensorless transition rpm",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 456,
      "subcommands": [],
      "usage": [
        "Usage: slrpm <transition_rpm> <fallback_rpm>"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "slguard",
      "title": "无感交接护栏",
      "description": "set I/F ceiling and takeover guard",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 495,
      "subcommands": [],
      "usage": [
        "Usage: slguard <openloop_max_rpm> <phase_tolerance_rad> <blend_s>"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "slmotor",
      "title": "无感电机模型",
      "description": "set sensorless motor model",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 569,
      "subcommands": [],
      "usage": [
        "Usage: slmotor <R_ohm> <L_mH> <flux_mWb>"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "slobs",
      "title": "观测器与 PLL 增益",
      "description": "set observer and PLL gains",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 629,
      "subcommands": [],
      "usage": [
        "Usage: slobs <observer_gain> <pll_kp> <pll_ki>"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "sldt",
      "title": "观测电压死区补偿",
      "description": "RAM observer deadtime fraction, stopped only",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 606,
      "subcommands": [],
      "usage": [
        "Invalid: sldt <fraction 0..0.02>, stopped only"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "slphase",
      "title": "观测器相位偏置",
      "description": "set observer phase offset <deg>",
      "category": "无感观测",
      "file": "App/cmd/cmd_sensorless.c",
      "line": 666,
      "subcommands": [],
      "usage": [
        "Sensorless phase offset = %.4f rad  (usage: slphase <rad>)"
      ],
      "gate": "#if MOTOR_CFG_CMD_SENSORLESS"
    },
    {
      "name": "show",
      "title": "霍尔电平",
      "description": "show hall sensor state",
      "category": "系统与存储",
      "file": "App/cmd/cmd_show_param.c",
      "line": 18,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "task",
      "title": "任务、堆栈与 CPU",
      "description": "task [watch [ms]]: stack, heap, CPU; key stops",
      "category": "系统与存储",
      "file": "App/cmd/cmd_task.c",
      "line": 471,
      "subcommands": [
        "watch"
      ],
      "usage": [
        "usage: task [watch [ms]]",
        "task watch ",
        "task watch stopped"
      ],
      "gate": "#endif"
    },
    {
      "name": "reset",
      "title": "复位 MCU",
      "description": "software reset MCU",
      "category": "系统与存储",
      "file": "App/cmd/cmd_misc.c",
      "line": 21,
      "subcommands": [],
      "usage": [],
      "gate": "#endif"
    },
    {
      "name": "duty",
      "title": "占空比调试",
      "description": "duty cycle control: create/destory/set/start/stop",
      "category": "基础控制",
      "file": "App/cmd/cmd_duty.c",
      "line": 86,
      "subcommands": [
        "create",
        "destory",
        "set",
        "start",
        "stop"
      ],
      "usage": [
        "usage: duty create|destory|set <pct>|start|stop  (debug only)",
        "duty task create (debug)",
        "duty task destory",
        "usage: duty set <0..100>",
        "duty percent must be 0..100",
        "duty set [%d%%]",
        "duty start (debug)",
        "duty blocked: fault latched, use olfault clear",
        "duty blocked: motor enabled, use olstop first"
      ],
      "gate": "#endif"
    },
    {
      "name": "encoder",
      "title": "编码器与齿槽补偿",
      "description": "AB encoder show/reset/detect",
      "category": "检测与校准",
      "file": "App/cmd/cmd_encoder.c",
      "line": 368,
      "subcommands": [
        "compare",
        "show",
        "reset",
        "detect",
        "watch",
        "cog"
      ],
      "usage": [
        "Usage: encoder compare <0|1> (stopped only)",
        "Usage: encoder detect [ram]",
        "usage: encoder [show|reset|detect|watch [seconds]|cog ...]",
        "usage: encoder cog learn [passes 1..5]",
        "usage: encoder cog [status|learn [passes 1..5]|on|off|stop|clear]"
      ],
      "gate": "#if MOTOR_CFG_CMD_ENCODER"
    },
    {
      "name": "plot",
      "title": "实时数据流",
      "description": "VOFA JustFloat: create current|speed|pos|sine",
      "category": "诊断与波形",
      "file": "App/cmd/cmd_serial_plot.c",
      "line": 171,
      "subcommands": [
        "create",
        "destory",
        "destroy",
        "stop",
        "help"
      ],
      "usage": [
        "  plot create current|speed|pos|sine",
        "  plot destory"
      ],
      "gate": "#endif"
    },
    {
      "name": "openloop",
      "title": "开环后台任务",
      "description": "openloop background task create",
      "category": "基础控制",
      "file": "App/cmd/cmd_openloop.c",
      "line": 1351,
      "subcommands": [
        "create"
      ],
      "usage": [
        "usage: openloop create"
      ],
      "gate": "#endif"
    },
    {
      "name": "help",
      "title": "固件命令列表",
      "category": "系统与存储",
      "description": "固件命令列表",
      "file": "App/cmd/command.c",
      "line": 447,
      "subcommands": [],
      "usage": [
        "help"
      ]
    }
  ],
  "config": [
    {
      "name": "ol_max_rpm",
      "type": "float",
      "array": "",
      "description": "max target speed RPM",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 16,
      "default": "MOTOR_FIXED_RATED_RPM"
    },
    {
      "name": "ol_v_boost",
      "type": "float",
      "array": "",
      "description": "V/F 启动补偿 V",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 17,
      "default": "MOTOR_OPEN_LOOP_VBOOST_DEFAULT"
    },
    {
      "name": "ol_kv_f",
      "type": "float",
      "array": "",
      "description": "V/F 斜率 V/Hz",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 18,
      "default": "0.057f"
    },
    {
      "name": "ol_ramp_rpm_s",
      "type": "float",
      "array": "",
      "description": "软启动速率 RPM/s",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 19,
      "default": "10.0f"
    },
    {
      "name": "ol_vq_limit",
      "type": "float",
      "array": "",
      "description": "Vq 上限 V",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 20,
      "default": "MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT"
    },
    {
      "name": "ol_foc_freq",
      "type": "float",
      "array": "",
      "description": "ADC-DMA 基准频率 Hz",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 21,
      "default": "30000.0f"
    },
    {
      "name": "ol_pole_pairs",
      "type": "float",
      "array": "",
      "description": "电机极对数",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 22,
      "default": "MOTOR_FIXED_POLE_PAIRS"
    },
    {
      "name": "ol_min_rpm",
      "type": "float",
      "array": "",
      "description": "旧启动策略兼容参数；平滑启动不作转速地板",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 23,
      "default": "72.0f"
    },
    {
      "name": "ol_current_kp",
      "type": "float",
      "array": "",
      "description": "开环电流环 Kp",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 24,
      "default": "MOTOR_CURRENT_KP_DEFAULT"
    },
    {
      "name": "ol_current_ki",
      "type": "float",
      "array": "",
      "description": "开环电流环 Ki",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 25,
      "default": "MOTOR_CURRENT_KI_DEFAULT"
    },
    {
      "name": "ol_start_phase_boost",
      "type": "float",
      "array": "",
      "description": "兼容旧启动方式的一拍相位补偿 rad",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 26,
      "default": "M_PI / 2.0f"
    },
    {
      "name": "ol_align_enable",
      "type": "bool",
      "array": "",
      "description": "true=每次 I/F 启动前执行预对齐",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 27,
      "default": "false"
    },
    {
      "name": "ol_align_current",
      "type": "float",
      "array": "",
      "description": "对齐 D 轴电流 Id (A)",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 28,
      "default": "MOTOR_ALIGN_CURRENT_DEFAULT"
    },
    {
      "name": "ol_align_time",
      "type": "float",
      "array": "",
      "description": "固定磁场保持时间 (s)",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 29,
      "default": "0.3f"
    },
    {
      "name": "ol_align_angle",
      "type": "float",
      "array": "",
      "description": "固定对齐电角度 (rad)",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 30,
      "default": "0.0f"
    },
    {
      "name": "hfi_start_enable",
      "type": "bool",
      "array": "",
      "description": "true=I/F 启动前先扫描转子凸极轴",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 31,
      "default": "false"
    },
    {
      "name": "hfi_start_voltage",
      "type": "float",
      "array": "",
      "description": "双极性注入电压幅值 V",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 32,
      "default": "2.0f"
    },
    {
      "name": "hfi_start_cycles",
      "type": "uint16_t",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 33,
      "default": "8U"
    },
    {
      "name": "hfi_start_min_confidence",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 34,
      "default": "0.05f"
    },
    {
      "name": "speed_enable",
      "type": "bool",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 35,
      "default": "false"
    },
    {
      "name": "speed_allow_braking",
      "type": "bool",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 36,
      "default": "false"
    },
    {
      "name": "speed_kp",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 37,
      "default": "MOTOR_SPEED_KP_DEFAULT"
    },
    {
      "name": "speed_ki",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 38,
      "default": "MOTOR_SPEED_KI_DEFAULT"
    },
    {
      "name": "speed_iq_limit",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 39,
      "default": "MOTOR_SPEED_IQ_LIMIT_DEFAULT"
    },
    {
      "name": "speed_loop_hz",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 40,
      "default": "1000.0f"
    },
    {
      "name": "sensorless_enable",
      "type": "bool",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 41,
      "default": "(MOTOR_SENSOR_TYPE == MOTOR_SENSOR_SENSORLESS)"
    },
    {
      "name": "sensorless_takeover",
      "type": "bool",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 42,
      "default": "false"
    },
    {
      "name": "sensorless_motor_r",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 43,
      "default": "MOTOR_FIXED_LINE_RESISTANCE"
    },
    {
      "name": "sensorless_motor_l",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 44,
      "default": "MOTOR_FIXED_LINE_INDUCTANCE"
    },
    {
      "name": "sensorless_flux_linkage",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 45,
      "default": "MOTOR_FIXED_FLUX_LINKAGE"
    },
    {
      "name": "sensorless_observer_gain",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 46,
      "default": "12000000.0f"
    },
    {
      "name": "sensorless_pll_kp",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 47,
      "default": "2000.0f"
    },
    {
      "name": "sensorless_pll_ki",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 48,
      "default": "30000.0f"
    },
    {
      "name": "sensorless_transition_rpm",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 49,
      "default": "500.0f"
    },
    {
      "name": "sensorless_fallback_rpm",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 50,
      "default": "300.0f"
    },
    {
      "name": "sensorless_openloop_max_rpm",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 51,
      "default": "800.0f"
    },
    {
      "name": "sensorless_lock_time",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 52,
      "default": "0.10f"
    },
    {
      "name": "sensorless_loss_time",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 53,
      "default": "0.10f"
    },
    {
      "name": "sensorless_blend_time",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 54,
      "default": "0.20f"
    },
    {
      "name": "sensorless_phase_tolerance",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 55,
      "default": "1.40f"
    },
    {
      "name": "sensorless_phase_offset",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 56,
      "default": "0.0f"
    },
    {
      "name": "sensorless_speed_filter",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 57,
      "default": "0.10f"
    },
    {
      "name": "hall_extra_samples",
      "type": "int",
      "array": "",
      "description": "霍尔 GPIO 额外滤波次数",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 58,
      "default": "1"
    }
  ],
  "calibration": [
    {
      "name": "current_offset",
      "type": "float",
      "array": "[3]",
      "description": "ADC count；0 表示尚未校准，上电检测后写入 Flash",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 62
    },
    {
      "name": "voltage_offset",
      "type": "float",
      "array": "[3]",
      "description": "采样点 V",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 63
    },
    {
      "name": "undriven_offset",
      "type": "float",
      "array": "[3]",
      "description": "采样点 V",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 64
    },
    {
      "name": "hall_table",
      "type": "uint8_t",
      "array": "[MOTOR_HALL_TABLE_SIZE]",
      "description": "0..200，255 未标定；保留旧表字节编码",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 65
    },
    {
      "name": "encoder_offset_rad",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 66
    },
    {
      "name": "encoder_inverted",
      "type": "bool",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 67
    },
    {
      "name": "encoder_calibrated",
      "type": "bool",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_settings.h",
      "line": 68
    }
  ],
  "runtime": [
    {
      "name": "currentAdc",
      "type": "float",
      "array": "[3]",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 25
    },
    {
      "name": "voltAdc",
      "type": "float",
      "array": "[3]",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 26
    },
    {
      "name": "ia",
      "type": "float",
      "array": "",
      "description": "A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 27
    },
    {
      "name": "ib",
      "type": "float",
      "array": "",
      "description": "A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 28
    },
    {
      "name": "ic",
      "type": "float",
      "array": "",
      "description": "A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 29
    },
    {
      "name": "va",
      "type": "float",
      "array": "",
      "description": "V",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 30
    },
    {
      "name": "vb",
      "type": "float",
      "array": "",
      "description": "V",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 31
    },
    {
      "name": "vc",
      "type": "float",
      "array": "",
      "description": "V",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 32
    },
    {
      "name": "v_bus",
      "type": "float",
      "array": "",
      "description": "V",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 33
    },
    {
      "name": "Ialpha",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 34
    },
    {
      "name": "Ibeta",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 35
    },
    {
      "name": "Id",
      "type": "float",
      "array": "",
      "description": "A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 36
    },
    {
      "name": "Iq",
      "type": "float",
      "array": "",
      "description": "A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 37
    },
    {
      "name": "id_target",
      "type": "float",
      "array": "",
      "description": "本拍电流环 Id 目标 A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 38
    },
    {
      "name": "iq_target",
      "type": "float",
      "array": "",
      "description": "本拍电流环 Iq 目标 A",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 39
    },
    {
      "name": "theta",
      "type": "float",
      "array": "",
      "description": "电角度 rad",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 40
    },
    {
      "name": "modD",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 41
    },
    {
      "name": "modQ",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 42
    },
    {
      "name": "modAlpha",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 43
    },
    {
      "name": "modBeta",
      "type": "float",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 44
    },
    {
      "name": "duty1",
      "type": "uint32_t",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 45
    },
    {
      "name": "duty2",
      "type": "uint32_t",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 46
    },
    {
      "name": "duty3",
      "type": "uint32_t",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 47
    },
    {
      "name": "curHallABC",
      "type": "uint8_t",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 48
    },
    {
      "name": "lastHallABC",
      "type": "uint8_t",
      "array": "",
      "description": "",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 49
    },
    {
      "name": "ol_current_rpm",
      "type": "float",
      "array": "",
      "description": "斜坡转速 RPM",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 50
    },
    {
      "name": "ol_theta",
      "type": "float",
      "array": "",
      "description": "开环电角度 rad",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 51
    },
    {
      "name": "ol_vd_int",
      "type": "float",
      "array": "",
      "description": "d 轴积分 V",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 52
    },
    {
      "name": "ol_vq_int",
      "type": "float",
      "array": "",
      "description": "q 轴积分 V",
      "file": "App/motor_param/inc/motor_param.h",
      "line": 53
    }
  ],
  "profiles": [
    {
      "file": "App/motor_param/inc/motor_3508.h",
      "name": "TYI-3508",
      "macros": [
        {
          "name": "MOTOR_3508_H",
          "value": "#ifdef __cplusplus",
          "description": ""
        },
        {
          "name": "MOTOR_3508_MODEL",
          "value": "\"TYI-3508\"",
          "description": "型号"
        },
        {
          "name": "MOTOR_SENSOR_TYPE",
          "value": "MOTOR_SENSOR_SENSORLESS",
          "description": "本台未接编码器、霍尔"
        },
        {
          "name": "MOTOR_3508_POLE_COUNT",
          "value": "(14U)",
          "description": "磁极数"
        },
        {
          "name": "MOTOR_3508_POLE_PAIRS",
          "value": "(7.0f)",
          "description": "极对数 = 磁极数 / 2"
        },
        {
          "name": "MOTOR_3508_SLOT_COUNT",
          "value": "(12U)",
          "description": "定子槽数，12 槽 14 极"
        },
        {
          "name": "MOTOR_3508_KV400_RPM_PER_V",
          "value": "(400.0f)",
          "description": "图纸 KV400，空载 RPM/V"
        },
        {
          "name": "MOTOR_3508_KV590_RPM_PER_V",
          "value": "(590.0f)",
          "description": "图纸 KV590，空载 RPM/V"
        },
        {
          "name": "MOTOR_3508_KV400_TEST_VOLTAGE_V",
          "value": "(22.2f)",
          "description": "KV400 测试电压 V，6S"
        },
        {
          "name": "MOTOR_3508_KV590_TEST_VOLTAGE_V",
          "value": "(14.8f)",
          "description": "KV590 测试电压 V，4S"
        },
        {
          "name": "MOTOR_3508_MAX_CURRENT_A",
          "value": "(25.0f)",
          "description": "最大连续电流 A"
        },
        {
          "name": "MOTOR_3508_MAX_POWER_W",
          "value": "(370.0f)",
          "description": "最大连续功率 W"
        },
        {
          "name": "MOTOR_3508_WEIGHT_G",
          "value": "(90.0f)",
          "description": "重量 g"
        },
        {
          "name": "MOTOR_3508_STATOR_DIAMETER_MM",
          "value": "(35.0f)",
          "description": "定子外径 mm"
        },
        {
          "name": "MOTOR_3508_STATOR_THICKNESS_MM",
          "value": "(8.0f)",
          "description": "定子厚度 mm"
        },
        {
          "name": "MOTOR_3508_ROTOR_DIAMETER_MM",
          "value": "(42.5f)",
          "description": "转子直径 mm"
        },
        {
          "name": "MOTOR_3508_PHASE_RESISTANCE_OHM",
          "value": "(0.4740f)",
          "description": "相电阻 Ω"
        },
        {
          "name": "MOTOR_3508_PHASE_INDUCTANCE_MH",
          "value": "(0.1120f)",
          "description": "相电感 mH"
        },
        {
          "name": "MOTOR_3508_PHASE_INDUCTANCE_H",
          "value": "(MOTOR_3508_PHASE_INDUCTANCE_MH * 0.001f)",
          "description": ""
        },
        {
          "name": "MOTOR_3508_FLUX_LINKAGE_WB",
          "value": "(0.002390f)",
          "description": "磁链 Wb，即 2.3900 mWb"
        },
        {
          "name": "MOTOR_3508_CONSERVATIVE_RPM_CAP",
          "value": "(MOTOR_3508_KV400_RPM_PER_V * MOTOR_3508_KV590_TEST_VOLTAGE_V)",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_MODEL",
          "value": "MOTOR_3508_MODEL",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_POLE_COUNT",
          "value": "MOTOR_3508_POLE_COUNT",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_POLE_PAIRS",
          "value": "MOTOR_3508_POLE_PAIRS",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_PHASE_COUNT",
          "value": "(3U)",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_VOLTAGE",
          "value": "MOTOR_3508_KV590_TEST_VOLTAGE_V",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_RPM",
          "value": "MOTOR_3508_CONSERVATIVE_RPM_CAP",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_CURRENT",
          "value": "MOTOR_3508_MAX_CURRENT_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_OUTPUT_POWER",
          "value": "MOTOR_3508_MAX_POWER_W",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_PEAK_CURRENT",
          "value": "MOTOR_3508_MAX_CURRENT_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_TORQUE_CONSTANT",
          "value": "(1.5f * MOTOR_3508_POLE_PAIRS * MOTOR_3508_FLUX_LINKAGE_WB)",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_RESISTANCE",
          "value": "MOTOR_3508_PHASE_RESISTANCE_OHM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE_MH",
          "value": "MOTOR_3508_PHASE_INDUCTANCE_MH",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE",
          "value": "MOTOR_3508_PHASE_INDUCTANCE_H",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_FLUX_LINKAGE",
          "value": "MOTOR_3508_FLUX_LINKAGE_WB",
          "description": ""
        },
        {
          "name": "MOTOR_OPEN_LOOP_IQ_DEFAULT",
          "value": "(1.0f)",
          "description": "开环 I/F 拖转电流 A"
        },
        {
          "name": "MOTOR_ALIGN_CURRENT_DEFAULT",
          "value": "(1.0f)",
          "description": "静态 D 轴对齐电流 A"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VBOOST_DEFAULT",
          "value": "(1.0f)",
          "description": "V/F 起步补偿电压 V"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT",
          "value": "(4.0f)",
          "description": "开环 Vq 上限 V"
        },
        {
          "name": "MOTOR_ENCODER_START_IQ_A",
          "value": "(0.0f)",
          "description": "速度环刚接管时的起步电流 A，0 表示不注入"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM",
          "value": "(0.0f)",
          "description": "低于此转速给满齿槽助推，RPM"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM",
          "value": "(0.0f)",
          "description": "高于此转速退出齿槽助推，RPM"
        },
        {
          "name": "MOTOR_CURRENT_KP_DEFAULT",
          "value": "(0.12f)",
          "description": "电流环比例，V/A"
        },
        {
          "name": "MOTOR_CURRENT_KI_DEFAULT",
          "value": "(18.0f)",
          "description": "电流环积分，V/(A·s)"
        },
        {
          "name": "MOTOR_SPEED_KP_DEFAULT",
          "value": "(0.003f)",
          "description": "速度比例，A/RPM"
        },
        {
          "name": "MOTOR_SPEED_KI_DEFAULT",
          "value": "(0.01f)",
          "description": "速度积分，A/(RPM·s)"
        },
        {
          "name": "MOTOR_SPEED_IQ_LIMIT_DEFAULT",
          "value": "(2.0f)",
          "description": "速度环电流上限 A"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLTAGE_MODE",
          "value": "(0U)",
          "description": "1：速度环出电压；0：出电流"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_PER_AMP",
          "value": "(0.4740f)",
          "description": "电流换成电压，V/A，约等于相电阻"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_LIMIT_V",
          "value": "(0.0f)",
          "description": "速度环电压上限 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_BOOST_V",
          "value": "(0.0f)",
          "description": "低速固定前馈 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_V_PER_RPM",
          "value": "(0.0f)",
          "description": "空载反电势前馈 V/RPM"
        },
        {
          "name": "MOTOR_ENCODER_PLL_BW_RAD_S",
          "value": "(100.0f)",
          "description": "速度 PLL 带宽 rad/s"
        },
        {
          "name": "MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG",
          "value": "(20.0f * 60.0f / 360.0f)",
          "description": "角度误差换成转速，约 3.33 RPM/deg"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_DEFAULT",
          "value": "(60.0f)",
          "description": "位置运动速度上限 RPM"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_ALLOWED",
          "value": "(1800.0f)",
          "description": "允许提到的最高速度 RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KP_DEFAULT",
          "value": "(0.050f)",
          "description": "位置环内的速度比例，V/RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KI_DEFAULT",
          "value": "(0.20f)",
          "description": "位置环内的速度积分，V/(RPM·s)"
        },
        {
          "name": "MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT",
          "value": "(2.0f)",
          "description": "位置环电压上限 V"
        },
        {
          "name": "MOTOR_POSITION_ACCEL_RPM_S_DEFAULT",
          "value": "(30000.0f)",
          "description": "速度变化上限 RPM/s"
        },
        {
          "name": "MOTOR_POSITION_FF_V_PER_RPM_DEFAULT",
          "value": "(0.0f)",
          "description": "位置环速度前馈 V/RPM"
        },
        {
          "name": "MOTOR_POSITION_HOLD_ENTER_DEG",
          "value": "(2.0f)",
          "description": "误差小于此角度进入保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HOLD_EXIT_DEG",
          "value": "(3.0f)",
          "description": "误差大于此角度退出保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HYBRID_SPEED_ENABLE",
          "value": "(0U)",
          "description": "0：禁止低速切电流环"
        },
        {
          "name": "MOTOR_POSITION_SMALL_TEST_SEQUENCE",
          "value": "(0U)",
          "description": "0：不跑 0°→2°→5° 小行程测试"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ENABLE",
          "value": "(0U)",
          "description": "0：大角度不走轨迹"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_VEL_DEG_S",
          "value": "(180.0f)",
          "description": "轨迹最高速度 deg/s"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2",
          "value": "(360.0f)",
          "description": "轨迹加速度 deg/s²"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2",
          "value": "(540.0f)",
          "description": "轨迹减速度 deg/s²"
        }
      ],
      "note": "@Description: TYI 3508 电机铭牌与无感检测初值\r@Author: lkc\r@Note: 图纸在 Doc/motor/3508，同一页含 KV400 与 KV590。极数、槽数、连续电流两档相同。\r       R/L/磁链是无感检测的大概值，还没和图纸 KV 核对。\r       控制默认值未上板标定。图纸 25 A 是连续电流，起步电流用下面的开环默认值。"
    },
    {
      "file": "App/motor_param/inc/motor_2804_220kv.h",
      "name": "2804-220KV",
      "macros": [
        {
          "name": "MOTOR_2804_220KV_H",
          "value": "#ifdef __cplusplus",
          "description": ""
        },
        {
          "name": "MOTOR_2804_220KV_MODEL",
          "value": "\"2804-220KV\"",
          "description": "型号"
        },
        {
          "name": "MOTOR_SENSOR_TYPE",
          "value": "MOTOR_SENSOR_ENCODER",
          "description": "MT6701 SSI"
        },
        {
          "name": "MOTOR_2804_220KV_POLE_COUNT",
          "value": "(14U)",
          "description": "磁极数"
        },
        {
          "name": "MOTOR_2804_220KV_POLE_PAIRS",
          "value": "(7.0f)",
          "description": "极对数 = 磁极数 / 2"
        },
        {
          "name": "MOTOR_2804_220KV_SLOT_COUNT",
          "value": "(12U)",
          "description": "定子槽数，12 槽 14 极"
        },
        {
          "name": "MOTOR_2804_220KV_KV_RPM_PER_V",
          "value": "(220.0f)",
          "description": "KV，空载 RPM/V"
        },
        {
          "name": "MOTOR_2804_220KV_RATED_VOLTAGE_V",
          "value": "(12.0f)",
          "description": "额定电压 V"
        },
        {
          "name": "MOTOR_2804_220KV_OPERATING_VOLTAGE_MIN",
          "value": "(7.4f)",
          "description": "工作电压下限 V"
        },
        {
          "name": "MOTOR_2804_220KV_OPERATING_VOLTAGE_MAX",
          "value": "(16.0f)",
          "description": "工作电压上限 V"
        },
        {
          "name": "MOTOR_2804_220KV_TRANSIENT_VOLTAGE_MAX",
          "value": "(20.0f)",
          "description": "瞬时最大电压 V，不作测试电压"
        },
        {
          "name": "MOTOR_2804_220KV_RATED_CURRENT_A",
          "value": "(0.5f)",
          "description": "额定电流 A"
        },
        {
          "name": "MOTOR_2804_220KV_MAX_CURRENT_A",
          "value": "(2.0f)",
          "description": "图纸最大电流 A，不是首次测试电流"
        },
        {
          "name": "MOTOR_2804_220KV_RATED_SPEED_RPM",
          "value": "(2600.0f)",
          "description": "额定转速 RPM，12 V"
        },
        {
          "name": "MOTOR_2804_220KV_RATED_TORQUE_NM",
          "value": "(0.03f)",
          "description": "额定转矩 N·m，图纸 300 g·cm"
        },
        {
          "name": "MOTOR_2804_220KV_TORQUE_CONSTANT_NM_A",
          "value": "(0.0434f)",
          "description": "力矩常数 N·m/A"
        },
        {
          "name": "MOTOR_2804_220KV_PHASE_RESISTANCE_OHM",
          "value": "(2.55f)",
          "description": "相电阻 Rs，Ω，观测器用这个"
        },
        {
          "name": "MOTOR_2804_220KV_WINDING_RESISTANCE_OHM",
          "value": "(5.1f)",
          "description": "绕线电阻 Ω，约两相，不要当 Rs"
        },
        {
          "name": "MOTOR_2804_220KV_PHASE_INDUCTANCE_H",
          "value": "(0.00086f)",
          "description": "相电感 Ls，H，即 0.86 mH"
        },
        {
          "name": "MOTOR_2804_220KV_WINDING_INDUCTANCE_H",
          "value": "(0.0028f)",
          "description": "绕线电感 H，即 2.8 mH，不要当 Ls"
        },
        {
          "name": "MOTOR_2804_220KV_FLUX_LINKAGE_WB",
          "value": "(0.0035f)",
          "description": "磁链 Wb"
        },
        {
          "name": "MOTOR_2804_220KV_ROTOR_INERTIA_KGM2",
          "value": "(0.0000037f)",
          "description": "转子惯量 kg·m²，小惯量易冲过目标"
        },
        {
          "name": "MOTOR_FIXED_MODEL",
          "value": "MOTOR_2804_220KV_MODEL",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_POLE_COUNT",
          "value": "MOTOR_2804_220KV_POLE_COUNT",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_POLE_PAIRS",
          "value": "MOTOR_2804_220KV_POLE_PAIRS",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_PHASE_COUNT",
          "value": "(3U)",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_VOLTAGE",
          "value": "MOTOR_2804_220KV_RATED_VOLTAGE_V",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_RPM",
          "value": "MOTOR_2804_220KV_RATED_SPEED_RPM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_TORQUE",
          "value": "MOTOR_2804_220KV_RATED_TORQUE_NM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_CURRENT",
          "value": "MOTOR_2804_220KV_RATED_CURRENT_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_OUTPUT_POWER",
          "value": "(8.17f)",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_PEAK_CURRENT",
          "value": "MOTOR_2804_220KV_MAX_CURRENT_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_TORQUE_CONSTANT",
          "value": "MOTOR_2804_220KV_TORQUE_CONSTANT_NM_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_RESISTANCE",
          "value": "MOTOR_2804_220KV_PHASE_RESISTANCE_OHM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE_MH",
          "value": "(0.86f)",
          "description": "相电感，mH"
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE",
          "value": "MOTOR_2804_220KV_PHASE_INDUCTANCE_H",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_FLUX_LINKAGE",
          "value": "MOTOR_2804_220KV_FLUX_LINKAGE_WB",
          "description": ""
        },
        {
          "name": "MOTOR_OPEN_LOOP_IQ_DEFAULT",
          "value": "(0.30f)",
          "description": "开环 I/F 拖转电流 A"
        },
        {
          "name": "MOTOR_ALIGN_CURRENT_DEFAULT",
          "value": "(0.30f)",
          "description": "静态 D 轴对齐电流 A"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VBOOST_DEFAULT",
          "value": "(1.0f)",
          "description": "V/F 起步补偿电压 V"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT",
          "value": "(9.0f)",
          "description": "开环 Vq 上限 V"
        },
        {
          "name": "MOTOR_ENCODER_START_IQ_A",
          "value": "(0.0f)",
          "description": "速度环刚接管时的起步电流 A"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM",
          "value": "(0.0f)",
          "description": "低于此转速给满齿槽助推，RPM"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM",
          "value": "(0.0f)",
          "description": "高于此转速退出齿槽助推，RPM"
        },
        {
          "name": "MOTOR_CURRENT_KP_DEFAULT",
          "value": "(0.12f)",
          "description": "电流环比例，V/A"
        },
        {
          "name": "MOTOR_CURRENT_KI_DEFAULT",
          "value": "(18.0f)",
          "description": "电流环积分，V/(A·s)"
        },
        {
          "name": "MOTOR_SPEED_KP_DEFAULT",
          "value": "(0.003f)",
          "description": "速度比例，A/RPM；电流后端实测不振荡"
        },
        {
          "name": "MOTOR_SPEED_KI_DEFAULT",
          "value": "(0.01f)",
          "description": "速度积分，A/(RPM·s)"
        },
        {
          "name": "MOTOR_SPEED_IQ_LIMIT_DEFAULT",
          "value": "(0.20f)",
          "description": "速度环电流上限 A"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLTAGE_MODE",
          "value": "(2U)",
          "description": "0：电流；1：电压；2：低速电压/高速电流"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLTAGE_MAX_RPM",
          "value": "(120.0f)",
          "description": "混合模式直接 Vq 的目标转速上限"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_PER_AMP",
          "value": "(2.55f)",
          "description": "电流换成电压，V/A，约等于相电阻"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_LIMIT_V",
          "value": "(9.0f)",
          "description": "速度环电压上限 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_BOOST_V",
          "value": "(0.0741f)",
          "description": "空载摩擦/死区电压，正反转八点拟合"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_V_PER_RPM",
          "value": "(0.0023871f)",
          "description": ""
        },
        {
          "name": "MOTOR_ENCODER_PLL_BW_RAD_S",
          "value": "(30.0f)",
          "description": "速度 PLL 带宽 rad/s，压低齿槽纹波"
        },
        {
          "name": "MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG",
          "value": "(1.0f)",
          "description": "角度误差换成转速，RPM/deg"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_DEFAULT",
          "value": "(2.0f)",
          "description": "位置运动速度上限 RPM"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_ALLOWED",
          "value": "(20.0f)",
          "description": "极限测试可逐级提高；默认仍保持 2 RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KP_DEFAULT",
          "value": "(0.020f)",
          "description": "位置环内的速度比例，V/RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KI_DEFAULT",
          "value": "(0.02f)",
          "description": "位置环内的速度积分，V/(RPM·s)"
        },
        {
          "name": "MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT",
          "value": "(0.35f)",
          "description": "位置环电压上限 V"
        },
        {
          "name": "MOTOR_POSITION_ACCEL_RPM_S_DEFAULT",
          "value": "(30.0f)",
          "description": "速度变化上限 RPM/s"
        },
        {
          "name": "MOTOR_POSITION_FF_V_PER_RPM_DEFAULT",
          "value": "(0.0f)",
          "description": "位置环速度前馈 V/RPM，先关"
        },
        {
          "name": "MOTOR_POSITION_HOLD_ENTER_DEG",
          "value": "(2.0f)",
          "description": "误差小于此角度进入保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HOLD_EXIT_DEG",
          "value": "(3.0f)",
          "description": "误差大于此角度退出保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HYBRID_SPEED_ENABLE",
          "value": "(0U)",
          "description": "0：禁止低速切电流环，盲区里会失稳"
        },
        {
          "name": "MOTOR_POSITION_SMALL_TEST_SEQUENCE",
          "value": "(1U)",
          "description": "1：允许 0°→2°→5° 小行程测试"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ENABLE",
          "value": "(1U)",
          "description": "1：关节位置命令走轨迹"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_VEL_RPM",
          "value": "(300.0f)",
          "description": "只改这个，RPM"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_VEL_DEG_S",
          "value": "(MOTOR_JOINT_TRAJECTORY_VEL_RPM * 6.0f)",
          "description": ""
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2",
          "value": "(MOTOR_JOINT_TRAJECTORY_VEL_DEG_S / 0.5f)",
          "description": ""
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2",
          "value": "(MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2 * 1.5f)",
          "description": ""
        }
      ],
      "note": "@Description: 2804-220KV 电机铭牌、电气参数与调试默认值\n@Author: lkc\n@Note: 图纸来自用户提供的 2804 工程图。控制默认值分成起步、电流环、速度环、位置环，\n       和另外两款电机是同一套宏。\n       空轴基础已测；带轮和负载要单独再调，不要几组参数一起放大。"
    },
    {
      "file": "App/motor_param/inc/motor_c2208_100t.h",
      "name": "C2208-100T",
      "macros": [
        {
          "name": "MOTOR_C2208_100T_H",
          "value": "#ifdef __cplusplus",
          "description": ""
        },
        {
          "name": "MOTOR_C2208_100T_MODEL",
          "value": "\"C2208-100T\"",
          "description": "型号"
        },
        {
          "name": "MOTOR_SENSOR_TYPE",
          "value": "MOTOR_SENSOR_ENCODER",
          "description": "MT6701 SSI"
        },
        {
          "name": "MOTOR_C2208_100T_TYPE",
          "value": "\"BLDC\"",
          "description": "直流无刷电机"
        },
        {
          "name": "MOTOR_C2208_100T_POLE_COUNT",
          "value": "(14U)",
          "description": "磁极数"
        },
        {
          "name": "MOTOR_C2208_100T_POLE_PAIRS",
          "value": "(7.0f)",
          "description": "极对数 = 磁极数 / 2"
        },
        {
          "name": "MOTOR_C2208_100T_SLOT_COUNT",
          "value": "(12U)",
          "description": "定子槽数"
        },
        {
          "name": "MOTOR_C2208_100T_PHASE_COUNT",
          "value": "(3U)",
          "description": "相数"
        },
        {
          "name": "MOTOR_C2208_100T_KV_RPM_PER_V",
          "value": "(120.0f)",
          "description": "KV，空载 RPM/V"
        },
        {
          "name": "MOTOR_C2208_100T_RATED_VOLTAGE_V",
          "value": "(12.0f)",
          "description": "额定电压 V"
        },
        {
          "name": "MOTOR_C2208_100T_MAX_VOLTAGE_V",
          "value": "(24.0f)",
          "description": "最大电压 V"
        },
        {
          "name": "MOTOR_C2208_100T_RATED_CURRENT_A",
          "value": "(0.8f)",
          "description": "额定电流 A"
        },
        {
          "name": "MOTOR_C2208_100T_MAX_CURRENT_A",
          "value": "(4.5f)",
          "description": "最大电流 A"
        },
        {
          "name": "MOTOR_C2208_100T_RATED_POWER_W",
          "value": "(14.0f)",
          "description": "额定功率 W"
        },
        {
          "name": "MOTOR_C2208_100T_RATED_SPEED_RPM",
          "value": "(1800.0f)",
          "description": "额定转速 RPM"
        },
        {
          "name": "MOTOR_C2208_100T_RATED_TORQUE_NM",
          "value": "(0.03f)",
          "description": "额定转矩约值 N·m"
        },
        {
          "name": "MOTOR_C2208_100T_RESISTANCE_OHM",
          "value": "(21.2f)",
          "description": "图纸电阻 Ω"
        },
        {
          "name": "MOTOR_C2208_100T_PHASE_INDUCTANCE_MH",
          "value": "(5.3f)",
          "description": "图纸相电感 mH"
        },
        {
          "name": "MOTOR_C2208_100T_PHASE_INDUCTANCE_H",
          "value": "(0.0053f)",
          "description": "相电感 H"
        },
        {
          "name": "MOTOR_C2208_100T_WINDING_TURNS",
          "value": "(100U)",
          "description": "每组绕组匝数"
        },
        {
          "name": "MOTOR_C2208_100T_WIRE_DIAMETER_MM",
          "value": "(0.14f)",
          "description": "漆包线线径 mm"
        },
        {
          "name": "MOTOR_C2208_100T_WINDING_CONNECTION",
          "value": "\"STAR\"",
          "description": "星形接法"
        },
        {
          "name": "MOTOR_C2208_100T_WEIGHT_G",
          "value": "(38.0f)",
          "description": "重量 g"
        },
        {
          "name": "MOTOR_C2208_100T_OUTER_DIAMETER_MM",
          "value": "(27.5f)",
          "description": "外径 mm"
        },
        {
          "name": "MOTOR_C2208_100T_BODY_HEIGHT_MM",
          "value": "(26.5f)",
          "description": "机身高度 mm"
        },
        {
          "name": "MOTOR_C2208_100T_TOTAL_HEIGHT_MM",
          "value": "(28.0f)",
          "description": "总高度 mm"
        },
        {
          "name": "MOTOR_C2208_100T_SHAFT_DIAMETER_MM",
          "value": "(3.0f)",
          "description": "输出轴直径 mm"
        },
        {
          "name": "MOTOR_FIXED_MODEL",
          "value": "MOTOR_C2208_100T_MODEL",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_POLE_COUNT",
          "value": "MOTOR_C2208_100T_POLE_COUNT",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_POLE_PAIRS",
          "value": "MOTOR_C2208_100T_POLE_PAIRS",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_PHASE_COUNT",
          "value": "MOTOR_C2208_100T_PHASE_COUNT",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_VOLTAGE",
          "value": "MOTOR_C2208_100T_RATED_VOLTAGE_V",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_RPM",
          "value": "MOTOR_C2208_100T_RATED_SPEED_RPM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_TORQUE",
          "value": "MOTOR_C2208_100T_RATED_TORQUE_NM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_RATED_CURRENT",
          "value": "MOTOR_C2208_100T_RATED_CURRENT_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_OUTPUT_POWER",
          "value": "MOTOR_C2208_100T_RATED_POWER_W",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_PEAK_CURRENT",
          "value": "MOTOR_C2208_100T_MAX_CURRENT_A",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_RESISTANCE",
          "value": "MOTOR_C2208_100T_RESISTANCE_OHM",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE_MH",
          "value": "MOTOR_C2208_100T_PHASE_INDUCTANCE_MH",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE",
          "value": "MOTOR_C2208_100T_PHASE_INDUCTANCE_H",
          "description": ""
        },
        {
          "name": "MOTOR_FIXED_FLUX_LINKAGE",
          "value": "(0.00928f)",
          "description": ""
        },
        {
          "name": "MOTOR_OPEN_LOOP_IQ_DEFAULT",
          "value": "(0.6f)",
          "description": "开环 I/F 拖转电流 A"
        },
        {
          "name": "MOTOR_ALIGN_CURRENT_DEFAULT",
          "value": "(0.5f)",
          "description": "静态 D 轴对齐电流 A"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VBOOST_DEFAULT",
          "value": "(5.0f)",
          "description": "V/F 起步补偿电压 V"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT",
          "value": "(18.0f)",
          "description": "开环 Vq 上限 V"
        },
        {
          "name": "MOTOR_ENCODER_START_IQ_A",
          "value": "(0.08f)",
          "description": "速度环刚接管时的起步电流 A"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM",
          "value": "(10.0f)",
          "description": "低于此转速给满齿槽助推，RPM"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM",
          "value": "(20.0f)",
          "description": "高于此转速退出齿槽助推，RPM"
        },
        {
          "name": "MOTOR_CURRENT_KP_DEFAULT",
          "value": "(0.12f)",
          "description": "电流环比例，V/A"
        },
        {
          "name": "MOTOR_CURRENT_KI_DEFAULT",
          "value": "(18.0f)",
          "description": "电流环积分，V/(A·s)"
        },
        {
          "name": "MOTOR_SPEED_KP_DEFAULT",
          "value": "(0.008f)",
          "description": "速度比例，A/RPM"
        },
        {
          "name": "MOTOR_SPEED_KI_DEFAULT",
          "value": "(0.002f)",
          "description": "速度积分，A/(RPM·s)"
        },
        {
          "name": "MOTOR_SPEED_IQ_LIMIT_DEFAULT",
          "value": "(0.20f)",
          "description": "速度环电流上限 A"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLTAGE_MODE",
          "value": "(1U)",
          "description": "1：速度环出电压；0：出电流"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_PER_AMP",
          "value": "(5.0f)",
          "description": "电流换成电压，V/A"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_LIMIT_V",
          "value": "(13.8f)",
          "description": "速度环电压上限 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_BOOST_V",
          "value": "(0.1666667f)",
          "description": "低速固定前馈 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_V_PER_RPM",
          "value": "(0.00586f)",
          "description": "空载反电势前馈 V/RPM"
        },
        {
          "name": "MOTOR_ENCODER_PLL_BW_RAD_S",
          "value": "(30.0f)",
          "description": "速度 PLL 带宽 rad/s，压低齿槽纹波"
        },
        {
          "name": "MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG",
          "value": "(20.0f * 60.0f / 360.0f)",
          "description": "角度误差换成转速，约 3.33 RPM/deg"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_DEFAULT",
          "value": "(60.0f)",
          "description": "位置运动速度上限 RPM"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_ALLOWED",
          "value": "(1800.0f)",
          "description": "允许提到的最高速度 RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KP_DEFAULT",
          "value": "(0.050f)",
          "description": "位置环内的速度比例，V/RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KI_DEFAULT",
          "value": "(0.20f)",
          "description": "位置环内的速度积分，V/(RPM·s)"
        },
        {
          "name": "MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT",
          "value": "(2.0f)",
          "description": "位置环电压上限 V"
        },
        {
          "name": "MOTOR_POSITION_ACCEL_RPM_S_DEFAULT",
          "value": "(30000.0f)",
          "description": "速度变化上限 RPM/s"
        },
        {
          "name": "MOTOR_POSITION_FF_V_PER_RPM_DEFAULT",
          "value": "(0.0f)",
          "description": "位置环速度前馈 V/RPM"
        },
        {
          "name": "MOTOR_POSITION_HOLD_ENTER_DEG",
          "value": "(2.0f)",
          "description": "误差小于此角度进入保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HOLD_EXIT_DEG",
          "value": "(3.0f)",
          "description": "误差大于此角度退出保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HYBRID_SPEED_ENABLE",
          "value": "(0U)",
          "description": "0：禁止低速切电流环"
        },
        {
          "name": "MOTOR_POSITION_SMALL_TEST_SEQUENCE",
          "value": "(0U)",
          "description": "0：不跑 0°→2°→5° 小行程测试"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ENABLE",
          "value": "(0U)",
          "description": "1：关节位置命令走轨迹"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_VEL_DEG_S",
          "value": "(180.0f)",
          "description": "轨迹最高速度 deg/s"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2",
          "value": "(360.0f)",
          "description": "轨迹加速度 deg/s²"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2",
          "value": "(540.0f)",
          "description": "轨迹减速度 deg/s²"
        }
      ],
      "note": "@Description: C2208-100T 电机铭牌与调试默认值\n@Author: lkc\n@Note: 图纸来自 Doc/motor/C2208-100T/2208电机工程图.png。\n       控制默认值分成起步、电流环、速度环、位置环，和另外两款电机是同一套宏。"
    },
    {
      "file": "App/motor_param/inc/motor_atk_bl57h95d20e.h",
      "name": "ATK-BL57H95D20E",
      "macros": [
        {
          "name": "MOTOR_ATK_BL57H95D20E_H",
          "value": "#ifdef __cplusplus",
          "description": ""
        },
        {
          "name": "MOTOR_SENSOR_TYPE",
          "value": "MOTOR_SENSOR_HALL",
          "description": "空载 Hall 路径已在这台电机上用过"
        },
        {
          "name": "MOTOR_FIXED_MODEL",
          "value": "\"ATK-BL57H95D20E\"",
          "description": "型号"
        },
        {
          "name": "MOTOR_FIXED_POLE_COUNT",
          "value": "(4U)",
          "description": "磁极数"
        },
        {
          "name": "MOTOR_FIXED_POLE_PAIRS",
          "value": "(2.0f)",
          "description": "极对数 = 磁极数 / 2"
        },
        {
          "name": "MOTOR_FIXED_PHASE_COUNT",
          "value": "(3U)",
          "description": "相数"
        },
        {
          "name": "MOTOR_FIXED_RATED_VOLTAGE",
          "value": "(24.0f)",
          "description": "额定电压 V"
        },
        {
          "name": "MOTOR_FIXED_RATED_RPM",
          "value": "(3000.0f)",
          "description": "额定转速 RPM"
        },
        {
          "name": "MOTOR_FIXED_RATED_TORQUE",
          "value": "(0.33f)",
          "description": "额定转矩 N·m"
        },
        {
          "name": "MOTOR_FIXED_RATED_CURRENT",
          "value": "(6.6f)",
          "description": "额定电流 A"
        },
        {
          "name": "MOTOR_FIXED_OUTPUT_POWER",
          "value": "(105.0f)",
          "description": "输出功率 W"
        },
        {
          "name": "MOTOR_FIXED_PEAK_TORQUE",
          "value": "(4.2f)",
          "description": "峰值转矩 N·m"
        },
        {
          "name": "MOTOR_FIXED_PEAK_CURRENT",
          "value": "(19.8f)",
          "description": "峰值电流 A"
        },
        {
          "name": "MOTOR_FIXED_TORQUE_CONSTANT",
          "value": "(0.063f)",
          "description": "力矩常数 N·m/A"
        },
        {
          "name": "MOTOR_FIXED_BACK_EMF",
          "value": "(6.6f)",
          "description": "线间 RMS 反电势 V/kRPM"
        },
        {
          "name": "MOTOR_FIXED_LINE_RESISTANCE",
          "value": "(0.4f)",
          "description": "线电阻 Ω"
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE_MH",
          "value": "(0.8f)",
          "description": "线电感 mH"
        },
        {
          "name": "MOTOR_FIXED_LINE_INDUCTANCE",
          "value": "(0.0008f)",
          "description": "线电感 H，即 0.8 mH"
        },
        {
          "name": "MOTOR_FIXED_INERTIA",
          "value": "(210.0f)",
          "description": "转动惯量 g·cm²"
        },
        {
          "name": "MOTOR_FIXED_BODY_LENGTH",
          "value": "(95.0f)",
          "description": "机身长度 mm"
        },
        {
          "name": "MOTOR_FIXED_WEIGHT",
          "value": "(1.0f)",
          "description": "重量 kg"
        },
        {
          "name": "MOTOR_FIXED_FLUX_LINKAGE",
          "value": "(0.02573f)",
          "description": ""
        },
        {
          "name": "MOTOR_OPEN_LOOP_IQ_DEFAULT",
          "value": "(0.6f)",
          "description": "开环 I/F 拖转电流 A"
        },
        {
          "name": "MOTOR_ALIGN_CURRENT_DEFAULT",
          "value": "(0.5f)",
          "description": "静态 D 轴对齐电流 A"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VBOOST_DEFAULT",
          "value": "(5.0f)",
          "description": "V/F 起步补偿电压 V"
        },
        {
          "name": "MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT",
          "value": "(18.0f)",
          "description": "开环 Vq 上限 V"
        },
        {
          "name": "MOTOR_ENCODER_START_IQ_A",
          "value": "(0.0f)",
          "description": "速度环刚接管时的起步电流 A，0 表示不注入"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_FULL_RPM",
          "value": "(0.0f)",
          "description": "低于此转速给满齿槽助推，RPM"
        },
        {
          "name": "MOTOR_ENCODER_COGGING_ASSIST_RELEASE_RPM",
          "value": "(0.0f)",
          "description": "高于此转速退出齿槽助推，RPM"
        },
        {
          "name": "MOTOR_CURRENT_KP_DEFAULT",
          "value": "(0.12f)",
          "description": "电流环比例，V/A"
        },
        {
          "name": "MOTOR_CURRENT_KI_DEFAULT",
          "value": "(18.0f)",
          "description": "电流环积分，V/(A·s)"
        },
        {
          "name": "MOTOR_SPEED_KP_DEFAULT",
          "value": "(0.003f)",
          "description": "速度比例，A/RPM"
        },
        {
          "name": "MOTOR_SPEED_KI_DEFAULT",
          "value": "(0.01f)",
          "description": "速度积分，A/(RPM·s)"
        },
        {
          "name": "MOTOR_SPEED_IQ_LIMIT_DEFAULT",
          "value": "(2.0f)",
          "description": "速度环电流上限 A"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLTAGE_MODE",
          "value": "(0U)",
          "description": "1：速度环出电压；0：出电流"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_PER_AMP",
          "value": "(0.0f)",
          "description": "电流换成电压，V/A，电压模式才用"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_VOLT_LIMIT_V",
          "value": "(0.0f)",
          "description": "速度环电压上限 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_BOOST_V",
          "value": "(0.0f)",
          "description": "低速固定前馈 V"
        },
        {
          "name": "MOTOR_ENCODER_SPEED_FF_V_PER_RPM",
          "value": "(0.0f)",
          "description": "空载反电势前馈 V/RPM"
        },
        {
          "name": "MOTOR_ENCODER_PLL_BW_RAD_S",
          "value": "(100.0f)",
          "description": "速度 PLL 带宽 rad/s"
        },
        {
          "name": "MOTOR_POSITION_KP_DEFAULT_RPM_PER_DEG",
          "value": "(20.0f * 60.0f / 360.0f)",
          "description": "角度误差换成转速，约 3.33 RPM/deg"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_DEFAULT",
          "value": "(60.0f)",
          "description": "位置运动速度上限 RPM"
        },
        {
          "name": "MOTOR_POSITION_MAX_RPM_ALLOWED",
          "value": "(1800.0f)",
          "description": "允许提到的最高速度 RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KP_DEFAULT",
          "value": "(0.050f)",
          "description": "位置环内的速度比例，V/RPM"
        },
        {
          "name": "MOTOR_POSITION_VEL_KI_DEFAULT",
          "value": "(0.20f)",
          "description": "位置环内的速度积分，V/(RPM·s)"
        },
        {
          "name": "MOTOR_POSITION_VOLTAGE_LIMIT_DEFAULT",
          "value": "(2.0f)",
          "description": "位置环电压上限 V"
        },
        {
          "name": "MOTOR_POSITION_ACCEL_RPM_S_DEFAULT",
          "value": "(30000.0f)",
          "description": "速度变化上限 RPM/s"
        },
        {
          "name": "MOTOR_POSITION_FF_V_PER_RPM_DEFAULT",
          "value": "(0.0f)",
          "description": "位置环速度前馈 V/RPM"
        },
        {
          "name": "MOTOR_POSITION_HOLD_ENTER_DEG",
          "value": "(2.0f)",
          "description": "误差小于此角度进入保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HOLD_EXIT_DEG",
          "value": "(3.0f)",
          "description": "误差大于此角度退出保持，deg"
        },
        {
          "name": "MOTOR_POSITION_HYBRID_SPEED_ENABLE",
          "value": "(0U)",
          "description": "0：禁止低速切电流环"
        },
        {
          "name": "MOTOR_POSITION_SMALL_TEST_SEQUENCE",
          "value": "(0U)",
          "description": "0：不跑 0°→2°→5° 小行程测试"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ENABLE",
          "value": "(0U)",
          "description": "0：大角度不走轨迹"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_VEL_DEG_S",
          "value": "(180.0f)",
          "description": "轨迹最高速度 deg/s"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_ACCEL_DEG_S2",
          "value": "(360.0f)",
          "description": "轨迹加速度 deg/s²"
        },
        {
          "name": "MOTOR_JOINT_TRAJECTORY_DECEL_DEG_S2",
          "value": "(540.0f)",
          "description": "轨迹减速度 deg/s²"
        }
      ],
      "note": "@Description: ATK-BL57H95D20E 电机铭牌与调试默认值\n@Author: lkc\n@Note: 铭牌直接写在 MOTOR_FIXED_*。控制默认值分成起步、电流环、速度环、位置环，\n       和另外两款电机是同一套宏。低速助推尚未上板标定，保持关闭。"
    }
  ],
  "defaults": {
    "ol_pole_pairs": "MOTOR_FIXED_POLE_PAIRS",
    "ol_enable": "false",
    "ol_mode": "MOTOR_OL_MODE_IF",
    "ol_target_rpm": "72.0f",
    "ol_max_rpm": "MOTOR_FIXED_RATED_RPM",
    "ol_v_boost": "MOTOR_OPEN_LOOP_VBOOST_DEFAULT",
    "ol_kv_f": "0.057f",
    "ol_ramp_rpm_s": "10.0f",
    "ol_vq_limit": "MOTOR_OPEN_LOOP_VQ_LIMIT_DEFAULT",
    "ol_foc_freq": "30000.0f",
    "ol_min_rpm": "72.0f",
    "ol_iq_set": "MOTOR_OPEN_LOOP_IQ_DEFAULT",
    "ol_id_set": "0.0f",
    "fw_current_max": "0.0f",
    "fw_duty_start": "0.80f",
    "fw_ramp_a_s": "8.0f",
    "ol_current_kp": "MOTOR_CURRENT_KP_DEFAULT",
    "ol_current_ki": "MOTOR_CURRENT_KI_DEFAULT",
    "ol_start_phase_boost": "M_PI / 2.0f",
    "ol_align_enable": "false",
    "ol_align_current": "MOTOR_ALIGN_CURRENT_DEFAULT",
    "ol_align_time": "0.3f",
    "ol_align_angle": "0.0f",
    "hfi_start_enable": "false",
    "hfi_start_voltage": "2.0f",
    "hfi_start_cycles": "8U",
    "hfi_start_min_confidence": "0.05f",
    "speed_enable": "false",
    "speed_allow_braking": "false",
    "speed_kp": "MOTOR_SPEED_KP_DEFAULT",
    "speed_ki": "MOTOR_SPEED_KI_DEFAULT",
    "speed_iq_limit": "MOTOR_SPEED_IQ_LIMIT_DEFAULT",
    "speed_loop_hz": "1000.0f",
    "sensorless_enable": "(MOTOR_SENSOR_TYPE == MOTOR_SENSOR_SENSORLESS)",
    "sensorless_takeover": "false",
    "sensorless_motor_r": "MOTOR_FIXED_LINE_RESISTANCE",
    "sensorless_motor_l": "MOTOR_FIXED_LINE_INDUCTANCE",
    "sensorless_flux_linkage": "MOTOR_FIXED_FLUX_LINKAGE",
    "sensorless_observer_gain": "12000000.0f",
    "sensorless_pll_kp": "2000.0f",
    "sensorless_pll_ki": "30000.0f",
    "sensorless_transition_rpm": "500.0f",
    "sensorless_fallback_rpm": "300.0f",
    "sensorless_openloop_max_rpm": "800.0f",
    "sensorless_lock_time": "0.10f",
    "sensorless_loss_time": "0.10f",
    "sensorless_blend_time": "0.20f",
    "sensorless_phase_tolerance": "1.40f",
    "sensorless_phase_offset": "0.0f",
    "sensorless_speed_filter": "0.10f",
    "hall_extra_samples": "1"
  },
  "baud": 2000000,
  "commandBytes": 159,
  "maxArgs": 8
};
