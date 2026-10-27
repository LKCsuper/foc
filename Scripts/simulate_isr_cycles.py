"""用本机 Keil 软件模拟器估算 ISR 指令周期；不连接或烧写硬件。"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
LABELS = ["空夹具基线", "跳过控制拍", "停机完整拍", "V/F 电压开环", "I/F 电流环（开环角）",
          "I/F 电流环＋无感观测", "故障已锁存", "无感 I/F＋细分计时", "无感 I/F（性能统计关闭）"]


def configure(out, samples):
    """从正式工程派生独立软件仿真工程，保留目标编译选项。"""
    out.mkdir(parents=True, exist_ok=True)
    source = ROOT / 'App/Project/foc.uvprojx'
    tree = ET.parse(source)
    project = tree.getroot()
    target = project.find('./Targets/Target')
    for tag, value in {'OutputDirectory': str(out / 'Objects') + '\\', 'ListingPath': str(out / 'Objects') + '\\',
                       'OutputName': 'isr_sim', 'CreateHexFile': '0'}.items():
        for node in project.iter(tag):
            node.text = value
    for node in project.iter('Cpu'):
        node.text = re.sub(r'CLOCK\(\d+\)', 'CLOCK(168000000)', node.text)
    for node in project.iter('IncludePath'):
        if node.text:
            node.text = ';'.join(str((source.parent / x.replace('\\', '/')).resolve())
                                 for x in node.text.split(';') if x) + ';' + str(ROOT / 'App/motor/diagnostic')
    excluded = {ROOT / 'App/user/main.c', ROOT / 'App/user/system_stm32f4xx.c'}
    for files in project.iter('Files'):
        for item in list(files):
            node = item.find('FilePath')
            if node is None or not node.text:
                continue
            path = (source.parent / node.text.replace('\\', '/')).resolve()
            if path in excluded:
                files.remove(item)
            else:
                node.text = str(path)
    group = ET.SubElement(target.find('Groups'), 'Group')
    ET.SubElement(group, 'GroupName').text = 'Simulation'
    item = ET.SubElement(ET.SubElement(group, 'Files'), 'File')
    ET.SubElement(item, 'FileName').text = 'isr_cycle_bench.c'
    ET.SubElement(item, 'FileType').text = '1'
    ET.SubElement(item, 'FilePath').text = str(ROOT / 'Tests/perf/isr_cycle_bench.c')
    definitions = target.find('TargetOption/TargetArmAds/Cads/VariousControls/Define')
    definitions.text = (definitions.text or '') + ',SIM_SAMPLES=' + str(samples)
    tree.write(out / 'isr_sim.uvprojx', encoding='utf-8', xml_declaration=True)
    options = ET.parse(ROOT / 'App/Project/foc.uvoptx')
    opt = options.getroot()
    for tag, value in {'RunSim': '1', 'RunTarget': '0', 'uSim': '1', 'uTrg': '0', 'sGomain': '0',
                       'sIfile': str(out / 'sim.ini'), 'CLKADS': '168000000', 'sRunDeb': '0',
                       'tLdApp': '0', 'tGomain': '0'}.items():
        for node in opt.iter(tag):
            node.text = value
    for node in list(opt):
        if node.tag == 'Group':
            opt.remove(node)
    options.write(out / 'isr_sim.uvoptx', encoding='utf-8', xml_declaration=True)
    # LOG 使用工作目录下的相对文件名，规避 µVision 对带引号绝对日志路径的解析差异。
    commands = ['LOG > full.log', 'MAP 0x10000000, 0x1000FFFF READ WRITE EXEC',
                'MAP 0x40000000, 0x400FFFFF READ WRITE', 'RESET']
    for _ in range(len(LABELS) * samples):
        commands.extend(['G, Sim_Begin',
            'printf("BEGIN case=%u iter=%u states=%u\\n", sim_case, sim_iteration, states)',
            'G, Sim_End', 'printf("END case=%u iter=%u states=%u\\n", sim_case, sim_iteration, states)',
            'G, Sim_After',
            'printf("META case=%u iter=%u fault=%u vbus=%f iqtarget=%f modq=%f profile=%u hash=%u\\n", '
            'sim_case, sim_iteration, sim_fault, sim_vbus, motor.iq_target, motor.modQ, sim_profile.count[0], sim_control_hash)'])
    commands.extend(['LOG OFF', 'EXIT'])
    (out / 'sim.ini').write_text('\n'.join(commands) + '\n', encoding='utf-8')


def invoke(uv4, out, args):
    """隐藏运行本机已安装的 Keil；超时仅结束本脚本启动的进程。"""
    info = subprocess.STARTUPINFO()
    info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    info.wShowWindow = subprocess.SW_HIDE
    result = subprocess.run([str(uv4), *args], cwd=out, startupinfo=info, timeout=180)
    if result.returncode != 0:
        raise RuntimeError(f'Keil returned {result.returncode}; inspect logs in {out}')


def summarize(out, samples):
    """解析配对断点计数；发现脚本错误、丢样或意外故障时拒绝给出成功结果。"""
    log = (out / 'full.log').read_text(encoding='utf-8', errors='replace')
    if re.search(r'\*\*\* error|access violation', log, re.I):
        raise RuntimeError('Simulator errors in full.log')
    starts, records = {}, {}
    for line in log.splitlines():
        match = re.match(r'BEGIN case=(\d+) iter=(\d+) states=(\d+)', line)
        if match:
            starts[int(match[1]), int(match[2])] = int(match[3])
        match = re.match(r'END case=(\d+) iter=(\d+) states=(\d+)', line)
        if match:
            key = int(match[1]), int(match[2])
            if key in records:
                raise RuntimeError('Duplicate sample')
            records[key] = {'case': key[0], 'iteration': key[1],
                            'raw_cycles': int(match[3]) - starts[key]}
        match = re.match(r'META case=(\d+) iter=(\d+) fault=(\d+) vbus=([\d.]+) '
                         r'iqtarget=([\d.eE+-]+) modq=([\d.eE+-]+) profile=(\d+) hash=(\d+)', line)
        if match:
            key = int(match[1]), int(match[2])
            records[key].update(fault=int(match[3]), vbus=float(match[4]),
                                iq_target=float(match[5]), mod_q=float(match[6]), profile_count=int(match[7]), control_hash=int(match[8]))
    expected = {(case, index) for case in range(len(LABELS)) for index in range(samples)}
    if set(records) != expected or any('fault' not in row for row in records.values()):
        raise RuntimeError('Incomplete simulation samples or metadata')
    if any(row['fault'] != (4 if row['case'] == 6 else 0) for row in records.values()):
        raise RuntimeError('Unexpected fault state')
    baseline = statistics.median(row['raw_cycles'] for row in records.values() if row['case'] == 0)
    summary = []
    for case, label in enumerate(LABELS):
        rows = [records[case, i] for i in range(samples)]
        cycles = [row['raw_cycles'] - baseline for row in rows]
        for row, value in zip(rows, cycles):
            row['corrected_cycles'] = value
        summary.append(dict(case=case, label=label, samples=samples, min_cycles=min(cycles),
                            mean_cycles=statistics.mean(cycles), max_cycles=max(cycles),
                            min_us=min(cycles)/168, mean_us=statistics.mean(cycles)/168, max_us=max(cycles)/168))
    axf = out / 'Objects/isr_sim.axf'
    result = dict(clock_hz=168000000, baseline_cycles=baseline, summary=summary,
                  image_sha256=hashlib.sha256(axf.read_bytes()).hexdigest(),
                  scope='IRQ function call chain; excludes hardware exception entry/exit and memory-system stalls')
    (out / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    with (out / 'samples.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0, 0]))
        writer.writeheader()
        writer.writerows(records[key] for key in sorted(records))
    for row in summary:
        print(f"case {row['case']}: {row['min_cycles']:.0f} / {row['mean_cycles']:.2f} / "
              f"{row['max_cycles']:.0f} cycles; max {row['max_us']:.3f} us")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uv4', type=Path, default=os.environ.get('KEIL_UV4'))
    parser.add_argument('--samples', type=int, default=512)
    parser.add_argument('--output', type=Path, default=ROOT / '.build/isr_sim')
    parser.add_argument('--summarize-only', action='store_true')
    parser.add_argument('--compare', type=Path, help='优化前 samples.csv；逐拍核对控制摘要与故障状态')
    args = parser.parse_args()
    if not 256 <= args.samples <= 4096:
        parser.error('--samples must be between 256 and 4096 to include periodic profiling')
    out = args.output.resolve()
    if not args.summarize_only:
        if args.uv4 is None or not args.uv4.is_file():
            parser.error('Provide an installed UV4.exe using --uv4 or KEIL_UV4')
        configure(out, args.samples)
        invoke(args.uv4.resolve(), out, ['-b', 'isr_sim.uvprojx', '-j0', '-o', 'build-full.log'])
        # Only software simulation (-d); never issue Flash download (-f).
        invoke(args.uv4.resolve(), out, ['-d', 'isr_sim.uvprojx', '-j0'])
    summarize(out, args.samples)
    if args.compare:
        with args.compare.open(encoding='utf-8', newline='') as stream:
            before = {(r['case'], r['iteration']): r for r in csv.DictReader(stream)}
        with (out / 'samples.csv').open(encoding='utf-8', newline='') as stream:
            after = {(r['case'], r['iteration']): r for r in csv.DictReader(stream)}
        if before.keys() != after.keys() or any(
                before[key][field] != after[key][field]
                for key in before for field in ('fault', 'control_hash')):
            raise RuntimeError('优化前后采样集合、故障状态或控制摘要不一致')
        print(f'Control comparison: {len(after)} samples match')


if __name__ == '__main__':
    main()
