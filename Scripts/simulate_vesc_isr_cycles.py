"""在 Keil 中执行已有 VESC HD75 ELF 的 ADC 回调；保持第三方源码与固件镜像只读。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess
import xml.etree.ElementTree as ET
from simulate_isr_cycles import ROOT, invoke

LABELS = ['空夹具', 'V7 跳过回调', '停机回调', '开环电流（含观测器）', '无感电流控制', '无感电流＋触发性能采样']


def run_logged(args, log):
    """执行已安装工具并保存诊断，不改工具链或源目录。"""
    result = subprocess.run(args, capture_output=True, text=True, errors='replace', timeout=90)
    log.write_text(result.stdout + result.stderr, encoding='utf-8')
    if result.returncode:
        raise RuntimeError(f'Tool failed: {log}')


def prepare(out):
    """推导私有状态地址、验证中断代码来源，并生成仅用于软件仿真的工程。"""
    out.mkdir(parents=True, exist_ok=True)
    vesc = ROOT / 'ThirtyLibrary/bldc'
    image = vesc / 'build/hd75/hd75.elf'
    old_object = vesc / 'build/hd75/obj/mcpwm_foc.o'
    map_path = vesc / 'build/hd75/hd75.map'
    tools = vesc / 'tools/gcc-arm-none-eabi-7-2018-q2-update/bin'
    gcc = tools / 'arm-none-eabi-gcc.exe'
    objcopy = tools / 'arm-none-eabi-objcopy.exe'
    for path in [image, old_object, map_path, gcc, objcopy]:
        if not path.is_file():
            raise RuntimeError(f'Required existing VESC artifact/tool missing: {path}')
    mapping = map_path.read_text(encoding='utf-8', errors='replace')

    def address(section, object_name):
        pattern = re.escape(section) + r'\s*\n\s*(0x[\da-fA-F]+)\s+(0x[\da-fA-F]+)\s+\S*[/\\]' + re.escape(object_name)
        found = re.search(pattern, mapping)
        if not found:
            raise RuntimeError(f'Cannot uniquely locate {section} from {object_name}')
        return int(found[1], 16), int(found[2], 16)

    motor, size = address('.bss.m_motor_1', 'mcpwm_foc.o')
    config, _ = address('.bss.m_motor_1', 'mc_interface.o')
    init_done, _ = address('.bss.m_init_done', 'mcpwm_foc.o')
    (out / 'vesc_symbols.h').write_text(
        f'#define VESC_MOTOR_ADDR 0x{motor:x}U\n#define VESC_CONFIG_ADDR 0x{config:x}U\n'
        f'#define VESC_INIT_DONE_ADDR 0x{init_done:x}U\n#define VESC_STATE_SIZE {size}U\n', encoding='ascii')
    dep = (vesc / 'build/hd75/.dep/mcpwm_foc.o.d').read_text(encoding='utf-8')
    includes = {vesc, vesc / 'motor', vesc / 'util', vesc / 'driver', vesc / 'applications',
                vesc / 'hwconf/trampa/hd', out}
    for word in dep.replace('\\\n', ' ').split():
        path = vesc / word
        if path.is_file() and path.suffix == '.h':
            includes.add(path.parent)
    common = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16',
              '-O2', '-g', '-std=gnu99', '-ffunction-sections', '-fdata-sections',
              '-fsingle-precision-constant', '-DSTM32F407xx=', '-DHW_HEADER="hw_hd75.h"',
              '-DHW_SOURCE="hw_hd75.c"', *['-I' + str(path) for path in sorted(includes)]]
    current_object = out / 'mcpwm_foc_current.o'
    run_logged([str(gcc), *common, '-fomit-frame-pointer', '-falign-functions=16', '-fno-common',
                '-c', str(vesc / 'motor/mcpwm_foc.c'), '-o', str(current_object)], out / 'source-check.log')
    checks = {}
    for section in ['.text.mcpwm_foc_adc_int_handler', '.text.update_valpha_vbeta.constprop.7']:
        old_bytes = out / (section.rsplit('.', 1)[-1] + '-old.bin')
        new_bytes = out / (section.rsplit('.', 1)[-1] + '-new.bin')
        for source, destination in [(old_object, old_bytes), (current_object, new_bytes)]:
            run_logged([str(objcopy), '--dump-section', section + '=' + str(destination), str(source),
                        str(out / 'section-copy.o')], out / 'objcopy.log')
        if not old_bytes.is_file() or not new_bytes.is_file() or old_bytes.read_bytes() != new_bytes.read_bytes():
            raise RuntimeError(f'Existing VESC object does not match current source: {section}')
        checks[section] = len(old_bytes.read_bytes())
    run_logged([str(gcc), *common, str(ROOT / 'Tests/perf/vesc_isr_cycle_bench.c'), '-nostdlib',
                '-Wl,--just-symbols=' + str(image), '-Wl,-Ttext=0x080e0000,-Tdata=0x2001c000,-e,VMain',
                '-o', str(out / 'bench.elf')], out / 'compile.log')
    (out / 'bench.axf').write_bytes((out / 'bench.elf').read_bytes())
    tree = ET.parse(ROOT / 'App/Project/foc.uvprojx')
    for tag, value in {'OutputDirectory': str(out) + '\\', 'ListingPath': str(out) + '\\', 'OutputName': 'bench'}.items():
        for node in tree.getroot().iter(tag):
            node.text = value
    for node in tree.getroot().iter('Cpu'):
        node.text = re.sub(r'CLOCK\(\d+\)', 'CLOCK(168000000)', node.text)
    tree.getroot().find('./Targets/Target/Groups').clear()
    tree.write(out / 'vesc.uvprojx', encoding='utf-8', xml_declaration=True)
    tree = ET.parse(ROOT / 'App/Project/foc.uvoptx')
    for tag, value in {'RunSim': '1', 'RunTarget': '0', 'uSim': '1', 'uTrg': '0', 'sLdApp': '1',
                       'sGomain': '0', 'sRunDeb': '0', 'sIfile': str(out / 'vesc.ini'), 'CLKADS': '168000000'}.items():
        for node in tree.getroot().iter(tag):
            node.text = value
    for node in list(tree.getroot()):
        if node.tag == 'Group':
            tree.getroot().remove(node)
    tree.write(out / 'vesc.uvoptx', encoding='utf-8', xml_declaration=True)
    commands = ['LOG > vesc.log', 'MAP 0x10000000, 0x1000FFFF READ WRITE EXEC',
                'MAP 0x20000000, 0x2001FFFF READ WRITE EXEC', 'MAP 0x40000000, 0x400FFFFF READ WRITE',
                f'LOAD "{image.as_posix()}" INCREMENTAL', f'LOAD "{(out / "bench.elf").as_posix()}" INCREMENTAL',
                '_WDWORD(0xE000ED88,0x00F00000)', 'XPSR=0x01000000', 'SP=0x2001FF00', 'PC=VMain']
    # GCC may save FP registers in VMain's prologue, so CPACR must be enabled before entering VMain.
    for _ in range(6 * 512):
        commands += ['G, VBegin', 'printf("BEGIN case=%u iter=%u states=%u\\n",v_case,v_iter,states)',
                     'G, VEnd', 'printf("END case=%u iter=%u states=%u\\n",v_case,v_iter,states)',
                     'G, VAfter', 'printf("META case=%u iter=%u fault=%u bus=%f iq=%f\\n",v_case,v_iter,v_fault,v_vbus,v_iq)']
    commands += ['LOG OFF', 'EXIT']
    (out / 'vesc.ini').write_text('\n'.join(commands) + '\n', encoding='utf-8')
    metadata = dict(image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(), object_code_checks=checks,
                    vesc_commit=subprocess.check_output(['git', '-C', str(vesc), 'rev-parse', 'HEAD'], text=True).strip())
    (out / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')


def summarize(out):
    """校验样本、故障与有效目标，输出周期汇总和原始样本。"""
    log = (out / 'vesc.log').read_text(encoding='utf-8', errors='replace')
    if re.search(r'\*\*\* error|access violation', log, re.I):
        raise RuntimeError('Simulator error; inspect vesc.log')
    begin, rows = {}, {}
    for line in log.splitlines():
        match = re.match(r'BEGIN case=(\d+) iter=(\d+) states=(\d+)', line)
        if match:
            begin[int(match[1]), int(match[2])] = int(match[3])
        match = re.match(r'END case=(\d+) iter=(\d+) states=(\d+)', line)
        if match:
            key = int(match[1]), int(match[2])
            rows[key] = dict(case=key[0], iteration=key[1], raw_cycles=int(match[3]) - begin[key])
        match = re.match(r'META case=(\d+) iter=(\d+) fault=(\d+) bus=([\d.eE+-]+) iq=([\d.eE+-]+)', line)
        if match:
            rows[int(match[1]), int(match[2])].update(fault=int(match[3]), vbus=float(match[4]), iq_target=float(match[5]))
    expected = {(case, i) for case in range(6) for i in range(512)}
    if set(rows) != expected or any(row.get('fault') != 0 for row in rows.values()):
        raise RuntimeError('Incomplete samples or unexpected faults')
    if any(row.get('iq_target') != 0.6 for row in rows.values() if row['case'] >= 3):
        raise RuntimeError('Current target was not active')
    baseline = statistics.median(rows[0, i]['raw_cycles'] for i in range(512))
    summary = []
    for case in range(6):
        values = [rows[case, i]['raw_cycles'] - baseline for i in range(512)]
        for i, value in enumerate(values):
            rows[case, i]['corrected_cycles'] = value
        summary.append(dict(case=case, label=LABELS[case], samples=512, min_cycles=min(values),
                            mean_cycles=statistics.mean(values), max_cycles=max(values),
                            mean_us=statistics.mean(values)/168, max_us=max(values)/168))
    result = dict(baseline_cycles=baseline, summary=summary)
    if (out / 'metadata.json').exists():
        result.update(json.loads((out / 'metadata.json').read_text(encoding='utf-8')))
    (out / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    with (out / 'samples.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0, 0]))
        writer.writeheader()
        writer.writerows(rows[key] for key in sorted(rows))
    for row in summary:
        print(f"case {row['case']}: {row['mean_cycles']:.2f} cycles avg, {row['max_cycles']:.0f} max; "
              f"{row['mean_us']:.3f} us avg, {row['max_us']:.3f} us max")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uv4', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT / '.build/vesc_isr_sim')
    parser.add_argument('--summarize-only', action='store_true')
    args = parser.parse_args()
    out = args.output.resolve()
    if not args.summarize_only:
        prepare(out)
        invoke(args.uv4.resolve(), out, ['-d', 'vesc.uvprojx', '-j0'])
    summarize(out)


if __name__ == '__main__':
    main()
