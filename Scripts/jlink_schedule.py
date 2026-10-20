"""Export frozen scheduler statistics using read-only SWD, after CLI stop/debug."""
import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import tempfile
from datetime import datetime
from pathlib import Path

from compact_trace import SnapshotChangedError
from jlink_fault_trace import check_ram, read_script

SNAPSHOT = struct.Struct('<70I')
TASK_FIELDS = ('task_count', 'task_max_gap_us', 'pi_count', 'pi_no_update',
               'pi_max_gap_us', 'pi_over_1ms', 'pi_streak_max', 'wake_late_max_ticks')
WAKE_FIELDS = ('valid', 'delay_us', 'dma_count', 'dma_us')
IRQ_FIELDS = ('seen', 'count', 'sum_us', 'max_us', 'first_us', 'last_us',
              'tick_pending', 'switch_pending')


def snapshot_address(map_text):
    matches = re.findall(r'^\s*g_motor_schedule_debug\s+0x([0-9a-fA-F]+)\s+Data\s+280\s', map_text, re.M)
    if len(matches) != 1:
        raise ValueError('Expected one 280-byte scheduler symbol in current firmware map')
    address = int(matches[0], 16)
    check_ram(address, SNAPSHOT.size)
    return address


def decode_snapshot(raw):
    if len(raw) != SNAPSHOT.size:
        raise ValueError('Truncated scheduler snapshot')
    words = SNAPSHOT.unpack(raw)
    if words[0] != 0x464f4353 or words[1] != 3 or words[2] & 1 or words[3] != 2:
        raise ValueError('Unsupported or unfrozen scheduler snapshot')
    if words[4] & ~15:
        raise ValueError('Unknown capture mode')
    gap_us = (words[54] - words[55]) & 0xffffffff
    if words[64] not in (0, 1) or (words[64] and sum(words[65:70]) != gap_us):
        raise ValueError('Invalid or inconsistent event phase breakdown')
    intervals = sum(words[47:53])
    over_1ms = sum(words[48:53])
    return {
        'version': words[1], 'sequence': words[2], 'mode_bits': words[4],
        'task': dict(zip(TASK_FIELDS, words[5:13])),
        'phase_max_us': dict(zip(('wait', 'position', 'speed', 'publish'), words[13:17])),
        'pi_gap_bins': dict(zip(('le1000', 'le1100', 'le1250', 'le1500', 'le2000', 'gt2000'), words[17:23])),
        'worst_pi_wake': dict(zip(WAKE_FIELDS, words[23:27])),
        'worst_wake': dict(zip(WAKE_FIELDS, words[27:31])),
        'dma': dict(zip(IRQ_FIELDS, words[31:39])),
        'systick': dict(zip(IRQ_FIELDS, words[39:47])),
        'task_gap_bins': dict(zip(('le1000', 'le1100', 'le1250', 'le1500', 'le2000', 'gt2000'), words[47:53])),
        'task_gap_summary': {'intervals': intervals, 'over_1ms': over_1ms,
                             'over_1ms_percent': 100.0 * over_1ms / intervals if intervals else None},
        'worst_task_gap': {
            'captured': words[6] > 0,
            'sequence': words[53], 'time_us': words[54], 'previous_us': words[55],
            'gap_us': gap_us,
            'wake': dict(zip(WAKE_FIELDS, words[56:60])),
            'previous_wake': dict(zip(WAKE_FIELDS, words[60:64])),
            'phase_valid': bool(words[64]), 'before_wait_us': words[65],
            'phase_us': dict(zip(('wait', 'position', 'speed', 'completion_snapshot'), words[66:70])),
        },
        'limitations': 'IRQ stride=31; sampled sums are not total occupancy; SysTick includes preemption; '
                        'wake DMA time excludes exception overhead; maxima need not belong to the same event; '
                        'zero counters in disabled modes do not prove zero latency',
    }


def read_snapshot(read_memory, address):
    check_ram(address, SNAPSHOT.size)
    first = read_memory(address, SNAPSHOT.size)
    report = decode_snapshot(first)
    if read_memory(address, SNAPSHOT.size) != first:
        raise SnapshotChangedError('Scheduler snapshot changed between reads')
    return first, report


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jlink', default=shutil.which('JLink.exe') or shutil.which('JLinkExe'))
    parser.add_argument('--map', type=Path, default=root.parent / 'build' / 'foc.map')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not args.jlink or not Path(args.jlink).is_file():
        parser.error('Provide --jlink with the J-Link Commander executable path')
    map_data = args.map.read_bytes()
    address = snapshot_address(map_data.decode('utf-8', errors='replace'))
    prefix = (args.output or root / 'Doc' / 'verify' /
              ('jlink_schedule_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))).resolve()
    if any(Path(str(prefix) + suffix).exists() for suffix in ('.bin', '.json', '.log')):
        parser.error('Output exists; refusing to overwrite')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='foc_schedule_') as folder, \
            Path(str(prefix) + '.log').open('x', encoding='utf-8') as log:
        def read_memory(target_address, size):
            output = Path(folder) / 'read.bin'
            if output.exists():
                output.unlink()
            script = read_script(output, target_address, size)
            commands = Path(folder) / 'read.jlink'
            commands.write_text(script, encoding='utf-8')
            result = subprocess.run([
                str(args.jlink), '-device', 'STM32F405RG', '-if', 'SWD', '-speed', '100',
                '-autoconnect', '1', '-ExitOnError', '1', '-CommanderScript', str(commands)
            ], capture_output=True, timeout=60)
            log.write(script + result.stdout.decode(errors='replace') + result.stderr.decode(errors='replace'))
            log.flush()
            if result.returncode or not output.is_file() or output.stat().st_size != size:
                raise RuntimeError('J-Link read failed or short dump')
            return output.read_bytes()
        raw, report = read_snapshot(read_memory, address)
        runtime_match = re.findall(r'^\s*g_motor_task_runtime_debug\s+0x([0-9a-fA-F]+)\s+Data\s+40\s',
                                   map_data.decode('utf-8', errors='replace'), re.M)
        if len(runtime_match) == 1:
            runtime_address = int(runtime_match[0], 16)
            check_ram(runtime_address, 40)
            runtime_raw = read_memory(runtime_address, 40)
            if read_memory(runtime_address, 40) != runtime_raw:
                raise SnapshotChangedError('Task runtime changed between reads')
            count, minimum, maximum, last, total, dma_total, max_dma, max_dma_count = struct.unpack('<4I2Q2I', runtime_raw)
            report['task_runtime_wall_us'] = {
                'count': count, 'min': minimum, 'max': maximum, 'last': last,
                'sum': total, 'mean': total / count if count else None,
                'dma_sum': dma_total, 'dma_mean': dma_total / count if count else None,
                'max_wall_dma_us': max_dma, 'max_wall_dma_count': max_dma_count,
                'scope': 'After periodic sleep through task bookkeeping; includes IRQ preemption; excludes end counter update',
            }
    report.update(address=address, matching_reads=2, map_sha256=hashlib.sha256(map_data).hexdigest(),
                  snapshot_sha256=hashlib.sha256(raw).hexdigest(),
                  access='SWD SRAM reads only; no halt/reset/write; export after CLI stop/debug')
    Path(str(prefix) + '.bin').write_bytes(raw)
    Path(str(prefix) + '.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Validated scheduler snapshot: {prefix}.json')


if __name__ == '__main__':
    main()
