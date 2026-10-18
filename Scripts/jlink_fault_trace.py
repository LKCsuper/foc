"""Read a frozen FOC RAM ring through J-Link/SWD without halt, reset or target writes."""
import argparse
import hashlib
import json
import math
import re
import shutil
import struct
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

from compact_trace import SnapshotChangedError, crc32_msb
from fault_trace import HEADER

DESCRIPTOR = struct.Struct('<14I')
SAMPLE = struct.Struct('<II4H17f')
FIELDS = ('magic', 'version', 'sequence', 'state', 'address', 'sample_bytes',
          'capacity', 'stride', 'head', 'count', 'fault', 'first_us', 'last_us', 'descriptor_bytes')


def check_ram(address, size):
    if address % 4 or size <= 0 or not 0x20000000 <= address < address + size <= 0x20020000:
        raise ValueError('Read outside STM32F405 normal SRAM or unaligned address')


def descriptor_address(map_text):
    matches = re.findall(r'^\s*g_motor_fault_debug\s+0x([0-9a-fA-F]+)\s+Data\s+56\s', map_text, re.M)
    if len(matches) != 1:
        raise ValueError('Expected one 56-byte g_motor_fault_debug symbol in current firmware map')
    address = int(matches[0], 16)
    check_ram(address, DESCRIPTOR.size)
    return address


def parse_descriptor(raw):
    if len(raw) != DESCRIPTOR.size:
        raise ValueError('Truncated debug descriptor')
    info = dict(zip(FIELDS, DESCRIPTOR.unpack(raw)))
    if (info['magic'] != 0x464f4344 or info['version'] != 1 or
            info['descriptor_bytes'] != DESCRIPTOR.size or info['sample_bytes'] != SAMPLE.size or
            info['capacity'] != 512 or info['stride'] != 4):
        raise ValueError('Unsupported debug ABI; check firmware/map pairing')
    if info['sequence'] & 1 or info['state'] != 2:
        raise ValueError('Trace is not a stable frozen snapshot; no data read')
    if not 0 < info['count'] <= info['capacity'] or info['head'] >= info['capacity']:
        raise ValueError('Empty or invalid ring geometry')
    check_ram(info['address'], info['capacity'] * info['sample_bytes'])
    if info['fault'] > 0xffff:
        raise ValueError('Invalid fault code')
    return info


def decode_ring(info, data):
    if len(data) != info['capacity'] * SAMPLE.size:
        raise ValueError('Truncated ring data')
    first = (info['head'] - info['count']) % info['capacity']
    rows = []
    stamps = []
    for sequence in range(info['count']):
        offset = ((first + sequence) % info['capacity']) * SAMPLE.size
        sample = SAMPLE.unpack_from(data, offset)
        stamp, flags, adc0, adc1, adc2, reserved = sample[:6]
        floats = sample[6:]
        if reserved or max(adc0, adc1, adc2) > 4095 or not all(map(math.isfinite, floats)) or floats[-1] < 0:
            raise ValueError('Invalid raw sample')
        stamps.append(stamp)
        fields = [str(sequence), str(stamp), str(flags), str(adc0), str(adc1), str(adc2)]
        fields.extend(format(value, '.9g') for value in floats)
        payload = ','.join(fields)
        rows.append(f'{payload},{crc32_msb(payload.encode("ascii")):08x}\n')
    if (stamps[0] != info['first_us'] or stamps[-1] != info['last_us'] or
            any(not 0 < ((current - previous) & 0xffffffff) < 0x80000000
                for previous, current in zip(stamps, stamps[1:]))):
        raise SnapshotChangedError('Timestamp sequence or frozen endpoints disagree')
    if (SAMPLE.unpack_from(data, ((info['head'] - 1) % info['capacity']) * SAMPLE.size)[1] >> 16) != info['fault']:
        raise ValueError('Last record fault differs from descriptor')
    return HEADER + '\n' + ''.join(rows)


def read_frozen(read_memory, address):
    check_ram(address, DESCRIPTOR.size)
    before = read_memory(address, DESCRIPTOR.size)
    info = parse_descriptor(before)
    size = info['capacity'] * info['sample_bytes']
    first = read_memory(info['address'], size)
    middle = read_memory(address, DESCRIPTOR.size)
    if middle != before:
        raise SnapshotChangedError('Descriptor changed during first dump')
    second = read_memory(info['address'], size)
    after = read_memory(address, DESCRIPTOR.size)
    if after != before or second != first:
        raise SnapshotChangedError('Frozen data changed between independent reads')
    csv = decode_ring(info, first)
    return info, before, first, csv


def read_script(path, address, size):
    check_ram(address, size)
    filename = str(Path(path).resolve())
    if any(char in filename for char in ('"', '\r', '\n')):
        raise ValueError('Unsafe output filename')
    return f'connect\nsavebin "{filename}" 0x{address:08x} 0x{size:x}\nq\n'


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jlink', default=shutil.which('JLink.exe') or shutil.which('JLinkExe'))
    parser.add_argument('--map', type=Path, default=root.parent / 'build' / 'foc.map')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not args.jlink or not Path(args.jlink).is_file():
        parser.error('Provide --jlink with the J-Link Commander executable path')
    address = descriptor_address(args.map.read_text(encoding='utf-8', errors='replace'))
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    prefix = (args.output or root / 'Doc' / 'verify' / f'jlink_fault_{stamp}').resolve()
    suffixes = ('.descriptor.bin', '.ring.bin', '.csv', '.json', '.log')
    if any(Path(str(prefix) + suffix).exists() for suffix in suffixes):
        parser.error('Output prefix already exists; refusing to overwrite')
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='foc_jlink_') as folder, Path(str(prefix) + '.log').open('w', encoding='utf-8') as log:
        def read_memory(target_address, size):
            path = Path(folder) / 'read.bin'
            if path.exists():
                path.unlink()
            command_file = Path(folder) / 'read.jlink'
            script = read_script(path, target_address, size)
            command_file.write_text(script, encoding='utf-8')
            result = subprocess.run([
                str(args.jlink), '-device', 'STM32F405RG', '-if', 'SWD', '-speed', '100',
                '-autoconnect', '1', '-ExitOnError', '1', '-CommanderScript', str(command_file)
            ], capture_output=True, timeout=60)
            log.write(script + result.stdout.decode(errors='replace') + result.stderr.decode(errors='replace'))
            log.flush()
            if result.returncode or not path.is_file() or path.stat().st_size != size:
                raise RuntimeError('J-Link read failed or short dump; no validated CSV emitted')
            return path.read_bytes()

        info, descriptor, data, csv = read_frozen(read_memory, address)
    Path(str(prefix) + '.descriptor.bin').write_bytes(descriptor)
    Path(str(prefix) + '.ring.bin').write_bytes(data)
    Path(str(prefix) + '.csv').write_text(csv, encoding='utf-8')
    report = {
        'descriptor': info, 'descriptor_address': address,
        'map_sha256': hashlib.sha256(args.map.read_bytes()).hexdigest(),
        'ring_sha256': hashlib.sha256(data).hexdigest(), 'matching_reads': 2,
        'csv_crc_origin': 'host-generated, not target transport CRC',
        'timing': 'ADC/abc/vbus/dt=current entry; prev_* fields=previous control; not ADC aperture timestamp',
        'access': 'SWD SRAM reads only; no halt/reset/write commands; export after motor stop',
    }
    Path(str(prefix) + '.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Validated frozen samples: {info["count"]}; CSV: {prefix}.csv')


if __name__ == '__main__':
    main()
