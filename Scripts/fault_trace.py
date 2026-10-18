"""Export a stopped fault ring with bounded retries and CRC validation."""
import argparse
import math
import re
from datetime import datetime
from pathlib import Path

from compact_trace import SnapshotChangedError, crc32_msb

HEADER = ('seq,time_us,flags,adc0,adc1,adc2,ia,ib,ic,prev_theta,prev_pll,prev_flux,'
          'prev_rpm,prev_v_alpha,prev_v_beta,prev_vd_int,prev_vq_int,prev_id,prev_iq,'
          'prev_id_cmd,prev_iq_cmd,vbus,dt,crc32')


def parse_export(reply):
    lines = reply.splitlines()
    count = re.search(r'(?m)^trace samples=(\d+)\s*$', reply)
    if not count or HEADER not in lines or 'trace format=fault stride=4;' not in reply:
        raise ValueError('Missing fault trace count, format or header')
    expected = int(count[1])
    if not 0 < expected <= 512:
        raise ValueError('Empty or invalid fault trace count')
    rows = {}
    rejected = 0
    for line in lines[lines.index(HEADER) + 1:]:
        if not re.match(r'^\d+,', line):
            continue
        try:
            payload, checksum = line.rsplit(',', 1)
            if not re.fullmatch(r'[0-9a-fA-F]{8}', checksum):
                raise ValueError('Invalid CRC field')
            if crc32_msb(payload.encode('ascii')) != int(checksum, 16):
                raise ValueError('CRC mismatch')
            fields = payload.split(',')
            if len(fields) != 23:
                raise ValueError('Invalid column count')
            if any(not re.fullmatch(r'\d+', field) for field in fields[:6]):
                raise ValueError('Invalid integer encoding')
            numbers = list(map(float, fields))
            if not all(map(math.isfinite, numbers)):
                raise ValueError('Nonfinite value')
            for index, maximum in enumerate((expected - 1, 0xffffffff, 0xffffffff, 4095, 4095, 4095)):
                if numbers[index] != int(numbers[index]) or not 0 <= numbers[index] <= maximum:
                    raise ValueError('Invalid integer field')
            if numbers[-1] < 0:
                raise ValueError('Negative dt')
            sequence = int(numbers[0])
        except (ValueError, UnicodeError):
            rejected += 1
            continue
        if sequence in rows and rows[sequence] != payload:
            raise SnapshotChangedError('Conflicting CRC-valid rows')
        rows[sequence] = payload
    return expected, rows, rejected


def export_fault_trace(run, attempts=3):
    if not 1 <= attempts <= 5:
        raise ValueError('Retry count must be 1..5')
    verified = {}
    expected = None
    rejected = 0
    last_error = None
    for attempt in range(attempts):
        status = run('olstatus', 4)
        if not (re.search(r'Enable\s*:\s*OFF\b', status) and
                re.search(r'Target RPM\s*:\s*0(?:\s|$)', status)):
            raise RuntimeError('Stopped state not confirmed; no export')
        try:
            count, candidates, damaged = parse_export(run('sltrace', 30))
        except SnapshotChangedError:
            raise
        except ValueError as error:
            last_error = error
            continue
        rejected += damaged
        if expected is not None and count != expected:
            raise SnapshotChangedError('Frozen fault trace count changed')
        expected = count
        if verified and candidates and not (verified.keys() & candidates.keys()):
            raise SnapshotChangedError('No common valid rows to identify frozen buffer')
        for sequence, payload in candidates.items():
            if sequence in verified and verified[sequence] != payload:
                raise SnapshotChangedError('Frozen fault trace data changed')
            verified[sequence] = payload
        if len(verified) != expected:
            last_error = ValueError(f'Missing {expected - len(verified)} CRC-valid rows')
            continue
        ordered = [verified[index] for index in range(expected)]
        stamps = [int(payload.split(',')[1]) for payload in ordered]
        if any(not 0 < ((current - previous) & 0xffffffff) < 0x80000000
               for previous, current in zip(stamps, stamps[1:])):
            raise SnapshotChangedError('Invalid timestamp order')
        csv = HEADER + '\n' + ''.join(
            f'{payload},{crc32_msb(payload.encode("ascii")):08x}\n' for payload in ordered)
        return csv, {'rows': expected, 'attempts': attempt + 1, 'rejected_rows': rejected}
    raise RuntimeError(f'Fault export incomplete after {attempts} attempts: {last_error}')


def main():
    from ol_verify_serial import REPO_ROOT, Shell
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--attempts', type=int, default=3, choices=range(1, 6))
    args = parser.parse_args()
    prefix = REPO_ROOT / 'Doc' / 'verify' / ('fault_export_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    shell = Shell(args.port, 2_000_000)
    try:
        with prefix.with_suffix('.log').open('w', encoding='utf-8') as log:
            def run(command, timeout):
                reply = shell.run(command, timeout=timeout, tries=1)
                log.write(f'\n$ {command}\n{reply}')
                log.flush()
                if not shell._got_prompt(reply.encode('utf-8')):
                    raise RuntimeError('Incomplete serial response')
                return reply
            csv, report = export_fault_trace(run, args.attempts)
            prefix.with_suffix('.csv').write_text(csv, encoding='utf-8')
            print(report)
            print(prefix.with_suffix('.csv'))
    finally:
        shell.close()


if __name__ == '__main__':
    main()
