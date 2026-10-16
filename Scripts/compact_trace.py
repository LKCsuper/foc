"""Validate stopped compact CSV exports and retry damaged serial transfers."""
import re

HEADER = 'seq,time_us,flags,theta,rpm,id,iq,id_cmd,iq_cmd,crc32'


class SnapshotChangedError(ValueError):
    pass


def crc32_msb(data):
    value = 0
    for byte in data:
        value ^= byte << 24
        for _ in range(8):
            value = ((value << 1) ^ (0x04C11DB7 if value & 0x80000000 else 0)) & 0xffffffff
    return value


def compact_lines(text):
    lines = text.splitlines()
    counts = re.findall(r'(?m)^trace samples=(\d+)\s*$', text)
    if not counts or HEADER not in lines:
        raise ValueError('Missing compact count or checksum header')
    expected = int(counts[-1])
    if not 0 <= expected <= 1664:
        raise ValueError('Invalid compact sample count')
    rows = [line for line in lines[lines.index(HEADER) + 1:] if re.match(r'^\d+,', line)]
    return expected, rows


def validate_frame(line):
    fields = line.split(',')
    if len(fields) != 10:
        raise ValueError('Wrong column count')
    payload, checksum = line.rsplit(',', 1)
    if not re.fullmatch(r'[0-9a-fA-F]{8}', checksum):
        raise ValueError('Invalid checksum')
    if crc32_msb(payload.encode('ascii')) != int(checksum, 16):
        raise ValueError('Checksum mismatch')
    sequence, stamp, flags = int(fields[0]), int(fields[1]), int(fields[2])
    if not 0 <= stamp <= 0xffffffff or not 0 <= flags <= 0xffffffff:
        raise ValueError('Invalid timestamp or flags')
    for field in fields[3:9]:
        float(field)
    return {'seq': sequence, 'time_us': stamp, 'flags': flags, 'csv': line}


def validate_compact_trace(text):
    expected, rows = compact_lines(text)
    if len(rows) != expected:
        raise ValueError(f'Expected {expected} frames, received {len(rows)}')
    result = []
    previous = None
    for sequence, line in enumerate(rows):
        row = validate_frame(line)
        if row['seq'] != sequence:
            raise ValueError(f'Missing or reordered frame {sequence}')
        stamp = row['time_us']
        if previous is not None and not 0 < ((stamp - previous) & 0xffffffff) < 0x80000000:
            raise ValueError(f'Invalid time order at frame {sequence}')
        previous = stamp
        result.append(row)
    return result


def export_compact_trace(run, attempts=3):
    last_error = None
    verified = {}
    expected = None
    for attempt in range(attempts):
        try:
            reply = run('sltrace', 30)
            count, candidates = compact_lines(reply)
            if expected is not None and count != expected:
                raise SnapshotChangedError('Frozen compact sample count changed')
            expected = count
            for line in candidates:
                try:
                    row = validate_frame(line)
                except ValueError:
                    continue
                sequence = row['seq']
                if not 0 <= sequence < expected:
                    continue
                if sequence in verified and verified[sequence] != line:
                    raise SnapshotChangedError('Frozen compact frame changed')
                verified[sequence] = line
            if len(verified) != expected:
                raise ValueError(f'{len(verified)}/{expected} frames verified')
            combined = '\n'.join([f'trace samples={expected}', HEADER,
                                  *(verified[sequence] for sequence in range(expected))])
            rows = validate_compact_trace(combined)
            print(f'Compact export verified: {len(rows)} frames, attempt {attempt + 1}', flush=True)
            return rows
        except SnapshotChangedError:
            raise
        except (ValueError, RuntimeError) as error:
            last_error = error
            print(f'Compact export rejected: {error}; attempt {attempt + 1}/{attempts}', flush=True)
    raise ValueError(f'Compact export integrity failed: {last_error}')


def main():
    import argparse
    from pathlib import Path
    from ol_verify_serial import REPO_ROOT, Shell

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    path = Path(args.output)
    if not path.is_absolute():
        path = REPO_ROOT / path
    path.parent.mkdir(parents=True, exist_ok=True)
    replies = []
    shell = Shell(args.port, 2000000)
    def run(command, timeout):
        reply = shell.run(command, timeout=timeout, progress=True, tries=1)
        replies.append(f'\n$ {command}\n{reply}')
        path.with_suffix('.uart.log').write_text(''.join(replies), encoding='utf-8')
        if not shell._got_prompt(reply.encode()):
            raise RuntimeError('UART response incomplete')
        return reply
    try:
        shell.kick(5)
        stopped = run('olsnap', 10)
        if not re.search(r'Enable\s*:\s*OFF\b', stopped):
            raise RuntimeError('Compact export requires a stopped motor')
        rows = export_compact_trace(run)
        path.write_text(HEADER + '\n' + '\n'.join(row['csv'] for row in rows) + '\n', encoding='ascii')
        print(f'Verified CSV: {path}', flush=True)
    finally:
        shell.close()


if __name__ == '__main__':
    main()
