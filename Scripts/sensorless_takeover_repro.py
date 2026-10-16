"""有界600RPM接管对照；显式开/关RAM记录，失败停机，不做高速测试。"""
import argparse
import re
import time
from pathlib import Path
from ol_verify_serial import Shell, REPO_ROOT
from compact_trace import HEADER, export_compact_trace

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM31')
    parser.add_argument('--modes', nargs='+', choices=['off', 'on', 'compact', 'entry', 'handoff', 'phase', 'phaselong', 'step', 'pair'], default=['off', 'on', 'off'])
    parser.add_argument('--timing', choices=['off', 'full', 'lean', 'lean-detail', 'task', 'probe'], default='off')
    parser.add_argument('--duration', type=int, default=0, help='CLOSED observation seconds, 0 keeps six samples')
    args = parser.parse_args()
    if not 0 <= args.duration <= 120:
        parser.error('--duration must be between 0 and 120 seconds')
    directory = REPO_ROOT / 'Doc' / 'verify' / time.strftime('3508_repro_%Y%m%d_%H%M%S')
    directory.mkdir(parents=True, exist_ok=True)
    results = []
    for index, mode in enumerate(args.modes, 1):
        shell = Shell(args.port, 2000000)
        path = directory / f'{index}_{mode}.log'
        log = []
        def run(command, timeout=5):
            reply = shell.run(command, timeout=timeout, tries=1)
            log.append('\n$ ' + command + '\n' + reply)
            path.write_text(''.join(log), encoding='utf-8')
            if not shell._got_prompt(reply.encode()): raise RuntimeError('UART response incomplete')
            if re.search(r'(?im)^\s*(Invalid|Unknown|.* rejected|.* failed)\b', reply):
                raise RuntimeError('Command rejected: ' + command)
            return reply
        outcome = 'FAIL'
        stop_confirmed = False
        try:
            shell.kick(5)
            run('olstop')
            precheck = run('olsnap')
            voltage = re.search(r'V_bus\s*:\s*([0-9.]+)', precheck)
            if not voltage or not 22 <= float(voltage[1]) <= 26 or 'Enable     : OFF' not in precheck or '0 NONE' not in precheck:
                raise RuntimeError('Stopped/voltage/fault precheck failed')
            for command in ['pos off','joint off','spenable 0','olfb sensorless','slenable 1','sltakeover 0',
                            'olmax 6000','olramp 100','olvlimit 4','oliq 1','spiq 1','spkp 0.003','spki 0.01']:
                run(command)
            # 两组都先重置记录；off立即导出空缓冲区并停止，on只读故障保持记录。
            run({'compact': 'sltrace compact', 'entry': 'sltrace entry'}.get(mode, 'sltrace arm'))
            run('sltrace' if mode in ('off', 'handoff', 'phase', 'phaselong', 'step', 'pair') else 'olfault', 15)
            if mode == 'handoff': run('slhandoff arm')
            if mode == 'phase': run('slhandoff phase')
            if mode == 'phaselong': run('slhandoff phaselong')
            if mode == 'step': run('slhandoff step')
            if mode == 'pair': run('slhandoff pair')
            time.sleep(0.3)
            run('olspeed 300')
            run('slcadence ' + {'off': 'stop', 'full': 'reset', 'lean': 'lean',
                                'lean-detail': 'lean detail', 'task': 'task', 'probe': 'probe'}[args.timing])
            run('olstart'); time.sleep(5)
            run('olrpm'); run('olspeed 600'); time.sleep(5)
            run('oliq 0.5'); time.sleep(3)
            ready = run('olrpm')
            if not re.search(r'chk F=1 S=1 D=1 P=1', ready): raise RuntimeError('Takeover not ready')
            run('spenable 1'); run('sltakeover 1')
            # LOCKING不是已确认CLOSED后的失锁；先有界等待真正接管。
            takeover_deadline = time.monotonic() + 5
            confirmed_closed = False
            while time.monotonic() < takeover_deadline:
                time.sleep(0.3)
                text = run('olrpm')
                match = re.search(r'rpm obs=([-0-9.]+).*?tgt=([-0-9.]+).*?iq=([-0-9.]+)\s+(\w+) tk=(\d).*?fault=(\d+)', text)
                if not match: raise RuntimeError('Takeover diagnostic frame incomplete')
                if float(match[2]) != 600 or int(match[6]) or not 100 < float(match[1]) < 950 or abs(float(match[3])) > 1.6:
                    raise RuntimeError('Takeover speed/current/fault guard')
                print(f'{index}/{len(args.modes)} trace={mode} waiting={match[4]} rpm={match[1]}', flush=True)
                if match[4] == 'CLOSED' and match[5] == '1':
                    confirmed_closed = True
                    break
            if not confirmed_closed: raise RuntimeError('Takeover timeout; CLOSED not confirmed')
            if args.duration > 0:
                run('slcadence ' + {'off': 'stop', 'full': 'reset', 'lean': 'lean',
                                    'lean-detail': 'lean detail', 'task': 'task', 'probe': 'probe'}[args.timing])
            values = []
            stable_start = time.monotonic()
            while len(values) < 6 or time.monotonic() - stable_start < args.duration:
                time.sleep(0.7)
                text = run('olrpm')
                match = re.search(r'rpm obs=([-0-9.]+).*?tgt=([-0-9.]+).*?iq=([-0-9.]+)\s+(\w+) tk=(\d).*?fault=(\d+)', text)
                if not match: raise RuntimeError('Diagnostic frame incomplete')
                rpm = float(match[1]); iq = float(match[3])
                print(f'{index}/{len(args.modes)} trace={mode} rpm={rpm:.1f} state={match[4]} iq={iq:.3f}', flush=True)
                if float(match[2]) != 600 or int(match[6]) or not 100 < rpm < 950 or abs(iq) > 1.6:
                    raise RuntimeError('Speed/current/fault guard')
                if match[4] != 'CLOSED' or match[5] != '1': raise RuntimeError('Closed-loop lost')
                if abs(rpm-600) > 60: raise RuntimeError('Speed tracking failed')
                values.append(rpm)
            elapsed = time.monotonic() - stable_start
            run('slcadence stop')
            outcome = (f'PASS avg={sum(values)/len(values):.1f} min={min(values):.1f} max={max(values):.1f}'
                       f' samples={len(values)} duration={elapsed:.1f}s')
        except Exception as error:
            outcome = f'FAIL {error}'
        finally:
            try:
                stopped = run('olstop', 20)
                stop_confirmed = 'Open-loop stopped' in stopped
                run('slcadence stop')
                run('olspeed 0')
                final_state = run('olsnap')
                stop_confirmed = stop_confirmed or (
                    re.search(r'Enable\s*:\s*OFF\b', final_state) is not None and
                    re.search(r'Target RPM\s*:\s*0(?:\.0+)?\s*$', final_state, re.M) is not None)
                if mode == 'compact':
                    try:
                        verified = export_compact_trace(run)
                        (directory / f'{index}_{mode}_verified.csv').write_text(
                            HEADER + '\n' + '\n'.join(row['csv'] for row in verified) + '\n', encoding='ascii')
                    except ValueError as error:
                        outcome = f'FAIL {error}'
                elif mode in ('handoff', 'phase', 'phaselong', 'step', 'pair'):
                    trace_text = run('slhandoff show', 20)
                    (directory / f'{index}_{mode}_trace.log').write_text(trace_text, encoding='utf-8')
                elif mode in ('on', 'entry'):
                    trace_text = run('sltrace', 20)
                    (directory / f'{index}_{mode}_trace.log').write_text(trace_text, encoding='utf-8')
                run('olvlimit 4'); run('olmax 5920')
            finally:
                shell.close()
                path.write_text(''.join(log)+'\nRESULT '+outcome, encoding='utf-8')
        results.append(f'{index} trace={mode} {outcome}')
        (directory/'summary.txt').write_text('\n'.join(results), encoding='utf-8')
        print(results[-1], flush=True)
        if not stop_confirmed: raise RuntimeError('Stop unconfirmed; remaining tests cancelled')
        time.sleep(2)
    (directory/'summary.txt').write_text('\n'.join(results), encoding='utf-8')
    print('Logs: '+str(directory), flush=True)
    return 0 if all('PASS' in result for result in results) else 1

if __name__ == '__main__':
    raise SystemExit(main())
