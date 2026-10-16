"""无需实机的统一测试入口；失败写报告并返回非零。"""
import argparse
import datetime
import json
from pathlib import Path
import subprocess
import shutil
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default=str(ROOT / '.build/tools/ziglang/zig.exe'))
    parser.add_argument('--skip-existing', action='store_true', help='仅运行新增动态测试')
    args = parser.parse_args()
    build = HERE / 'build'
    results = HERE / 'results'
    build.mkdir(exist_ok=True)
    results.mkdir(exist_ok=True)
    records = []

    def run(name, command):
        print(f'Running {name}', flush=True)
        start = time.monotonic()
        try:
            completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                       encoding='utf-8', errors='replace')
            code, output = completed.returncode, completed.stdout + completed.stderr
        except OSError as error:
            code, output = 1, str(error)
        (results / f'{name}.log').write_text(output, encoding='utf-8')
        records.append(dict(name=name, status='PASS' if code == 0 else 'FAIL',
                            exit_code=code, seconds=round(time.monotonic() - start, 2)))
        print(f'{records[-1]["status"]}: {name}', flush=True)
        if code:
            print(output[-3000:])
        return code == 0

    if not args.skip_existing:
        run('existing_host_suite', [shutil.which('pwsh') or 'powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                                   '-File', str(ROOT / 'Tests/host/run.ps1'), '-Compiler', args.compiler])
        run('python_host_suite', [sys.executable, '-m', 'unittest', 'discover',
                                 '-s', 'Tests/host', '-p', 'test_*.py', '-v'])
    executable = build / 'test_current_plant.exe'
    compiled = run('current_plant_compile', [args.compiler, 'cc', '-std=c99', '-Wall', '-Wextra',
                      '-Werror', '-D_DEFAULT_SOURCE', '-Dassert_param(x)=((void)0)', '-IApp/motor/inc',
                      'App/motor/motor_pid.c', 'App/motor/motor_foc.c', 'App/motor/motor_utils.c',
                      str(HERE / 'test_current_plant.c'), '-o', str(executable)])
    if compiled:
        run('current_plant', [str(executable)])
    else:
        records.append(dict(name='current_plant', status='NOT_RUN', reason='编译失败'))
    report = dict(time=datetime.datetime.now(datetime.timezone.utc).isoformat(), results=records,
                  limitations=['主机已有 C 套件若遇到失败会停止，届时后续用例未执行。',
                               'RL 模型仅验证锁轴电流响应，不代表完整电机或硬件验收。'])
    (results / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# 无实机测试结果', '', '| 测试组 | 结果 |', '| --- | --- |']
    lines += [f'| {record["name"]} | {record["status"]} |' for record in records]
    lines += ['', '详细输出见同目录日志。', '', *report['limitations']]
    (results / 'REPORT.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return 0 if all(record['status'] == 'PASS' for record in records) else 1


if __name__ == '__main__':
    sys.exit(main())
