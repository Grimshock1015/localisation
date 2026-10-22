"""Check real navigation and relocalization orchestration without a robot."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', help='gcc/g++ path')
    args = parser.parse_args()
    compiler = args.compiler or shutil.which('gcc') or shutil.which('g++')
    if not compiler:
        parser.error('Install gcc/g++ or supply --compiler PATH')
    source = (ROOT / 'EV3_Localization.c').read_text()
    start = source.index('int heading_toward_target(')
    end = source.index('static int calibration_command(', start)
    with tempfile.TemporaryDirectory(prefix='c85_mission_') as directory:
        work = Path(directory)
        (work / 'mission_under_test.inc').write_text(source[start:end])
        executable = work / ('mission_test.exe' if sys.platform == 'win32' else 'mission_test')
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', '-I', str(work),
                        str(ROOT / 'verification' / 'mission_recovery_test.c'), '-lm',
                        '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True)
    rows = [line.split(',')[1:] for line in result.stdout.splitlines() if line.startswith('TEST,')]
    if result.returncode not in (0, 1) or len(rows) != 16:
        print(result.stdout + result.stderr, file=sys.stderr)
        raise RuntimeError('Mission tests crashed or did not complete')
    failed = [row for row in rows if row[1] == 'FAIL']
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report = ROOT / 'logs' / f'mission_recovery_tests_{stamp}.md'
    report.parent.mkdir(parents=True, exist_ok=True)
    lines = ['# Mission recovery tests', '',
             'Actual mission/navigation code; scripted localization and hardware outcomes.', '',
             '| Case | Result |', '|---|---|']
    lines.extend('| ' + ' | '.join(row) + ' |' for row in rows)
    lines.extend(['', 'Includes 900 full pose/destination routes.', '',
                  '## Diagnostic output', '', '```text', result.stderr.rstrip(), '```', ''])
    report.write_text('\n'.join(lines))
    print(f'{len(rows)-len(failed)}/{len(rows)} mission cases passed, including 900 full routes.')
    for name, _ in failed:
        print(f'FAIL: {name}')
    print(f'Report: {report}')
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
