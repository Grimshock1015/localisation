"""Test real red-boundary recovery control flow without linking robot hardware."""
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

    def section(start, end):
        begin = source.index(start)
        finish = source.index(end, begin)
        return source[begin:finish]

    functions = '\n'.join([
        section('static int is_street_marker(MoveResult result)\n{',
                'static int circular_angle_distance('),
        section('static int turn_at_red_and_search(const int *stop_colours)\n',
                'int drive_along_street('),
        section('static int recover_from_red_boundary(void)\n{',
                'static int window_wipe_for_colour(int target_colour)\n{'),
        section('int update_beliefs_after_forward(', 'int robot_localization('),
    ])
    with tempfile.TemporaryDirectory(prefix='c85_red_') as directory:
        work = Path(directory)
        (work / 'boundary_under_test.inc').write_text(functions)
        executable = work / ('red_test.exe' if sys.platform == 'win32' else 'red_test')
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', '-I', str(work),
                        str(ROOT / 'verification' / 'red_boundary_test.c'), '-lm',
                        '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True)
    rows = [line.split(',')[1:] for line in result.stdout.splitlines() if line.startswith('TEST,')]
    if result.returncode not in (0, 1) or len(rows) != 64:
        print(result.stdout + result.stderr, file=sys.stderr)
        raise RuntimeError('Boundary tests crashed or did not complete')
    failed = [row for row in rows if row[2] == 'FAIL']
    text = ['# Red-boundary control tests', '',
            'These tests compile the actual recovery/search/drive helpers and forward belief update.',
            'Motor/sensor outcomes are scripted; physical gyro and line-following accuracy are not tested.', '',
            '| Case | Starting heading | Result |', '|---|---:|---|']
    text.extend('| ' + ' | '.join(row) + ' |' for row in rows)
    text.extend(['', '## Failure details', '', '```text', result.stderr.rstrip(), '```', ''])
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report = ROOT / 'logs' / f'red_boundary_tests_{stamp}.md'
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text('\n'.join(text))
    print(f'{len(rows)-len(failed)}/{len(rows)} cases passed.')
    for name, heading, _ in failed:
        print(f'FAIL: {name}, starting heading {heading}')
    print(f'Report: {report}')
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
