"""Run production histogram localization against a PPM-based simulated robot.

Requires Python 3 and gcc or g++; no Python packages, robot, Bluetooth or BlueZ.
The map reader is independent of the production parser. Motion noise is optional.
"""
import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def read_map(path):
    data = path.read_bytes()
    position = 0

    def token():
        nonlocal position
        while position < len(data):
            if data[position:position + 1].isspace():
                position += 1
            elif data[position] == ord('#'):
                newline = data.find(b'\n', position)
                if newline < 0:
                    raise ValueError('Unterminated PPM comment')
                position = newline + 1
            else:
                break
        start = position
        while position < len(data) and not data[position:position + 1].isspace():
            position += 1
        return data[start:position]

    if token() != b'P6':
        raise ValueError('Simulator expects a binary P6 PPM map')
    width, height, maximum = int(token()), int(token()), int(token())
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError('PPM must have positive dimensions and maximum value 255')
    # Consume the raster separator without eating whitespace-valued pixel bytes.
    if position >= len(data) or not data[position:position + 1].isspace():
        raise ValueError('Missing PPM raster separator')
    position += 2 if data[position:position + 2] == b'\r\n' else 1
    pixels = data[position:]
    if len(pixels) != width * height * 3:
        raise ValueError('PPM pixel payload does not match its dimensions')

    def pixel(x, y):
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError('Building sample falls outside the PPM')
        offset = (y * width + x) * 3
        return tuple(pixels[offset:offset + 3])

    first = next(((x, y) for y in range(height) for x in range(width)
                  if pixel(x, y) == (255, 255, 0)), None)
    if first is None:
        raise ValueError('No yellow intersections in PPM')
    x0, y0 = first

    def runs(values):
        groups = []
        for value in values:
            if not groups or value != groups[-1][-1] + 1:
                groups.append([])
            groups[-1].append(value)
        return groups

    columns = runs(x for x in range(width) if pixel(x, y0) == (255, 255, 0))
    rows = runs(y for y in range(height) if pixel(x0, y) == (255, 255, 0))
    if not 1 <= len(columns) * len(rows) <= 400:
        raise ValueError('Map must have 1..400 intersections')
    ids = {(0, 0, 255): 2, (0, 255, 0): 3, (255, 255, 255): 6}
    landmarks = []
    for row in rows:
        for column in columns:
            x, y = column[0] + len(column) // 2, row[0] + len(row) // 2
            if pixel(x, y) != (255, 255, 0):
                raise ValueError('Yellow intersections do not form a rectangular grid')
            corners = [pixel(x + dx * len(column), y + dy * len(row))
                       for dx, dy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]]
            try:
                landmarks.append([ids[colour] for colour in corners])
            except KeyError as error:
                raise ValueError(f'Invalid building colour near intersection ({x}, {y})') from error
    return len(columns), len(rows), landmarks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=ROOT / 'Map1.ppm')
    parser.add_argument('--noise', type=float, default=0.0,
                        help='Wrong-colour probability per non-missing corner')
    parser.add_argument('--unknown', type=float, default=0.0,
                        help='Missing-reading probability per corner')
    parser.add_argument('--turn-error', type=float, default=0.0,
                        help='Chance a reported-success turn ends a quarter-turn away from intended')
    parser.add_argument('--stay', type=float, default=0.0,
                        help='Chance an interior trip reports yellow without advancing a cell')
    parser.add_argument('--scan-offset', type=int, choices=range(4), default=0,
                        help='Deliberate corner-order fault: offset the scan by 0..3 corners')
    parser.add_argument('--trials', type=int, default=1,
                        help='Repetitions of every start pose (or selected --start)')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--start', type=int, nargs=3, metavar=('X', 'Y', 'HEADING'))
    parser.add_argument('--verbose', action='store_true', help='Print production trace for a selected start')
    parser.add_argument('--report', type=Path, help='CSV output (default: timestamped file in logs/)')
    parser.add_argument('--compiler', help='Path to gcc or g++')
    args = parser.parse_args()
    if any(not math.isfinite(value) or not 0 <= value <= 1
           for value in (args.noise, args.unknown, args.turn_error, args.stay)):
        parser.error('--noise, --unknown, --turn-error and --stay must be finite probabilities in 0..1')
    if not 1 <= args.trials <= 10000 or not 0 <= args.seed <= 0xffffffff:
        parser.error('--trials must be 1..10000 and --seed must be a 32-bit unsigned integer')
    if args.verbose and not args.start:
        parser.error('--verbose requires --start X Y HEADING to keep the trace readable')
    compiler = args.compiler or shutil.which('gcc') or shutil.which('g++')
    if not compiler:
        parser.error('Install gcc/g++ or provide --compiler PATH')
    width, height, landmarks = read_map(args.map)
    if args.start:
        x, y, heading = args.start
        if not (0 <= x < width and 0 <= y < height and 0 <= heading < 4):
            parser.error(f'--start must be within {width}x{height}, heading 0..3')
    # Keep hardware code out of the build while testing the current functions,
    # rather than maintaining a separate Python copy of their mathematics.
    source = (ROOT / 'EV3_Localization.c').read_text()
    begin = source.index('double score_intersection_scan(')
    end = source.index('int heading_toward_target(', begin)
    functions = source[begin:end]
    for name in ('update_beliefs_from_scan', 'update_beliefs_after_turn',
                 'update_beliefs_after_forward', 'robot_localization'):
        if f'int {name}(' not in functions:
            raise ValueError(f'Cannot find production function {name}')

    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report = args.report or ROOT / 'logs' / f'localization_simulation_{stamp}.csv'
    if report.exists():
        parser.error(f'Report already exists: {report}')
    with tempfile.TemporaryDirectory(prefix='c85_sim_') as directory:
        work = Path(directory)
        (work / 'localization_under_test.inc').write_text(functions)
        initializers = ',\n'.join('{' + ','.join(str(c) for c in row) + '}' for row in landmarks)
        (work / 'simulation_map.inc').write_text(
            f'#define SIM_WIDTH {width}\n#define SIM_HEIGHT {height}\n'
            f'static const int simulation_map[{len(landmarks)}][4]={{\n{initializers}\n}};\n')
        executable = work / ('simulator.exe' if sys.platform == 'win32' else 'simulator')
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror',
                        '-I', str(work), str(ROOT / 'verification' / 'localization_simulator.c'),
                        '-lm', '-o', str(executable)], check=True)
        csv_path, trace_path = work / 'results.csv', work / 'trace.log'
        with trace_path.open('w') as trace:
            result = subprocess.run(
                [str(executable), str(args.noise), str(args.unknown), str(args.trials), str(args.seed),
                 *(str(v) for v in (args.start or (-1, -1, -1))), str(csv_path), str(int(args.verbose)),
                 str(args.turn_error), str(args.stay), str(args.scan_offset)],
                stdout=trace, stderr=subprocess.STDOUT)
        if result.returncode not in (0, 1):
            print(trace_path.read_text(errors='replace'), file=sys.stderr)
            raise RuntimeError(f'Simulator failed an assertion or crashed (exit {result.returncode})')
        if args.verbose:
            # Fixture traces precede the selected production run.
            trace_text = trace_path.read_text(errors='replace')
            print(trace_text[trace_text.index('SIM case:'):])
        with csv_path.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        report.parent.mkdir(parents=True, exist_ok=True)
        with report.open('xb') as output:
            output.write(csv_path.read_bytes())

    print(f'Map: {width} x {height}; runs: {len(rows)}; noise: {args.noise:.0%}; unknown: {args.unknown:.0%}')
    print(f'Turn errors: {args.turn_error:.0%}; false arrival/stay: {args.stay:.0%}; scan offset: {args.scan_offset}')
    for outcome in ('correct', 'incorrect', 'unresolved'):
        print(f'{outcome.capitalize()}: {sum(row["result"] == outcome for row in rows)}')
    print(f'Average scans: {sum(int(row["scans"]) for row in rows) / len(rows):.2f}')
    print('Corner-order, motion-model, maximum-size, ambiguous-map, missing-scan and belief-invariant checks passed.')
    print(f'CSV report: {report.resolve()}')
    print('Discrete simulated movement; hardware control and production PPM parser are not exercised.')
    return result.returncode


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError, RuntimeError) as error:
        print(f'Simulation failed: {error}', file=sys.stderr)
        sys.exit(2)
