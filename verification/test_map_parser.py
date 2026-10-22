"""Exercise the production PPM reader/grid parser on valid and malformed files."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile

from simulate_localization import read_map

ROOT = Path(__file__).resolve().parents[1]
COLOURS = {2: bytes((0, 0, 255)), 3: bytes((0, 255, 0)), 6: bytes((255, 255, 255))}


def grid(columns, rows, starts=None):
    xs = starts or [4 + 8 * x for x in range(columns)]
    ys = [4 + 8 * y for y in range(rows)]
    width, height = xs[-1] + 6, ys[-1] + 6
    raster = bytearray(b'\xff' * (width * height * 3))

    def paint(x, y, colour):
        index = (y * width + x) * 3
        raster[index:index + 3] = colour

    expected = []
    for y, top in enumerate(ys):
        for x, left in enumerate(xs):
            for py in range(top, top + 2):
                for px in range(left, left + 2):
                    paint(px, py, bytes((255, 255, 0)))
            corners = [[2, 3, 6, 2][(c + x + y) % 4] for c in range(4)]
            expected.append(corners)
            for c, (dx, dy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
                paint(left + 1 + dx * 2, top + 1 + dy * 2, COLOURS[corners[c]])
    return width, height, raster, expected


def ppm(width, height, raster):
    return f'P6\n{width} {height}\n255\n'.encode() + raster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', help='gcc/g++ path')
    parser.add_argument('--sanitize', action='store_true', help='Use Linux address/undefined-behaviour sanitizers')
    args = parser.parse_args()
    compiler = args.compiler or shutil.which('gcc') or shutil.which('g++')
    if not compiler:
        parser.error('Install gcc/g++ or supply --compiler PATH')
    if args.sanitize and sys.platform == 'win32':
        parser.error('--sanitize requires a Linux toolchain (for example, WSL)')
    source = (ROOT / 'EV3_Localization.c').read_text()
    functions = source[source.index('static int map_pixel_is_yellow('):]
    outcomes = []
    with tempfile.TemporaryDirectory(prefix='c85_parser_') as directory:
        work = Path(directory)
        (work / 'map_parser_under_test.inc').write_text(functions)
        executable = work / ('parser_test.exe' if sys.platform == 'win32' else 'parser_test')
        flags = ['-fsanitize=address,undefined', '-fno-omit-frame-pointer'] if args.sanitize else []
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', *flags, '-I', str(work),
                        str(ROOT / 'verification' / 'map_parser_test.c'), '-o', str(executable)], check=True)

        def invoke(mode, path=None):
            command = [str(executable), mode] + ([str(path)] if path else [])
            result = subprocess.run(command, capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                raise RuntimeError(f'Parser test crashed: {command}\n{result.stdout}\n{result.stderr}')
            if args.sanitize and 'runtime error:' in result.stderr:
                raise RuntimeError(result.stderr)
            return json.loads(result.stdout)

        def record(name, passed):
            outcomes.append((name, bool(passed)))
            if not passed:
                print(f'FAIL: {name}')

        def fixture(name, data, read_ok, parse_ok=False, shape=None, expected=None, mode='parse', prefix=None):
            path = work / (name + '.ppm')
            path.write_bytes(data)
            result = invoke(mode, path)
            passed = bool(result['read']) == read_ok and bool(result['parsed']) == parse_ok and result['canaries'] == 1
            if not read_ok:
                passed &= result['width'] == result['height'] == 0
            if not parse_ok:
                passed &= result['unchanged'] == 1 and result['sx'] == result['sy'] == 0
            if shape:
                passed &= (result['sx'], result['sy']) == shape
            if expected is not None:
                passed &= result['map'] == expected
            if prefix is not None:
                passed &= result['prefix'] == list(prefix)
            record(name, passed)

        result = invoke('invalid_arguments')
        record('invalid API arguments', result['arguments_ok'] == result['unchanged'] == 1)
        actual = ROOT / 'Map1.ppm'
        fingerprint = hashlib.sha256(actual.read_bytes()).digest()
        result = invoke('parse', actual)
        nx, ny, landmarks = read_map(actual)
        record('actual Map1 matches independent reader', result['parsed'] == result['canaries'] == 1 and
               (result['sx'], result['sy']) == (nx, ny) and result['map'] == landmarks)
        record('actual map file unchanged', hashlib.sha256(actual.read_bytes()).digest() == fingerprint)
        for nx, ny in [(1, 1), (1, 5), (5, 1), (2, 2), (3, 5), (20, 20)]:
            w, h, raster, expected = grid(nx, ny)
            fixture(f'valid_{nx}x{ny}', ppm(w, h, raster), True, True, (nx, ny), expected)
        for nx, ny in [(21, 20), (401, 1), (1, 401)]:
            w, h, raster, _ = grid(nx, ny)
            fixture(f'oversized_{nx}x{ny}', ppm(w, h, raster), True)

        w, h, raster, _ = grid(2, 2)
        mutations = {
            'missing_intersection': [(x, y, b'\xff\xff\xff') for x in (12, 13) for y in (12, 13)],
            'ragged_intersection': [(13, 13, b'\xff\xff\xff')],
            'extra_yellow': [(2, 2, b'\xff\xff\x00')],
            'invalid_building': [(3, 3, b'\xff\x00\x00')],
        }
        for name, changes in mutations.items():
            changed = raster.copy()
            for x, y, colour in changes:
                offset = (y * w + x) * 3
                changed[offset:offset + 3] = colour
            fixture(name, ppm(w, h, changed), True)
        w, h, raster, expected = grid(3, 2, starts=[4, 12, 22])
        fixture('nonuniform_spacing', ppm(w, h, raster), True, True, (3, 2), expected)
        fixture('no_intersections', ppm(8, 8, b'\xff' * 8 * 8 * 3), True)
        fixture('corner_outside_image', ppm(2, 2, b'\xff\xff\x00' * 4), True)
        fixture('single_yellow_pixel', ppm(1, 1, b'\xff\xff\x00'), True)

        for separator in (b'\n', b'\r\n', b'\r', b' '):
            for first in (0, 9, 10, 13, 32, 35):
                raster = bytes((first, 1, 2, 3, 4, 5))
                header = separator.join((b'P6', b'# comment', b'2 1', b'255')) + separator
                # Comments require a line ending, regardless of token separators.
                if separator == b' ':
                    header = b'P6 2 1 255 '
                fixture(f'pixel_prefix_{separator.hex()}_{first}', header + raster,
                        True, mode='read', prefix=raster)
        fixture('comments_between_tokens', b'P6\n#first\n2 #width\n1\n#max\n255\n' + bytes(range(6)),
                True, mode='read', prefix=bytes(range(6)))
        invalid = {
            'empty': b'', 'ascii_format': b'P3\n1 1\n255\n0 0 0',
            'bad_magic': b'P60\n1 1\n255\nabc', 'zero_width': b'P6\n0 1\n255\n',
            'negative_width': b'P6\n-1 1\n255\n', 'zero_height': b'P6\n1 0\n255\n',
            'signed_width': b'P6\n+1 1\n255\nabc', 'numeric_suffix': b'P6\n1x 1\n255\nabc',
            'dimension_overflow': b'P6\n99999999999999999999999 1\n255\nabc',
            'huge_dimensions_no_payload': b'P6\n2147483647 2147483647\n255\n',
            'wrong_maxval': b'P6\n1 1\n256\nabc', 'zero_maxval': b'P6\n1 1\n0\nabc',
            'missing_maxval': b'P6\n1 1\n', 'missing_separator': b'P6\n1 1\n255',
            'truncated_raster': b'P6\n2 2\n255\nabc',
            'extra_raster': b'P6\n1 1\n255\nabcdef', 'unterminated_comment': b'P6\n#unfinished',
            'long_token': b'P6\n' + b'1' * 100 + b' 1\n255\nabc',
            'nul_in_magic': b'P6\x00junk\n1 1\n255\nabc',
            'nul_in_dimension': b'P6\n1\x00junk 1\n255\nabc',
        }
        for name, data in invalid.items():
            fixture(name, data, False)
        # Check a broken interior on the supplied map, separately from the
        # tolerated one-pixel irregularities at its marker edges.
        original_map = actual.read_bytes()
        raster_start = original_map.index(b'\n255\n') + len(b'\n255\n')
        damaged = bytearray(original_map)
        offset = raster_start + (169 * 750 + 194) * 3
        damaged[offset:offset + 3] = b'\xff\xff\xff'
        fixture('broken_marker_interior', damaged, True)
        readonly = work / 'readonly.ppm'
        readonly.write_bytes(ppm(2, 1, bytes(range(6))))
        readonly.chmod(0o444)
        try:
            result = invoke('read', readonly)
            record('read-only file', result['read'] == 1 and result['prefix'] == list(range(6)))
        finally:
            readonly.chmod(0o644)

        rng = random.Random(20261020)
        for number in range(100):
            width, height = rng.randint(1, 12), rng.randint(1, 12)
            # Deliberately inconsistent payloads must fail before allocation/read.
            expected_size = width * height * 3
            size = max(0, expected_size + rng.choice((-3, -1, 1, 3)))
            fixture(f'payload_length_fuzz_{number:03}', ppm(width, height, rng.randbytes(size)), False)

    failures = [name for name, passed in outcomes if not passed]
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    report = ROOT / 'logs' / f'map_parser_tests_{stamp}.md'
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text('\n'.join(['# Production map parser tests', '',
        f'Sanitizers: {args.sanitize}. Passed {len(outcomes) - len(failures)}/{len(outcomes)}.', '',
        '| Case | Result |', '|---|---|',
        *(f'| {name} | {"PASS" if passed else "FAIL"} |' for name, passed in outcomes), '']))
    print(f'{len(outcomes)-len(failures)}/{len(outcomes)} production parser cases passed.')
    print(f'Report: {report}')
    return int(bool(failures))


if __name__ == '__main__':
    sys.exit(main())
