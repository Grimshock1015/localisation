"""Exercise production centering and turn control with a simulated gyro; no robot."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'EV3_Localization.c').read_text()
parts = []
for start, end in (
    ('static int window_wipe_for_colour(int target_colour)\n{', 'static int wiggle_to_black('),
    ('static MoveResult rotate_gyro_to_target(', 'static int center_turn_to_heading('),
    ('static int center_turn_to_heading(int target)\n{', 'static MoveResult rotate_gyro('),
    ('static int sample_marker_alignment_colour(', 'static int follow_black_street(void)\n{'),
):
    begin = source.index(start)
    parts.append(source[begin:source.index(end, begin)])
with tempfile.TemporaryDirectory(prefix='c85_centering_') as name:
    directory = Path(name)
    (directory / 'centering_under_test.inc').write_text('\n'.join(parts))
    for compiler in ('gcc', 'g++'):
        executable = directory / 'test'
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', '-I', str(directory),
                        str(ROOT / 'verification/centering_test.c'), '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, check=True, timeout=10)
        print(compiler + ': ' + result.stdout.splitlines()[-1])
