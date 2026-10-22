"""Exercise the real Bluetooth gyro reader with scripted I/O; no robot/socket."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'EV3_RobotControl/btcomm.c').read_text()
begin = source.index('int BT_read_gyro(')
end = source.index('int BT_play_sound_file(', begin)
with tempfile.TemporaryDirectory(prefix='c85_gyro_reader_') as name:
    directory = Path(name)
    (directory / 'gyro_reader_under_test.inc').write_text(source[begin:end])
    for compiler in ('gcc', 'g++'):
        executable = directory / 'test'
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-I', str(directory),
                        str(ROOT / 'verification/gyro_reader_test.c'), '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        print(compiler + ': ' + result.stdout.strip())
