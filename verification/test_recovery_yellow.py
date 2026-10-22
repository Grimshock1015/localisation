"""Test actual recovery centring with scripted stationary yellow confirmations."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'EV3_Localization.c').read_text()
begin = source.index('static int recover_black_heading(void)\n{')
end = source.index('static int follow_black_street(void)\n{', begin)
with tempfile.TemporaryDirectory(prefix='c85_recovery_yellow_') as name:
    directory = Path(name)
    (directory / 'recovery_yellow_under_test.inc').write_text(source[begin:end])
    for compiler in ('gcc', 'g++'):
        executable = directory / 'unit'
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', '-I', str(directory),
                        str(ROOT / 'verification/recovery_yellow_test.c'), '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=5)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        print(compiler + ': ' + result.stdout.splitlines()[-1])
