"""Robot-free counted intersection sweep and production scan integration checks."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'EV3_Localization.c').read_text()
parts = []
begin = source.index('static int sample_colour_agreement_mode(')
parts.append(source[begin:source.index('static int sample_colour_agreement(', begin)])
begin = source.index('static int sample_intersection_colour(')
parts.append(source[begin:source.index('static int continuous_black_sweep(', begin)])
begin = source.index('  int scan_intersection(')
parts.append(source[begin:source.index('static int recover_from_red_boundary(', begin)])
with tempfile.TemporaryDirectory(prefix='c85_intersection_sweep_') as name:
    directory = Path(name)
    (directory / 'intersection_sweep_under_test.inc').write_text('\n'.join(parts))
    for compiler in ('gcc', 'g++'):
        executable = directory / 'test'
        subprocess.run([compiler, '-Wall', '-Wextra', '-Werror', '-I', str(directory),
                        str(ROOT / 'verification/intersection_sweep_test.c'), '-lm', '-o', str(executable)], check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError(result.stdout[-2500:] + result.stderr[-2500:])
        print(compiler + ': ' + result.stdout.splitlines()[-1])
        if compiler == 'gcc':
            print(next(line for line in result.stdout.splitlines() if line.startswith('Nominal scan:')))
    executable = directory / 'trial'
    subprocess.run(['g++', str(ROOT / 'verification/robot_trial.cpp'),
                    str(ROOT / 'EV3_RobotControl/btcomm.c'), '-lbluetooth', '-o', str(executable)], check=True)
    for args in ([], ['missing-calibration'], ['missing-calibration', 'extra']):
        result = subprocess.run([str(executable), '--test-intersection-sweep', *args],
                                capture_output=True, text=True, timeout=5)
        assert result.returncode != 0 and 'Request to connect' not in result.stdout
    subprocess.run([str(executable), '--stop-self-test'], check=True)
    print('Direct scan diagnostic rejects 3 invalid setups before Bluetooth; signal brake self-test passed.')
