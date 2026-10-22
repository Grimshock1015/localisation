"""Reject invalid navigation-step setups before opening Bluetooth (Linux/WSL)."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
wrapper = ROOT / 'verification' / 'robot_trial.cpp'
if '--test-navigation-step' not in wrapper.read_text():
    raise RuntimeError('The navigation-step diagnostic is not present in robot_trial.cpp')
with tempfile.TemporaryDirectory(prefix='c85_navigation_step_') as name:
    directory = Path(name)
    executable = directory / 'trial'
    subprocess.run(['g++', str(wrapper), str(ROOT / 'EV3_RobotControl' / 'btcomm.c'),
                    '-lbluetooth', '-o', str(executable)], check=True)
    calibration = str(ROOT / 'calibration' / 'run-5')
    missing = str(directory / 'missing-calibration')
    empty = directory / 'empty-calibration'
    empty.mkdir()
    plain_file = directory / 'not-a-calibration-directory.txt'
    plain_file.write_text('not a calibration directory\n')
    invalid = [[], [calibration], [calibration, 'up'], [calibration, '0'],
               [calibration, 'RIGHT'], [calibration, ''],
               [calibration, 'right', 'extra'], [missing, 'right'],
               [missing, 'left'], [str(empty), 'right'], [str(plain_file), 'left']]
    for index, args in enumerate(invalid):
        result = subprocess.run([str(executable), '--test-navigation-step', *args],
                                capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr
        assert result.returncode != 0, args
        assert 'Request to connect' not in output, (args, output)
        if index == 0:
            assert '--test-navigation-step' in output, output
    subprocess.run([str(executable), '--stop-self-test'], check=True, timeout=5)
    print(f'{len(invalid)}/{len(invalid)} invalid navigation-step setups rejected before Bluetooth; signal brake self-test passed.')
