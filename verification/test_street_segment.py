"""Robot-free diagnostic preflight, outcome and geometry CLI checks (Linux/WSL)."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
source=(ROOT/'EV3_Localization.c').read_text()
start=source.index('static int street_test_preflight(void)\n{')
end=source.index('static int street_test_main(',start)
with tempfile.TemporaryDirectory(prefix='c85_street_test_') as name:
    directory=Path(name)
    (directory/'street_segment_under_test.inc').write_text(source[start:end])
    for compiler in ('gcc','g++'):
        executable=directory/'unit'
        subprocess.run([compiler,'-Wall','-Wextra','-Werror','-I',str(directory),
                        str(ROOT/'verification/street_segment_test.c'),'-o',str(executable)],check=True)
        result=subprocess.run([str(executable)],capture_output=True,text=True,check=True)
        print(compiler+': '+ '; '.join(line for line in result.stdout.splitlines() if 'cases passed' in line))
    executable=directory/'application'
    subprocess.run(['g++',str(ROOT/'EV3_Localization.c'),str(ROOT/'EV3_RobotControl/btcomm.c'),
                    '-lbluetooth','-o',str(executable)],check=True)
    result=subprocess.run([str(executable),'--check-geometry','47.5','56'],capture_output=True,text=True,check=True)
    value=float(re.search(r'advance: ([0-9.]+)',result.stdout)[1])
    assert abs(value-0.27)<0.00001
    invalid=[['--check-geometry'],['--check-geometry','0','56'],
             ['--check-geometry','47.5','0'],['--check-geometry','nan','56'],
             ['--check-geometry','47.5','inf'],['--check-geometry','12oops','56'],
             ['--test-street'],['--test-street','missing-calibration'],
             ['--test-street','missing-calibration','0'],
             ['--test-street','missing-calibration','21'],
             ['--test-street','missing-calibration','10oops'],
             ['--test-turn-street'],['--test-turn-street','missing-calibration','up'],
             ['--test-turn-street','missing-calibration','right'],
             ['--test-turn-street','missing-calibration','left','0'],
             ['--test-turn-street','missing-calibration','right','10oops']]
    for args in invalid:
        result=subprocess.run([str(executable),*args],capture_output=True,text=True,timeout=5)
        assert result.returncode!=0 and 'Request to connect' not in result.stdout,args
    print(f'Geometry calculation and {len(invalid)} invalid-argument/calibration checks passed without Bluetooth.')
