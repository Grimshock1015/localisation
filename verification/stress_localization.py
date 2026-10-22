"""Run a reproducible sensor/motion fault sweep and save every result under logs/."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=100,
                        help='Repetitions per starting pose in stochastic scenarios')
    parser.add_argument('--seed', type=int, default=10001)
    args = parser.parse_args()
    if not 1 <= args.trials <= 10000 or not 0 <= args.seed <= 0xffffffff:
        parser.error('Use trials 1..10000 and an unsigned 32-bit seed')
    scenarios = [
        ('ideal', {}, 1),
        ('wrong_01', {'noise': .01}, args.trials),
        ('wrong_05', {'noise': .05}, args.trials),
        ('wrong_10', {'noise': .10}, args.trials),
        ('wrong_20', {'noise': .20}, args.trials),
        ('wrong_30', {'noise': .30}, args.trials),
        ('missing_10', {'unknown': .10}, args.trials),
        ('missing_25', {'unknown': .25}, args.trials),
        ('missing_50', {'unknown': .50}, args.trials),
        ('sensor_mixed', {'noise': .10, 'unknown': .10}, args.trials),
        ('turn_04', {'turn-error': .04}, args.trials),
        ('stay_06', {'stay': .06}, args.trials),
        ('motion_mixed', {'turn-error': .04, 'stay': .06}, args.trials),
        ('sensor_motion', {'noise': .10, 'turn-error': .04, 'stay': .06}, args.trials),
        ('all_mixed', {'noise': .10, 'unknown': .10, 'turn-error': .04, 'stay': .06}, args.trials),
        ('all_missing', {'unknown': 1.0}, 1),
        ('all_wrong', {'noise': 1.0}, 1),
        ('all_turns_wrong', {'turn-error': 1.0}, 1),
        ('never_advance', {'stay': 1.0}, 1),
        ('corner_offset_1', {'scan-offset': 1}, 1),
        ('corner_offset_2', {'scan-offset': 2}, 1),
        ('corner_offset_3', {'scan-offset': 3}, 1),
    ]
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    directory = ROOT / 'logs' / f'localization_stress_{stamp}'
    directory.mkdir(parents=True)

    def run(scenario):
        name, settings, trials = scenario
        report = directory / f'{name}.csv'
        command = [sys.executable, str(ROOT / 'verification' / 'simulate_localization.py'),
                   '--trials', str(trials), '--seed', str(args.seed), '--report', str(report)]
        for flag, value in settings.items():
            command.extend([f'--{flag}', str(value)])
        process = subprocess.run(command, capture_output=True, text=True)
        (directory / f'{name}.log').write_text(process.stdout + process.stderr)
        if process.returncode not in (0, 1):
            raise RuntimeError(f'{name}: simulator failed; inspect {directory / (name + ".log")}')
        with report.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        counts = {outcome: sum(row['result'] == outcome for row in rows)
                  for outcome in ('correct', 'incorrect', 'unresolved')}
        result = dict(scenario=name, runs=len(rows), **counts,
                      average_scans=round(sum(int(row['scans']) for row in rows) / len(rows), 3),
                      noise=settings.get('noise', 0), unknown=settings.get('unknown', 0),
                      turn_error=settings.get('turn-error', 0), stay=settings.get('stay', 0),
                      scan_offset=settings.get('scan-offset', 0), seed=args.seed)
        print(f'{name}: {counts["correct"]}/{len(rows)} correct, '
              f'{counts["incorrect"]} incorrect, {counts["unresolved"]} unresolved', flush=True)
        return result

    print(f'Running {len(scenarios)} scenarios; reports: {directory}', flush=True)
    results = {}
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures = {pool.submit(run, scenario): scenario[0] for scenario in scenarios}
        for future in as_completed(futures):
            results[futures[future]] = future.result()
    ordered = [results[name] for name, _, _ in scenarios]
    summary = directory / 'summary.csv'
    with summary.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(ordered[0]))
        writer.writeheader()
        writer.writerows(ordered)
    text = ['# Localization stress results', '',
            f'Seed: {args.seed}; stochastic repetitions per pose: {args.trials}.', '',
            '| Scenario | Runs | Correct | Incorrect | Unresolved | Average scans |',
            '|---|---:|---:|---:|---:|---:|']
    for row in ordered:
        text.append('| ' + ' | '.join(str(row[key]) for key in
                    ('scenario', 'runs', 'correct', 'incorrect', 'unresolved', 'average_scans')) + ' |')
    text.extend(['', 'Fault definitions and replay commands: verification/README.md.', '',
                 'These are synthetic discrete-state tests, not measured robot accuracy.',
                 'Wrong estimates and unresolved runs are findings, not simulator crashes.',
                 'All-missing and corner-offset scenarios deliberately remove/corrupt evidence.', '',
                 '## Source fingerprints', ''])
    for name in ('EV3_Localization.c', 'Map1.ppm', 'verification/localization_simulator.c',
                 'verification/simulate_localization.py', 'verification/stress_localization.py'):
        text.append(f'- {name}: `{hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}`')
    (directory / 'summary.md').write_text('\n'.join(text) + '\n')
    print(f'Total requested simulation runs: {sum(row["runs"] for row in ordered)}')
    print(f'Summary: {directory / "summary.md"}')
    # Noisy scenarios expose algorithm failures; these are recorded, not hidden.
    # The ideal and no-evidence controls must still satisfy their basic contracts.
    return int(results['ideal']['correct'] != results['ideal']['runs'] or
               results['all_missing']['unresolved'] != results['all_missing']['runs'])


if __name__ == '__main__':
    sys.exit(main())
