"""Interactive one-street trials using production control and an existing-connection stop handler."""
import argparse
import csv
from datetime import datetime,timezone
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration',type=Path)
    parser.add_argument('--trials',type=int,default=10)
    parser.add_argument('--power',type=int,default=10)
    parser.add_argument('--start-now',action='store_true',help='Skip first positioning prompt only when already ready')
    args=parser.parse_args()
    if not 1<=args.trials<=100 or not 1<=args.power<=20:
        parser.error('Use trials 1..100 and power 1..20')
    if args.start_now and args.trials!=1:
        parser.error('--start-now requires --trials 1; repeated trials require manual repositioning')
    calibration=args.calibration.resolve()
    if not all((calibration/(colour+'.txt')).is_file() for colour in ('red','green','blue','yellow','black','white')):
        parser.error('Calibration folder must contain all six captures')
    build=ROOT/'build'
    build.mkdir(exist_ok=True)
    executable=build/'EV3_StreetTrial'
    subprocess.run(['g++',str(ROOT/'verification/robot_trial.cpp'),str(ROOT/'EV3_RobotControl/btcomm.c'),
                    '-lbluetooth','-o',str(executable)],check=True)
    subprocess.run([str(executable),'--stop-self-test'],check=True)
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    logs=ROOT/'logs'/('street_trials_'+stamp)
    logs.mkdir(parents=True)
    summary=logs/'summary.csv'
    passed=0
    with summary.open('x',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['trial','power','returncode','sensor_outcome','operator_outcome','line_losses','log'])
        for trial in range(1,args.trials+1):
            if not (trial==1 and args.start_now):
                try:
                    response=input(f'\nTrial {trial}/{args.trials}: place sensor on black just beyond yellow, aimed at next yellow. Enter to start; q to finish: ')
                except EOFError: break
                if response.strip().lower()=='q': break
            log=logs/f'trial-{trial:02d}.log'
            print(f'\nStarting trial {trial}; log {log}',flush=True)
            command=[str(executable),'--test-street',str(calibration),str(args.power)]
            with log.open('x') as output:
                process=subprocess.Popen(['stdbuf','-oL','-eL',*command],stdout=output,stderr=subprocess.STDOUT)
                try:
                    process.wait(timeout=120)
                except (subprocess.TimeoutExpired,KeyboardInterrupt):
                    if process.poll() is None:
                        process.send_signal(signal.SIGTERM)
                    try: process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill();process.wait()
                    print('Trial interrupted/time-limited; verify physical standstill.',flush=True)
            text=log.read_text()
            print(text,flush=True)
            matches=re.findall(r'STREET_RESULT,([a-z_]+)',text)
            outcome=matches[-1] if matches else 'interrupted_or_setup_error'
            operator='unverified'
            if not args.start_now:
                try:
                    response=input('Did it reach the intended next yellow intersection and stop correctly? y/n/q: ').strip().lower()
                except EOFError: response='q'
                operator='confirmed' if response=='y' and outcome=='yellow_detected' else 'failed' if response=='n' else 'unverified'
            else: response='q'
            passed+=operator=='confirmed'
            writer.writerow([trial,args.power,process.returncode,outcome,operator,text.count('Black lost ('),str(log)])
            stream.flush()
            if outcome in ('brake_failed','interrupted_or_setup_error') or response=='q': break
            time.sleep(1)
    print(f'Operator-confirmed successes: {passed}. Summary: {summary}',flush=True)
    return 0

if __name__=='__main__':
    sys.exit(main())
