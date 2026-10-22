# Robot-free localization checks

## One-street hardware diagnostic

In Ubuntu/WSL, build and run an interactive series. It waits for manual placement
between trials and never drives back automatically:

```sh
python3 verification/run_street_trials.py calibration/run-5 --trials 10 --power 10
```

Start with the colour sensor on black just beyond yellow, facing the intended
next yellow intersection. Each trial checks stationary gyro stability, runs the
production street follower once, and stops at yellow, red or failure. It has no
localization loop, boundary turnaround or automatic mission retry. The follower
can still turn to center/recover black. Confirm that yellow is the intended next
intersection; the sensor alone cannot verify which intersection it reached.
Logs and a CSV of sensor/operator outcomes are saved in `logs/street_trials_*`.
The runner uses the existing-connection signal brake and a 120-second outer limit.

`--start-now --trials 1` starts one already-positioned trial without prompts and
leaves physical success unverified. Compare lower powers only after recording
a baseline; changing diagnostic power does not alter normal mission power.

Run the robot-free preflight/outcome tests with:

```sh
python3 verification/test_street_segment.py
```

For geometry, measure the forward horizontal axle-to-sensor-reading-point offset
and wheel diameter in millimetres, then calculate the advance in wheel rotations:

```sh
./build/EV3_Localization --check-geometry OFFSET_MM WHEEL_DIAMETER_MM
```

The formula is offset / (pi * wheel diameter). This command does not connect to
the robot or change configuration. The current `SENSOR_TO_AXLE_OFFSET_REV=0.27`
remains an assumption until physical measurements are supplied. Keep the sensor
at its calibrated mounted height and hold the robot still during gyro startup.

From the project directory, with Python 3 and gcc or g++ available:

```sh
python verification/simulate_localization.py
```

This tests all 60 first-intersection poses in `Map1.ppm`, plus explicit corner
rotation, ambiguous-map and all-missing-reading cases. Each run checks that the
current C localization loop returns the simulated robot's true position/heading
and that beliefs remain positive and normalized. Generated C and executables
live in a temporary directory. No robot control code or Bluetooth is linked.

Inspect a particular start (heading 0 up, 1 right, 2 down, 3 left):

```sh
python verification/simulate_localization.py --start 1 2 0 --verbose
```

Stress the sensor assumptions with reproducible noise:

```sh
python verification/simulate_localization.py --noise 0.10 --trials 20 --seed 1
python verification/simulate_localization.py --unknown 0.10 --trials 20 --seed 1
python verification/simulate_localization.py --turn-error 0.04 --stay 0.06 --trials 100
python verification/stress_localization.py
python verification/test_red_boundary.py
python verification/test_mission_recovery.py
python verification/test_map_parser.py
```

`--noise` is the chance of selecting either incorrect building colour for a
non-missing corner. `--unknown` independently controls missing corners. Trials
repeat each starting pose. `--map PATH`, `--compiler PATH` and `--report PATH`
override defaults. Timestamped CSV reports are saved in the ignored `logs/`
directory and include starting, actual and estimated poses for every run.
Existing reports are never overwritten.

`--turn-error` models an undetected discrete heading error: a commanded quarter
turn instead performs no turn or a half turn (equally likely). On a boundary
turnaround, it ends one quarter-turn either side of the intended reversed
heading. `--stay` models an interior trip that reports yellow but remains at the
previous intersection. These are hypothetical faults, not measured robot rates.
Zero motion-error settings preserve the original deterministic sensor seeds.

The red-boundary test separately compiles the actual `find_street()`,
`turn_at_red_and_search()`, `recover_from_red_boundary()` and
`drive_to_next_intersection()` helpers. It scripts sensor/motor outcomes and
checks 16 cases in all four headings, including same-intersection belief updates,
turn/search failures, alignment fallback, repeated red and nested turnarounds.
Failures return exit status 1 and are saved in a timestamped Markdown report.
This tests recovery control flow; it does not verify the physical 180-degree turn.
After the boundary turnaround, a second red detection (including during a recovery
sweep) must fail recovery without another 180-degree turn. Initial street search
still permits its normal single red turnaround.

The mission-recovery suite compiles the real navigation helpers and `run_mission()`.
It checks a wrong initial pose causing red, successful relocalization and arrival,
retry exhaustion, temporary localization/turn failures, failed braking, invalid
poses/inputs, stale-belief/colour-history reset, and 900 complete routes. Localization
results and movement feedback are scripted for this suite, so it tests orchestration
rather than the filter (which is exercised separately by the simulator).

The application allows two recovery retries (`MAX_RELOCALIZATIONS`), in addition
to the initial localization/navigation attempt. Each attempt starts with uniform
beliefs and cleared sensor/alignment history. Pose loss or a failed localization
can retry after braking; invalid navigation data or a failed brake stops the mission.

The production parser suite tests the real `readPPMimage()` and `parse_map()`:
Map1's decoded landmarks are compared with the independent reader, and fixtures
cover the 400-intersection limit, single-row/column grids, invalid geometry and
colours, read-only files, malformed headers, truncated/extra raster data, and
whitespace-valued first pixels under LF/CRLF/CR/space separators. Invalid maps
must leave map storage unchanged, reset dimensions, and preserve guard values
around the map array. The supplied map's small edge irregularities are accepted;
broken marker interiors or missing intersections are rejected. Only single P6
images with 8-bit channels and an exact raster length are supported.

On Linux/WSL, also run memory and undefined-behaviour sanitizers:

```sh
python3 verification/test_map_parser.py --sanitize
```

`--scan-offset 1` deliberately rotates all observed corner labels by one corner;
values 2 and 3 rotate further. This is a systematic scan-order fault test, not
ordinary independent sensor noise. It can produce confidently incorrect poses.

The stress runner covers 22 scenarios and saves a summary CSV/Markdown, each
scenario's pose results and console log, and source fingerprints in one folder
under `logs/`. By default stochastic scenarios repeat every start 100 times;
use `--trials 20` for a shorter sweep. It returns nonzero for an ideal/no-evidence
control failure or simulator crash. Incorrect noisy results remain visible in
the summary and individual CSVs rather than stopping the sweep.

To replay a particular CSV row, supply its `start_x`, `start_y`, `start_heading`
as `--start`, its `seed` as `--seed`, and the original noise/unknown settings.
Use `--trials 1 --verbose` to see that case's full production trace.
Supply the original motion-error/scan-offset options too, when applicable.

Exit status: 0 means all requested runs localized correctly; 1 means at least one
incorrect or unresolved run; 2 means setup/build/assertion failure. Noisy tests
can expose confidently wrong estimates: inspect incorrect CSV rows rather than
treating every successful return from localization as correct.

The Python runner extracts the current contiguous sensing/motion/localization
functions from `EV3_Localization.c` for compilation, so it exercises the actual
algorithm rather than a rewritten filter. If those functions are reorganized,
update the runner's extraction boundaries. The PPM is decoded independently;
the production `parse_map()` is not tested. Starting poses represent the first
scanned intersection. Street acquisition and scan geometry are idealized, with
optional discrete motion faults; this does not establish physical robot accuracy.
