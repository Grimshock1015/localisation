# Robot-free localization checks

Localisation confirms when the physical sweep verifies the street heading and
the accumulated best position/heading belief is at least 10 times as likely as
the runner-up, or its probability is over 70% with at least a 5x lead. There is
no minimum number of visible corners and no consecutive-scan requirement.
Unknown or tied corner readings are ignored by the likelihood update; earlier
scans and movement retain their evidence. With no visible corners, confirmation
is possible only if the accumulated belief already passes the same thresholds.
A failed physical sweep/heading verification cannot confirm, even with strong
belief or four valid colours. `scan_intersection` reports physical heading
verification separately from corner availability; missing corners remain -1.
The limit remains 24 scans per localisation attempt. Simulator fixtures check
immediate acceptance, both sides of the 70% and 5x thresholds, failed headings,
and missing corner readings.

The first intersection search may accept stationary confirmed yellow without
a prior black departure, including yellow found during alignment or recovery.
When initial acquisition already stops on confirmed yellow, localisation skips
the extra full alignment/forward approach and scans at that intersection.
This establishes the first scan location and does not apply a forward belief
transition. The exception is cleared immediately after the initial approach,
including failure; all later trips still require black before yellow.

Navigation distinguishes an axle already positioned by a scan from a sensor-only
arrival on yellow. Before leaving an ordinary arrival, it verifies/centres yellow
and advances the existing sensor-to-axle offset before turning. The street follower
verifies outgoing black and requires an actual black reading during departure
before accepting the next yellow. Unknown readings cannot count as that black
observation. The 0.27-revolution offset still needs physical calibration.
Street reacquisition and marker-edge checks try right before left; a missed side
returns to the entry heading before searching the other side. The search remains
bounded to 30 degrees per side in 5-degree increments.
During recovery from a lost street while driving, confirmed yellow also ends
black centring immediately, including edge and midpoint probes. Yellow is checked
again while stopped, then returned as an intersection event. The follower still
requires a prior black departure before counting it as the next intersection.
Normal turn alignment and the four-street building scan retain their existing
rules; yellow during those operations does not trigger this recovery shortcut.

Run `python3 verification/test_street_departure.py` for scripted departure events
and `python3 verification/test_mission_recovery.py` for placement, route and retry
checks. These tests do not establish physical centring or turn accuracy.
`python3 verification/test_recovery_yellow.py` checks yellow detection during
driving recovery and verifies that confirmed yellow is not swept away from.

## One-street hardware diagnostic

In Ubuntu/WSL, build and run an interactive series. It waits for manual placement
between trials and never drives back automatically:

```sh
python3 verification/run_street_trials.py calibration/run-5 --trials 10 --power 10
```

Keep the EV3 at its main menu with Port View closed and no competing program
running. In stationary tests, leaving Port View resolved repeated invalid sensor
values in both the independent reader and the production C reader.

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

Then test a turn followed by one street:

```sh
python3 verification/run_street_trials.py calibration/run-5 --turn right --trials 5
python3 verification/run_street_trials.py calibration/run-5 --turn left --trials 5
```

For these tests, manually place the drive-wheel axle over the center of an
interior yellow intersection, facing a street with the sensor on black ahead.
The selected turn must lead to another yellow intersection. The diagnostic turns
90 degrees and checks black before any alignment recovery. If it misses black,
it stops instead of disguising a bad turn with a recovery sweep. Logs record the
gyro angle after the turn and the separate street outcome. Operator confirmation
must verify that it turned the requested way and reached the correct next marker.

Run the robot-free preflight/outcome tests with:

```sh
python3 verification/test_street_segment.py
```

Centering keeps one gyro reference across its edge probes, returns to the measured
entry heading when an edge is missing, and checks the marker before continuing.
Its robot-free regression covers accumulated small-turn error, missing edges,
midpoint alignment, search acquisition and failed reads/brakes:

```sh
python3 verification/test_centering.py
```

The Bluetooth gyro reader rejects EV3 invalid-data sentinels before changing the
reference or calculating relative angles. It transfers complete reply frames,
validates message ID/status/size, and checks subtraction in 64-bit arithmetic.
The motion caller brakes when a gyro read fails. These checks prevent invalid
data from becoming a huge commanded heading; they do not correct sensor drift.

```sh
python3 verification/test_gyro_reader.py
```

## One-step navigation hardware diagnostic

This diagnostic starts with the sensor over the centre of a yellow intersection,
facing along a street. Do not manually move the axle onto yellow first: the test
uses normal navigation to position it with the configured sensor-to-axle offset.
Use an interior intersection with a clear street on the chosen side leading to
the next yellow intersection. Keep Port View closed and the robot still at startup.

In Ubuntu/WSL, from the project directory:

```sh
mkdir -p build
g++ verification/robot_trial.cpp EV3_RobotControl/btcomm.c -lbluetooth -o build/EV3_RobotTrial
./build/EV3_RobotTrial --test-navigation-step calibration/run-5 right
# Reposition manually before testing the other direction:
./build/EV3_RobotTrial --test-navigation-step calibration/run-5 left
```

The test calls the production navigation step once: position the axle, turn,
reacquire black, and follow the street until yellow, red, or failure. There is no
localisation scan or automatic mission retry. Its coordinates describe the test's
relative origin/destination; they do not establish an absolute pose on Map1.
Verify physically that it turns the selected way, reaches the intended adjacent
yellow intersection, and stops both wheels.

Run `python3 verification/test_navigation_step.py` for robot-free CLI validation.
It rejects invalid arguments/calibration before Bluetooth and checks signal brake
dispatch locally; it does not run a valid driving command or prove physical braking.

## Counted intersection scan

After initial street acquisition, intersection scans stop at gyro targets to
sample colours. Steps are 10 degrees between streets and 5 degrees near expected
crossings and while measuring black strips. A clean sample ends after four
agreeing reads; mixed samples still use up to five and require four votes.
Start and final-black checks use the same scan sampling rule. Other alignment
and street-event sampling retain their existing five-reading behaviour.
The initial black strip is excluded from the count.
Two confirmed nonblack stops rearm detection; unknown readings do not. Black
entries must be within 25 degrees of the next expected quarter turn (90, 180,
270, 360). After the fourth strip, the scan measures its exit and returns to the
midpoint, verifies black, and brakes. The old post-scan black recovery wiggle is
removed. Initial street alignment still uses its existing continuous sweep.

Missing streets/readings, an unverified midpoint, or failures stop the scan; it
does not assume that another street is the starting heading. The sweep is bounded
to 405 degrees and 180 seconds, with individual turns also bounded. This strict
four-street requirement can reject intersections where the sensor cannot observe
all four streets, including map edges. A centred axle and physical testing are
still needed; simulated success does not establish robot reliability.

```sh
python3 verification/test_intersection_sweep.py
```

To isolate the actual production sweep from initial street acquisition, place
the axle at an interior yellow intersection's centre with the sensor on black
ahead. Build `verification/robot_trial.cpp` with `EV3_RobotControl/btcomm.c` and
`-lbluetooth`, then run the wrapper with
`--test-intersection-sweep calibration/run-5`. It checks stationary gyro/black,
rotates in place once using the production counted sweep, then brakes and closes.
It has no forward drive or mission retry. Confirm that it returns to the original
street and that both wheels stop; sensor confirmation alone is not ground truth.

For geometry, measure the forward horizontal axle-to-sensor-reading-point offset
and wheel diameter in millimetres, then calculate the advance in wheel rotations:

```sh
./build/EV3_Localization --check-geometry OFFSET_MM WHEEL_DIAMETER_MM
```

The formula is offset / (pi * wheel diameter). This command does not connect to
the robot or change configuration. The current `SENSOR_TO_AXLE_OFFSET_REV=0.27`
remains an assumption until physical measurements are supplied. Keep the sensor
at its calibrated mounted height and hold the robot still during gyro startup.

The classifier also learns a dark-black brightness threshold when black is
strictly darker than all nonblack calibration captures. This allows a dark,
red-tinted black reading to remain black instead of selecting red by hue. If the
captured brightness ranges overlap, the added gate is disabled. Test the captured
black-street regression and overlap handling with:

```sh
python3 verification/test_colour_classifier.py
```

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
