# USBODOM 5: manual checkpoint capture

Dedicated passive program in slot 5. No mechanism objects, no USB input parser,
no nonzero motor command, and no odometry feedback into motor outputs. Six drive
motors are kept at zero voltage in COAST mode. Controller A marks a reference;
R1 and other buttons cannot enable movement. User moves the robot manually.

Native IMU calibration runs while stationary at startup; failure stops startup.
The first hardware startup failed calibration; the second passed. Verification
`docs/usb-passive-20261003-191313-406.csv`: 400 records, 10 ms cycles, validity
mask 31, zero drops, marker zero. ARM build and host PowerShell parse passed.

Recorded at nominal 100 Hz: both rotation sensors in raw centidegrees and nominal
millimetres; all six signed motor positions in configured revolutions; native
IMU continuous angle, gyro XYZ, acceleration XYZ, pitch and roll; controller
marker; sequential record number, board cycle, validity and queue-drop counters;
microsecond API-call bracketing times for each pod, IMU reads and drive reads.
Call times are not proof of simultaneous physical sensor sampling. Do not mix
absolute RTOS millisecond origin with the microsecond clock origin.

Port mapping/nominal pod diameters remain the prior diagnostic mapping. Motors
on ports 4 and 9 have physically different cartridges/external gearing; their
raw positions must not be averaged with the four blue motors as equal wheel
distances without per-port normalization. This capture does not fit that ratio.

Protocol `UP1` is decoded by `tools/usb_odom_passive.ps1`. That host opens the
serial port for capture and sends no data to the robot. `-UntilArrival` requires
initial marker zero, records all frames, prints each marker, and finishes 2.5 s
after marker 3. Default hard recording duration is configurable up to 600 s.
No asynchronous UI reply is used as a sensor timestamp: physical controller A
edges create the board-timestamped marks.

## Agreed physical protocol

User confirmed 60 **cm**, or 600 mm, on both axes. All reference headings are
the same as at departure. They chose a forward-only route with a broad return
curve. One fixed, recognizable point of the chassis is aligned at every mark;
its centre of mass is irrelevant. Distances are along the starting X-right /
Y-forward axes, not diagonal travel distance.

1. Stationary at (0,0), heading 0: wait 2 s, press/release A once (marker 1),
   wait 2 s, then move.
2. Align the SAME chassis point at (600,600), with initial orientation; wait
   2 s, press/release A (marker 2), wait 2 s.
3. Return by the broad forward curve, align the same position AND orientation
   as start, wait 2 s, press/release A (marker 3), remain stopped 3 s.

Reference plan: `docs/usb-checkpoint-plan-20261003.json`.

## Analysis

`tools/fit_usb_checkpoint_offsets.py` validates frame continuity, sensor validity,
exact marker progression and pre-marker stationarity. It integrates the raw
readings with exact constant-twist SE(2) steps, producing a linear basis for
the two geometric offsets. It solves the continuous least-squares problem and
its bounded optimum, then scans 1,002,001 pairs (both offsets -500..500 mm in
1 mm steps). This domain is a diagnostic range, not measured robot geometry.
It reports errors at BOTH reference points and never writes a calibration.

Same external headings make the intended offset-calibration design unobservable,
even if the robot always moves forward or makes a full 360-degree loop between
marks. Small measured heading errors can produce numerical sensitivity and an
artificial fitted offset; that does not establish physical calibration. The
program therefore reports reference excitation separately and sets
`apply_calibration=false`. It does not promise a pair that forces both errors
to zero. Different headings at intermediate known positions would be needed
for an identifiable offset test; the user's explicit heading-zero choice is
preserved.

Four focused mathematical tests passed: pure fixed-point rotation, forward-only
closed-circle offset cancellation, recovery from rotated references, and box
constraint/unobservable cases (`tools/test_usb_checkpoint_offsets.py`).

## Display feedback correction (5B)

The first attempted capture `docs/usb-passive-20261003-191707-097.csv` contains
no pod movement: F stays -501.418747 mm, L -2.881539 mm throughout. The user
reported that A seemed not to change the display. The CSV records markers
1/2/3/4; a read-only Brain screenshot `docs/usb-passive-display-20261003.png`
shows "attuale 5". Thus button edges arrived, while the fixed numbered legend
gave ambiguous feedback. This file is NOT a completed trajectory/reference
test, and is not used to fit offsets.

USBODOM 5B replaces the fixed legend with explicit PARTENZA SALVATA /
CHECKPOINT SALVATO / ARRIVO SALVATO states and ignores further A edges after
marker 3. Motors remain zero/coast and the serial protocol is unchanged.
Build passed; upload to slot 5 and native calibration passed. Read-only check
`docs/usb-passive-20261003-192130-669.csv`: 299 frames, all 10 ms, valid=31,
drops=0, marker=0. A fresh capture was then started for the same authorized
physical protocol.

## Completed checkpoint experiment

`docs/usb-passive-20261003-192152-873.csv` contains 13,843 consecutive records,
all 10 ms, all validity masks 31 and no queue drops. Markers were received at
board times 100374 (start), 121844 (checkpoint), and 168614 ms (return).
The host stopped normally after marker 3. All pre-marker half-second windows
passed stationarity checks. The reference locations are the user's physically
marked points, not a separately surveyed ground-truth system.

With configured offsets (+20,-70) mm, replay relative to the marked start gives:

| Point | Reference X/Y, mm | Estimated X/Y, mm | Position error, mm |
| --- | --- | --- | --- |
| Checkpoint | 600 / 600 | 553.019 / 667.736 | 82.434 |
| Return | 0 / 0 | -55.259 / 96.785 | 111.449 |

Measured heading changes are -0.3294 degrees at the checkpoint and +360.8340
degrees on return (equivalent final yaw residual +0.8340 degrees). The design
still specifies equal headings, so this is not an identifiable offset calibration.

`docs/usb-checkpoint-fit-20261003.json` records the full 1,002,001-pair grid
search and continuous solutions. The best bounded pair is at the box boundary
(-500,-500) mm and leaves 86.299 mm checkpoint / 101.773 mm return error.
Even allowing arbitrary, physically implausible offsets yields (-4114.55,
-2297.48) mm and residuals 109.314 / 43.176 mm. The nonzero least-squares
minimum proves that no constant offset pair fits both marked positions exactly
for these recorded inputs. No candidate is applied.

Additional substitutions are recorded in `docs/usb-checkpoint-signals-20261003.json`:
integrating raw gyro Z leaves 83.702 / 81.156 mm errors; the best constant IMU
shift scanned at 5 ms steps over +/-200 ms leaves 83.538 / 89.709 mm; a yaw
scale inferred from assuming exactly one complete final turn leaves 83.581 /
99.970 mm. These are diagnostics, not independently validated corrections.
Whole acquisition call span max 44 us, sample intervals 9992..10009 us on the
microsecond clock. API-call times are not internal sensor measurement timestamps.

Independent pinned original LemLib replay also reproduces the large closure:
`docs/usb-checkpoint-lemlib-20261003.txt`. 6850 frames from the marked starting
plateau through arrival, one segment, no skipped inputs or rejected Nexus
increments. Final Nexus (-55.257153,96.785779) mm; original LemLib
(-55.258134,96.785456) mm; maximum trajectory disagreement 0.003873196 mm.
The small difference versus the averaged-marker report is due to selecting
one start/end sample rather than averaging stationary windows.

`tools/prepare_passive_lemlib_replay.py` writes the explicit compatibility input
`docs/usb-checkpoint-replay-input-20261003.csv`: F/L/native heading from this
capture, four blue motor readings only for drive evidence, zero initial-pose
placeholders (not measured poses). It is used only in whole-file mode. This
compares integration algorithms, not an emulation of hardware acquisition.

The experiment reproduces the position discrepancy and rules out repairing
these two residuals solely by constant pod offsets. It does not identify which
physical sensing, sensor timing/model, or reference measurement effect is the
root cause. Production configuration/build remain unchanged; robot remains in
zero-voltage passive mode.
