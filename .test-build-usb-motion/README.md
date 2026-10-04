# USBODOM 3 — supervised USB odometry diagnostics

Dedicated program in Brain slot **5**, named **USBODOM**. Slots 1–4 were occupied
when inspected on 2026-10-03. Production NEXUS and its build are unchanged.

The app runs unchanged LemLib v0.5.6 odometry (commit
`3388145e0d90ee3c7c17d7267a30ef64367df4cb`) and a copy of the current NEXUS
estimator core with native IMU heading. Both use the same physical sensors, but
acquire independently; instantaneous poses can differ by a sampling interval.
This is **not** the complete production NEXUS Chassis adapter. Compare settled
endpoints relative to their initial baselines, and retain the raw sensor trace.

Hardware: IMU 1, forward rotation 2, horizontal rotation +3 (LemLib left-positive;
USB and NEXUS negate this to right-positive), 2-inch pods, F offset +20 mm,
L offset -70 mm. Drive ports `{5,-4,6}` and `{-7,9,-10}`, blue cartridges,
3.25-inch drive wheels, external ratio 0.6, track width 287 mm.

Only the six drive motors are instantiated. No mechanism/pneumatic routines are
called. IMU native calibration runs at startup. No automatic motion at startup.

## Motion interlocks

- Controller R1 must first be seen released after startup, then held continuously.
- B, controller loss, competition connection, disabled/autonomous state inhibit motion.
- Every pulse requires a fresh USB command and heartbeat; robot-side heartbeat timeout 120 ms.
- Robot-issued timestamp in commands must be at most 100 ms old.
- Max 3000 mV, 2000 ms per pulse, 1000 mA current limit per motor.
- Stop on 600 mm indicated excursion of **any** motor during a pulse.
- Latch a fault after 900 mm indicated accumulated absolute travel of **any** motor per app run.
- Additional estimated envelope: X -50..450 mm, Y -50..800 mm, heading -10..55 deg.
  This check supplements the time, encoder, controller and USB limits; it is not
  an independent ground-truth position measurement.
- Invalid sensors or cycle longer than 30 ms latch a fault.
- USB input and output run below the motor watchdog task and cannot block it.
- Every stop cancels the active pulse; heartbeat alone cannot restart it.

These are motor/encoder thresholds, not guaranteed physical stopping distances:
coasting continues after voltage removal. They are not obstacle detection or independently
measured ground displacement. A person must supervise the clear test area and
check physical endpoints. The diagnostic session initially authorizes only a
small forward pulse; further commands require checking the actual result and
available space. Never infer an actual closed path from an estimated zero pose.

## Host

`tools/usb_odom_probe.ps1` defaults to capture without motion. It writes a new
timestamped CSV, opens COM9 (V5 User), and always sends STOP on exit. `-Pulse`
sends exactly one bounded pulse after one second, only if telemetry reports R1
consent. It does not wait indefinitely for someone to press the button.
`-WaitForConsent` permits a bounded wait for the physical button within the
specified capture duration; without it, absent consent rejects the attempt.
Once a command is sent, capture ends after 3.5 seconds of subsequent data.
`-Pulse -SSequence` runs 3 V/1 V for 2000 ms, then 1 V/3 V for 2000 ms.
The second command requires the first to have ended normally, continuing R1
consent, fresh telemetry/heartbeat, and no board fault. It never restarts a
sequence after a release. The host reserves enough capture time for both pulses.
`-ProtocolCheck` sends an invalid zero-voltage pulse, which must be consumed but
not accepted. Every run creates a new log; no old log is overwritten.

Raw serial is explicitly enabled using PROS `SERCTL_DISABLE_COBS`. The official
PROS 4.2.1 implementation routes raw USB input to stdin. Input blocking is
confined to the dedicated input task. Source references:

- https://github.com/purduesigbots/pros/blob/4.2.1/src/system/dev/ser_driver.c
- https://github.com/purduesigbots/pros/blob/4.2.1/src/system/dev/ser_daemon.c

## Validation on 2026-10-03

- Host C++ guard tests pass: expiry, deadman release, stale/missing heartbeat,
  consumed sequence replay, voltage/duration limits, individual encoder limit,
  session travel budget, invalid sensor, and no restart after stopping.
- ARM build passes; upstream LemLib files remain byte-identical to the pinned copy.
- Uploaded to verified-free slot 5, then started disarmed.
- First startup reported generic sensor configuration failure; no motion command
  was sent. Added per-operation error/errno diagnostics; next startup calibrated
  successfully. Cause of the first startup failure is not established.
- Initial hardware capture: 500 frames, all cycles 10 ms, no record drops, no
  active pulse, no fault. `docs/usb-odom-20261003-173335-591.csv`.
- USB protocol check passed: sequence 1 received, zero pulse rejected, no active
  output. `docs/usb-odom-20261003-173354-577.csv`.

Physical motion and ground-truth comparison are recorded separately once run.

### Additional startup checks

The first requested pulse was **not sent**: the added test harness initially
tested `Controller::is_connected()==1`. PROS passes VEX connection states through
(including radio), despite the header documenting only 0/1. The production robot
already handled this correctly. The harness now uses the same predicate:
positive status other than PROS_ERR. Host tests cover connected states 1 and 2,
disconnected 0, negative, and PROS_ERR. This was a test-harness error, not an
explanation of the original odometry drift.

The CSV now includes an `interlocks` mask: 1 controller absent/error, 2 R1 not
pressed, 4 B pressed/error, 8 no initial release yet, 16 driver callback inactive,
32 competition connected, 64 disabled, 128 autonomous. All must be clear to move.

One subsequent startup reported `IMU1 calibrazione, errno=11, battery_mv=12860`.
The program stayed stopped. A disarmed restart then completed calibration.
Do not infer that the battery was disconnected or bypass calibration failure.
The source of the intermittent native IMU EAGAIN remains unestablished.

Capture `docs/usb-odom-20261003-174058-920.csv`: 299 frames, cycle 10 ms,
interlocks=2 only, zero motor outputs, protocol sequence 1 rejected as intended.

### Actual motion, 2026-10-03

Version 1 had limits 1.8 V / 250 ms / 40 mm pulse / 300 mm session. Tests:

- `174213-260`: 1.2 V forward, exactly 200 ms; forward pod +1.481 mm.
- `174324-560`: 1.8 V forward, 250 ms; forward pod approximately +5.337 mm.
- `174431-159`: 1.8 V opposite sides, 250 ms; yaw approximately +0.347 deg.

The user then explicitly requested at least 3 V and 1 second. Version 2 raises
the command maxima to exactly 3 V / 1000 ms, with the current 150/450 mm encoder
thresholds and the same local/USB interlocks. Guard tests were rerun, including
a full 1000 ms simulated pulse with fresh heartbeats.

- `docs/usb-odom-20261003-174747-595.csv`: requested straight 3 V / 1000 ms;
  encoder threshold stopped power after **660 ms**, with coast-down afterward.
  Pod +168.167 mm; NEXUS settled change (+3.895,+168.193) mm and LemLib
  (+3.886,+168.194) mm, disagreement 0.00955 mm. User confirmed roughly 17 cm
  forward with regular motion. All logged control cycles 10 ms, zero record drops.
- User confirmed ports **4 and 9 have 200 RPM cartridges with different external
  gearing**, while the other four drive motors have 600 RPM cartridges. Thus the
  individually logged `motor_*_mm` fields, all currently scaled with the same 0.6
  ratio, are **not all calibrated wheel distances**. Do not interpret their
  approximately threefold difference as slipping, disconnection, or pod failure.
  Per-port conversion/gearing readback still needs verification before changing
  production hardware configuration. The pod/IMU pose comparison does not use
  these drive readings as its active heading/translation source in this test.
- `docs/usb-odom-20261003-175313-762.csv`: forward right curve, L 3 V / R 1.5 V,
  requested 1000 ms, bounded by the pulse encoder threshold. Final pose NEXUS
  (18.904,311.464,6.15427 deg), LemLib (18.886,311.463,6.15156 deg).
- The following attempted opposite curve timed out waiting for R1 and did not
  send a pulse (`175501-409`). A later explicitly confirmed R1 press allowed the
  opposite curve; the **450 mm accumulated session threshold** then latched the
  motor stop (reason 5). Subsequent stopped capture
  `docs/usb-odom-20261003-175704-352.csv`: NEXUS (27.459,452.893,1.32703 deg),
  LemLib (27.435,452.891,1.32449 deg). All voltages zero, fault latched.

These are real hardware tests, but agreement of the two estimates does not
establish physical accuracy or resolve the original closed-S drift. The final
physical lateral displacement has been requested from the user. No claim of
fixing the original 30 mm error has been made.

### Larger S requested by user

The user confirmed the approximately 3 cm right / 46 cm forward physical result,
but asked for a larger S with at least approximately four seconds of motion.
They then returned the robot to the initial clear area and marked two physical
references for position and orientation. USBODOM 3 was built with the limits
above; host tests include full 2000 ms pulses and all envelope exits.

`docs/usb-odom-20261003-180826-227.csv` is the continuous larger-S recording:

- Pulse sequence 2: L 3000 mV / R 1000 mV, **2000 ms**, 200 active records.
- Pulse sequence 3: L 1000 mV / R 3000 mV, **2000 ms**, 200 active records.
- Both pulses ended by normal duration (reason 3), without early limits.
- Max IMU heading 22.78869 deg; max cycle 10 ms; no record drops.
- Final NEXUS (130.987,681.733,-0.32988 deg), LemLib
  (130.951,681.738,-0.33258 deg), units mm/degrees.
- The small initial heading-reference difference is approximately 0.003 deg;
  compare changes from baseline, not just absolute displayed headings.
- Summary: `docs/usb-odom-wide-s-20261003.json`.

The user was asked to release R1 and manually align both physical reference
marks **without restarting or zeroing the program**, to measure closure against
external ground truth. The first 60-second passive capture
`docs/usb-odom-20261003-181022-797.csv` still shows the robot stationary at the
outbound endpoint. Thus it does **not** yet contain a completed closed-path test.
No movement commands are sent during the passive return recordings.

### Closed-path results and second larger S

The user subsequently confirmed exact alignment to the two initial physical
marks. The settled first-S read `docs/usb-odom-20261003-181619-822.csv` gives
NEXUS (2.446,-5.662,-1.46214 deg), LemLib (2.523,-5.660,-1.46522 deg):
approximately 6 mm positional closure error, shared by both algorithms. The
first return was only partly captured (`181218-564`), so do not replay that
partial recording as a complete loop.

For a second, more curved S, the unchanged USBODOM 3 firmware was restarted
at the marked origin. `docs/usb-odom-20261003-181754-035.csv` records the whole
outbound motion and manual return continuously:

- L 3000 / R 500 mV for 2000 ms, then L 500 / R 3000 mV for 2000 ms.
- Both pulses finished by duration; IMU reached 30.38221 degrees.
- Outbound settled NEXUS approximately (134.780,511.001) mm.
- 12,303 consecutive records at exactly 10 ms, zero dropped records.
- After manual return: NEXUS (-9.163,13.088,0.19628 deg), LemLib
  (-9.167,13.088,0.19302 deg), approximately 16 mm closure error in both.
- Later physical alignment confirmation and passive read
  `docs/usb-odom-20261003-182638-365.csv`: NEXUS (-9.157,13.130,0.20121 deg),
  LemLib (-9.162,13.129,0.19748 deg). Both motor voltages remain zero.
- The 900 mm cumulative-travel stop latched during the manual return, as
  expected. Odometry and logging continued; no further powered command was
  issued after the two pulses completed.

`tools/analyze_usb_odom_closure.py` independently integrates the raw pod/IMU
increments. Results are in `docs/usb-odom-closure-20261003.json`. Relative to
the trace baseline, replay closure is (-9.2062,13.0761) mm; its maximum
trajectory discrepancy from NEXUS is 0.033 mm. This is numerical reproduction,
not a new physical measurement. The initial heading references differ slightly.

A constant relative IMU shift scanned from -200 to +200 ms (10 ms steps) leaves
closure magnitude at least 15.62 mm. This particular constant-lag model alone
does not explain the closure error; it does not exclude variable latency or
separate pod delays. Powered wheel-yaw correlation is best at +55 ms advance
of IMU, but drivetrain slip/effective track width confound that comparison.
No time shift or calibration was applied to firmware based on this fit.

The next isolated test is prepared, but has not moved yet: straight 3000 mV on
both sides for 2000 ms, then passive logging of manual return to the same marks.
Disarmed restart and zero-command rejection passed in
`docs/usb-odom-20261003-182927-110.csv`. A new local R1 confirmation is pending.

That confirmation subsequently arrived. Straight trace `183057-077` completed
183 active records (1830 ms) before the 600 mm per-pulse encoder bound stopped
power. Settled position approximately (8.073,617.144) mm. The host previously
recognized only timeout as the end of its command phase, so it was interrupted
while the robot was stopped to switch to passive recording. There is one
incomplete final CSV record, explicitly omitted by the closure analysis tool.
The host now also enters passive mode after other final-pulse stop reasons;
the firmware bounds did not change.

Passive return trace `docs/usb-odom-20261003-183207-344.csv` contains the whole
manual return. The user confirmed exact alignment to both physical marks:
NEXUS (6.412,0.683,-0.33412 deg), LemLib (6.407,0.683,-0.33781 deg).
The session motor stop latched during manual return as expected. All motor
outputs remained zero. The user then requested a controlled rotation to check
pod offsets; its isolated diagnostic is `.test-build-usb-rotation` (USBODOM 4).
