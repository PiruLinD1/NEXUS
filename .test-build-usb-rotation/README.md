# USBODOM 4: supervised rotation diagnostic

Separate copy of USBODOM 3, with original LemLib and NEXUS estimator sources
unchanged. Only the drive-test guard/UI changes. No mechanism objects exist.
Upload to the existing diagnostic slot 5; never start production NEXUS for this test.

The user explicitly requested motor-controlled full rotation because manually
keeping the centre fixed is difficult. This test estimates effective pod offsets
about the actual rotation trajectory. Unequal slip/centre translation can bias
that estimate; the result is not automatically saved as calibration.

Only equal/opposite voltage commands are accepted, at most 3 V and 2000 ms.
Every pulse stops at 90 degrees of measured rotation, 300 mm indicated travel
of any motor, or the original R1/USB/mode/sensor interlocks. Session limits:
1250 mm indicated accumulated path per motor, 400 degrees from initial IMU,
and estimated centre position within +/-150 mm on both axes. An opposite yaw
over 5 degrees latches stop. Angle/pose limits supplement the independent time
and encoder bounds; they do not establish actual physical clearance.

`tools/usb_odom_turn.ps1` sends four such pulses, with 1.5-second pauses and
continuous R1 required. Any early stop cancels subsequent pulses. After the
last angular stop, logging becomes passive (no heartbeat or movement commands).
The user must check the full rotating footprint and cable slack before R1.
Angular coast-down can make the total exceed 360 degrees slightly; retain raw
heading and distinguish the rotation endpoint from later manual realignment.

`tools/analyze_usb_odom_rotation.py` reports candidates at stable cardinal
plateaus: F offset = -delta_forward / delta_heading; L offset = delta_lateral /
delta_heading. These formulas assume a stationary chosen centre and calibrated
pod scale. A second direction and physical measurements are needed to assess
repeatability and confounding translation. Never describe a candidate as a
confirmed mechanical distance from motor motion alone.

Host guard assertions enabled: angle stop in both directions, wrong-direction
stop, rejection of straight/zero/out-of-range commands, R1, heartbeat, timeout,
each-motor travel, total travel, invalid readings, envelope, and four quarters.
Host tests and PowerShell syntax check passed before upload.

## Hardware preparation

Uploaded to slot 5 and started disarmed. Zero-command rejection passed in
`docs/usb-odom-20261003-184638-275.csv`. First requested rotation attempt
`docs/usb-turn-20261003-185356-518.csv` sent **no pulse**: telemetry already
reported a latched envelope fault, pose (2.765,734.228) mm and IMU 16.27666 deg,
after displacement since the diagnostic startup. The cause of that displacement
was not inferred. The user was instructed to release R1 and remain stationary.
Disarmed restart established a new origin at the current cleared location;
`docs/usb-odom-20261003-185436-712.csv` verifies near-zero pose, fault=0,
R1 released, zero motor voltages, and zero-pulse rejection. Awaiting renewed R1
press, with two physical reference marks requested at this new origin.

## Completed hardware rotations, 2026-10-03

Both directions were explicitly armed by the user holding R1. All eight powered
quarters reached the local 90-degree stop (reason 10) within 2000 ms. Coast-down
added about 4-5 degrees per quarter; do not call these exact 360-degree powered
trajectories. The user subsequently aligned position AND orientation to the
same two physical reference marks, separately for each run.

| Direction / trace | Powered durations, ms | Heading before manual alignment | Final NEXUS X/Y, mm | Final LemLib X/Y, mm |
| --- | --- | --- | --- | --- |
| CW `docs/usb-turn-20261003-185527-596.csv` | 1440, 1360, 1290, 1280 | 380.006 deg | 5.867 / 5.448 | 5.863 / 5.447 |
| CCW `docs/usb-turn-20261003-185846-589.csv` | 1610, 1390, 1370, 1320 | -379.296 deg | -1.891 / 4.409 | -1.895 / 4.408 |

CW recorded 10,098 contiguous 10 ms frames; CCW 10,128; neither dropped records.
Native IMU after physical alignment was +360.705 and -360.711 degrees,
respectively (initial approximately +0.037/+0.036 degrees). This is a measured
residual, not a certified gyro scale calibration: reference alignment accuracy
has not been quantified. Both logs finish with R1 released and zero outputs.

The quarter-wise raw candidates are in `docs/usb-turn-clockwise-20261003.json`
and `docs/usb-turn-counterclockwise-20261003.json`:

- Forward candidate after longitudinal drive compensation: mean 15.08 mm CW,
  14.79 mm CCW. Compensation uses only the four blue motor counters with their
  nominal wheel scale; drive slip can still bias it.
- Lateral candidate: mean -36.34 mm CW, -28.52 mm CCW. Individual quarters span
  -41.92 to -21.02 mm across both runs. Unknown lateral centre motion is not
  observed independently and remains confounded with this parameter.
- Configured physical offsets are +20 and -70 mm. These data **do not prove**
  that the physical mounting has those candidate dimensions or justify saving
  them as calibrated values. Forward-translation compensation cannot remove
  unmeasured lateral translation.

Offline replays in `docs/usb-offset-comparison-20261003.json` compare configured
offsets (+20,-70), candidates (+15,-36), (+15,-29), (+15,-32.5), and zero offsets.
The complete S closure is 15.9918 mm with configured offsets and 16.0415 mm
with (+15,-32.5); its endpoint changes only 0.1066 mm. Therefore these candidate
offsets do not resolve the S closure error. The CW/CCW closure magnitudes change
from 7.9613/4.8036 to 7.6081/4.6929 mm (trace-baseline-relative values).

This follows the rigid-body geometry: a constant offset correction integrated
around a path depends on the initial/final orientation, and cancels when those
orientations coincide. A full-turn endpoint alone cannot identify the offset.
No production configuration, calibration profile, or estimator source was
changed on the basis of these rotations. Production build hash remains
`0526431F5FB812A6077BAB928F3EA099386801E20697162B1004E863BA9B3ED4`.
