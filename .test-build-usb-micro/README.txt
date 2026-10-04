USBMICRO 1 - isolated small-motion diagnostic, 2026-10-03

Derived from USBODOM 3 without changing production firmware or earlier builds.
User authorized slight powered motion and previously reported 1 m clear ahead
and 60 cm clear to the right. The existing local R1 deadman is retained.

Planned command: two arcs L/R = 3000/1000 mV then 1000/3000 mV, each 500 ms,
with 1000 ms stopped between them. Expected forward travel is approximately
100-150 mm; this is an estimate, not an independently measured endpoint.

Board limits: 500 ms per command, 3000 mV, 120 mm per individual encoder per
pulse, 250 mm cumulative travel per encoder. Pose envelope X [-30,60] mm,
Y [-30,200] mm, heading [-10,20] degrees is an additional check. It is not the
sole travel bound. R1 release, B, controller loss, field mode, invalid sensors,
USB heartbeat older than 120 ms, excessive cycle time, and STOP stop power.
No mechanism objects. All six drive motors use a 1000 mA current limit.

NEXUS drive reference now averages only the two 600-RPM motors on each side.
Ports 4/9 remain actuated but are excluded from that reference. Legacy UD1
motor_l2_mm and motor_r2_mm still use the blue conversion and must NOT be
interpreted as calibrated ground travel; their encoder ratios are different.
Native IMU + forward/lateral pods provide the primary pose. Original LemLib
is also running; online snapshots may differ in sample timing.

Build: ARM make -j4 PASS (existing upstream LemLib warnings).
Host guard tests PASS: consent, heartbeat loss, stale/replayed commands,
duration, all six encoder bounds, cumulative travel, invalid sensors,
pose envelope, two short pulses, and no restart from heartbeat alone.
Host PowerShell parser PASS.

Uploaded to slot 5, USBMICRO, initially disarmed. Protocol check log:
docs/usb-micro-20261003-200907-109.csv
451 frames, 10 ms cycles, zero drops, no faults, no accepted pulse.
Controller connected, R1 released (interlocks=2), zero motor commands.
At preparation completion the powered trial is pending local R1 readiness.

Host command once R1 readiness is confirmed:
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tools/usb_odom_micro.ps1 -Seconds 15 -Pulse -SSequence -InnerMv 1000 -AfterCommandSeconds 5

Never infer that matching computed poses proves physical endpoint accuracy.
