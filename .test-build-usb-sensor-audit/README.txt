Isolated motor/sensor readback audit — 2026-10-03

This program was briefly run in diagnostic slot 5, then USBODOM 5B restored.
It has no positive/nonzero motor output or motion-command parser. All six
drivetrain motors receive constant zero-voltage commands and coast braking.
It does not address other mechanisms or calibrate the IMU.

Stages:
0. Read runtime defaults, which are not the previous program's configuration.
1. At 1 s, construct the same blue/rotations motor groups as USBODOM 5B.
2. At 4 s, set only ports 4/9 to green and record return values.

Final trace: docs/usb-sensor-audit-20261003-195751-949.txt, 113 records.
Before stage 2, 4/9 already report motor type 1 (5.5 W), gearing 1 (200 RPM).
The four other ports report type 0 (11 W), gearing 2 (600 RPM).
The 5.5 W firmware retains its fixed 200 RPM gearing despite the blue request.
PROS setter success does not guarantee a device changed its gearing.

AUDIT1 fields after prefix: board_ms, signed_port, gearing_enum, units_enum,
raw_count, raw_timestamp_ms, position, velocity_rpm, voltage_mv, current_ma,
temperature_c, errno, motor_type.
AUDITS: board_ms, forward_centidegrees, lateral_centidegrees, imu_degrees,
imu_status, forward_reversed, lateral_reversed.
AUDITCFG records configuration stage transitions and setter return codes.

tools/capture_usb_sensor_audit.ps1 only reads the USB user port.
It creates a new log; it sends no serial input. Reboots reset sensor counter
references. Never splice measurements across program starts as one trajectory.

Hardware labels/values:
pros::E_MOTOR_TYPE_V5=0 (11 W); E_MOTOR_TYPE_EXP=1 (5.5 W).
Gearing: 0 red/100 RPM, 1 green/200 RPM, 2 blue/600 RPM.
Encoder units: 0 degrees, 1 rotations, 2 counts.
Manufacturer: https://kb.vex.com/hc/en-us/articles/10002101702932-Understanding-V5-Smart-Motor-5-5W-Performance
