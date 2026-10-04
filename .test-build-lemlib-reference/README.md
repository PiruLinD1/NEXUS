# LemLib 0.5.6 differential audit

This standalone host target compiles **the unmodified upstream `odom.cpp`**, not a reimplementation of its position equations. It does not change the robot firmware or its estimator configuration.

Source: [LemLib v0.5.6](https://github.com/LemLib/LemLib/releases/tag/v0.5.6), commit `3388145e0d90ee3c7c17d7267a30ef64367df4cb`, downloaded on 2026-10-02. The original [odom.cpp](https://github.com/LemLib/LemLib/blob/3388145e0d90ee3c7c17d7267a30ef64367df4cb/src/lemlib/chassis/odom.cpp) is 7,920 bytes; SHA256 `26a84c6a3448c0443a398fef437ae6597d44cb3d0d5485748c3f4604005df5f6`. CMake verifies this hash. The upstream MIT license is included as `LICENSE`; copyright 2024 Liam Teale and LemLib contributors. Original `util.hpp/.cpp` and `pose.hpp/.cpp` are also retained for auditing the minimal helper definitions.

`fake/` provides sensor getters, an inert task API, and the float Pose/math operations used by `odom.cpp`. It does not emulate PROS acquisition, caches, task scheduling, or physical sensors. The drive-wheel substitute is type 1, as supplied by upstream `Chassis::calibrate` for this one-vertical/one-horizontal/IMU configuration. Thus upstream uses the IMU for heading.

## Input mapping

The robot convention is X right, Y forward, heading clockwise, forward counter positive forward and lateral counter positive right. The pinned LemLib code projects a **positive horizontal counter toward negative world X** (lines 168–171). This is explicitly verified by a negative-control test: an unadapted +100 mm horizontal increment gives X = −100.000001 mm. Consequently, identical physical input is mapped as follows:

| Value | Nexus | Pinned LemLib API |
|---|---|---|
| Forward distance | F in meters | F / 0.0254 in inches |
| Lateral distance | L in meters | −L / 0.0254 in inches |
| Forward offset | +0.020 m | +0.020 / 0.0254 inches |
| Lateral offset | −0.070 m | −0.070 / 0.0254 inches |
| Heading change | IMU, clockwise radians | Same IMU, clockwise degrees |

The lateral **offset is not negated** with its counter. The stable configuration tutorial currently says the horizontal counter grows when pushed right; that description is inconsistent with this pinned source's projection, so this audit records and tests the source-derived mapping instead of assuming the guide and code agree. No wheel-diameter constants are substituted: input is already linear distance, with the robot's scales of 1.

The common integration comparison sets Nexus gyro bias to zero and withholds drive-wheel validity to disable stationary learning/heading constraints. No distance corrections are provided. This compares the two position integration algorithms on identical F/L/IMU samples; it does **not** compare uncertainty, hardware drivers, or all the robot's fusion features. Float arithmetic in upstream LemLib versus double arithmetic in Nexus accounts for tiny numerical differences.

The additional `--native` mode instead enables `EstimatorConfig::nativeImuHeading`, retains the real drive readings and validity flags, and allows the ordinary quiet diagnostic to run. Bias learning/correction and quiet heading replacement are disabled by that policy itself. Thus it also checks that quiet transitions leave native integration equivalent to upstream LemLib. Geometric scales and offsets remain in force.

## Reproduction

From the repository root, with CMake and a C++23 host compiler:

```text
cmake -S .test-build-lemlib-reference -B .test-build-lemlib-reference/out
cmake --build .test-build-lemlib-reference/out --config Release
ctest --test-dir .test-build-lemlib-reference/out -C Release --output-on-failure
.test-build-lemlib-reference/out/Release/lemlib_compare.exe
.test-build-lemlib-reference/out/Release/lemlib_compare.exe docs/odometry-usb-01.csv
.test-build-lemlib-reference/out/Release/lemlib_compare.exe docs/odometry-usb-01.csv 130 166
.test-build-lemlib-reference/out/Release/lemlib_compare.exe --native docs/odometry-usb-01.csv
.test-build-lemlib-reference/out/Release/lemlib_compare.exe --native docs/odometry-usb-01.csv 130 166
```

The CSV must be the cleaned output of `tools/inspect_odometry_trace.py`. Replay splits at sequence gaps, epoch changes, nonincreasing host time, or invalid F/L/IMU packets. It does not interpolate. An entire-file replay starts at pose zero; a requested time subrange starts once from its first recorded X/Y/heading, then integrates only F/L/IMU. The board's recorded pose and bias are not reused thereafter. Replay fails if Nexus rejects a supplied increment or unexpectedly enters stationary mode.

## Results on the captured run

Synthetic suite: **33,266 checks passed per policy**, including straight/diagonal reversals, 500 mm / 30 degree arc plus inverse, 20 closed manual cycles, varying simultaneous F/L/yaw, small angles, both turning directions, and heading wrap. Maximum position difference among these cases: **0.002468 mm**. The deliberately stale-gyro cases affect both implementations similarly; they demonstrate an input-timing limitation, not a measured physical latency in this robot.

Real `docs/odometry-usb-01.csv`: **27,838 frames**, one contiguous segment, no invalid F/L/IMU frames. Comparing every sample, the maximum position difference is **0.000708984 mm**, maximum heading difference **0.000009110 degree**. Final positions with a zero origin are Nexus (−37.372330, −23.571190) mm and LemLib (−37.372165, −23.570857) mm.

The 130–166 second subrange contains **3,600 frames** and starts at (−1.3279, −0.2953) mm, heading −1.027894 degree. Maximum position difference across the path is **0.001177737 mm**, heading **0.000024255 degree**. Final positions are Nexus (−38.116950, −24.967402) mm and LemLib (−38.116962, −24.966390) mm.

The actual upstream integration therefore reproduces the substantial residual in this recorded movement. This rules out a meaningful difference between these two kinematic implementations as the explanation for that residual. It does not establish which physical measurement, timing relationship, or mechanical effect caused it.

Native-policy replay gives the same path differences and endpoints above with **12,837 quiet frames** across the full recording, and **387 quiet frames** in the 130–166 second subrange. Both CTest targets pass. This verifies the policy through actual quiet transitions, rather than disabling quiet by withholding wheel evidence. `native-usb-results.txt` and `native-usb-window-results.txt` contain the full replay summaries.
