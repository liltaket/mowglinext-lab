# Field status — 2026-07-28

## Confirmed baseline

- Platform: YardForce SA650ECO, Raspberry Pi 5 (8 GB).
- Wheel scale was measured over a tape-measured straight run and validated:
  `ticks_per_meter = 332.2`.
  - Old active value: 365.0.
  - Validation: 5.009 m wheel odometry over a 5.000 m tape distance (0.18%).
- Active persistent appliance config was updated at
  `docker/config/mowgli/mowgli_robot.yaml`; a timestamped backup was retained
  on the mower before the change.

## Yaw / straight-line finding

In a command with `v=0.25 m/s`, `w=0` over approximately 5 m:

- Wheel odometry reported only -0.53 degrees of yaw change.
- Fused map pose and GNSS both indicated roughly 33 degrees of left yaw drift.
- The built-in `/hardware_bridge/imu` topic does not publish usable data.
- The external USB H7/ICM IMU is the available yaw-rate source on `/imu/data`.
  It is currently delivered at about 8.9 Hz, despite the bridge's requested
  50 Hz polling loop.

**Conclusion:** changing `wheel_track` would mask the symptom rather than
correct the physical drive imbalance. The firmware yaw loop cannot be trusted
until it has a usable gyro input. The first lab experiment is a bounded,
external-IMU host yaw hold tool.

## External-IMU yaw-hold smoke test

- Date: 2026-07-28.
- Tool: `tools/lab/external_imu_yaw_hold.py`.
- Commanded a blade-free, bounded 0.80 m forward run in `RECORDING` mode.
- Result: stopped automatically at 0.834 m of wheel odometry; wheel-local
  cross-track was -0.027 m and wheel yaw was -3.71 degrees.
- The controller completed and published zero velocity before `RECORDING` was
  cancelled. The mower was subsequently verified `IDLE`, blade-disabled,
  0 RPM, and clear of active/latched emergency and lift warning.

This proves the external-IMU control path and fail-safe stop path work in a
short field test. It does **not** establish final gains or a straight-line
correction: a longer tape-measured run with a physical heading observation is
still required.

## 2026-07-28 handoff: external-IMU tuning

- The USB flight-controller IMU bridge was changed on the mower from 8.9 Hz to
  a stable 23.8 Hz by reducing its MSP serial-read timeout. The live bridge is
  a local replacement container named `mowgli-fc-imu-fast`; its predecessor is
  retained stopped as `mowgli-fc-imu`.
- Bounded, blade-free 5 m tests used `RECORDING` mode only. The most stable
  tested yaw-hold settings were `kp=0.5`, `ki=0`, and `max_wz_rad_s=0.08`.
- Wheel odometry disagreed with field observation about turn direction during
  trim experiments. Do not use wheel yaw alone for trim direction; record and
  compare `/odometry/filtered_map`, `/gps/fix`, and `/gps/pose_cov`.
- The temporary constant-trim experiment was removed from the lab tool. The
  tool retains only filtered gyro-rate feedback (`gyro_filter_alpha`, default
  0.35) and its existing blade-off/fresh-data guards.
- No appliance credentials, IP addresses, NTRIP values, or bag files are in
  this repository. Live bags remain under `/tmp` on the mower.

## Safety state used for field tests

- Tests use BehaviorTree `RECORDING` mode only, then `COMMAND_RECORD_CANCEL`.
- Blade command was verified off (`mow_enabled=false`, 0 RPM).
- No active or latched emergency; lift warning clear.
- All test tools must publish zero velocity in `finally` and use a bounded
  distance and time.

## 2026-07-28 safety-supervisor shadow deployment

- The isolated `mowgli_safety` + `mowgli_bringup` overlay was built on the
  Raspberry Pi from the lab branch and mounted over the normal runtime image.
- `/safety_supervisor` is running with `shadow_mode=true`. Its first live
  diagnostic was `NORMAL`, with zero impact evidence and no emergency request.
- Deployment preflight was blade-disabled, 0 RPM, no active/latched emergency,
  and no lift warning. No motion or blade command was issued for this deploy.
- After the ROS container restart, `/hardware_bridge/status` reported
  `firmware_compatible=false` while still reporting firmware `129.8.131` and
  protocol `5`. Treat this as a fail-closed preflight blocker: do not enable
  motion or change safety enforcement until the existing bridge handshake is
  understood and returns compatible again.

## 2026-07-28 dedicated safety diagnostics deployment

- `/safety_supervisor/diagnostics` was deployed as a dedicated Foxglove topic,
  separate from the high-volume shared `/diagnostics` stream.
- Live verification found one publisher (`/safety_supervisor`) and a stable
  approximately 4–5 Hz normal-state rate. Safety state/trip/reason changes are
  published immediately.
- The verified sample was `NORMAL`, `trip=NONE`, `shadow_mode=true`, with zero
  impact evidence and no emergency request. Blade status remained disabled at
  0 RPM; no motion or blade command was issued.

## 2026-07-28 live enforcement-mode parameter change

- With explicit owner approval, `shadow_mode` was changed live to `false`
  while the existing physical emergency remained latched. This was a
  parameter-only change: no motion, blade, emergency-release, or firmware
  command was issued.
- Post-change verification: Safety Supervisor was `NORMAL`, `trip=NONE`,
  `shadow_mode=false`; `mow_enabled=false`, 0 RPM; emergency was still
  `latched=true` with the existing physical-release instruction.
- This live parameter value is not persistent and returns to the YAML default
  (`safety_shadow_mode: true`) after a ROS container restart.

## 2026-07-28 safety crash repair and conservative blade-off tuning

- A post-test crash of `safety_supervisor` was diagnosed as mixed ROS/system
  time subtraction in the dedicated diagnostics rate limiter. The repaired
  node was deployed and verified alive, publishing at approximately 4–5 Hz
  with enforcement enabled (`shadow_mode=false`).
- The detector now explicitly reports its `rearmed after stable stop`
  transition after physical rearm plus two seconds of level, stationary state.
- Temporary blade-off tuning was applied live: absolute tilt 35 deg, hard tilt
  50 deg, relative tilt 20 deg, rapid tilt 25 deg at 60 deg/s; impact accel
  4.5 m/s2, jerk 30 m/s3, gyro 1.2 rad/s, and one required evidence signal.
  No motion or blade command was issued. These thresholds reset on restart.
