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

## Safety state used for field tests

- Tests use BehaviorTree `RECORDING` mode only, then `COMMAND_RECORD_CANCEL`.
- Blade command was verified off (`mow_enabled=false`, 0 RPM).
- No active or latched emergency; lift warning clear.
- All test tools must publish zero velocity in `finally` and use a bounded
  distance and time.
