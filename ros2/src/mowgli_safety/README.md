# Safety supervisor

`safety_supervisor_node` consumes the externally published USB IMU at `/imu/data`, raw wheel odometry at `/wheel_odom`, the post-mux command at `/cmd_vel`, and `/hardware_bridge/status`. It never opens the STM32 serial port and never sends an emergency release.

The detector is deterministic (`NORMAL`, `SUSPECT`, `TRIPPED`). Rollover trips require sustained absolute/relative tilt or a rapid confirmed tilt. Impact trips require two corroborating signals inside the confirmation window: dynamic IMU acceleration/jerk, unexpected wheel-speed loss while `/cmd_vel` remains active, and gyro shock. A command falling to zero starts a grace period, suppressing the expected stop response.

The central default is `shadow_mode: true`; it records candidates and publishes `Safety Supervisor` diagnostics without calling the emergency service. After supervised replay and blade-off field validation, enable enforcement live:

```bash
ros2 param set /safety_supervisor shadow_mode false
ros2 param set /safety_supervisor tilt.absolute_trip_deg 52.0
```

On a real trip the only STM32 request is `/hardware_bridge/emergency_stop` with `emergency: 1`; request retry is bounded to one outstanding request and 10 Hz. The node also publishes zero to the existing `/cmd_vel_emergency` mux lane as defense in depth. It does not reset an emergency; after an external reset it only clears its own latch after two seconds stable, upright and stationary.

For tuning, run in shadow mode and record `/imu/data /wheel_odom /cmd_vel /hardware_bridge/status /diagnostics`, then replay with `use_sim_time:=true`. Sensor timestamps are used by the detector; the node reports stale sensor data and can fail-safe on an active USB-IMU timeout.
