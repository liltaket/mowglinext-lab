# Safety supervisor field-validation gate

The safety supervisor uses the Raspberry Pi USB IMU at `/imu/data`, `/wheel_odom`, final twist-mux output `/cmd_vel`, `/hardware_bridge/status`, and `/hardware_bridge/emergency`. It does not read `/hardware_bridge/imu/data_raw`, open the serial port, change STM32 firmware, or issue an emergency release.

## First deployment: shadow mode only

1. Keep `safety_shadow_mode: true` in `mowgli_robot.yaml`.
2. Confirm blade off, an operator is present, there is no active emergency or lift warning, and use a bounded RECORDING-mode forward test only.
3. Rebuild/restart the ROS container, then verify:

```bash
ros2 node list | grep safety_supervisor
ros2 param get /safety_supervisor shadow_mode
ros2 topic echo /diagnostics --once
```

4. Record a bag during representative flat grass, slope, stop, turn, bump, and blade-off controlled tests:

```bash
ros2 bag record /imu/data /wheel_odom /cmd_vel /hardware_bridge/status \
  /hardware_bridge/emergency /diagnostics
```

5. Review every `Safety Supervisor` warning. A candidate is expected to remain a diagnostic event in shadow mode; no `/hardware_bridge/emergency_stop` request may be made.

### Pi without the lab fork

Keep the Pi's current Mowgli checkout as the runtime checkout. After the safety branch has been pushed to `liltaket/mowglinext-lab`, copy or run the bootstrap script and set the two paths explicitly:

```bash
MOWGLI_RUNTIME_DIR="$HOME/mowglinext" \
MOWGLI_SAFETY_REPO_REF="<pushed-safety-branch>" \
bash bootstrap_safety_shadow.sh
```

This clones the lab fork into `$HOME/mowgli-safety-shadow-source`, builds only the two overlay packages, and mounts the resulting overlay over the existing runtime container. It does not replace the Pi's runtime checkout. Roll back from the runtime checkout with:

```bash
MOWGLI_RUNTIME_DIR="$HOME/mowglinext" \
MOWGLI_SAFETY_SOURCE="$HOME/mowgli-safety-shadow-source" \
$HOME/mowgli-safety-shadow-source/tools/lab/deploy_safety_shadow.sh rollback
```

## Replay and tuning

Replay bags with the node in shadow mode and use ROS parameters for temporary changes:

```bash
ros2 param set /safety_supervisor tilt.absolute_trip_deg 52.0
ros2 param set /safety_supervisor shadow_mode true
```

Do not change the central configuration to enforcement until replay and supervised blade-off observations show no unexplained candidates. GUI settings are saved for the next container restart; ROS parameters are the live tuning path.

## Enforcement gate

Only after the owner explicitly approves the observed replay/field result may `shadow_mode` be changed to false. Before doing so, verify the service type and a safe, supervised stationary test plan. The node may then request only:

```text
/hardware_bridge/emergency_stop: {emergency: 1}
```

It never sends `{emergency: 0}`. Emergency reset remains a physical/operator and existing Mowgli/STM32 responsibility.
