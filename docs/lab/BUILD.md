# Lab build and deployment notes

## Repository layout

- `tools/lab/`: explicit, supervised field-test tools.
- `docs/lab/`: state, procedure, and collaboration notes.
- Production ROS, firmware, and GUI code remain in their normal upstream
  locations until an experiment is validated.

## External-IMU yaw hold experiment

`tools/lab/external_imu_yaw_hold.py` is intentionally a standalone ROS 2 node.
It publishes only on `/cmd_vel_tuning`, which is already routed through
`twist_mux`. It will not move unless `--enable-motion` is supplied and all
runtime guards are healthy.

Example field sequence (operator present, blade off):

```bash
# Enter blade-free RECORDING mode through the normal high-level control path.
python3 tools/lab/external_imu_yaw_hold.py --enable-motion --distance-m 0.8
```

The first test must be 0.8 m. Increase to 5 m only after checking the command
sign and stopping behaviour. Return to IDLE / cancel recording immediately
after every pass.

## Do not commit

- `.env`, tokens, passwords, and SSH material
- ROS bags, maps, or raw sensor captures
- `build/`, `install/`, `log/`, Docker image exports, or generated overlays
