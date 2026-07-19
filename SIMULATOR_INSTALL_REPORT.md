# MowgliNext simulator — local installation record

Date: 2026-07-19  
Host: Windows, Docker Desktop Linux engine (`x86_64`)

## Source checked out

- Repository: `https://github.com/mowglinext/mowglinext.git`
- Branch: `main`
- Commit: `cb7176c67537c71d3e85cd70a67ce2ef83c5c382` (`2026-07-19`, `Merge branch 'dev'`)
- Git submodules required by the Docker build:
  - `ros2/src/external/universal-gnss` at `d0f59a510f19fa834d9344c186d21583cca28cd1`
  - `ros2/src/opennav_coverage` at `d6e41a298c5e9767cc2a42c9fafa7ca41b9bf089`

The directory was an empty, already-initialized Git repository, so the upstream
history was fetched into it rather than deleting existing Git metadata.

## Dependencies

Already present on the host:

- Docker Desktop 28.5.1 (Engine 28.5.1)
- Docker Compose v2.40.3-desktop.1

No ROS, Gazebo, Webots, sensor, RTK, firmware, or hardware packages were
installed on the host. Docker built the container from
`ros2/Dockerfile`, whose declared runtime/build dependencies include ROS 2
Kilted, Nav2, Foxglove Bridge, Cyclone DDS, GTSAM 4.3a1, Fields2Cover 2.0.0
and a pinned Fields2Cover v3 revision, plus the project workspace and its two
submodules. The complete authoritative package list is the `apt-get install`
instructions in that Dockerfile.

The resulting local image is `docker-simulation-gui:latest` (5.64 GB).

## Commands run

```powershell
# Obtain current upstream source in the existing empty repository
git remote add origin https://github.com/mowglinext/mowglinext.git
git fetch --tags origin main
git checkout -B main --track origin/main

# Obtain build-required nested repositories
git submodule sync --recursive
git submodule update --init --recursive

# Start Docker Desktop, then wait until its Linux engine answers
Start-Process -FilePath 'C:\Program Files\Docker\Docker\Docker Desktop.exe' -WindowStyle Hidden
docker version

cd docker
docker compose -f docker-compose.simulation.yaml build simulation-gui
docker compose -f docker-compose.simulation.yaml up -d --force-recreate simulation-gui
docker compose -f docker-compose.simulation.yaml ps --all
docker compose -f docker-compose.simulation.yaml logs --tail=200 simulation-gui
docker compose -f docker-compose.simulation.yaml down
```

## Windows line ending correction

The machine-wide Git setting `core.autocrlf=true` checked out shell scripts
with CRLF. Linux then rejected the Docker entrypoint with:

```text
exec /ros2_entrypoint.sh: no such file or directory
```

The actual file existed; its shebang ended with `CRLF`. The local
`.gitattributes` file sets `*.sh text eol=lf`, and the two scripts copied and
executed by the GUI service were rewritten with LF only:

- `ros2/scripts/ros2_entrypoint.sh`
- `ros2/scripts/start_vnc.sh`

No application logic was changed.

## Verification result

| Check | Result |
| --- | --- |
| Docker Linux engine running | Pass |
| Compose configuration parses | Pass |
| Required submodules available | Pass |
| `simulation-gui` image builds | Pass |
| GUI service can create VNC/noVNC | Pass |
| Simulator process starts | **Fail** |
| Virtual robot loads | **Fail** |
| Critical-error-free simulation | **Fail** |

The current official Compose file and wiki still label the simulator as
Gazebo, but the current source has migrated its launch path to Webots. The
container fails immediately with:

```text
[ERROR] [launch]: Caught exception in launch (see debug for traceback): No module named 'webots_ros2_driver'
```

This is also declared directly in `ros2/Dockerfile`: the simulation stage says
the Gazebo packages were removed and Webots plus `webots_ros2_driver` are
still TODO. Therefore no running simulator or virtual robot can honestly be
verified from the current upstream revision. This is an upstream source/image
defect, not a hardware or RTK configuration issue.

## Start, stop, and logs

The official command currently reaches the known upstream error above:

```powershell
cd C:\Users\bruno\OneDrive\Dokument\Mowgli-next\docker
docker compose -f docker-compose.simulation.yaml up -d simulation-gui
```

Once upstream supplies the missing simulation driver (or restores a supported
Gazebo stack), use the same command to start it. Then open the noVNC URL
advertised by the project, `http://localhost:6080/vnc.html`.

```powershell
# Status and logs
docker compose -f docker-compose.simulation.yaml ps --all
docker compose -f docker-compose.simulation.yaml logs -f simulation-gui

# Stop and remove the service created by this Compose project
docker compose -f docker-compose.simulation.yaml down
```

No Yard Force, LC29H, RTK, external sensor, motor-control, or firmware files
were configured or changed.

## Remediation completed after the initial check

The current upstream simulation stage was incomplete on Windows + Docker
Desktop. The following local simulator-only fixes were applied and verified:

- installed the official `ros-kilted-webots-ros2` package in the simulation
  image, including its `webots_ros2_driver`;
- installed the official Cyberbotics Webots R2025a Linux package and set
  `WEBOTS_HOME=/usr/local/webots`;
- prevented the ROS Webots driver from treating a Docker Desktop Linux
  container as a WSL host;
- restored `docker/config/cyclonedds.xml` from the repository's installer
  default. The directory Docker had created at that path was retained as
  `docker/config/cyclonedds.xml.created-by-docker-backup`;
- corrected the GUI Compose command so `start_vnc.sh` does not start the ROS
  simulation a second time;
- exposed noVNC and Foxglove through Docker Desktop with ports `6080:6080`
  and `8765:8765`.

Final verification passed:

- `mowgli_sim_gui` is running;
- `http://localhost:6080/vnc.html` returns HTTP 200 and connects to noVNC;
- Webots visibly renders and its world tree contains `MowgliMower` and
  `Ros2Supervisor`;
- the ROS graph includes `/MowgliMower`, `/Ros2Supervisor/Ros2Supervisor`,
  Nav2 nodes, and Foxglove Bridge;
- `/clock`, `/scan`, `/cmd_vel_wheels`, and `/wheel_odom` are present.

The Webots console warns that rendering uses the CPU software renderer. This
affects performance only; it did not prevent the robot or simulator from
loading.

## Reproducible build and startup from a clean checkout

```bash
git clone --recurse-submodules https://github.com/mowglinext/mowglinext.git
cd mowglinext
git submodule update --init --recursive

docker compose -f docker/docker-compose.simulation.yaml build simulation-gui
docker compose -f docker/docker-compose.simulation.yaml up -d simulation-gui
```

Open `http://localhost:6080/vnc.html` after Webots has started. To stop the
stack while retaining the named `mowgli_sim_maps` volume, run:

```bash
docker compose -f docker/docker-compose.simulation.yaml down
```

To verify that a clean container recreation preserves the simulator
interfaces, rebuild and start it again:

```bash
docker compose -f docker/docker-compose.simulation.yaml build simulation-gui
docker compose -f docker/docker-compose.simulation.yaml up -d --force-recreate simulation-gui
docker exec mowgli_sim_gui bash -lc '
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 topic info /wheel_odom
  ros2 topic info /imu/data
  ros2 topic info /gps/fix
  ros2 topic info /odometry/filtered
  ros2 topic info /sim/ground_truth_pose
'
```

Expected publishers are `sim_wheel_slip`, `sim_imu_noise`,
`sim_navsat_rtk_fix`, `fusion_graph_node`, and
`MowgliMower_kinematic_drive`, respectively. The simulated raw GPS topic
`/gps/fix_raw` has status `-2`; `sim_navsat_rtk_fix` deliberately promotes it
to status `2` on the production-facing `/gps/fix` topic. `/wheel_odom_raw`
comes from `diffdrive_controller`, and `sim_wheel_slip` relays it to
`/wheel_odom` with the production covariance contract.
