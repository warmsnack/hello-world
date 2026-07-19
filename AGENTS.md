# AGENTS.md

## Cursor Cloud specific instructions

This repo is a **ROS 2 Humble + Gazebo Classic 11** colcon workspace for a TurtleBot3
dynamic-obstacle-avoidance A/B experiment (see `README.md`, which is authoritative for
build/run commands — it is in Korean). It is **not** an empty hello-world repo.

### Runs inside Docker (host VM is Ubuntu 24.04)
ROS 2 Humble + Gazebo Classic 11 only ship for Ubuntu 22.04, so everything runs in a
container built from `docker/Dockerfile` (image tag `tb3-ros:humble`, based on
`osrf/ros:humble-desktop-full`). The update script starts the Docker daemon (no systemd
here — PID 1 is `tini`, so use `sudo service docker start`), builds the image if missing,
and runs `colcon build` in the mounted workspace.

Standard dev loop (workspace is bind-mounted at `/workspace`):
```bash
sudo docker exec -it tb3 bash    # or: docker run ... tb3-ros:humble
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
export TURTLEBOT3_MODEL=waffle
colcon build --symlink-install   # build
ros2 launch tb3_experiment experiment.launch.py use_gt:=true gui:=false record:=false
```
A long-lived container named `tb3` (host networking, `ROS_DOMAIN_ID=42`) is typically left
running; use `sudo docker exec tb3 ...` to talk to it.

### Non-obvious gotchas
- **Headless only**: no display/GPU. Always pass `gui:=false` (Gazebo runs `gzserver` only).
  The lidar is a CPU `ray` sensor and IMU-only, so headless works fine; `Can't open display`
  / RenderEngine warnings and `walk.dae` actor-mesh warnings are harmless.
- **Nav2 needs `diagnostic_updater`**: `libdiagnostic_updater.so` is a runtime dep of
  `nav2_lifecycle_manager`; it is installed in the image (do NOT use `--no-install-recommends`
  for the ROS packages or Nav2 silently fails to activate).
- **Nav2 activation is slow**: give the stack ~30–40s before the `navigate_to_pose` action
  server is available and the goal client fires.
- **Mode A works; Mode B (default) has a pre-existing bug**: `use_gt:=false` starts the
  `robot_localization` EKF, which crashes because `src/tb3_state_estimation/config/ekf.yaml`
  mixes ints (`0`) and floats in `process_noise_covariance` / `initial_estimate_covariance`
  sequences — ROS 2 Humble's param parser rejects mixed-type sequences. This is an
  application/config bug, not an environment issue. Use `use_gt:=true` (Mode A) to
  demonstrate the full pipeline; fixing Mode B requires editing that YAML (out of scope for
  env setup).
- **numpy**: `evo` (Phase 5, optional) pulls numpy 2.x via pip, shadowing ROS's numpy 1.x.
  Core ROS Python (`rclpy`, the tb3 nodes) still works; only mention if a numpy ABI error
  appears.
- **Outputs**: CSVs and rosbags go to `~/tb3_eval` **inside the container** (`/root/tb3_eval`),
  not the mounted workspace — copy them out with `docker cp` if needed.
- `build/`, `install/`, `log/` live in the bind-mounted `/workspace` and are gitignored.
