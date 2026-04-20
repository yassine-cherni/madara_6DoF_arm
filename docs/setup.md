# Madara 6DoF Arm — Setup Guide

`madara_ws/src/` is created and symlinked automatically by `entrypoint.sh` — do not create it manually.

---

## Step 1 — Prerequisites (Raspberry Pi 5 or dev machine)

```bash
# Install Docker
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in

# Allow GUI apps from Docker (needed for RViz, Gazebo)
sudo apt install -y x11-xserver-utils
xhost +local:docker
```

---

## Step 2 — Clone the project

```bash
git clone https://github.com/yassine-cherni/madara_6DoF_arm.git
cd madara_6DoF_arm
```

---

## Step 3 — Build the image

```bash
docker compose build
```

The base image is `ros:humble-ros-base-jammy` from the official ROS Docker library.
It ships proper `linux/amd64` and `linux/arm64` manifests, so Docker automatically
pulls the correct variant for your host — no `--platform` flags or env vars needed.

> **First build time:** ~15–25 min on a Pi 5 (downloads ~245 MB base + installs ROS packages).  
> Subsequent builds use the layer cache and are much faster.

---

## Step 4 — Start the container

```bash
xhost +local:docker
docker compose up -d madara
docker exec -it madara bash
```

---

## Step 5 — First time only: install dependencies and build

```bash
# Inside the container
cd /madara_ws
apt-get update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Expected: all packages build cleanly. `madara_hardware_interface` takes the longest (~30s on Pi 5).

---

## Step 6 — Verify camera (real hardware only)

```bash
# Inside the container — libcamera should see the IMX219 sensor
libcamera-hello --list-cameras
# Expected: 0 : imx219 [3280x2464 10-bit RGGB] (...)

# Test the ROS camera node
ros2 run camera_ros camera_node --ros-args \
  -p width:=1280 \
  -p height:=720 \
  -p frame_id:=camera_link_optical

# In a second terminal
ros2 topic hz /camera/image_raw
# Expected: ~30 Hz
```

If `libcamera-hello --list-cameras` shows no cameras, check the host:
```bash
# On the Pi host (not inside Docker)
rpicam-hello --list-cameras
```
If it works on host but not in Docker, verify `/dev/dma_heap/` is visible inside
the container — `ls /dev/dma_heap/` should return two entries.

---

## Step 7 — Launch options

### Mock demo — no hardware, no Gazebo, no camera

```bash
ros2 launch madara_moveit_config demo.launch.py
```

Expected: RViz opens with MoveIt2 MotionPlanning panel. Camera Feed panel shows
"No data" — this is normal in mock mode.

---

### Gazebo simulation — arm + simulated camera

```bash
ros2 launch madara_bringup gz_launch.py
```

Expected sequence:
1. Ignition Gazebo opens with arm spawned in empty world
2. Controllers activate: `joint_state_broadcaster → madara_arm_controller → hand_controller`
3. After ~5 seconds: MoveIt2 move_group + RViz open automatically
4. Camera Feed panel in RViz shows the simulated camera view from Gazebo
5. Plan and execute trajectories — sim time aligned to Gazebo clock

Alternatively, use the sim profile:
```bash
docker compose --profile sim up madara-sim
```

---

### Real hardware — arm + camera

```bash
# Terminal 1 — arm controllers + camera
docker exec -it madara bash
source install/setup.bash
ros2 launch madara_bringup bringup_launch.py serial_port:=/dev/ttyUSB0

# Terminal 2 — MoveIt2 + RViz (after Terminal 1 shows all controllers active)
docker exec -it madara bash
source install/setup.bash
ros2 launch madara_moveit_config moveit_rviz.launch.py
```

Expected: RViz opens with Camera Feed panel showing live RPi Camera v2 feed
alongside the arm model and MoveIt2 MotionPlanning panel.

To launch arm only (no camera):
```bash
ros2 launch madara_bringup bringup_launch.py start_camera:=false
```

---

## Daily use (after first setup)

```bash
xhost +local:docker
docker compose up -d madara
docker exec -it madara bash
source install/setup.bash

# Pick one:
ros2 launch madara_moveit_config demo.launch.py   # mock demo
ros2 launch madara_bringup gz_launch.py           # Gazebo simulation
```

---

## Clean rebuild

```bash
# On host
docker compose down
docker compose build --no-cache
docker compose up -d madara
docker exec -it madara bash

# Inside container
cd /madara_ws
apt-get update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Expected output

| Window | What you see |
|---|---|
| **RViz2 — 3D view** | Madara arm model, camera TF frame (green box on base) |
| **RViz2 — MotionPlanning panel** | Plan + Execute works for `madara_arm` and `hand` groups |
| **RViz2 — Camera Feed panel** | Live image from RPi Camera v2 (real) or Gazebo sensor (sim) |
| **Gazebo** | Arm spawned in empty world, joint states updating |
| **Terminal** | All controllers: `Configured and activated` |

---

## Known harmless messages

| Message | Why | Action |
|---|---|---|
| `No 3D sensor plugin for octomap` | No depth camera | None |
| `Segmentation fault` on Ctrl-C | Upstream Humble MoveIt2 shutdown bug | None |
| QML `TypeError` / deprecation warnings | Ignition Gazebo 6 + Qt | None |
| `allow_nonzero_velocity_at_trajectory_end` | JTC default changing | None |
| `Camera Feed: No data` in demo.launch | No camera in mock mode | None — expected |

---

## Troubleshooting

### Architecture / Docker

| Symptom | Fix |
|---|---|
| `exec format error` during build | Wrong arch image cached — run `docker system prune -af --volumes` then rebuild |
| Build pulls amd64 on arm64 host | Base image had no arm64 manifest — fixed by switching to `ros:humble-ros-base-jammy` |
| `Cannot connect to Docker daemon` | Run `sudo systemctl start docker` |

### Camera

| Symptom | Fix |
|---|---|
| `libcamera-hello` shows no cameras inside Docker | Check host: `rpicam-hello --list-cameras`. If host works, verify `/dev/dma_heap/` is in container |
| `/camera/image_raw` at 0 Hz | Camera node not running — check `ros2 node list` |
| Image in RViz is black/grey | Camera not initialized yet — wait 3–5 seconds |
| No `Camera Feed` panel in RViz | Add Image display manually via Add button, topic `/camera/image_raw` |
| `failed to allocate buffers` | Increase CMA on host: add `dtoverlay=vc4-kms-v3d,cma-512` to `/boot/firmware/config.txt` |
