# Madara 6DoF Arm

> **Status: 🚧 In Progress**

A 6-DOF robotic arm  **ROS 2 Humble · MoveIt2 · Ignition Gazebo Fortress · Raspberry Pi 5 · Docker**.

<p align="center">
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros"/></a>
  <a href="https://moveit.ros.org/"><img src="https://img.shields.io/badge/MoveIt2-Humble-orange"/></a>
  <a href="https://gazebosim.org/"><img src="https://img.shields.io/badge/Gazebo-Fortress-green"/></a>
  <img src="https://img.shields.io/badge/arch-amd64%20%7C%20arm64-lightgrey"/>
  <img src="https://img.shields.io/badge/license-MIT-yellow"/>
</p>

---

## Hardware

| Component | Part |
|---|---|
| SBC | Raspberry Pi 5 |
| Microcontroller | Arduino Uno |
| DC Motor (base) | JGA25-371 12V 18RPM + quadrature encoder |
| Servos (×5) | MG966R |
| Motor driver | L298N H-Bridge |
| Camera | Raspberry Pi Camera Module v2 (IMX219) |

---

## Photos

<p align="center">
  <img src="docs/images/IMG1.jpg" alt="Physical build" width="600"/>
</p>
<p align="center"><em>Physical build</em></p>

<p align="center">
  <img src="docs/images/IMG2.jpg" alt="Simulation in RViz2 + Ignition Gazebo" width="600"/>
</p>
<p align="center"><em>Hardware Setup</em></p>

---

## Quick Start

```bash
# 1 — Prerequisites
curl -fsSL https://get.docker.com | sh && sudo usermod -aG docker $USER
sudo apt install -y x11-xserver-utils

# 2 — Clone
git clone https://github.com/yassine-cherni/madara_6DoF_arm.git
cd madara_6DoF_arm

# 3 — Build & start
xhost +local:docker
docker compose build
docker compose up -d madara
docker exec -it madara bash

# 4 — First time only (inside container)
cd /madara_ws
apt-get update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

> **Build time:** ~15–25 min first run on Pi 5. Layer cache used on subsequent builds.
> Base image `ros:humble-ros-base-jammy` ships proper `linux/amd64` + `linux/arm64` manifests — no `--platform` flags needed.

---

## Usage

| Mode | Command |
|---|---|
| Mock demo (no HW, no Gazebo) | `ros2 launch madara_moveit_config demo.launch.py` |
| Gazebo simulation | `ros2 launch madara_bringup gz_launch.py` |
| Gazebo via Compose | `docker compose --profile sim up madara-sim` |
| Real hardware (arm + camera) | `ros2 launch madara_bringup bringup_launch.py serial_port:=/dev/ttyACM0` |
| Real hardware (arm only) | `ros2 launch madara_bringup bringup_launch.py start_camera:=false` |
| MoveIt2 + RViz (after bringup) | `ros2 launch madara_moveit_config moveit_rviz.launch.py` |

---

## Package Overview

| Package | Role |
|---|---|
| `madara_description` | URDF/xacro model, meshes |
| `madara_moveit_config` | MoveIt2 config, SRDF, kinematics |
| `madara_bringup` | Top-level launch files |
| `madara_hardware_interface` | ros2_control `SystemInterface` for Arduino/servos |
| `madara_controllers` | Controller YAML config |
| `madara_gazebo` | World, plugins, ROS↔Gazebo bridge |
| `camera_ros` | RPi Camera v2 node (libcamera) |

---

## Docker Notes

| | Detail |
|---|---|
| Base image | `ros:humble-ros-base-jammy` — multi-arch, installs MoveIt2, ros2_control, RViz2, Gazebo Fortress, libcamera |
| Services | `madara` (dev / real HW) · `madara-sim` (sim profile, auto-launches `gz_launch.py`) |
| Device access | `privileged: true` + `/dev` volume required for `/dev/dma_heap/` (libcamera DMA on RPi 5) |
| Build persistence | `madara_ws_build` named volume preserves `colcon` build across container restarts |
| Groups | Container root added to `dialout` (UART), `video` + `render` (camera DMA) |

> **`privileged` note:** needed for libcamera DMA heap on RPi 5. For production, replace with fine-grained `devices:` entries once `/dev/dma_heap/` paths are confirmed.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `exec format error` | Wrong arch cached — `docker system prune -af --volumes` then rebuild |
| No cameras in container | Check host: `rpicam-hello --list-cameras`; verify `ls /dev/dma_heap/` inside container |
| `failed to allocate buffers` | Add `dtoverlay=vc4-kms-v3d,cma-512` to `/boot/firmware/config.txt` |
| `Segfault` on Ctrl-C | Known Humble MoveIt2 shutdown bug — harmless |
| `Camera Feed: No data` | Expected in `demo.launch.py` (mock mode) |

---

## TODO

> Each item is a separate planned extension — independent branch or sub-package when implemented.

### 🔲 RTEMS Support
Real-time joint control on an RTEMS target (RP2350 / Hazard3 RISC-V core).
- [ ] Minimal `madara_rtems_controller` task: PWM output + quadrature decoding
- [ ] Serial/SPI bridge to the ROS 2 hardware interface on Pi 5
- [ ] CI: RTEMS RSB cross-compiler (`rtems-riscv`) + OpenOCD flash/debug

### 🔲 Pixi — ROS 2 Jazzy & Kilted
Native host install via [Pixi](https://prefix.dev/) alongside Docker.
- [ ] `pixi.toml` with `[feature.jazzy]` + `[feature.kilted]` environments (robostack channels)
- [ ] MoveIt2 Jazzy API migration
- [ ] CI matrix: `humble` (Docker) | `jazzy` (Pixi) | `kilted` (Pixi nightly)

### 🔲 Zenoh-pico + Zephyr
DDS-native comms on the microcontroller via [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico) on Zephyr RTOS — replaces serial bridge.
- [ ] Zephyr workspace (`west`) for RP2040/RP2350 with zenoh-pico module
- [ ] Publish `/joint_states`, subscribe `/joint_commands` over Zenoh (UART or UDP)
- [ ] Bridge to ROS 2 graph via `zenoh-bridge-ros2dds` on Pi 5

### 🔲 ros2_control — Hardware Interface Refactor
Harden the hardware interface and close the control loop.
- [ ] Lifecycle-managed `SystemInterface` with `HW_IF_POSITION` + `HW_IF_VELOCITY` for all 6 DOF
- [ ] `HandSystemInterface` for the gripper (1 DOF)
- [ ] Encoder ticks as `StateInterface` → closed-loop PID via `pid_controller`
- [ ] `controller_manager` mock hardware tests in CI

### 🔲 Policy Training
Learn a manipulation policy on top of the MoveIt2 stack.
- [ ] Record demonstrations via RViz2 (rosbag2): `joint_states` + `camera/image_raw`
- [ ] Train a behaviour-cloning baseline (PyTorch)
- [ ] Inference node publishing to `madara_arm_controller` trajectory topic
- [ ] RL fine-tuning (sim-to-real) using Gazebo as training environment
- [ ] Evaluate with pick-and-place benchmark (ArUco marker target)

---

## License

MIT — see [LICENSE](LICENSE).

---
