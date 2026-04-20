# Madara 6DoF Arm
> **Status: 🚧 In Progress**

A 6-degree-of-freedom robotic arm built with ROS 2 Humble, MoveIt2, and Ignition Gazebo Fortress.  
Designed to run on a Raspberry Pi 5 inside Docker with full simulation and real hardware support.

---

## Hardware

| Component | Part |
|---|---|
| SBC | Raspberry Pi 5 |
| Microcontroller | Arduino Uno |
| DC Motor (base rotation) | JGA25-371 12V 18RPM + quadrature encoder |
| Servo motors (×5) | MG966R |
| Motor driver | L298N H-Bridge |
| Camera | Raspberry Pi Camera Module v2 (IMX219) |

---

## Photos

<!-- Place your images in docs/images/ and update the filenames below -->
<p align="center">
  <img src="docs/images/IMG1.jpg" alt="Madara Arm Physical Build" width="600"/>
</p>
<p align="center"><em>Physical build</em></p>

<p align="center">
  <img src="docs/images/IMG2.jpg" alt="Some Hardware Setup" width="600"/>
</p>
<p align="center"><em>Simulation in RViz2 + Ignition Gazebo</em></p>

---

## Quick Start

```bash
