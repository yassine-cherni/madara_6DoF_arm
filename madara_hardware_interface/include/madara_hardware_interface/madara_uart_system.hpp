// Copyright 2026 Yassine Cherni — madara_6DoF_arm
// SPDX-License-Identifier: Apache-2.0
//
// madara_uart_system.hpp
// ─────────────────────
// ros2_control SystemInterface plugin that communicates with an Arduino Uno
// over UART using a compact binary framing protocol.
//
// ┌─────────────────────────────────────────────────────────────────┐
// │  UART PROTOCOL  (binary, little-endian, 115200 8N1)            │
// │                                                                 │
// │  CMD frame (ROS → Arduino)  17 bytes                           │
// │  ┌──────┬──────┬──────┬─────┬──────────────────────┬──────┐   │
// │  │ 0xA5 │ 0x5A │ 'C'  │ seq │ j0..j5 int16 [mrad]  │ crc8 │   │
// │  └──────┴──────┴──────┴─────┴──────────────────────┴──────┘   │
// │  Bytes:  0      1      2     3    4-5 6-7 8-9 10-11 12-13 14-15 16│
// │  CRC-8/SMBUS over bytes [2 .. 15]                               │
// │                                                                 │
// │  STA frame (Arduino → ROS)  11 bytes                           │
// │  ┌──────┬──────┬──────┬─────┬──────────────┬──────────┬──────┐│
// │  │ 0xA5 │ 0x5A │ 'S'  │ seq │ enc  int32   │ vel int16│ crc8 ││
// │  └──────┴──────┴──────┴─────┴──────────────┴──────────┴──────┘│
// │  Bytes:  0      1      2     3    4-7            8-9      10    │
// │  enc : raw encoder ticks (int32, zeroed at Arduino boot)        │
// │  vel : velocity in mrad/s (int16)                               │
// │  CRC-8/SMBUS over bytes [2 .. 9]                                │
// └─────────────────────────────────────────────────────────────────┘
//
// Joint index mapping (must match Arduino firmware):
//   0  top_plate_joint   — DC motor (JGA25-371), encoder feedback
//   1  lower_arm_joint   — MG966R servo, open-loop
//   2  upper_arm_joint   — MG966R servo, open-loop
//   3  wrist_joint       — MG966R servo, open-loop
//   4  claw_base_joint   — MG966R servo, open-loop
//   5  right_claw_joint  — MG966R servo, open-loop (gripper)
//   6  left_claw_joint   — mimic (= -right_claw), no physical channel

#pragma once

#include <array>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <cmath>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace madara_hardware_interface
{

// ── Protocol constants ─────────────────────────────────────────────────────
constexpr uint8_t PROTO_SOF1    = 0xA5;
constexpr uint8_t PROTO_SOF2    = 0x5A;
constexpr uint8_t PROTO_TYPE_CMD = 0x43;   // 'C'
constexpr uint8_t PROTO_TYPE_STA = 0x53;   // 'S'

constexpr std::size_t CMD_FRAME_LEN = 17;
constexpr std::size_t STA_FRAME_LEN = 11;

// Position resolution: 1 unit = 0.001 rad → int16_t covers ±32.767 rad
constexpr double MRAD_PER_RAD = 1000.0;
constexpr double RAD_PER_MRAD = 0.001;

// Total joints managed by this interface
constexpr std::size_t NUM_JOINTS = 7;


class MadaraUARTSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MadaraUARTSystem)

  // ── ros2_control lifecycle ─────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Parameters (from xacro hardware block) ────────────────────────
  std::string serial_port_;
  int         baud_rate_         = 115200;
  double      ticks_per_rev_     = 21696.0;  // JGA25-371: 11cpr×4×34
  int         uart_timeout_ms_   = 50;

  // ── Serial file descriptor ────────────────────────────────────────
  int fd_ = -1;

  // ── Joint storage ─────────────────────────────────────────────────
  std::array<double, NUM_JOINTS> hw_pos_{};   // position state  [rad]
  std::array<double, NUM_JOINTS> hw_vel_{};   // velocity state  [rad/s]
  std::array<double, NUM_JOINTS> hw_cmd_{};   // position command [rad]

  // Joint names in index order (populated from HardwareInfo)
  std::array<std::string, NUM_JOINTS> joint_names_;

  // Sequence counter (wraps 0–255)
  uint8_t cmd_seq_ = 0;

  // ── Serial helpers ─────────────────────────────────────────────────
  bool open_serial();
  void close_serial();

  bool send_cmd_frame();
  bool recv_sta_frame();

  /// Read exactly @p n bytes from fd_, respecting uart_timeout_ms_.
  bool timed_read(uint8_t * buf, std::size_t n);

  /// CRC-8/SMBUS (polynomial 0x07), computed over @p len bytes.
  static uint8_t crc8(const uint8_t * data, std::size_t len);
};

}  // namespace madara_hardware_interface