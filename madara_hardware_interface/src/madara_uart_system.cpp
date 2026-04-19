// Copyright 2026 Yassine Cherni — madara_6DoF_arm
// SPDX-License-Identifier: Apache-2.0

#include "madara_hardware_interface/madara_uart_system.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace madara_hardware_interface
{

// ═══════════════════════════════════════════════════════════════════════════
// CRC-8 / SMBUS  (polynomial 0x07, no reflection, no final XOR)
// ═══════════════════════════════════════════════════════════════════════════
uint8_t MadaraUARTSystem::crc8(const uint8_t * data, std::size_t len)
{
  uint8_t crc = 0x00;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1u) ^ 0x07u)
                           : static_cast<uint8_t>(crc << 1u);
    }
  }
  return crc;
}

// ═══════════════════════════════════════════════════════════════════════════
// on_init
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
MadaraUARTSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Read parameters from URDF hardware block ──────────────────────
  auto get_param = [&](const std::string & key, const std::string & def) {
    auto it = info_.hardware_parameters.find(key);
    return (it != info_.hardware_parameters.end()) ? it->second : def;
  };

  serial_port_     = get_param("serial_port",               "/dev/ttyAMA0");
  baud_rate_       = std::stoi(get_param("baud_rate",        "115200"));
  ticks_per_rev_ = std::stod(get_param("dc_encoder_ticks_per_rev", "21696"));
  uart_timeout_ms_ = std::stoi(get_param("uart_timeout_ms", "50"));

  RCLCPP_INFO(rclcpp::get_logger("MadaraUARTSystem"),
    "Params — port: %s  baud: %d  ticks/rev: %.0f  timeout: %d ms",
    serial_port_.c_str(), baud_rate_, ticks_per_rev_, uart_timeout_ms_);

  // ── Validate joint count ──────────────────────────────────────────
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL(rclcpp::get_logger("MadaraUARTSystem"),
      "Expected %zu joints in <ros2_control>, got %zu.",
      NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Map joint names and load initial values from xacro ───────────
  hw_pos_.fill(0.0);
  hw_vel_.fill(0.0);   // servo velocity is always 0.0 (open-loop)
  hw_cmd_.fill(0.0);

  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    joint_names_[i] = info_.joints[i].name;

    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION &&
          !si.initial_value.empty())
      {
        hw_pos_[i] = std::stod(si.initial_value);
        hw_cmd_[i] = hw_pos_[i];
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("MadaraUARTSystem"),
      "  [%zu] %s  init_pos=%.4f rad", i, joint_names_[i].c_str(), hw_pos_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// export_state_interfaces
// ═══════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
MadaraUARTSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;

  // ── Joint 0 (DC motor): position + velocity from encoder ──────────
  si.emplace_back(joint_names_[0], hardware_interface::HW_IF_POSITION, &hw_pos_[0]);
  si.emplace_back(joint_names_[0], hardware_interface::HW_IF_VELOCITY,  &hw_vel_[0]);

  // ── Joints 1-6 (servos + mimic): position + velocity ─────────────
  // FIX: velocity state is now exported for ALL joints (hw_vel_[1..6]
  //      stays 0.0 for open-loop servos).  JointTrajectoryController
  //      on ROS 2 Humble requests velocity from every joint it manages.
  //      Without this, the controller manager refuses to activate the
  //      arm controller because the claimed interfaces are incomplete.
  for (std::size_t i = 1; i < NUM_JOINTS; ++i) {
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
    si.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY,  &hw_vel_[i]);
  }

  return si;
}

// ═══════════════════════════════════════════════════════════════════════════
// export_command_interfaces
// ═══════════════════════════════════════════════════════════════════════════
std::vector<hardware_interface::CommandInterface>
MadaraUARTSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (std::size_t i = 0; i < NUM_JOINTS; ++i) {
    ci.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_cmd_[i]);
  }
  return ci;
}

// ═══════════════════════════════════════════════════════════════════════════
// on_activate
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
MadaraUARTSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_serial()) {
    RCLCPP_FATAL(rclcpp::get_logger("MadaraUARTSystem"),
      "Cannot open serial port '%s'. "
      "Check cable and permissions (add user to 'dialout').",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MadaraUARTSystem"),
    "Serial port %s opened at %d baud. Hardware interface ACTIVE.",
    serial_port_.c_str(), baud_rate_);

  hw_cmd_ = hw_pos_;   // no startup jump
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// on_deactivate
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
MadaraUARTSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  close_serial();
  RCLCPP_INFO(rclcpp::get_logger("MadaraUARTSystem"), "Serial port closed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
// read  — receive one STA frame from Arduino and update DC motor state
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type
MadaraUARTSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fd_ < 0) return hardware_interface::return_type::ERROR;

  // FIX: use a persistent steady clock for RCLCPP_WARN_THROTTLE.
  // The previous code called rclcpp::Clock::make_shared() on every
  // invocation, creating a new clock each time — the throttle counter
  // always saw t=0 and never suppressed repeated warnings.
  static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);

  if (!recv_sta_frame()) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MadaraUARTSystem"),
      throttle_clock, 2000,
      "Missed/invalid STA frame — holding last encoder state.");
  }

  // Mimic joint: left_claw state = -right_claw state (no physical channel)
  hw_pos_[6] = -hw_pos_[5];
  // hw_vel_[6] stays 0.0 (mimic, no actuator)

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
// write — send one CMD frame to Arduino with all joint targets
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type
MadaraUARTSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fd_ < 0) return hardware_interface::return_type::ERROR;

  static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);

  // Mirror command for mimic joint before sending
  hw_cmd_[6] = -hw_cmd_[5];

  if (!send_cmd_frame()) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MadaraUARTSystem"),
      throttle_clock, 1000, "CMD frame write failed.");
    return hardware_interface::return_type::ERROR;
  }

  // Servo state = echo last command (open-loop — joints 1-6)
  // hw_vel_ stays 0.0 for all servo joints
  for (std::size_t i = 1; i < NUM_JOINTS; ++i) {
    hw_pos_[i] = hw_cmd_[i];
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
// open_serial
// ═══════════════════════════════════════════════════════════════════════════
bool MadaraUARTSystem::open_serial()
{
  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) { return false; }

  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    ::close(fd_); fd_ = -1; return false;
  }

  speed_t speed = B115200;
  switch (baud_rate_) {
    case   9600: speed = B9600;   break;
    case  57600: speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("MadaraUARTSystem"),
        "Unsupported baud %d, defaulting to 115200.", baud_rate_);
  }
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);

  // 8N1, no flow control, raw mode
  tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |=  CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_iflag  =  IGNBRK;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tty.c_lflag  =  0;
  tty.c_oflag  =  0;
  tty.c_cc[VMIN]  = 0;   // poll() handles timeout
  tty.c_cc[VTIME] = 0;

  ::tcsetattr(fd_, TCSANOW, &tty);
  ::tcflush(fd_, TCIOFLUSH);
  return true;
}

void MadaraUARTSystem::close_serial()
{
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ═══════════════════════════════════════════════════════════════════════════
// timed_read — reads exactly n bytes within uart_timeout_ms_
// ═══════════════════════════════════════════════════════════════════════════
bool MadaraUARTSystem::timed_read(uint8_t * buf, std::size_t n)
{
  std::size_t total = 0;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(uart_timeout_ms_);

  while (total < n) {
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (now_ms <= 0) return false;

    struct pollfd pfd{fd_, POLLIN, 0};
    int ret = ::poll(&pfd, 1, static_cast<int>(now_ms));
    if (ret <= 0) return false;

    ssize_t r = ::read(fd_, buf + total, n - total);
    if (r <= 0) return false;
    total += static_cast<std::size_t>(r);
  }
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// send_cmd_frame
//
// CMD frame (17 bytes):
//   [0]    0xA5          SOF1
//   [1]    0x5A          SOF2
//   [2]    0x43 'C'      frame type
//   [3]    seq           rolling counter 0-255
//   [4-5]  j0 int16 LE   DC motor target [mrad]
//   [6-7]  j1 int16 LE   servo lower_arm [mrad]
//   [8-9]  j2 int16 LE   servo upper_arm [mrad]
//   [10-11] j3 int16 LE  servo wrist     [mrad]
//   [12-13] j4 int16 LE  servo claw_base [mrad]
//   [14-15] j5 int16 LE  servo right_claw[mrad]
//   [16]   crc8           CRC-8/SMBUS of bytes [2..15]
// ═══════════════════════════════════════════════════════════════════════════
bool MadaraUARTSystem::send_cmd_frame()
{
  uint8_t frame[CMD_FRAME_LEN];
  frame[0] = PROTO_SOF1;
  frame[1] = PROTO_SOF2;
  frame[2] = PROTO_TYPE_CMD;
  frame[3] = cmd_seq_++;

  for (int j = 0; j < 6; ++j) {
    double clamped = std::clamp(hw_cmd_[j], -32.0, 32.0);
    auto mrad = static_cast<int16_t>(std::round(clamped * MRAD_PER_RAD));
    std::memcpy(&frame[4 + j * 2], &mrad, 2);
  }

  frame[16] = crc8(&frame[2], 14);

  ssize_t written = ::write(fd_, frame, CMD_FRAME_LEN);
  return written == static_cast<ssize_t>(CMD_FRAME_LEN);
}

// ═══════════════════════════════════════════════════════════════════════════
// recv_sta_frame
//
// STA frame (11 bytes):
//   [0]    0xA5            SOF1
//   [1]    0x5A            SOF2
//   [2]    0x53 'S'        frame type
//   [3]    seq echo        echoes last CMD seq
//   [4-7]  enc   int32 LE  encoder ticks (raw, zeroed at Arduino boot)
//   [8-9]  vel   int16 LE  DC motor velocity [mrad/s]
//   [10]   crc8            CRC-8/SMBUS of bytes [2..9]
// ═══════════════════════════════════════════════════════════════════════════
bool MadaraUARTSystem::recv_sta_frame()
{
  // Scan for SOF1
  uint8_t b = 0;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(uart_timeout_ms_);

  while (true) {
    auto ms_left = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now()).count();
    if (ms_left <= 0) return false;

    struct pollfd pfd{fd_, POLLIN, 0};
    if (::poll(&pfd, 1, static_cast<int>(ms_left)) <= 0) return false;
    if (::read(fd_, &b, 1) != 1) return false;
    if (b == PROTO_SOF1) break;
  }

  if (!timed_read(&b, 1) || b != PROTO_SOF2) return false;

  uint8_t rest[STA_FRAME_LEN - 2];
  if (!timed_read(rest, sizeof(rest))) return false;

  if (rest[0] != PROTO_TYPE_STA) return false;

  uint8_t expected = crc8(rest, 8);
  if (rest[8] != expected) {
    static rclcpp::Clock crc_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MadaraUARTSystem"),
      crc_clock, 1000,
      "STA CRC mismatch: got 0x%02X expected 0x%02X", rest[8], expected);
    return false;
  }

  // Encoder ticks → position [rad]
  int32_t ticks = 0;
  std::memcpy(&ticks, &rest[2], 4);
  hw_pos_[0] = static_cast<double>(ticks) / ticks_per_rev_ * (2.0 * M_PI);

  // Velocity [mrad/s] → [rad/s]
  int16_t vel_mrad = 0;
  std::memcpy(&vel_mrad, &rest[6], 2);
  hw_vel_[0] = static_cast<double>(vel_mrad) * RAD_PER_MRAD;

  return true;
}

}  // namespace madara_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  madara_hardware_interface::MadaraUARTSystem,
  hardware_interface::SystemInterface)
