// Copyright 2026 Yassine Cherni — madara_6DoF_arm
// SPDX-License-Identifier: Apache-2.0

#include "madara_hardware_interface/madara_uart_system.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>   // FIONREAD — buffer-size query for drain in recv_sta_frame
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

  // Default /dev/ttyACM0 — Arduino Uno on Raspberry Pi 5.
  serial_port_ = get_param("serial_port", "/dev/ttyACM0");
  try {
    baud_rate_       = std::stoi(get_param("baud_rate",                "115200"));
    ticks_per_rev_   = std::stod(get_param("dc_encoder_ticks_per_rev", "21696"));
    // FIX (WARNING) — uart_timeout_ms default reduced from 50 ms → 8 ms.
    //
    // The Arduino sends STA frames at 100 Hz (one every 10 ms). With the
    // old default of 50 ms, a single late/missing frame caused recv_sta_frame()
    // to block for 50 ms — five full 100 Hz control cycles. This caused the
    // controller manager to fall behind real time and triggered late-cycle
    // warnings. 8 ms = one cycle minus 2 ms margin. A timeout holds the last
    // known encoder state, which is already handled gracefully by the
    // WARN_THROTTLE log below. Override via <param name="uart_timeout_ms">8</param>
    // in the URDF hardware block (already the recommended value).
    uart_timeout_ms_ = std::stoi(get_param("uart_timeout_ms", "8"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("MadaraUARTSystem"),
      "Failed to parse hardware parameters from URDF: %s — "
      "check baud_rate, dc_encoder_ticks_per_rev, uart_timeout_ms are valid numbers.",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  hw_vel_.fill(0.0);
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

  // Joint 0 (DC motor): real position + velocity from encoder
  si.emplace_back(joint_names_[0], hardware_interface::HW_IF_POSITION, &hw_pos_[0]);
  si.emplace_back(joint_names_[0], hardware_interface::HW_IF_VELOCITY,  &hw_vel_[0]);

  // Joints 1-6 (servos + mimic): position + velocity for all.
  // JointTrajectoryController on Humble requires velocity from every
  // joint it manages. Servo joints report 0.0 rad/s (open-loop).
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
//
// IMPORTANT: Do NOT call usleep() / sleep() / any blocking wait here or
// inside open_serial(). on_activate() runs in the ros2_control lifecycle
// thread. Blocking it for more than ~100 ms causes an internal watchdog
// to fire SIGABRT (exit code -6, process dies immediately).
//
// The Arduino bootloader boot-wait is handled passively:
//   - open_serial() flushes the RX buffer twice (before and after termios)
//   - recv_sta_frame() drains any residual bootloader garbage each cycle
//     via the FIONREAD drain at the top of the function
// The first 1-2 seconds of throttled CRC warnings are expected and harmless.
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn
MadaraUARTSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_serial()) {
    RCLCPP_FATAL(rclcpp::get_logger("MadaraUARTSystem"),
      "Cannot open serial port '%s'. "
      "Check cable and permissions (user must be in 'dialout' group).",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MadaraUARTSystem"),
    "Serial port %s opened at %d baud. Hardware interface ACTIVE.",
    serial_port_.c_str(), baud_rate_);

  hw_cmd_ = hw_pos_;   // seed commands from initial positions — no startup jump
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
// read  — receive one STA frame from Arduino, update DC motor state
// ═══════════════════════════════════════════════════════════════════════════
hardware_interface::return_type
MadaraUARTSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fd_ < 0) return hardware_interface::return_type::ERROR;

  // Persistent steady clock — re-creating it each call resets the throttle
  // counter to t=0 and spams the log on every cycle.
  static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);

  if (!recv_sta_frame()) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MadaraUARTSystem"),
      throttle_clock, 2000,
      "Missed/invalid STA frame — holding last encoder state.");
  }

  // FIX (WARNING) — mimic joint state update belongs in read(), not write().
  // Semantically: read() is where all state is refreshed from hardware.
  // The redundant copy that was previously in write() has been removed.
  hw_pos_[6] = -hw_pos_[5];
  // hw_vel_[6] stays 0.0

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

  // Mirror mimic joint command before sending
  hw_cmd_[6] = -hw_cmd_[5];

  // INTENTIONAL: if the serial port is closed (fd_ < 0, caught above) or
  // send_cmd_frame() fails here, we return ERROR immediately without updating
  // the servo echo state below. This correctly freezes servo state at the
  // last successfully sent value rather than advancing it on a failed write.
  // Do not move the echo loop above this early return.
  if (!send_cmd_frame()) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MadaraUARTSystem"),
      throttle_clock, 1000, "CMD frame write failed.");
    return hardware_interface::return_type::ERROR;
  }

  // Servo state = echo of last command (open-loop, joints 1-6).
  // hw_vel_[1..6] stays 0.0.
  // NOTE: hw_pos_[6] (mimic joint) is updated in read(), not here.
  for (std::size_t i = 1; i < NUM_JOINTS - 1; ++i) {   // joints 1-5 only
    hw_pos_[i] = hw_cmd_[i];
  }

  return hardware_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
// open_serial
//
// FIX — HUPCL disabled:
//   Linux asserts/de-asserts DTR when a serial port is opened or closed.
//   On Arduino Uno, a DTR edge triggers the hardware reset line, causing
//   the bootloader to run for ~1.5 s and flood the RX buffer with garbage.
//   Setting ~HUPCL stops every subsequent open/close from resetting the
//   Arduino. The very first open() already triggered the reset before we
//   can clear HUPCL, so we flush twice — once immediately after open()
//   and once after tcsetattr is applied — to discard bootloader output.
//   recv_sta_frame()'s FIONREAD drain handles any residual garbage that
//   arrives in the first ~1 second of operation.
// ═══════════════════════════════════════════════════════════════════════════
bool MadaraUARTSystem::open_serial()
{
  fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) { return false; }

  // First flush: discard whatever the DTR reset sent before we got here
  ::tcflush(fd_, TCIOFLUSH);

  // Switch to blocking I/O — poll() handles timeouts explicitly
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

  // 8N1, raw mode, no flow control
  tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |=  CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

  // Disable HUPCL: prevents DTR toggle on future open/close → no Arduino reset
  tty.c_cflag &= ~HUPCL;

  tty.c_iflag  =  IGNBRK;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
  tty.c_lflag  =  0;
  tty.c_oflag  =  0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  ::tcsetattr(fd_, TCSANOW, &tty);

  // Second flush: discard any bootloader bytes that arrived during termios setup
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
// CMD frame layout (17 bytes, little-endian):
//   [0]      0xA5        SOF1
//   [1]      0x5A        SOF2
//   [2]      0x43 'C'    frame type
//   [3]      seq         rolling counter 0-255
//   [4-5]    j0 int16    DC motor target  [mrad]
//   [6-7]    j1 int16    lower_arm        [mrad]
//   [8-9]    j2 int16    upper_arm        [mrad]
//   [10-11]  j3 int16    wrist            [mrad]
//   [12-13]  j4 int16    claw_base        [mrad]
//   [14-15]  j5 int16    right_claw       [mrad]
//   [16]     crc8        CRC-8/SMBUS over bytes [2..15]
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
// STA frame layout (11 bytes, little-endian):
//   [0]     0xA5          SOF1
//   [1]     0x5A          SOF2
//   [2]     0x53 'S'      frame type
//   [3]     seq           rolling counter from Arduino
//   [4-7]   enc  int32    encoder ticks (zeroed at Arduino boot)
//   [8-9]   vel  int16    DC motor velocity [mrad/s]
//   [10]    crc8          CRC-8/SMBUS over bytes [2..9]
//
// FIX — buffer drain before parsing:
//   The Arduino sends 11 bytes every 10 ms (100 Hz). When the Pi is
//   briefly busy the kernel UART buffer accumulates multiple complete
//   frames. Reading the oldest queued frame puts the CRC byte at the
//   position occupied by the SOF bytes of the next frame — producing
//   the exact 0x5A / 0x53 / 0xA5 CRC mismatch pattern in the logs.
//
//   Fix: use FIONREAD to check buffer depth and discard all but one
//   frame's worth before the SOF1 scan, so we always parse the freshest
//   available frame.
// ═══════════════════════════════════════════════════════════════════════════
bool MadaraUARTSystem::recv_sta_frame()
{
  // ── Drain stale frames ────────────────────────────────────────────
  {
    int available = 0;
    if (::ioctl(fd_, FIONREAD, &available) == 0) {
      int to_discard = available - static_cast<int>(STA_FRAME_LEN);
      while (to_discard > 0) {
        uint8_t sink[64];
        int chunk = std::min(to_discard, static_cast<int>(sizeof(sink)));
        ssize_t r = ::read(fd_, sink, static_cast<std::size_t>(chunk));
        if (r <= 0) break;
        to_discard -= static_cast<int>(r);
      }
    }
  }

  // ── Scan for SOF1 ────────────────────────────────────────────────
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

  // ── SOF2 ─────────────────────────────────────────────────────────
  if (!timed_read(&b, 1) || b != PROTO_SOF2) return false;

  // ── Remaining 9 bytes: type + seq + enc[4] + vel[2] + crc ────────
  uint8_t rest[STA_FRAME_LEN - 2];   // 9 bytes
  if (!timed_read(rest, sizeof(rest))) return false;

  if (rest[0] != PROTO_TYPE_STA) return false;

  // ── CRC check ────────────────────────────────────────────────────
  uint8_t expected = crc8(rest, 8);
  if (rest[8] != expected) {
    static rclcpp::Clock crc_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MadaraUARTSystem"),
      crc_clock, 1000,
      "STA CRC mismatch: got 0x%02X expected 0x%02X", rest[8], expected);
    return false;
  }

  // ── Decode encoder → position [rad] ──────────────────────────────
  // top_plate_joint is continuous — raw ticks are valid across any
  // number of full revolutions in either direction.
  int32_t ticks = 0;
  std::memcpy(&ticks, &rest[2], 4);
  hw_pos_[0] = static_cast<double>(ticks) / ticks_per_rev_ * (2.0 * M_PI);

  // ── Decode velocity [mrad/s] → [rad/s] ───────────────────────────
  int16_t vel_mrad = 0;
  std::memcpy(&vel_mrad, &rest[6], 2);
  hw_vel_[0] = static_cast<double>(vel_mrad) * RAD_PER_MRAD;

  return true;
}

}  // namespace madara_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  madara_hardware_interface::MadaraUARTSystem,
  hardware_interface::SystemInterface)
