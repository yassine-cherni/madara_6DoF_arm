// ═══════════════════════════════════════════════════════════════════════════
// madara_firmware.ino
// Madara 6-DoF Arm — Arduino Uno firmware
// ───────────────────────────────────────────────────────────────────────────
// Responsibilities:
//   • Parse CMD frames from ROS2 over UART (115200 baud)
//   • Drive 5× MG966R servos via Servo library (open-loop position)
//   • Drive JGA25-371 DC motor via L298N H-bridge
//   • Read quadrature encoder (INT0 + INT1) for DC motor position/velocity
//   • Run positional PID at 100 Hz for DC motor position control
//   • Send STA frames to ROS2 at 100 Hz
//
// UART Protocol (matches madara_hardware_interface):
//   CMD (ROS→Arduino) 17 bytes: 0xA5 0x5A 'C' seq j0..j5[int16 mrad] crc8
//   STA (Arduino→ROS) 11 bytes: 0xA5 0x5A 'S' seq enc[int32] vel[int16] crc8
//
// Pin map (Arduino Uno):
//   DC Motor (L298N)    ENA=6(PWM)  IN1=7  IN2=8
//   Encoder             A=2(INT0)   B=3(INT1)
//   Servo lower_arm     = 4
//   Servo upper_arm     = 5
//   Servo wrist         = 9
//   Servo claw_base     = 10
//   Servo right_claw    = 11
//
// CALIBRATION NOTES (from your bench tests):
//   PPR_MOTOR   = 24   (encoder pulses per motor shaft revolution, one channel)
//   REDUCTION   = 226  (gearbox ratio)
//   QUADRATURE  = ×4   (both channels, CHANGE mode → 4 edges per pulse)
//   → TICKS_PER_REV = 24 × 4 × 226 = 21,696
//
//   PID form: positional (matches your tested position.py)
//   PID gains below are STARTING POINTS — retune with arm loaded.
//   Your Python test used error in "tours"; firmware uses error in ticks:
//     Kp_ticks ≈ Kp_tours / TICKS_PER_REV
//
//   Servo: writeMicroseconds() is used (more precise than write()).
//   Pin 11 confirmed working for right_claw in your bench test.
// ═══════════════════════════════════════════════════════════════════════════

#include <Servo.h>

// ── Pin definitions ────────────────────────────────────────────────────────
#define PIN_MOTOR_PWM   6    // L298N ENA — Timer0 OC0A (does not conflict with Servo)
#define PIN_MOTOR_IN1   7    // L298N IN1
#define PIN_MOTOR_IN2   8    // L298N IN2
#define PIN_ENC_A       2    // Encoder channel A — INT0
#define PIN_ENC_B       3    // Encoder channel B — INT1

#define PIN_SERVO_0     4    // lower_arm  (joint index 1)
#define PIN_SERVO_1     5    // upper_arm  (joint index 2)
#define PIN_SERVO_2     9    // wrist      (joint index 3)
#define PIN_SERVO_3    10    // claw_base  (joint index 4)
#define PIN_SERVO_4    11    // right_claw (joint index 5) — pin confirmed

// ── Protocol constants ─────────────────────────────────────────────────────
#define SOF1        0xA5
#define SOF2        0x5A
#define TYPE_CMD    0x43   // 'C'
#define TYPE_STA    0x53   // 'S'
#define CMD_LEN     17
#define STA_LEN     11
#define BAUD_RATE   115200

// ── Motor / encoder constants ──────────────────────────────────────────────
// Measured on your motor:
//   PPR_MOTOR = 24  (pulses per revolution, single channel)
//   REDUCTION = 226 (gearbox ratio)
//   Quadrature ×4 (both channels, CHANGE ISR)
//   → 24 × 4 × 226 = 21,696 ticks per output-shaft revolution
#define PPR_MOTOR        24
#define REDUCTION        226
#define TICKS_PER_REV_F  21696.0f          // 24 × 4 × 226
#define TWO_PI_F         6.28318530f
#define RAD_PER_TICK     (TWO_PI_F / TICKS_PER_REV_F)  // ~0.0002896 rad/tick

// PWM limits — matches your bench test (max duty 80/100 → 204/255)
// Keeping 200 for round number and safe headroom below 255.
#define PWM_MAX      200
#define PWM_DEADBAND   8   // ticks of deadband to avoid jitter at rest

// ── PID — Positional form ──────────────────────────────────────────────────
// Matches your tested position.py which used:
//   output = Kp*error + Ki*integral + Kd*derivative
//
// Error here is in TICKS (not tours).
// Conversion from tours-domain gains you tested:
//   Kp_ticks = Kp_tours / TICKS_PER_REV = Kp_tours / 21696
//
// STARTING POINTS — retune these with arm loaded:
//   If your best Kp_tours ≈ 200, then Kp_ticks ≈ 200/21696 ≈ 0.009
#define PID_KP    0.009f   // ← replace with (your_Kp_tours / 21696.0)
#define PID_KI    0.0003f  // ← replace with (your_Ki_tours / 21696.0)
#define PID_KD    0.0008f  // ← replace with (your_Kd_tours / 21696.0)
#define PID_T     0.01f    // sampling period [s] = 1/100 Hz

// ── Braking zones (from your position.py bench test) ──────────────────────
// Python used error in tours; converted here to ticks:
//   0.50 tours × 21696 = 10848 ticks → output × 0.5
//   0.20 tours × 21696 =  4339 ticks → output × 0.3
//   0.05 tours × 21696 =  1085 ticks → output × 0.2
//   0.02 tours × 21696 =   434 ticks → full stop + brake
#define BRAKE_ZONE_1    10848   // ticks — start slowing
#define BRAKE_ZONE_2     4339   // ticks — slow more
#define BRAKE_ZONE_3     1085   // ticks — near target
#define STOP_THRESHOLD    434   // ticks — declare reached, brake

// ── Servo calibration ──────────────────────────────────────────────────────
// 1500 µs = 0 rad center for all MG966R servos.
// 318 µs/rad: ±π/2 rad → ±500 µs (range 1000–2000 µs).
// Trim SERVO_CENTER_US per joint if physical center is not at 0 rad.
static const int   SERVO_CENTER_US[5] = {1500, 1500, 1500, 1500, 1500};
static const float SERVO_SCALE_US[5]  = {318.0f, 318.0f, 318.0f, 318.0f, 318.0f};

// ── Global state ───────────────────────────────────────────────────────────
volatile int32_t enc_ticks = 0;

Servo servos[5];

// cmd_j[0] = DC motor target [mrad], cmd_j[1..5] = servo targets [mrad]
volatile int16_t cmd_j[6] = {0, 0, 0, 0, 0, 0};

// PID state — positional form
float pid_integral  = 0.0f;
float pid_prev_err  = 0.0f;

// Motor stopped flag — set when STOP_THRESHOLD reached, cleared on new command
bool  motor_stopped = false;

// Sequence counter for STA frames
uint8_t sta_seq = 0;


// ── ISR — Quadrature encoder ───────────────────────────────────────────────
void isr_enc_a() {
  if (digitalRead(PIN_ENC_B) != digitalRead(PIN_ENC_A)) enc_ticks++;
  else                                                   enc_ticks--;
}
void isr_enc_b() {
  if (digitalRead(PIN_ENC_A) == digitalRead(PIN_ENC_B)) enc_ticks++;
  else                                                   enc_ticks--;
}

// ── CRC-8/SMBUS ───────────────────────────────────────────────────────────
uint8_t crc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// ── Motor drive via L298N ──────────────────────────────────────────────────
void motor_set(int pwm_signed) {
  pwm_signed = constrain(pwm_signed, -PWM_MAX, PWM_MAX);
  int mag = abs(pwm_signed);
  if (mag < PWM_DEADBAND) mag = 0;

  if (pwm_signed > 0) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
  } else if (pwm_signed < 0) {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
  } else {
    // Active brake — both HIGH, zero PWM
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
  }
  analogWrite(PIN_MOTOR_PWM, mag);
}

// ── Active brake — full stop, hold position ────────────────────────────────
// Matches your Python: set_motor(0) then stop_motor() with 100ms pause
void motor_brake() {
  analogWrite(PIN_MOTOR_PWM, 0);
  digitalWrite(PIN_MOTOR_IN1, HIGH);   // both HIGH = active brake
  digitalWrite(PIN_MOTOR_IN2, HIGH);
}

// ── Positional PID update ──────────────────────────────────────────────────
// Matches the form you tested in position.py:
//   output = Kp*error + Ki*integral + Kd*derivative
// With progressive braking zones near target.
void pid_update(int32_t target_ticks, int32_t cur_ticks) {

  int32_t err_ticks = target_ticks - cur_ticks;

  // ── Precise stop (from your Python: abs(error) < 0.02 tours) ──────────
  if (abs(err_ticks) < STOP_THRESHOLD) {
    motor_brake();
    pid_integral = 0.0f;    // reset integrator at stop
    pid_prev_err = 0.0f;
    motor_stopped = true;
    return;
  }

  float e = (float)err_ticks;

  // ── Integrate + differentiate ─────────────────────────────────────────
  pid_integral += e * PID_T;

  // Anti-windup: clamp integral contribution
  // Max integral contribution = PWM_MAX / Ki to avoid runaway
  if (PID_KI > 0.0f) {
    float max_i = (float)PWM_MAX / PID_KI;
    pid_integral = constrain(pid_integral, -max_i, max_i);
  }

  float derivative = (e - pid_prev_err) / PID_T;
  pid_prev_err = e;

  float output = (PID_KP * e) + (PID_KI * pid_integral) + (PID_KD * derivative);

  // ── Progressive braking zones (from your position.py) ─────────────────
  int32_t abs_err = abs(err_ticks);
  if      (abs_err < BRAKE_ZONE_3) output *= 0.2f;   // < 0.05 tours
  else if (abs_err < BRAKE_ZONE_2) output *= 0.3f;   // < 0.20 tours
  else if (abs_err < BRAKE_ZONE_1) output *= 0.5f;   // < 0.50 tours

  output = constrain(output, (float)-PWM_MAX, (float)PWM_MAX);

  motor_set((int)output);
}

// ── Servo write: mrad → microseconds ──────────────────────────────────────
void servo_write(uint8_t idx, int16_t mrad) {
  float rad = (float)mrad * 0.001f;
  int us = SERVO_CENTER_US[idx] + (int)(rad * SERVO_SCALE_US[idx]);
  us = constrain(us, 500, 2500);
  servos[idx].writeMicroseconds(us);
}

// ── CMD frame parser (non-blocking state machine) ─────────────────────────
uint8_t rx_buf[CMD_LEN];
uint8_t rx_idx = 0;

bool parse_cmd() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    switch (rx_idx) {
      case 0:
        if (b == SOF1) rx_buf[rx_idx++] = b;
        break;
      case 1:
        if (b == SOF2) rx_buf[rx_idx++] = b;
        else           rx_idx = 0;
        break;
      default:
        rx_buf[rx_idx++] = b;
        if (rx_idx == CMD_LEN) {
          rx_idx = 0;
          if (rx_buf[2] != TYPE_CMD)               return false;
          if (crc8(&rx_buf[2], 14) != rx_buf[16]) return false;
          for (uint8_t j = 0; j < 6; j++) {
            int16_t v;
            memcpy(&v, &rx_buf[4 + j * 2], 2);
            cmd_j[j] = v;
          }
          return true;
        }
        break;
    }
  }
  return false;
}

// ── STA frame sender ──────────────────────────────────────────────────────
void send_sta(int32_t enc_now, int16_t vel_mrad_s) {
  uint8_t frame[STA_LEN];
  frame[0] = SOF1;
  frame[1] = SOF2;
  frame[2] = TYPE_STA;
  frame[3] = sta_seq++;
  memcpy(&frame[4], &enc_now,    4);
  memcpy(&frame[8], &vel_mrad_s, 2);
  frame[10] = crc8(&frame[2], 8);
  Serial.write(frame, STA_LEN);
}

// ── setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);

  // Motor
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  motor_brake();   // start braked, not coasting

  // Encoder
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isr_enc_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isr_enc_b, CHANGE);

  // Servos — centre all on startup
  servos[0].attach(PIN_SERVO_0);
  servos[1].attach(PIN_SERVO_1);
  servos[2].attach(PIN_SERVO_2);
  servos[3].attach(PIN_SERVO_3);
  servos[4].attach(PIN_SERVO_4);
  for (uint8_t i = 0; i < 5; i++)
    servos[i].writeMicroseconds(SERVO_CENTER_US[i]);

  // Pause to let servos settle and motor brake before ROS sends commands
  delay(500);
}

// ── loop ───────────────────────────────────────────────────────────────────
void loop() {
  static uint32_t last_pid_ms  = 0;
  static int32_t  last_ticks   = 0;
  static int32_t  target_ticks = 0;

  // ── Non-blocking CMD reception ────────────────────────────────────────
  if (parse_cmd()) {
    // Convert DC motor target: mrad → encoder ticks
    int32_t new_target = (int32_t)(
      (float)cmd_j[0] * 0.001f / TWO_PI_F * TICKS_PER_REV_F);

    // New command received — clear stopped flag and reset PID
    // only if the target actually changed (avoids resetting on repeated frames)
    if (new_target != target_ticks) {
      target_ticks  = new_target;
      motor_stopped = false;
      pid_integral  = 0.0f;
      pid_prev_err  = 0.0f;
    }

    // Update servos immediately on new command
    for (uint8_t i = 0; i < 5; i++) {
      servo_write(i, cmd_j[i + 1]);
    }
  }

  // ── 100 Hz PID + state publish ────────────────────────────────────────
  uint32_t now = millis();
  if (now - last_pid_ms >= 10) {
    last_pid_ms = now;

    // Atomically read encoder
    noInterrupts();
    int32_t cur_ticks = enc_ticks;
    interrupts();

    // Velocity: ticks per 10 ms → rad/s → mrad/s for STA frame
    int32_t delta = cur_ticks - last_ticks;
    last_ticks = cur_ticks;
    float vel_rad_s = (float)delta * RAD_PER_TICK / PID_T;
    int16_t vel_mrad = (int16_t)constrain(
      vel_rad_s * 1000.0f, -32767.0f, 32767.0f);

    // Only run PID if not stopped at target
    if (!motor_stopped) {
      pid_update(target_ticks, cur_ticks);
    }

    // Send STA frame to ROS2
    send_sta(cur_ticks, vel_mrad);
  }
}