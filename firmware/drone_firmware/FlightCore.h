#pragma once
// Hardware-independent control, packet validation, and arming logic.
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "Config.h"

namespace flight {
inline float clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
constexpr uint8_t ARM = 1, KILL = 2, TEST = 4, DEBUG = 8;
constexpr size_t PACKET_SIZE = 16;
struct Command {
  uint32_t sequence = 0;
  int16_t roll = 0, pitch = 0, yaw = 0;
  uint16_t throttle = cfg::STOP_US;
  uint8_t flags = 0;
};
inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= uint16_t(p[i]) << 8;
    for (int j = 0; j < 8; ++j)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
inline uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline bool decode(const uint8_t* p, size_t n, Command& c) {
  if (n != PACKET_SIZE || p[0] != 0xa1 || (p[1] & 0xf0) ||
      crc16(p, 14) != u16(p + 14)) return false;
  Command v;
  v.flags = p[1];
  v.sequence = uint32_t(p[2]) | uint32_t(p[3]) << 8 |
               uint32_t(p[4]) << 16 | uint32_t(p[5]) << 24;
  v.roll = int16_t(u16(p + 6)); v.pitch = int16_t(u16(p + 8));
  v.yaw = int16_t(u16(p + 10)); v.throttle = u16(p + 12);
  if (v.roll < -1000 || v.roll > 1000 || v.pitch < -1000 || v.pitch > 1000 ||
      v.yaw < -1000 || v.yaw > 1000 || v.throttle < 1000 || v.throttle > 2000)
    return false;
  if ((v.flags & TEST) && ((v.flags & ARM) || v.roll < 1 || v.roll > 4 ||
      v.pitch != 0 || v.yaw != 0 || v.throttle > cfg::TEST_MAX_US)) return false;
  c = v;
  return true;
}
inline bool newer(uint32_t a, uint32_t b) {
  const uint32_t delta = a - b;
  return delta != 0 && delta < 0x80000000u;
}

struct Imu {
  float ax = 0, ay = 0, az = 1; // g; body X forward, Y left, Z up
  float gx = 0, gy = 0, gz = 0; // calibrated deg/s, same body axes
};
inline bool finite(const Imu& s) {
  return isfinite(s.ax) && isfinite(s.ay) && isfinite(s.az) &&
         isfinite(s.gx) && isfinite(s.gy) && isfinite(s.gz);
}
struct Attitude {
  float roll = 0, pitch = 0;
  bool initialized = false;
  bool update(const Imu& s, float dt) {
    if (!finite(s) || !isfinite(dt) || dt <= 0 || dt > 0.02f) return false;
    const float norm = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
    const float ar = atan2f(s.ay, s.az) * 57.2957795f;
    const float ap = atan2f(-s.ax, sqrtf(s.ay*s.ay + s.az*s.az)) * 57.2957795f;
    if (!initialized) {
      if (norm < 0.8f || norm > 1.2f || s.az < 0.5f) return false;
      roll = ar; pitch = ap; initialized = true;
    }
    // Body-rate -> Euler-rate conversion; this controller is for small angles.
    const float r = roll * 0.0174532925f, p = pitch * 0.0174532925f;
    roll += (s.gx + sinf(r)*tanf(p)*s.gy + cosf(r)*tanf(p)*s.gz) * dt;
    pitch += (cosf(r)*s.gy - sinf(r)*s.gz) * dt;
    if (norm > 0.8f && norm < 1.2f) {
      const float a = 0.5f / (0.5f + dt);
      roll = a * roll + (1-a) * ar;
      pitch = a * pitch + (1-a) * ap;
    }
    return isfinite(roll) && isfinite(pitch) && fabsf(roll) < 70 && fabsf(pitch) < 70;
  }
};
struct Pid {
  float kp, ki, kd, integral = 0, previous = 0, derivative = 0;
  bool initialized = false;
  Pid(float p, float i, float d): kp(p), ki(i), kd(d) {}
  void reset() { integral = previous = derivative = 0; initialized = false; }
  float update(float target, float measured, float dt, bool integrate) {
    if (initialized) {
      const float raw = -(measured - previous) / dt;
      derivative += dt / (0.02f + dt) * (raw - derivative);
    }
    previous = measured; initialized = true;
    const float error = target - measured;
    const float proposal = clamp(integral + ki*error*dt, -cfg::I_LIMIT, cfg::I_LIMIT);
    const float proposedOutput = kp*error + proposal + kd*derivative;
    if (integrate && (fabsf(proposedOutput) < cfg::CORRECTION_LIMIT ||
                     error * proposedOutput < 0)) integral = proposal;
    return clamp(kp*error + integral + kd*derivative,
                 -cfg::CORRECTION_LIMIT, cfg::CORRECTION_LIMIT);
  }
};

// Positive roll = right bank; positive pitch = nose down; positive yaw = left.
// FL/BR spin CW viewed from above, FR/BL CCW. Verify physically before enabling.
inline bool mix(float throttle, float roll, float pitch, float yaw, uint16_t out[4]) {
  float d[4] = {roll-pitch+yaw, -roll-pitch-yaw,
                roll+pitch-yaw, -roll+pitch+yaw};
  float lo = d[0], hi = d[0];
  for (int i = 1; i < 4; ++i) { lo = fminf(lo, d[i]); hi = fmaxf(hi, d[i]); }
  const float span = cfg::MAX_US - cfg::IDLE_US;
  bool saturated = hi - lo > span;
  if (saturated) {
    const float scale = span / (hi-lo);
    for (float& v : d) v *= scale;
    lo *= scale; hi *= scale;
  }
  const float base = clamp(throttle, cfg::IDLE_US-lo, cfg::MAX_US-hi);
  saturated = saturated || fabsf(base-throttle) > 0.5f;
  for (int i = 0; i < 4; ++i)
    out[i] = uint16_t(lroundf(clamp(base+d[i], cfg::IDLE_US, cfg::MAX_US)));
  return saturated;
}

enum class State { DISARMED, ARMING, ARMED, FAILSAFE, DEBUG_MODE };
class Safety {
 public:
  State state = State::DISARMED;
  const char* reason = "boot";
  Command command;
  bool havePacket = false;
  uint32_t receivedUs = 0;
  void stop(const char* why) {
    if (state != State::DEBUG_MODE) state = State::FAILSAFE;
    reason = why; lowSeen = false;
  }
  void debug() { state = State::DEBUG_MODE; reason = "debug latched until reset"; lowSeen = false; }
  void resetLink() {
    stop("Bluetooth session changed"); havePacket = false;
    command = Command{};
  }
  bool accept(const Command& c, uint32_t timestamp) {
    if (havePacket && !newer(c.sequence, command.sequence)) return false;
    command = c; receivedUs = timestamp; havePacket = true;
    if (c.flags & DEBUG) debug();
    if (c.flags & KILL) stop("kill");
    return true;
  }
  bool fresh(uint32_t now) const {
    return havePacket && uint32_t(now-receivedUs) < cfg::LINK_TIMEOUT_US;
  }
  void update(uint32_t now, bool linked, bool imuReady, bool flightAllowed,
              float roll, float pitch) {
    if (state == State::DEBUG_MODE) return;
    if (!linked || !fresh(now)) { stop("control link timeout"); return; }
    if (command.flags & (KILL | TEST)) { stop("kill or bench test"); return; }
    if (!imuReady) { stop("IMU not ready"); return; }
    if (!(command.flags & ARM)) {
      state = State::DISARMED; reason = "arm released";
      lowSeen = command.throttle <= cfg::LOW_THROTTLE_US;
      return;
    }
    if (state == State::ARMED) return;
    const bool centered = abs(int(command.roll)) < 100 && abs(int(command.pitch)) < 100 &&
                          abs(int(command.yaw)) < 100;
    if (!flightAllowed || !lowSeen || command.throttle > cfg::LOW_THROTTLE_US ||
        !centered || !isfinite(roll) || !isfinite(pitch) ||
        fabsf(roll) > cfg::MAX_ARM_ANGLE || fabsf(pitch) > cfg::MAX_ARM_ANGLE) {
      state = State::DISARMED; reason = "arming conditions not met";
      // An interrupted hold requires releasing ARM before another attempt.
      lowSeen = false;
      return;
    }
    if (state != State::ARMING) { armStarted = now; state = State::ARMING; }
    if (uint32_t(now-armStarted) >= cfg::ARM_HOLD_US) {
      state = State::ARMED; reason = "armed";
    }
  }
 private:
  bool lowSeen = false;
  uint32_t armStarted = 0;
};

class Controller {
 public:
  Attitude attitude;
  Pid roll{cfg::RATE_KP, cfg::RATE_KI, cfg::RATE_KD};
  Pid pitch{cfg::RATE_KP, cfg::RATE_KI, cfg::RATE_KD};
  Pid yaw{cfg::YAW_KP, cfg::YAW_KI, 0};
  void reset() { roll.reset(); pitch.reset(); yaw.reset(); saturated = false; }
  void run(const Imu& s, const Command& c, float dt, uint16_t out[4]) {
    if (c.throttle <= cfg::LOW_THROTTLE_US) {
      reset(); for (int i = 0; i < 4; ++i) out[i] = cfg::IDLE_US;
      return;
    }
    const float desiredRoll = cfg::MAX_ANGLE * c.roll / 1000.0f;
    const float desiredPitch = cfg::MAX_ANGLE * c.pitch / 1000.0f;
    const float rr = clamp(cfg::ANGLE_KP*(desiredRoll-attitude.roll), -cfg::MAX_RATE, cfg::MAX_RATE);
    const float pr = clamp(cfg::ANGLE_KP*(desiredPitch-attitude.pitch), -cfg::MAX_RATE, cfg::MAX_RATE);
    // The controller's Y axis points left; joystick positive yaw means turn right.
    const float yr = -cfg::MAX_YAW_RATE * c.yaw / 1000.0f;
    const float r = roll.update(rr, s.gx, dt, !saturated);
    const float p = pitch.update(pr, s.gy, dt, !saturated);
    const float y = yaw.update(yr, s.gz, dt, !saturated);
    saturated = mix(c.throttle, r, p, y, out);
  }
 private:
  bool saturated = false;
};

class BenchTest {
 public:
  void reset() { testing = false; released = false; }
  bool update(uint32_t now, const Command& c, bool permitted, uint16_t out[4]) {
    const bool request = permitted && (c.flags & TEST) && !(c.flags & (ARM|KILL|DEBUG));
    if (!request) {
      testing = false;
      released = permitted && !(c.flags & (TEST|ARM|KILL|DEBUG));
      return false;
    }
    if (!testing && released) {
      testing = true; released = false; started = now; motor = c.roll;
    }
    if (testing && uint32_t(now-started)<cfg::TEST_DURATION_US && c.roll==motor &&
        motor>=1 && motor<=4 && c.throttle<=cfg::TEST_MAX_US) {
      out[motor-1] = c.throttle;
      return true;
    }
    return false;
  }
 private:
  bool testing = false, released = false;
  uint32_t started = 0;
  int motor = 0;
};
}
