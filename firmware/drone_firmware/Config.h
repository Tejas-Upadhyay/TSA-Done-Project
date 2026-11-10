#pragma once
#include <stdint.h>

namespace cfg {
// Confirm the ESC actually accepts conventional 1000–2000 us PWM at 250 Hz.
// Defaults generate NO ESC pulses. Do not guess a protocol from the ESC name.
constexpr bool ESC_PWM_CONFIRMED = false;
// Enable only after the propeller-off checks in README.md. Gains are untuned.
constexpr bool FLIGHT_CONFIGURATION_VERIFIED = false;
constexpr bool DEBUG_ON_BOOT = false;
constexpr uint8_t MOTOR_PINS[4] = {12, 13, 14, 15}; // FL, FR, BL, BR
constexpr uint8_t SDA_PIN = 21, SCL_PIN = 22, MPU_ADDRESS = 0x68;
constexpr uint16_t PWM_HZ = 250;
constexpr uint8_t PWM_BITS = 16;
constexpr uint32_t LOOP_US = 4000, OUTPUT_TIMEOUT_US = 30000;
constexpr uint32_t LINK_TIMEOUT_US = 250000;
constexpr uint16_t STOP_US = 1000, IDLE_US = 1050, MAX_US = 2000;
constexpr uint16_t LOW_THROTTLE_US = 1050, TEST_MAX_US = 1150;
constexpr uint32_t ARM_HOLD_US = 1000000, TEST_DURATION_US = 2000000;
constexpr float MAX_ARM_ANGLE = 10.0f;
constexpr float MAX_ANGLE = 20.0f, ANGLE_KP = 4.0f, MAX_RATE = 120.0f;
constexpr float MAX_YAW_RATE = 90.0f;
// Starting values only: output in microseconds, input in degrees/second.
constexpr float RATE_KP = 0.7f, RATE_KI = 0.25f, RATE_KD = 0.003f;
constexpr float YAW_KP = 1.0f, YAW_KI = 0.2f;
constexpr float I_LIMIT = 60.0f, CORRECTION_LIMIT = 180.0f;
constexpr char BLE_NAME[] = "TSA-Drone";
constexpr char SERVICE_UUID[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char RX_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char TX_UUID[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static_assert(1000000 / PWM_HZ > MAX_US, "PWM frame must exceed pulse width");
static_assert(OUTPUT_TIMEOUT_US > LOOP_US, "Output deadline must exceed loop period");
}
