# TSA Drone Project Plan

## Parts List

- **Flight Controller**: ESP32-wroom-32
- **Power System**: OVONIC 4S 14.8V 100C 550mAh LiPo Battery
- **Motors**: Readytosky RS2205 230kv Brushless Motor (CW/CCW pairs) - 4 units
- **ESC**: AERO SELFIE 45A 4in1 ESC (2-6S BL_S)
- **Sensors**: MPU 6050 (Gyro/Accelerometer)

## Pre-Flight Checks

### Safety Verification
- Battery voltage and connector security
- Motor propeller installation (CW/CCW pairing)
- RC transmitter binding and signal check
- Firmware version compatibility

### Component Connection Validation
- All 4 motors connected to correct ESC ports
- Gyro/MPU 6050 properly wired to flight controller
- ESC-BLHeli_S firmware version check
- Power distribution board integrity

## Boot Up Sequence

### Mode 1: Set-Up / Troubleshoot Mode

Activated by holding both trigger buttons on startup.

1. **Gyro Initialization Check**
   - System powers up and reads MPU 6050 raw values
   - Expects ~0° pitch/roll when drone is level
   - If gyro does not return to 0 within 2 seconds: **flash red LED error pattern** and sound buzzer
   - Display: "GYRO ERROR - Check connections"

2. **Motor Detection & Direction Test**
   - System sequentially commands each motor:
     - **Front Right**: Should cause drone to tilt backward-left
     - **Front Left**: Should cause drone to tilt backward-right
     - **Back Left**: Should cause drone to tilt forward-right
     - **Back Right**: Should cause drone to tilt forward-left (default)
   - If any motor produces unexpected direction: **flash alternating red/green**, log error code
   - All motors spin up to 30% throttle, then slowly ramp down over 5 seconds

3. **Final Status**
   - Green LED solid: All checks passed, ready for normal mode
   - Red LED flash pattern: Specific failure mode identified

### Mode 2: Normal Mode

Default boot mode if no triggers held.

1. **Startup Sequence**
   - Power-on self-test (POST): 200ms
   - Gyro calibration: 1 second (drone must remain stationary)
   - ESC startup beep sequence
   - RSSI signal strength display

2. **Ready State**
   - Solid green LED
   - Bluetooth/telemetry connection established
   - Ready for flight mode selection

## Flight Modes

- **Manual**: Full stick control, no assistance
- **Stabilize**: Auto-leveling via gyro input
- **Position Hold**: GPS-assisted hovering (if module added)
- **Return-to-Launch**: Auto-return on signal loss

## Post-Flight

- Battery level logging
- Motor heat check
- Firmware update check
- Log data review for tuning adjustments

## Troubleshooting Guide

| Symptom | Likely Cause | Action |
|---------|-------------|--------|
| Red LED on boot | Gyro misconnection | Check MPU 6050 wiring, re-seat connector |
| Motors not spinning | ESC not armed | Arm via throttle stick bottom-left sequence |
| Drift in stabilize mode | Gyro offset needs calibration | Re-run gyro calibration on level surface |
| One motor not responding | Motor/ESC failure | Swap motor ports to isolate fault |
| No RSSI signal | TX/RX mismatch | Check binding, antenna connection, frequency |

## Testing Protocol

1. **Bench Test**: All motors spin, direction verification, ESC beep tones
2. **Hover Test**: 30-second hover at 1m altitude, check stability
3. **Control Response**: Full pitch/roll/yaw movement in all directions
4. **Fail-Safe**: Signal loss triggers expected behavior (land or hover)