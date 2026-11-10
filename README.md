# TSA medical supply drone — experimental ESP32 controller

ESP32-WROOM-32, MPU6050, four ESC outputs, and **Bluetooth Low Energy (BLE)**
control. This implements a small-angle self-leveling quad-X control loop. It is
not flight-validated. The PID gains are starting values, not a tune for this frame.
There is no GPS, altitude hold, position hold, return-to-home, autonomous delivery,
battery sensing, or payload-release control. A disconnect cuts motor thrust; it
does not land the aircraft. Do not test a disconnect in free flight.

## What was wrong with the previous firmware?

- It generated sequential software pulses instead of continuous independent ESC PWM.
- There was no radio/Bluetooth input or running stabilization loop.
- Calibration printed offsets without applying them.
- Setup/normal startup automatically commanded substantial motor throttle.
- Blocking startup tests prevented serial debug commands from being serviced.

The old sensor-only debug logic was useful, but its empty `.ino`, separate `.cpp`,
and folder naming made Arduino sketch selection confusing. It now has a canonical
sketch folder and its own build target. The unused ESP32PWM dependency is removed.

## Code and build

| File | Purpose |
| --- | --- |
| `firmware/drone_firmware/drone_firmware.ino` | Main sketch: hardware PWM, MPU6050, BLE, serial diagnostics |
| `firmware/drone_firmware/Config.h` | Pins, output locks, timing, and gains |
| `firmware/drone_firmware/FlightCore.h` | Packet parser, state machine, filter, PID, mixer, bench interlocks |
| `firmware/drone_debug/drone_debug.ino` | Independent USB-only sensor diagnostics; never generates motor pulses |
| `tools/ble_bench.py` | Mac/Windows/Linux BLE scan, status, debug, and single-motor bench test |
| `tools/ble_gamepad.py` | Experimental gamepad-to-BLE control bridge |

Install Python and PlatformIO, then from the repository root:

```sh
python -m pip install platformio==6.2.0
python -m platformio run
python -m platformio run -e esp32dev -t upload
python -m platformio device monitor -b 115200
```

The platform pins Arduino-ESP32 2.0.17 through `espressif32@6.9.0`.
The main sketch also has API branches for Arduino-ESP32 3.x, which need their own
build/hardware verification. BLE needs the large application partition selected
by `huge_app.csv`. In Arduino IDE, open the **canonical sketch folder**, select
ESP32 Dev Module, and choose a partition with sufficient app space, e.g. Huge APP.
Do not combine the root scanner `.ino` files: each has its own setup and loop.

For sensor-only debug:

```sh
python -m platformio run -e esp32debug -t upload
```

Alternatively open `firmware/drone_debug/drone_debug.ino` in Arduino IDE.
Root `drone_firmware.ino` and `Debug mode/debug.cpp` are compatibility includes;
the canonical sketch folders above are the recommended entry points.

## Wiring and conventions — verify before enabling output

| Connection | ESP32 pin / expected behavior |
| --- | --- |
| MPU6050 SDA / SCL | GPIO21 / GPIO22, address 0x68 (AD0 low) |
| Motor 1 / front left | GPIO12, CW viewed from above |
| Motor 2 / front right | GPIO13, CCW viewed from above |
| Motor 3 / back left | GPIO14, CCW viewed from above |
| Motor 4 / back right | GPIO15, CW viewed from above |
| Signal ground | Common ESP32 / IMU / ESC ground |

GPIO12 and GPIO15 are ESP32 strapping pins. An attached ESC pulling GPIO12 high
at reset can prevent boot. If needed, move motor outputs to suitable free pins
(e.g. 25, 26, 18, 19), updating **both** main Config.h and standalone debug pins.
Power the ESP32 from a suitable regulated supply; never connect the 4S battery
directly to its 3.3 V or USB/5 V input. Use 3.3 V-compatible I2C signal levels.

The IMU must be rigidly mounted with **X forward, Y left, Z up**. The filter
expects approximately `[0,0,+1] g` when flat. Positive roll means right bank;
positive pitch means nose down; positive gyro Z means turning left. The BLE yaw
stick is positive for turning right. Remap sensor axes in `readImu` if its mounting
differs; do not fix an axis error by experimenting with PID signs.

The ESC name alone does not establish its input protocol. Confirm that the
installed ESC firmware accepts conventional **1000–2000 microsecond PWM at
250 Hz**. This code does not implement DShot, OneShot, or ESC endpoint calibration.
The plan's `RS2205 230kv` motor entry also needs checking against the actual label
and motor/prop/battery ratings.

## Output locks and initial checks

All propellers must be removed for these steps.

1. Leave both Config.h switches false. Upload; verify `PWM=LOCKED`. BLE advertises
   as `TSA-Drone`. No ESC pulses are generated in this configuration.
2. Open USB Serial Monitor at 115200 baud, newline enabled. Use `status`, `scan`,
   `imu`, `stream on`, and `stream off`. Keep the frame still during boot calibration.
   A failed/moving/disconnected IMU cannot arm. Use `calibrate` while disarmed to
   retry; offsets are actually subtracted in subsequent flight samples.
3. Tilt the frame by hand. Check roll and pitch signs, calibrated rates, and
   near-zero gyro values when stationary. Debug cannot infer the physical motor
   rotation, motor identity, thrust direction, battery condition, or ESC health.
4. After confirming the ESC protocol and wiring, set **only**
   `ESC_PWM_CONFIRMED=true`, rebuild, and upload. Normal disarmed output is 1000 us.
   Keep `FLIGHT_CONFIGURATION_VERIFIED=false` for all initial motor tests.
5. Run each BLE motor test below. Confirm that exactly the named motor turns,
   in the documented direction. Watch pulse widths with an oscilloscope/logic
   analyzer where available. PWM duty calculations do not prove electrical timing.
6. Verify the Bluetooth/gamepad mappings, neutral/low throttle, arm-release stop,
   debug/kill, packet-loss stop, and board-reset behavior with props removed.
7. Review motor mixing against the mounted IMU. For a right bank, stabilization
   must raise right-side relative to left-side output; for nose-down tilt it must
   raise front-side relative to rear-side output. Confirm using measured PWM on
   a secured propeller-free frame with flight configuration deliberately enabled.
8. Only after these checks, decide whether to set `FLIGHT_CONFIGURATION_VERIFIED=true`
   and proceed with a restrained, supervised tuning procedure appropriate to the
   frame. The switch is an operator attestation, not an automatic hardware check.
   Compilation and software tests do not establish that the drone can fly safely.

Serial `debug` is available after startup and latches zero motor duties until
reset. Set `DEBUG_ON_BOOT=true` to start there. There are no automatic startup
spin sequences, and GPIO0 is not used to trigger tests (holding it during reset
can select the ESP32 bootloader). `kill` disarms immediately at the next control
tick. Scans, printing, and calibration are refused while armed/arming/testing.
Motor power must still be physically disconnected when working on the frame.

## Bluetooth bench testing from a Mac, Windows, or Linux computer

This is BLE GATT, not Classic Bluetooth SPP. Existing SPP-only phone apps are not
compatible. The included Python tool avoids browser Web Bluetooth requirements.
Give the terminal/Python app Bluetooth permission on macOS when prompted.

```sh
python -m venv .venv
source .venv/bin/activate
python -m pip install -r tools/requirements.txt
python tools/ble_bench.py --scan
python tools/ble_bench.py --address DEVICE_ID
python tools/ble_bench.py --address DEVICE_ID --debug
```

On Windows activate with `.venv\Scripts\activate`. Use the scanner's exact
address/UUID; macOS normally returns a UUID rather than a MAC address.

After confirming PWM support, with props removed:

```sh
python tools/ble_bench.py --address DEVICE_ID --motor 1 --pulse 1100
```

Repeat for motor numbers 2, 3, and 4. The tool sends a fresh packet roughly every
20 ms for 1.8 s. Firmware also enforces a **2 s maximum**, **1150 us maximum**, and
single-motor selection. A fresh non-test packet is required before each new test.
The bench tool cannot arm the flight controller. It ends by sending kill.
No Bluetooth hardware or flight was exercised by the automated software tests.

## Experimental gamepad control

```sh
python -m pip install -r tools/requirements-pilot.txt
python tools/ble_gamepad.py --inspect
```

Identify every axis and button first. Supply four distinct axis indices and two
distinct buttons explicitly; their layout varies between gamepads. For example,
**only if these match your inspection**:

```sh
python tools/ble_gamepad.py --address DEVICE_ID --roll-axis 0 --pitch-axis 1 --yaw-axis 2 --throttle-axis 3 --arm-button 4 --deadman-button 5 --throttle-sign -1
```

Use `--roll-sign`, `--pitch-sign`, `--yaw-sign`, and `--throttle-sign` as needed.
Positive roll commands right bank; positive pitch nose down; positive yaw right.
The full throttle axis maps -1 to 1000 us and +1 to 2000 us after sign correction.
A spring-centered throttle stick therefore commands about half throttle when
released. Validate this mapping and use appropriate controls before considering flight.

Start with both buttons released and throttle low. Center roll/pitch/yaw and hold
both buttons for at least one second to arm. Releasing either cuts the motors.
Arming at low throttle runs all four at the configured idle pulse, even at zero stick.
The bridge stops on gamepad removal, BLE write failure, or a host loop stall; it
does not reconnect automatically. Bluetooth packets must continue arriving. BLE
latency/interference, OS scheduling, and gamepad drivers have not been flight-tested.
This BLE test protocol is **unauthenticated**; keep it in a controlled bench setting.
It is not an authenticated production radio link.

## Control and fault behavior

- Main loop: target 250 Hz. MPU6050 is sampled at 1 kHz with a ~42 Hz gyro filter;
  the controller reads a coherent fresh burst, applies calibrated biases, uses a
  complementary attitude filter, angle-to-rate outer loop, and rate PID inner loop.
- Mixer: quad-X with bounded/desaturated outputs; derivative filtering and
  conditional integral updates reduce windup. No tuned response is claimed.
- Arming needs a fresh low-throttle ARM-off packet, valid IMU, level frame within
  10 degrees, centered sticks, verified configuration, then a one-second ARM hold.
  An interrupted hold needs ARM release. No automatic arming or automatic rearming.
- Missing/old/invalid control packets never refresh the link. After 250 ms without
  valid fresh control, or on disconnect/kill, outputs go to 1000 us. Recovering
  the link requires another ARM release and hold. A new BLE session resets sequence
  tracking and all bench-test interlocks.
- Incomplete/stale IMU reads and sensor saturation inhibit flight. IMU read faults
  latch until explicit disarmed calibration. Excessive tilt or a control interval
  above 20 ms also disarms and requires release/rearm.
- A separate motor-output task expires active commands after 30 ms and latches
  an output fault requiring board reset. It runs about every 2 ms, subject to OS
  scheduling. This is software protection, not an independent hardware cutoff;
  complete MCU/RTOS failure can still defeat it. Normal disarm is minimum PWM;
  debug mode is zero duty on every motor pin.
- Detailed BLE notifications and USB printing are suppressed while motors run,
  keeping blocking diagnostics out of the control path.

## BLE wire format

Nordic UART service UUIDs appear in Config.h. RX accepts exactly one **16-byte
binary packet per write**; this fits the default BLE payload without negotiation.
All multibyte fields are little-endian.

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u8 | Version/magic 0xA1 |
| 1 | u8 | flags: ARM=1, KILL=2, TEST=4, DEBUG=8; other bits rejected |
| 2 | u32 | Increasing sequence number, wraps modulo 2^32 |
| 6 | i16 | Roll -1000..1000; in TEST mode motor number 1..4 |
| 8 | i16 | Pitch -1000..1000; zero during TEST |
| 10 | i16 | Yaw -1000..1000; zero during TEST |
| 12 | u16 | Throttle 1000..2000 us, TEST maximum 1150 |
| 14 | u16 | CRC-16/CCITT-FALSE of bytes 0..13 (poly 0x1021, init 0xFFFF) |

Duplicate and out-of-order sequence numbers are rejected. Kill/debug events are
latched across the BLE callback mailbox so a subsequent packet cannot erase the
event before the main loop sees it. TEST+ARM packets are rejected. CRC detects
corruption; it is not authentication. A newly connected session must send ARM off
before it can arm; a first packet containing TEST cannot start a motor test.
TX provides compact status text while disarmed (I=IMU, P=PWM enabled, V=verified).

## Software checks

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined tests/flight_core_test.cpp -o /tmp/flight-tests
/tmp/flight-tests
python -m unittest discover -s tests -v
python -m platformio run
```

Tests cover packet integrity/bounds/sequence wrap, boot arming prevention, throttle
interlocks, interrupted holds, disconnect/timeout/kill, debug latch, attitude signs,
invalid samples, PID limits, mixer bounds/correction signs, and test-motor expiry.
They do not simulate frame dynamics, evaluate a PID tune, or verify ESC/IMU/BLE hardware.

API references: [Espressif LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html),
[Espressif BLE UART example](https://github.com/espressif/arduino-esp32/blob/master/libraries/BLE/examples/UART/UART.ino),
[TDK MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf).
