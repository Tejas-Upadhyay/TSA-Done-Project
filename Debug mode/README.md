# Sensor-only debug mode

The canonical standalone sketch is `firmware/drone_debug/drone_debug.ino`.
Open it in Arduino IDE (ESP32 Dev Module), or run from the repository root:

```sh
python -m platformio run -e esp32debug -t upload
python -m platformio device monitor -b 115200
```

It holds FL=12, FR=13, BL=14, BR=15 LOW. It never attaches ESC PWM.
Remove propellers and disconnect motor power for sensor bench work.
Commands: `help`, `status`, `scan`, `imu`, `stream on`, `stream off`.
Use newline at 115200 baud. MPU6050: SDA21, SCL22, address 0x68.
Sensor values are uncalibrated in this standalone tool. A scan only verifies ACK.
It cannot verify motor direction, motor identity, battery voltage, or ESC health.

The main controller also supports `debug` (USB or BLE), latching zero motor
duty until reset. Set `DEBUG_ON_BOOT=true` in its Config.h for repeatable use.
Main debug does not auto-test or auto-arm ESCs. See the root README for BLE
commands and the two output locks. Main `imu` shows the latest calibrated state;
`calibrate` retries stationary sensor calibration while disarmed.

The former `Debug mode/debug.cpp` is now a compatibility include. Do not combine
root scanner sketches, since they each define setup/loop. No ESP32PWM package
is required. Both sketches use libraries bundled with Arduino-ESP32.
