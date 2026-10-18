# Drone debug mode

Debug mode provides USB serial diagnostics while holding all four motor signal
pins LOW. It bypasses ESC arming and the automatic startup motor sequences.
Remove propellers before bench work. A LOW signal is not a physical power disconnect.

## Starting debug mode

1. Set `DEBUG_ON_BOOT = true` in `drone_firmware.ino` for repeatable bench use.
2. Upload the firmware to the ESP32, then open Serial Monitor at **115200 baud**
   with Newline or Both NL & CR enabled.
3. Reset the board. Look for `DEBUG MODE: motor signals disabled`.

With `DEBUG_ON_BOOT = false`, send `debug` followed by Enter during the first
three seconds after startup. You can also send it once normal startup finishes,
but that does not prevent the existing startup motor sequences. Serial commands
are not serviced during the existing setup tests or their failure loop; use
`DEBUG_ON_BOOT = true` to bypass those tests.

## Commands

| Command | Result |
| --- | --- |
| `help` | List commands |
| `status` | Mode, uptime, streaming state, and pin mapping |
| `scan` | Scan I2C addresses 0x01–0x7E |
| `imu` | One MPU6050 sample: acceleration in g, angular velocity in degrees/second, temperature in Celsius |
| `stream on` | Print samples every 250 ms |
| `stream off` | Stop streaming |

The sensor command expects address **0x68**, verifies WHO_AM_I, and reports
failed or incomplete reads. Readings are uncalibrated; with the board stationary,
expect roughly 1 g total acceleration and gyro values near zero with some bias.
An I2C scan confirms an address acknowledges; it does not verify device health.
Motor rotation, ESC health, battery voltage, and radio signal are not measured.

Debug mode cannot arm or spin motors. To return to the existing startup behavior,
set `DEBUG_ON_BOOT = false` if necessary, upload, and reset without sending `debug`.

## Arduino sketch layout

Compile `drone_firmware.ino` in its own sketch folder named `drone_firmware`.
The other `.ino` files in the repository are standalone diagnostics and have
their own `setup()` and `loop()` functions; do not compile them together.
The firmware uses the ESP32 Arduino core and its existing ESP32PWM dependency.
