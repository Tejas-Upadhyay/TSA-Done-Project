/**
 * I2C Scanner for ESP32
 * Scans I2C bus to find all connected devices
 * Useful for verifying MPU6050 address and other I2C peripherals
 */

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Wire.begin();
  Serial.println("I2C Scanner Started");
  Serial.println("Scanning I2C bus...");
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);  // Wait 5 seconds for next scan
}