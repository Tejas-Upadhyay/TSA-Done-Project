/**
 * MPU6050 Detection Sketch for ESP32 Drone
 * Checks I2C connection to MPU6050 and reports status
 */

#include <Wire.h>

#define MPU6050_ADDR 0x68

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("MPU6050 Detection Test");
  Serial.println("======================");

  Wire.begin();

  // Attempt to communicate with MPU6050
  // Wake up MPU6050 (write 0 to PWR_MGMT_1 register)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wakes up the MPU-6050)
  uint8_t result = Wire.endTransmission();

  if (result == 0) {
    Serial.println("✓ MPU6050 detected at address 0x68");
    
    // Read WHO_AM_I register to confirm identity
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);  // WHO_AM_I register
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDR, 1);
    if (Wire.available()) {
      uint8_t whoami = Wire.read();
      Serial.print("  WHO_AM_I: 0x");
      Serial.println(whoami, HEX);
      
      if (whoami == 0x68) {
        Serial.println("  ✓ Valid MPU6050 identity confirmed");
      } else {
        Serial.println("  ✗ Unexpected WHO_AM_I value");
      }
    }
  } else {
    Serial.println("✗ MPU6050 NOT detected at address 0x68");
    Serial.println("  Possible issues:");
    Serial.println("  - Wrong I2C address (try 0x69 if AD0 is HIGH)");
    Serial.println("  - MPU6050 not powered");
    Serial.println("  - Wiring issue");
    Serial.println("  - Different I2C device at this address");
  }

  Serial.println("\nDetection complete.");
}

void loop() {
  // Nothing to repeat - this is a one-shot detection
  delay(5000); // Check again every 5 seconds
  
  // Re-run detection
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  if (Wire.endTransmission() == 0) {
    Serial.println("Re-detected: MPU6050 present");
  } else {
    Serial.println("Re-detected: MPU6050 absent");
  }
}