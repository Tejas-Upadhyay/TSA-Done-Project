/**
 * TSA Drone Flight Controller Firmware - ESP32
 * MPU6050 detection + 4in1 ESC control + Boot modes
 * 
 * Hardware:
 * - ESP32 dev board
 * - MPU6050 (I2C: GPIO 21 SDA, GPIO 22 SCL)
 * - AERO SELFIE 45A 4in1 ESC (PWM signals on GPIO 12,13,14,15)
 * - OVONIC 4S 14.8V LiPo battery
 */

#include <Wire.h>
#include <ESP32PWM.h>

// ===== Pin Definitions =====
// Motor pins for 4in1 ESC (AERO SELFIE 45A)
const int motorPins[4] = {12, 13, 14, 15}; // FL, FR, BL, BR - matching CW/CCW pattern

// Boot mode selection
const int MODE_PIN = 0; // GPIO 0 - check grounded for setup mode

// MPU6050
#define MPU6050_ADDR 0x68

// PWM constants
const int MIN_THROTTLE = 1000;
const int MAX_THROTTLE = 2000;
const int ARM_THROTTLE = 1000;

// State
// Set true to enter diagnostics on every boot, before any ESC initialization.
const bool DEBUG_ON_BOOT = false;
enum DroneMode { NORMAL_MODE, SETUP_MODE, DEBUG_MODE };
DroneMode currentMode = NORMAL_MODE;
String serialCommand;
bool serialOverflow = false;
bool debugStreaming = false;
unsigned long lastDebugSample = 0;

void enterDebugMode();
void pollSerialCommands();
void printDebugSample();

// Timing
unsigned long previousMillis = 0;
const long interval = 50;

void setup() {
  // Initialize serial
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("\n=== TSA Drone Firmware ===");
  Serial.println("Boot sequence starting...");

  // Set motor pins as output
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // Check boot mode - GPIO 0 grounded = Setup mode
  pinMode(MODE_PIN, INPUT_PULLUP);
  bool setupMode = (digitalRead(MODE_PIN) == LOW);
  
  if (setupMode) {
    Serial.println(">>> SETUP/TROUBLESHOOT MODE");
    currentMode = SETUP_MODE;
  } else {
    Serial.println(">>> NORMAL MODE");
    currentMode = NORMAL_MODE;
  }

  // Initialize I2C
  Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
  Wire.setTimeOut(50);

  Serial.println("Type debug + Enter within 3 seconds for motor-disabled diagnostics.");
  if (DEBUG_ON_BOOT) enterDebugMode();
  unsigned long debugWindow = millis();
  while (currentMode != DEBUG_MODE && millis() - debugWindow < 3000) {
    pollSerialCommands();
    delay(1);
  }
  if (currentMode == DEBUG_MODE) return;

  // ===== MPU6050 Detection =====
  Serial.println("Initializing MPU6050...");
  bool mpuFound = false;

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Clear sleep mode
  if (Wire.endTransmission() == 0) {
    delay(100);
    
    // Read WHO_AM_I to confirm
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x75);  // WHO_AM_I register
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDR, 1);
    if (Wire.available()) {
      uint8_t whoami = Wire.read();
      if (whoami == 0x68) {
        Serial.println("  MPU6050 detected! WHO_AM_I: 0x68");
        mpuFound = true;
      } else {
        Serial.print("  Found device at 0x68 but WHO_AM_I: 0x");
        Serial.println(whoami, HEX);
      }
    }
  } else {
    Serial.println("  MPU6050 not found at 0x68");
  }

  if (!mpuFound) {
    Serial.println("  WARNING: MPU6050 not detected!");
    // Continue anyway - limited functionality without gyro
  }

  // ===== ESC Initialization =====
  Serial.println("Arming ESCs...");
  armESCs();
  delay(2000);

  if (currentMode == SETUP_MODE) {
    runSetupMode();
  } else {
    runNormalMode();
  }
}

void loop() {
  pollSerialCommands();
  if (currentMode == DEBUG_MODE && debugStreaming && millis() - lastDebugSample >= 250) {
    lastDebugSample = millis();
    printDebugSample();
  }
  // Main loop - minimal for now, mode-specific handling in respective functions
  // Could add normal flight stabilization loop here later
  delay(10);
}

// ===== ARM ESCs =====
void armESCs() {
  // Send minimum throttle to arm all ESCs
  // BLHeli_S ESCs: send 1000µs for 2 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    setAllMotors(MIN_THROTTLE);
    // Short delay for PWM stability
  }
  setAllMotors(0); // Stop motors
  Serial.println("  ESCs armed");
}

// ===== SETUP MODE: Troubleshooting =====
void runSetupMode() {
  Serial.println("\n=== SETUP MODE: Component Verification ===");
  unsigned long startTime = millis();
  bool allPassed = true;

  // 1. Gyro check
  Serial.println("\n1. Gyro Initialization Check");
  if (checkGyro()) {
    Serial.println("   ✓ Gyro OK - reading 0 when level");
  } else {
    Serial.println("   ✗ Gyro ERROR - not reading 0");
    allPassed = false;
    flashErrorLED(3); // 3 red flashes
  }

  // 2. Motor direction test
  Serial.println("\n2. Motor Direction Test");
  Serial.println("   Sequence: FR -> FL -> BL -> All up/down");
  
  // Front Right motor - should make drone go backward-left
  Serial.println("   Front Right motor: BACKWARD-LEFT");
  setMotor(0, 1500); // Neutral, just to test direction understanding
  delay(1000);
  
  // Front Left motor - should make drone go backward-right
  Serial.println("   Front Left motor: BACKWARD-RIGHT");
  setMotor(1, 1500);
  delay(1000);
  
  // Back Left motor - should make drone go forward-right
  Serial.println("   Back Left motor: FORWARD-RIGHT");
  setMotor(2, 1500);
  delay(1000);
  
  // Spin all motors up then down
  Serial.println("   All motors: SPIN UP then SLOW DOWN");
  setAllMotors(1800);
  delay(2000);
  setAllMotors(1200);
  delay(2000);
  setAllMotors(1000);
  delay(2000);
  setAllMotors(0);

  // 3. Final status
  unsigned long elapsed = millis() - startTime;
  Serial.print("\n=== Setup complete in ");
  Serial.print(elapsed / 1000);
  Serial.println(" seconds ===");

  if (allPassed) {
    Serial.println("✓ ALL CHECKS PASSED - Ready for Normal Mode");
    // Wait 3 seconds then exit setup mode
    delay(3000);
    currentMode = NORMAL_MODE;
  } else {
    Serial.println("✗ SOME CHECKS FAILED - Check connections");
    // Stay in setup mode, loop error indicators
    while (currentMode == SETUP_MODE) {
      flashErrorLED(1);
      delay(500);
      flashErrorLED(2);
      delay(500);
    }
  }
}

// ===== NORMAL MODE: Startup Sequence =====
void runNormalMode() {
  Serial.println("\n=== NORMAL MODE: Startup Sequence ===");
  unsigned long startTime = millis();

  // 1. Power-on self-test
  Serial.println("1. POST: Power-on self-test");
  delay(200);
  
  // 2. Gyro calibration - drone must be stationary
  Serial.println("2. Gyro calibration - keep drone stationary");
  calibrateGyro();
  delay(1000);
  
  // 3. ESC startup beeps
  Serial.println("3. ESC startup sequence");
  setAllMotors(MIN_THROTTLE);
  delay(100);
  setAllMotors(0);
  // BLHeli_S beep pattern: 2 short beeps
  setAllMotors(1200);
  delay(100);
  setAllMotors(0);
  delay(100);
  setAllMotors(1200);
  delay(100);
  setAllMotors(0);
  
  // 4. Ready state
  unsigned long elapsed = millis() - startTime;
  Serial.print("4. Ready! ");
  Serial.print(elapsed);
  Serial.println("ms elapsed");
  Serial.println("   - Solid green LED (simulated)");
  Serial.println("   - RSSI check would happen here");
  Serial.println("   - Ready for flight mode selection");
  
  // Stay in normal mode - could add flight stabilization loop here
  // For now, just maintain armed state
  maintainArmedState();
}

// ===== GYRO CHECK =====
bool checkGyro() {
  // Read MPU6050 registers
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Accelerometer XOUTH
  Wire.endTransmission();
  
  Wire.requestFrom(MPU6050_ADDR, 6);
  if (Wire.available() >= 6) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    
    // Calculate pitch and roll from accelerometer
    // This is simplified - real code would use complementary filter
    float pitch = atan2(ay, az) * 180 / PI;
    float roll = atan2(-ax, az) * 180 / PI;
    
    // Check if close to 0 when drone is level
    if (abs(pitch) < 5 && abs(roll) < 5) {
      return true;
    }
  }
  return false;
}

// ===== GYRO CALIBRATION =====
void calibrateGyro() {
  Serial.println("   Collecting gyro offsets (5 seconds)...");
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  int samples = 0;
  
  unsigned long calStart = millis();
  while (millis() - calStart < 5000) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // Gyro XOUTH
    Wire.endTransmission();
    
    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() >= 6) {
      int16_t gx = Wire.read() << 8 | Wire.read();
      int16_t gy = Wire.read() << 8 | Wire.read();
      int16_t gz = Wire.read() << 8 | Wire.read();
      
      gyroXSum += gx;
      gyroYSum += gy;
      gyroZSum += gz;
      samples++;
    }
    delay(10);
  }
  
  if (samples > 0) {
    long gyroXOffset = gyroXSum / samples;
    long gyroYOffset = gyroYSum / samples;
    long gyroZOffset = gyroZSum / samples;
    
    Serial.print("   Gyro offsets - X: ");
    Serial.print(gyroXOffset);
    Serial.print(" Y: ");
    Serial.print(gyroYOffset);
    Serial.print(" Z: ");
    Serial.println(gyroZOffset);
    Serial.println("   Calibration complete");
  }
}

// ===== MOTOR CONTROL =====
void setMotor(int motorIndex, int microseconds) {
  if (motorIndex < 0 || motorIndex > 3) return;
  if (currentMode == DEBUG_MODE || microseconds <= 0) {
    digitalWrite(motorPins[motorIndex], LOW);
    return;
  }
  
  // Generate PWM signal
  // Standard ESC PWM: 1000-2000µs
  // Set pin HIGH, delay, then LOW
  digitalWrite(motorPins[motorIndex], HIGH);
  delayMicroseconds(microseconds);
  digitalWrite(motorPins[motorIndex], LOW);
}

void setAllMotors(int microseconds) {
  for (int i = 0; i < 4; i++) {
    setMotor(i, microseconds);
  }
}

// ===== MAINTAIN ARMED STATE =====
void maintainArmedState() {
  // Keep ESCs armed - send periodic signals
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 100) {
    setAllMotors(MIN_THROTTLE); // Keep minimum throttle
    lastSend = millis();
  }
}

// ===== ERROR LED FLASH =====
void flashErrorLED(int pattern) {
  // In a real implementation, this would drive an LED pin
  // For now, just use serial as indicator
  Serial.print("   LED flash pattern: ");
  for (int i = 0; i < pattern; i++) {
    Serial.print("RED ");
    delay(100);
    Serial.print("OFF ");
    delay(100);
  }
  Serial.println();
}

// ===== SERIAL DEBUG MODE (115200 baud) =====
void enterDebugMode() {
  currentMode = DEBUG_MODE;
  debugStreaming = false;
  for (int i = 0; i < 4; i++) digitalWrite(motorPins[i], LOW);
  Serial.println("\n=== DEBUG MODE: motor signals disabled ===");
  Serial.println("Commands: help, status, scan, imu, stream on, stream off");
  Serial.println("Reset the board to leave debug mode.");
}

bool readDebugRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU6050_ADDR, count) != count) return false;
  for (uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return true;
}

void printDebugSample() {
  // Wake the device; explicitly select +/-2g and +/-250 degrees/second.
  const uint8_t registers[] = {0x6B, 0x1B, 0x1C};
  for (uint8_t i = 0; i < 3; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(registers[i]);
    Wire.write(0);
    if (Wire.endTransmission() != 0) {
      Serial.println("IMU ERROR: MPU6050 did not acknowledge at 0x68.");
      return;
    }
  }
  delay(10);
  uint8_t identity;
  uint8_t data[14];
  if (!readDebugRegisters(0x75, &identity, 1) || identity != 0x68 ||
      !readDebugRegisters(0x3B, data, sizeof(data))) {
    Serial.println("IMU ERROR: identity mismatch or incomplete sensor read.");
    return;
  }
  int16_t values[7];
  for (int i = 0; i < 7; i++) {
    values[i] = (int16_t)(((uint16_t)data[2 * i] << 8) | data[2 * i + 1]);
  }
  Serial.print("Accel g [X Y Z]: ");
  for (int i = 0; i < 3; i++) { Serial.print(values[i] / 16384.0f, 3); Serial.print(' '); }
  Serial.print(" | Gyro deg/s [X Y Z]: ");
  for (int i = 4; i < 7; i++) { Serial.print(values[i] / 131.0f, 2); Serial.print(' '); }
  Serial.print(" | Temp C: ");
  Serial.println(values[3] / 340.0f + 36.53f, 2);
}

void pollSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialOverflow) {
        serialOverflow = false;
        serialCommand = "";
        Serial.println("Command too long; type help.");
        continue;
      }
      serialCommand.trim();
      serialCommand.toLowerCase();
      if (serialCommand == "debug") {
        enterDebugMode();
      } else if (serialCommand.length() && currentMode == DEBUG_MODE) {
        if (serialCommand == "help") {
          Serial.println("help | status | scan | imu | stream on | stream off; reset to exit");
        } else if (serialCommand == "status") {
          Serial.print("DEBUG; motor signals LOW; uptime ms: ");
          Serial.print(millis());
          Serial.print("; streaming: ");
          Serial.println(debugStreaming ? "on" : "off");
          Serial.println("I2C SDA=21 SCL=22; MPU address=0x68; motors FL=12 FR=13 BL=14 BR=15");
        } else if (serialCommand == "imu") {
          printDebugSample();
        } else if (serialCommand == "stream on") {
          debugStreaming = true;
          Serial.println("Sensor stream enabled (250 ms interval).");
        } else if (serialCommand == "stream off") {
          debugStreaming = false;
          Serial.println("Sensor stream disabled.");
        } else if (serialCommand == "scan") {
          int found = 0;
          for (uint8_t address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0) {
              Serial.print("I2C device: 0x");
              Serial.println(address, HEX);
              found++;
            }
          }
          Serial.print("Devices found: ");
          Serial.println(found);
        } else {
          Serial.println("Unknown command. Type help.");
        }
      }
      serialCommand = "";
    } else if (serialOverflow) {
      continue;
    } else if (serialCommand.length() < 48) {
      serialCommand += c;
    } else {
      // Discard the entire line even when it arrives across multiple loop calls.
      serialCommand = "";
      serialOverflow = true;
    }
  }
}
