// Debug mode for drone (reference the README.md in this folder).
#include <Arduino.h>
#include <Wire.h>

const byte motor[] = {12, 13, 14, 15}; // Front Left, Front Right, Back Left, Back Right
const byte MPU = 0x68; // Gyro address
const unsigned long SAMPLE_INTERVAL_MS = 250;

String command;
bool discardLine = false;
bool streamEnabled = false;
unsigned long lastSampleTime = 0;

void printHelp();
void handleCommand(String text);
void readCommands();
void scanI2C();
void printIMU();
bool writeRegister(byte reg, byte value);
bool readRegister(byte reg, byte *buffer, byte count);

void setup() {
    for (byte pin : motor) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    Serial.begin(115200);
    Wire.begin(21,22); // ESP32
    Wire.setTimeOut(50); // Limit how long a failed I2C transaction can block.

    Serial.println("===Debug mode for drone===");
    printHelp();
}

void loop() {
    readCommands();

    unsigned long now = millis();
    if (streamEnabled && now - lastSampleTime >= SAMPLE_INTERVAL_MS){
        lastSampleTime = now;
        printIMU();
    }
}

void printHelp() {
  Serial.println("help       - show commands");
  Serial.println("status     - show uptime and pin assignments");
  Serial.println("scan       - find devices on the I2C bus");
  Serial.println("imu        - read the MPU6050 once");
  Serial.println("stream on  - read the MPU6050 every 250 ms");
  Serial.println("stream off - stop repeated readings");
}

void readCommands(){
    while (Serial.available() > 0){
        char c = Serial.read();
        if (c == '\n' || c == '\r'){
            if (discardLine){
                Serial.println("Command too long. Type help");
            }
            else if (command.length() > 0){
                handleCommand(command);
            }
            command = "";
            discardLine = false;
        }
        else if (!discardLine) {
            if (command.length() < 48){
                command += c;
            } else {
                command = "";
                discardLine = true;
            }
        }
    }
}

void handleCommand(String text){
    text.trim();
    text.toLowerCase();
    if (text == "help") {
        printHelp();
    }
    else if (text == "status") {
        Serial.print("debug mode for drone, uptime: ");
        Serial.println(millis());
        Serial.println("Motor signals LOW: FL=12, FR=13, BL=14, BR=15");
        Serial.println("I2C bus: SDA=21, SCL=22; expected MPU6050 address: 0x68");
        Serial.print("Streaming: ");
        Serial.println(streamEnabled ? "ON" : "OFF");
    }
    else if (text == "scan") {
        scanI2C();
    }
    else if (text == "imu") {
        printIMU();
    }
    else if (text == "stream on") {
        streamEnabled = true;
        Serial.println("Streaming enabled");
    }
    else if (text == "stream off") {
        streamEnabled = false;
        Serial.println("Streaming stopped");
    }
    else if (text.length() > 0) {
        Serial.println("Unknown command. Type help");
    }
}

void scanI2C(){
    int devices = 0;
    Serial.println("Scanning I2C bus...");
    for(byte address = 1; address < 127; address++){
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0){
            Serial.print("Found device at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            devices++;
        }
    }
    Serial.print("Total devices found: ");
    Serial.println(devices);
}

bool writeRegister(byte reg, byte value){
    Wire.beginTransmission(MPU);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readRegister(byte reg, byte *buffer, byte count){
    Wire.beginTransmission(MPU);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU, count) != count) return false;
    for (byte i = 0; i < count; i++) buffer[i] = Wire.read();
    return true;
}

void printIMU() {
    byte identity;
    if (!readRegister(0x75, &identity, 1) || identity != 0x68) {
        Serial.println("MPU6050 not found");
        return;
    }
    if (!writeRegister(0x6B, 0) || !writeRegister(0x1B, 0) || !writeRegister(0x1C, 0)) {
        Serial.println("Failed to configure MPU6050");
        return;
    }
    delay(10); // Short settling time after waking the sensor.

    byte buffer[14];
    if (!readRegister(0x3B, buffer, 14)) {
        Serial.println("Failed to read MPU6050 data");
        return;
    }

    // Sensor measurements can be negative, so use a signed 16-bit type.
    int16_t raw[7];
    for (byte i = 0; i < 7; i++) {
        raw[i] = (int16_t)(((uint16_t)buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    }

  Serial.print("Accel g [X Y Z]: ");
  for (byte i = 0; i < 3; i++) {
    Serial.print(raw[i] / 16384.0f, 3);
    Serial.print(' ');
  }
  Serial.print("| Gyro deg/s [X Y Z]: ");
  for (byte i = 4; i < 7; i++) {
    Serial.print(raw[i] / 131.0f, 2);
    Serial.print(' ');
  }
  Serial.print("| Temperature C: ");
  Serial.println(raw[3] / 340.0f + 36.53f, 2);
}
