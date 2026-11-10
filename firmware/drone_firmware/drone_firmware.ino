/** ESP32-WROOM-32 + MPU6050 experimental BLE quad-X controller.
 * Read README.md before enabling output. No automatic motor tests or arming.
 * This is not flight-validated firmware. Remove propellers for initial work.
 */
#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_arduino_version.h>
#include "FlightCore.h"

using namespace flight;
Safety safety;
Controller controller;
Imu imu;
float gyroBias[3] = {};
bool imuReady = false, pwmReady = false, streamEnabled = false;
bool usbKillRequested = false;
uint32_t lastTick = 0, lastPrint = 0, lastNotify = 0;

// BLE callbacks never touch I2C, PWM, or the controller. Fixed-size mailbox only.
portMUX_TYPE linkMux = portMUX_INITIALIZER_UNLOCKED;
struct LinkMailbox {
  Command command;
  uint32_t timestamp = 0, epoch = 0;
  bool connected = false, have = false, kill = false, debug = false;
} linkBox;
uint32_t observedEpoch = 0;
BenchTest bench;
BLECharacteristic* telemetry = nullptr;
BLEServer* bleServer = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    portENTER_CRITICAL(&linkMux);
    linkBox.connected = true; linkBox.have = false; ++linkBox.epoch;
    portEXIT_CRITICAL(&linkMux);
  }
  void onDisconnect(BLEServer*) override {
    portENTER_CRITICAL(&linkMux);
    linkBox.connected = false; linkBox.have = false; ++linkBox.epoch;
    portEXIT_CRITICAL(&linkMux);
  }
};
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const auto value = characteristic->getValue();
    Command c;
    if (!decode(reinterpret_cast<const uint8_t*>(value.c_str()), value.length(), c)) return;
    const uint32_t now = micros();
    portENTER_CRITICAL(&linkMux);
    if (linkBox.connected && (!linkBox.have || newer(c.sequence, linkBox.command.sequence))) {
      linkBox.command = c; linkBox.timestamp = now; linkBox.have = true;
      linkBox.kill |= bool(c.flags & KILL); linkBox.debug |= bool(c.flags & DEBUG);
    }
    portEXIT_CRITICAL(&linkMux);
  }
};

// A separate task OWNS all PWM duty writes. A stuck main loop cannot leave its
// last high throttle indefinitely: commands expire and the fault latches.
portMUX_TYPE outputMux = portMUX_INITIALIZER_UNLOCKED;
struct OutputMailbox {
  uint16_t pulse[4] = {1000,1000,1000,1000};
  uint32_t timestamp = 0;
  bool pulsesEnabled = false, active = false, tripped = false;
} outputBox;

uint32_t pulseDuty(uint16_t us) {
  return uint32_t((uint64_t(us) * cfg::PWM_HZ * (1UL << cfg::PWM_BITS) + 500000) / 1000000);
}
void writeDuty(int i, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(i, duty);
#else
  ledcWrite(i, duty);
#endif
}
void outputTask(void*) {
  for (;;) {
    OutputMailbox snapshot;
    const uint32_t now = micros();
    portENTER_CRITICAL(&outputMux);
    if (outputBox.active && uint32_t(now-outputBox.timestamp) > cfg::OUTPUT_TIMEOUT_US)
      outputBox.tripped = true;
    snapshot = outputBox;
    portEXIT_CRITICAL(&outputMux);
    for (int i = 0; i < 4; ++i) {
      const uint16_t pulse = snapshot.tripped ? cfg::STOP_US : snapshot.pulse[i];
      writeDuty(i, snapshot.pulsesEnabled ? pulseDuty(pulse) : 0);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
void submitOutputs(const uint16_t pulse[4], bool pulsesEnabled, bool active) {
  portENTER_CRITICAL(&outputMux);
  for (int i = 0; i < 4; ++i) outputBox.pulse[i] = pulse[i];
  outputBox.timestamp = micros(); outputBox.pulsesEnabled = pulsesEnabled;
  outputBox.active = active;
  portEXIT_CRITICAL(&outputMux);
}
bool outputTripped() {
  portENTER_CRITICAL(&outputMux);
  const bool result = outputBox.tripped;
  portEXIT_CRITICAL(&outputMux);
  return result;
}
bool initPwm() {
  for (uint8_t pin : cfg::MOTOR_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  if (!cfg::ESC_PWM_CONFIRMED) return false;
  for (int i = 0; i < 4; ++i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttachChannel(cfg::MOTOR_PINS[i], cfg::PWM_HZ, cfg::PWM_BITS, i)) {
      for (int j = 0; j < i; ++j) ledcDetach(cfg::MOTOR_PINS[j]);
      return false;
    }
#else
    if (ledcSetup(i, cfg::PWM_HZ, cfg::PWM_BITS) == 0) {
      for (int j = 0; j < i; ++j) ledcDetachPin(cfg::MOTOR_PINS[j]);
      return false;
    }
    ledcAttachPin(cfg::MOTOR_PINS[i], i);
#endif
    writeDuty(i, 0);
  }
  if (xTaskCreatePinnedToCore(outputTask, "ESC-output", 3072, nullptr, 3, nullptr, 0) != pdPASS) {
    for (int i = 0; i < 4; ++i) writeDuty(i, 0);
    return false;
  }
  return true;
}

bool readRegisters(uint8_t reg, uint8_t* data, uint8_t count) {
  Wire.beginTransmission(cfg::MPU_ADDRESS); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(cfg::MPU_ADDRESS, count) != count) return false;
  for (int i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(cfg::MPU_ADDRESS); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool configureImu() {
  uint8_t identity;
  if (!readRegisters(0x75, &identity, 1) || identity != 0x68) return false;
  if (!writeRegister(0x6b, 0x80)) return false; // reset, with motors stopped
  delay(100);
  if (!writeRegister(0x6b, 0x01) || !writeRegister(0x6c, 0x00) ||
      !writeRegister(0x1a, 0x03) || // DLPF ~42 Hz gyro / 44 Hz accel
      !writeRegister(0x19, 0x00) || // 1 kHz sensor, read at 250 Hz
      !writeRegister(0x1b, 0x08) || // +/-500 deg/s, 65.5 LSB/(deg/s)
      !writeRegister(0x1c, 0x08)) return false; // +/-4 g, 8192 LSB/g
  delay(100);
  // Verify scaling-critical registers rather than trusting ACK alone.
  uint8_t regs[4];
  return readRegisters(0x19, regs, 4) && regs[0] == 0 && regs[1] == 3 &&
         regs[2] == 8 && regs[3] == 8;
}
bool readImu(Imu& sample, bool calibrated = true) {
  uint8_t data[15];
  // INT_STATUS + coherent 14-byte measurement burst. Reject stale samples.
  if (!readRegisters(0x3a, data, sizeof(data)) || !(data[0] & 1)) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; ++i) {
    raw[i] = int16_t(uint16_t(data[1+2*i]) << 8 | data[2+2*i]);
    if (i != 3 && abs(int(raw[i])) >= 32700) return false;
  }
  sample.ax = raw[0]/8192.0f; sample.ay = raw[1]/8192.0f; sample.az = raw[2]/8192.0f;
  sample.gx = raw[4]/65.5f - (calibrated ? gyroBias[0] : 0);
  sample.gy = raw[5]/65.5f - (calibrated ? gyroBias[1] : 0);
  sample.gz = raw[6]/65.5f - (calibrated ? gyroBias[2] : 0);
  return finite(sample);
}
bool calibrateImu() {
  Serial.println("Keep the frame level and motionless: calibrating IMU...");
  imuReady = false;
  controller.reset(); controller.attitude = Attitude{};
  if (!configureImu()) return false;
  double sum[6] = {}, square[6] = {};
  constexpr int count = 500;
  for (int n = 0; n < count; ++n) {
    delay(2);
    Imu s;
    if (!readImu(s, false)) return false;
    const float v[6] = {s.gx,s.gy,s.gz,s.ax,s.ay,s.az};
    for (int j = 0; j < 6; ++j) { sum[j] += v[j]; square[j] += v[j]*v[j]; }
  }
  for (int j = 0; j < 6; ++j) {
    const double mean = sum[j]/count;
    const double variance = fmax(0.0, square[j]/count-mean*mean);
    if (sqrt(variance) > (j < 3 ? 0.8 : 0.025) || (j < 3 && fabs(mean) > 5))
      return false;
  }
  const double ax = sum[3]/count, ay = sum[4]/count, az = sum[5]/count;
  const double norm = sqrt(ax*ax+ay*ay+az*az);
  if (norm < 0.9 || norm > 1.1 || az < 0.8) return false;
  for (int j = 0; j < 3; ++j) gyroBias[j] = sum[j]/count;
  imuReady = true;
  return true;
}

void startBle() {
  BLEDevice::init(cfg::BLE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());
  BLEService* service = bleServer->createService(cfg::SERVICE_UUID);
  auto* rx = service->createCharacteristic(cfg::RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  telemetry = service->createCharacteristic(cfg::TX_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  telemetry->addDescriptor(new BLE2902());
  service->start();
  auto* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(cfg::SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

const char* stateName() {
  switch (safety.state) {
    case State::DISARMED: return "DISARMED";
    case State::ARMING: return "ARMING";
    case State::ARMED: return "ARMED";
    case State::FAILSAFE: return "FAILSAFE";
    default: return "DEBUG";
  }
}
void printStatus() {
  Serial.printf("%s; %s; IMU=%s PWM=%s flightVerified=%d outputTrip=%d\n",
      stateName(), safety.reason, imuReady ? "ready" : "fault",
      pwmReady ? "enabled" : "LOCKED", cfg::FLIGHT_CONFIGURATION_VERIFIED, outputTripped());
  Serial.printf("FL=%u FR=%u BL=%u BR=%u; SDA=%u SCL=%u MPU=0x%02x\n",
      cfg::MOTOR_PINS[0],cfg::MOTOR_PINS[1],cfg::MOTOR_PINS[2],cfg::MOTOR_PINS[3],
      cfg::SDA_PIN,cfg::SCL_PIN,cfg::MPU_ADDRESS);
  Serial.printf("roll=%.2f pitch=%.2f; gyro=[%.2f %.2f %.2f] deg/s\n",
      controller.attitude.roll,controller.attitude.pitch,imu.gx,imu.gy,imu.gz);
}
void enterDebug() {
  safety.debug(); controller.reset();
  const uint16_t stopped[4] = {1000,1000,1000,1000};
  submitOutputs(stopped, false, false);
  Serial.println("DEBUG latched: motor duties zero; reset to leave.");
}
void handleSerial(const char* command) {
  if (!strcmp(command, "debug")) { enterDebug(); return; }
  if (!strcmp(command, "kill")) { usbKillRequested = true; safety.stop("USB kill"); return; }
  // Printing and I2C scans can block. Only the immediate stop/debug commands
  // above are allowed during a motor run or arming hold.
  if (safety.state == State::ARMED || safety.state == State::ARMING ||
      (safety.havePacket && (safety.command.flags & TEST))) return;
  if (!strcmp(command,"help"))
    Serial.println("help | status | debug | kill | calibrate | scan | imu | stream on | stream off");
  else if (!strcmp(command,"status")) printStatus();
  else if (!strcmp(command,"calibrate")) {
    safety.resetLink();
    // Discard packets received during the blocking calibration; require release/re-arm.
    imuReady = calibrateImu();
    portENTER_CRITICAL(&linkMux); linkBox.have = false; ++linkBox.epoch; portEXIT_CRITICAL(&linkMux);
    Serial.println(imuReady ? "Calibration passed" : "Calibration FAILED");
    lastTick = micros();
  } else if (!strcmp(command,"scan")) {
    for (uint8_t a=1; a<127; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) Serial.printf("I2C 0x%02x\n", a);
    }
    lastTick = micros();
  } else if (!strcmp(command,"imu")) printStatus();
  else if (!strcmp(command,"stream on")) streamEnabled = true;
  else if (!strcmp(command,"stream off")) streamEnabled = false;
  else Serial.println("Unknown command; type help.");
}
void pollSerial() {
  static char line[49]; static uint8_t used = 0; static bool overflow = false;
  // Bound work per tick so serial flooding cannot starve the control loop.
  for (int budget=0; budget<64 && Serial.available(); ++budget) {
    const char c = Serial.read();
    if (c=='\r' || c=='\n') {
      if (!overflow && used) { line[used]=0; handleSerial(line); }
      used=0; overflow=false;
    } else if (!overflow) {
      if (used<sizeof(line)-1) line[used++]=c;
      else { overflow=true; used=0; }
    }
  }
}

void setup() {
  for (uint8_t pin : cfg::MOTOR_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  Serial.begin(115200);
  Wire.begin(cfg::SDA_PIN,cfg::SCL_PIN,400000); Wire.setTimeOut(5);
  Serial.println("TSA experimental BLE controller; NO automatic spin tests.");
  if (cfg::DEBUG_ON_BOOT) safety.debug();
  pwmReady = initPwm();
  imuReady = calibrateImu();
  startBle();
  printStatus();
  lastTick = micros();
}

void loop() {
  pollSerial();
  const uint32_t now = micros(), elapsed = now-lastTick;
  if (elapsed < cfg::LOOP_US) { delayMicroseconds(100); return; }
  lastTick = now;
  LinkMailbox link;
  portENTER_CRITICAL(&linkMux);
  link = linkBox; linkBox.kill = false; linkBox.debug = false;
  portEXIT_CRITICAL(&linkMux);
  if (link.epoch != observedEpoch) {
    observedEpoch = link.epoch; safety.resetLink(); controller.reset();
    bench.reset();
    if (!link.connected) BLEDevice::startAdvertising();
  }
  if (link.have) safety.accept(link.command,link.timestamp);
  const bool emergency = link.kill || usbKillRequested;
  usbKillRequested = false;
  if (link.debug) safety.debug();
  if (emergency) safety.stop("USB/Bluetooth kill");
  const bool timingOk = elapsed <= 20000;
  const bool sampleOk = readImu(imu);
  if (!sampleOk) imuReady = false; // latched until explicit stationary calibration
  const float dt = elapsed*1.0e-6f;
  const bool attitudeOk = sampleOk && controller.attitude.update(imu, dt);
  const bool wasArmed = safety.state == State::ARMED;
  const bool health = imuReady && attitudeOk && timingOk && !outputTripped();
  safety.update(now,link.connected,health,
      pwmReady && cfg::FLIGHT_CONFIGURATION_VERIFIED,
      controller.attitude.roll,controller.attitude.pitch);
  if (!health && wasArmed) safety.stop("sensor, attitude, timing, or output fault");

  uint16_t out[4] = {1000,1000,1000,1000};
  bool active = false;
  const bool testPermitted = !emergency && link.connected && safety.fresh(now) && !wasArmed &&
      safety.state != State::DEBUG_MODE && pwmReady && !outputTripped();
  active = bench.update(now,safety.command,testPermitted,out);
  if (safety.state == State::ARMED && health) {
    controller.run(imu,safety.command,dt,out); active=true;
  } else controller.reset();
  submitOutputs(out,pwmReady && safety.state!=State::DEBUG_MODE,active);

  // No blocking printing or BLE notifications while motors are active.
  if (!active && safety.state!=State::ARMING) {
    if (streamEnabled && now-lastPrint>=250000) { lastPrint=now; printStatus(); }
    if (link.connected && now-lastNotify>=500000) {
      lastNotify=now;
      char message[21];
      snprintf(message,sizeof(message),"%s I%d P%d V%d",stateName(),imuReady,pwmReady,
          cfg::FLIGHT_CONFIGURATION_VERIFIED);
      telemetry->setValue(message); telemetry->notify();
    }
  }
}
