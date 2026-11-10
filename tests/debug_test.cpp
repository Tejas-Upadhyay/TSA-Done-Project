#include <assert.h>
#include <stdio.h>
#include "../firmware/drone_debug/drone_debug.ino"
static void commandLine(const std::string& s) {Serial.input+=s;loop();}
int main(){
  setup();assert(writtenLevels.size()==4);
  for(int v:writtenLevels)assert(v==LOW);
  commandLine("  StAtUs  \r\n");assert(Serial.output.find("Motor signals LOW")!=std::string::npos);
  commandLine("scan\n");assert(Serial.output.find("Total devices found: 1")!=std::string::npos);
  Serial.output.clear();commandLine("imu\n");
  assert(Serial.output.find("-1.000")!=std::string::npos);
  assert(Serial.output.find("-1.00")!=std::string::npos);
  assert(Serial.output.find("36.53")!=std::string::npos);
  Wire.present=false;Serial.output.clear();commandLine("imu\n");
  assert(Serial.output.find("MPU6050 not found")!=std::string::npos);
  Wire.present=true;Wire.shortRead=true;Serial.output.clear();commandLine("imu\n");
  assert(Serial.output.find("MPU6050 not found")!=std::string::npos);
  Wire.shortRead=false;
  Serial.output.clear();commandLine(std::string(55,'x'));commandLine("status\n");
  assert(Serial.output.find("Command too long")!=std::string::npos);
  assert(Serial.output.find("Motor signals LOW")==std::string::npos);
  commandLine("status\n");assert(Serial.output.find("Motor signals LOW")!=std::string::npos);
  commandLine("stream on\n");Serial.output.clear();fakeTime+=250;loop();
  assert(Serial.output.find("Accel g")!=std::string::npos);
  commandLine("stream off\n");Serial.output.clear();fakeTime+=500;loop();assert(Serial.output.empty());
  for(int v:writtenLevels)assert(v==LOW);
  puts("PASS: standalone debug command parsing, signed IMU data, read failures, streaming, LOW motor pins");
}
