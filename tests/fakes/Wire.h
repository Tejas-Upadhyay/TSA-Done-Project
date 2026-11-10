#pragma once
#include "Arduino.h"
class WireFake {
 public:
  byte address=0,reg=0;
  std::vector<byte> tx,rx;
  bool present=true,shortRead=false;
  void begin(int,int){}
  void setTimeOut(int){}
  void beginTransmission(byte a){address=a;tx.clear();}
  void write(byte v){tx.push_back(v);}
  int endTransmission(bool=true){if(!tx.empty())reg=tx[0];return present&&address==0x68?0:2;}
  byte requestFrom(byte,byte n){
    rx.clear();
    if(!present)return 0;
    if(reg==0x75)rx={0x68};
    else if(reg==0x3b) // -1g X, +1g Z, -1 deg/s gyro X, 36.53 C
      rx={0xc0,0,0,0,0x40,0,0,0,0xff,0x7d,0,0,0,0};
    if(shortRead&&rx.size())rx.pop_back();
    return std::min(size_t(n),rx.size());
  }
  byte read(){byte v=rx.front();rx.erase(rx.begin());return v;}
} inline Wire;
