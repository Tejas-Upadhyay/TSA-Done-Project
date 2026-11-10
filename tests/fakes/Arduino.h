#pragma once
#include <stdint.h>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
using byte = uint8_t;
constexpr int OUTPUT=1, LOW=0, HIGH=1, HEX=16;
inline unsigned long fakeTime=0;
inline std::vector<int> writtenLevels;
inline void pinMode(int,int) {}
inline void digitalWrite(int,int level) { writtenLevels.push_back(level); }
inline unsigned long millis() { return fakeTime; }
inline void delay(unsigned long ms) { fakeTime+=ms; }
class String {
 public:
  std::string s;
  String()=default;
  String(const char* v):s(v){}
  unsigned length() const { return s.size(); }
  void trim() {
    auto begin=s.find_first_not_of(" \t\r\n"), end=s.find_last_not_of(" \t\r\n");
    s=begin==std::string::npos ? "" : s.substr(begin,end-begin+1);
  }
  void toLowerCase() { for(char& c:s)c=std::tolower(static_cast<unsigned char>(c)); }
  String& operator+=(char c) {s+=c;return *this;}
  bool operator==(const char* v) const {return s==v;}
};
class SerialFake {
 public:
  std::string input,output;
  void begin(int){}
  int available() const{return input.size();}
  char read(){char c=input.front();input.erase(0,1);return c;}
  void print(const char* v){output+=v;}
  void print(char v){output+=v;}
  template<class T>void print(T v){std::ostringstream o;o<<+v;output+=o.str();}
  void print(byte v,int base){std::ostringstream o;if(base==HEX)o<<std::hex<<int(v);else o<<int(v);output+=o.str();}
  void print(float v,int places){std::ostringstream o;o<<std::fixed<<std::setprecision(places)<<v;output+=o.str();}
  void println(){output+='\n';}
  template<class T>void println(T v){print(v);println();}
  template<class T>void println(T v,int p){print(v,p);println();}
} inline Serial;
