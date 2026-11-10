#include <assert.h>
#include <stdio.h>
#include <limits>
#include "../firmware/drone_firmware/FlightCore.h"
using namespace flight;

static void tick(Safety& s, uint32_t t, uint8_t flags=0, uint16_t throttle=1000,
                 bool linked=true, bool healthy=true, bool allowed=true) {
  Command c; c.sequence=s.command.sequence+1; c.flags=flags; c.throttle=throttle;
  assert(s.accept(c,t)); s.update(t,linked,healthy,allowed,0,0);
}
static void arm(Safety& s, uint32_t base=0) {
  tick(s,base);
  for (uint32_t t=20000; t<=1020000; t+=20000) tick(s,base+t,ARM);
  assert(s.state==State::ARMED);
}
static void testSafety() {
  Safety s;
  tick(s,0,ARM); assert(s.state!=State::ARMED); // no arm-on-connect
  tick(s,20000,0,1700); tick(s,40000,ARM); assert(s.state!=State::ARMED);
  arm(s,100000); tick(s,1140000,ARM,1500); assert(s.state==State::ARMED);
  s.update(s.receivedUs+cfg::LINK_TIMEOUT_US,true,true,true,0,0);
  assert(s.state==State::FAILSAFE);
  tick(s,1500000,ARM); assert(s.state!=State::ARMED); // no auto-rearm
  arm(s,1600000); tick(s,2640000,ARM|KILL); assert(s.state==State::FAILSAFE);
  arm(s,2700000); tick(s,3740000,ARM,1000,false); assert(s.state==State::FAILSAFE);
  arm(s,3800000); tick(s,4840000,ARM,1000,true,false); assert(s.state==State::FAILSAFE);
  arm(s,4900000); s.resetLink(); assert(!s.havePacket && s.state==State::FAILSAFE);
  tick(s,6000000); tick(s,6020000,ARM,1000,true,true,false);
  assert(s.state!=State::ARMING && s.state!=State::ARMED);
  Safety q; tick(q,0); tick(q,20000,ARM); tick(q,40000,ARM,1500);
  assert(q.state==State::DISARMED);
  for(uint32_t t=60000;t<1500000;t+=20000) tick(q,t,ARM);
  assert(q.state!=State::ARMED); // interrupted hold needs release
  arm(q,1600000); q.debug(); tick(q,2640000,ARM);
  assert(q.state==State::DEBUG_MODE); q.resetLink();
  assert(q.state==State::DEBUG_MODE); // debug remains latched across reconnect
  Safety wrap; arm(wrap,0xffff0000u); assert(wrap.state==State::ARMED);
  Safety tilt; tick(tilt,0);
  Command c; c.sequence=2;c.flags=ARM;
  tilt.accept(c,20000);tilt.update(20000,true,true,true,20,0);
  assert(tilt.state!=State::ARMING);
  Safety sequence;
  c.sequence=0xffffffffu; c.flags=0; assert(sequence.accept(c,0));
  c.sequence=0; assert(sequence.accept(c,20000));
  assert(!sequence.accept(c,40000)); assert(sequence.receivedUs==20000);
  c.sequence=0xffffffffu; assert(!sequence.accept(c,50000));
}
static void testPackets() {
  uint8_t p[16] = {0xa1,0,1,0,0,0,0,0,0,0,0,0,0xe8,3,0,0};
  auto crc=[&](){uint16_t v=crc16(p,14);p[14]=v&255;p[15]=v>>8;};
  crc(); assert(p[14]==0xb7 && p[15]==0xb2);
  Command c; assert(decode(p,16,c)); assert(c.throttle==1000 && c.sequence==1);
  assert(!decode(p,15,c)); p[7]^=1; assert(!decode(p,16,c)); p[7]^=1;
  p[1]=0x80;crc();assert(!decode(p,16,c));
  p[1]=TEST;p[6]=1;crc();assert(decode(p,16,c));
  p[1]=TEST|ARM;crc();assert(!decode(p,16,c));
  p[1]=TEST;p[6]=0;crc();assert(!decode(p,16,c));
  p[6]=4;p[12]=0x80;p[13]=0x04;crc();assert(!decode(p,16,c)); // 1152 us
  p[1]=0;p[6]=0;p[12]=0xe7;p[13]=3;crc();assert(!decode(p,16,c)); // 999
  p[12]=0xe8;p[6]=0xff;p[7]=0x7f;crc();assert(!decode(p,16,c));
  assert(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29b1);
}
static void testControl() {
  uint16_t out[4];
  mix(1500,10,0,0,out);assert(out[0]>out[1] && out[2]>out[3]);
  mix(1500,0,10,0,out);assert(out[2]>out[0] && out[3]>out[1]);
  mix(1500,0,0,10,out);assert(out[0]>out[1] && out[3]>out[2]);
  for(int t=1050;t<=2000;t+=25) for(int r=-500;r<=500;r+=50) {
    mix(t,r,-r,300,out);
    for(auto v:out) assert(v>=cfg::IDLE_US && v<=cfg::MAX_US);
  }
  Attitude a;Imu s;
  assert(a.update(s,0.004f));assert(fabsf(a.roll)<0.001 && fabsf(a.pitch)<0.001);
  s.ay=0.173648f;s.az=0.984808f; a=Attitude{};
  assert(a.update(s,0.004f));assert(fabsf(a.roll-10)<0.01);
  s=Imu{};s.ax=-0.173648f;s.az=0.984808f;a=Attitude{};
  assert(a.update(s,0.004f));assert(fabsf(a.pitch-10)<0.01);
  s=Imu{};assert(!a.update(s,0.1f));
  s.gx=std::numeric_limits<float>::quiet_NaN();assert(!a.update(s,0.004f));
  a=Attitude{};s=Imu{};s.az=-1;assert(!a.update(s,0.004f));
  Controller controller;Command c;c.throttle=1500;
  s=Imu{};controller.attitude.roll=10;
  controller.run(s,c,0.004f,out);assert(out[1]>out[0]); // correct right bank
  controller.attitude.roll=0;controller.attitude.pitch=10;controller.reset();
  controller.run(s,c,0.004f,out);assert(out[0]>out[2]); // correct nose down
  c.throttle=1000;controller.run(s,c,0.004f,out);
  for(auto v:out) assert(v==cfg::IDLE_US);
  assert(controller.roll.integral==0);
  Pid pid(1,1,0);
  for(int i=0;i<10000;++i) pid.update(1000,0,0.004f,true);
  assert(fabsf(pid.integral)<=cfg::I_LIMIT);
}
static void testBench() {
  BenchTest bench;Command c;c.flags=TEST;c.roll=1;c.throttle=1100;
  uint16_t out[4]={1000,1000,1000,1000};
  assert(!bench.update(0,c,true,out)); // first packet cannot spin
  Command released;bench.update(20000,released,true,out);
  assert(bench.update(40000,c,true,out));assert(out[0]==1100 && out[1]==1000);
  assert(!bench.update(2040000,c,true,out));
  assert(!bench.update(2060000,c,true,out)); // no automatic repeated test
  bench.update(2080000,released,true,out);
  assert(bench.update(2100000,c,true,out));
  assert(!bench.update(2120000,c,false,out)); // timeout/disconnect stops
  assert(!bench.update(2140000,c,true,out));
  bench.update(2160000,released,true,out);bench.reset();
  assert(!bench.update(2180000,c,true,out)); // fresh session requires release
  bench.update(2200000,released,true,out);c.flags=TEST|KILL;
  assert(!bench.update(2220000,c,true,out));
}
int main() {
  testSafety();testPackets();testControl();testBench();
  puts("PASS: packet validation, arming, failsafes, debug latch, filter, PID, mixer, bench interlocks");
}
