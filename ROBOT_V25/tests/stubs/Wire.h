#pragma once
#include "Arduino.h"
#include <array>
#include <functional>
struct FakeWire {
  uint32_t hz=0,timeoutUs=0;
  bool begun=false,resetTimeout=false,timeout=false,present=false;
  bool timeoutOnEnd=false,timeoutOnRead=false,missingByte=false;
  int nackRegister=-1,shortRead=-1,ignoreWriteRegister=-1;
  uint32_t endDelayMs=0,readDelayMs=0;
  unsigned endCalls=0,readCalls=0;
  std::function<void()> onEnd,onRead;
  uint8_t actualAddress=0x68,address=0,reg=0,lastReadReg=0,lastReadCount=0;
  std::vector<uint8_t> tx;
  std::deque<int> rx;
  std::array<uint8_t,128> registers{};
  FakeWire(){registers[0x75]=0x68;}
  void begin(){begun=true;digitalWrite(20,HIGH);digitalWrite(21,HIGH);}
  void setClock(uint32_t v){hz=v;}
  void setWireTimeout(uint32_t v,bool reset){timeoutUs=v;resetTimeout=reset;}
  void clearWireTimeoutFlag(){timeout=false;}
  bool getWireTimeoutFlag(){return timeout;}
  void beginTransmission(uint8_t a){address=a;tx.clear();}
  size_t write(uint8_t v){tx.push_back(v);return 1;}
  uint8_t endTransmission(bool stop=true){
    CHECK(!tx.empty());endCalls++;clockMs+=endDelayMs;
    if(onEnd)onEnd();
    if(timeoutOnEnd){timeout=true;return 0;}
    if(!present||address!=actualAddress||tx[0]==nackRegister)return 2;
    reg=tx[0];
    if(tx.size()==2){
      CHECK(stop);
      if(reg==0x6B&&tx[1]==0x80){uint8_t id=registers[0x75];registers.fill(0);registers[0x75]=id;registers[0x6B]=0x40;}
      else if(reg!=ignoreWriteRegister)registers[reg]=tx[1];
    }else CHECK(tx.size()==1&&!stop);
    return 0;
  }
  uint8_t requestFrom(uint8_t a,uint8_t count,uint8_t stop){
    CHECK(stop);readCalls++;clockMs+=readDelayMs;
    if(onRead)onRead();
    rx.clear();lastReadReg=reg;lastReadCount=count;
    if(timeoutOnRead){timeout=true;return count;}
    if(!present||a!=actualAddress)return 0;
    const uint8_t got=shortRead>=0?uint8_t(std::min(int(count),shortRead)):count;
    for(uint8_t i=0;i<got;i++)rx.push_back(registers[reg+i]);
    if(missingByte&&!rx.empty())rx.pop_back();
    return got;
  }
  int read(){if(rx.empty())return -1;int v=rx.front();rx.pop_front();return v;}
};
inline FakeWire Wire;
