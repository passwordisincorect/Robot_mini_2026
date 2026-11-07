#pragma once
#include "Arduino.h"
enum { DPAD_UP=1,DPAD_DOWN=2,DPAD_RIGHT=4,DPAD_LEFT=8 };
struct Controller {
  uint16_t keys=0,misc=0;
  uint8_t hat=0;
  int32_t lx=0,ly=0,rx=0,ry=0;
  bool connected=true,gamepad=true,fresh=false,disconnected=false;
  int disconnectCalls=0;
  bool delayedDisconnect=false;
  bool isConnected()const{return connected;}
  bool isGamepad()const{return gamepad;}
  bool hasData()const{return fresh;}
  void disconnect(){disconnectCalls++;if(!delayedDisconnect)connected=false;disconnected=true;}
  uint8_t dpad()const{return hat;}
  int32_t axisX()const{return lx;}
  int32_t axisY()const{return ly;}
  int32_t axisRX()const{return rx;}
  int32_t axisRY()const{return ry;}
  bool a()const{return keys&1;}
  bool b()const{return keys&2;}
  bool x()const{return keys&4;}
  bool y()const{return keys&8;}
  bool l1()const{return keys&16;}
  bool r1()const{return keys&32;}
  bool l2()const{return keys&64;}
  bool r2()const{return keys&128;}
  bool thumbL()const{return keys&256;}
  bool thumbR()const{return keys&512;}
  bool miscSystem()const{return misc&1;}
  bool miscSelect()const{return misc&2;}
  bool miscStart()const{return misc&4;}
};
using ControllerPtr=Controller*;
struct FakeBluepad {
  void(*connectCallback)(ControllerPtr)=nullptr;
  void(*disconnectCallback)(ControllerPtr)=nullptr;
  bool nextUpdate=false,virtualEnabled=true;
  uint8_t address[6]={1,2,3,4,5,6};
  void setup(void(*c)(ControllerPtr),void(*d)(ControllerPtr)){connectCallback=c;disconnectCallback=d;}
  void enableVirtualDevice(bool enabled){virtualEnabled=enabled;}
  const uint8_t *localBdAddress(){return address;}
  const char *firmwareVersion(){return "TEST STUB";}
  bool update(){bool result=nextUpdate;nextUpdate=false;return result;}
};
inline FakeBluepad BP32;
