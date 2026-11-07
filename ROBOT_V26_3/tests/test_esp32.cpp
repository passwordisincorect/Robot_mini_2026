#define ARDUINO_ARCH_ESP32 1
#define CONFIG_IDF_TARGET_ESP32 1
#include "Arduino.h"
#include <fstream>
#include "../ESP32_PS3_BRIDGE_V11/ESP32_PS3_BRIDGE_V11.ino"

Controller padTest;
void connect(){
  BP32.connectCallback(&padTest);
  padTest.fresh=true;BP32.nextUpdate=true;loop();
  CHECK(!sentValid); // A reconnect must first send an explicit stop.
  loop();CHECK(sentValid);
}
uint16_t wireSeq(){size_t n=Serial2.tx.size();CHECK(n>=14);return uint16_t(Serial2.tx[n-10])|(uint16_t(Serial2.tx[n-9])<<8);}
void validWire(){
  CHECK(!Serial2.tx.empty()&&Serial2.tx.size()%14==0);
  for(size_t i=0;i<Serial2.tx.size();i+=14){
    const auto *f=Serial2.tx.data()+i;
    CHECK(f[0]==0xa5&&f[1]==0x5a&&f[2]==1&&f[3]<=1);
    CHECK(crc16(f+2,10)==(uint16_t(f[12])|(uint16_t(f[13])<<8)));
  }
}
void boot(){
  CHECK(REPORT_TIMEOUT_MS==200&&STALE_LINK_MS==5000&&SEND_MS==20);
  CHECK(Serial.baud==115200&&Serial2.baud==115200&&Serial2.txPin==17);
  CHECK(!BP32.virtualEnabled);
  CHECK(Serial.log.find("01:02:03:04:05:06")!=std::string::npos);
  loop();CHECK(Serial2.tx.size()==14&&!sentValid);
  CHECK(Serial2.tx[3]==0&&Serial2.tx[6]==0xff&&Serial2.tx[7]==0xff);
  for(int i=8;i<12;i++)CHECK(Serial2.tx[i]==128);
  validWire();
}
void mapping(){
  const uint16_t blueKeys[]={0,256,512,0,0,0,0,0,64,128,16,32,8,2,1,4};
  const uint8_t blueHat[]={0,0,0,0,1,4,2,8,0,0,0,0,0,0,0,0};
  CHECK(readButtons(&padTest)==0xffff);
  for(int i=0;i<16;i++){
    padTest.keys=blueKeys[i];padTest.hat=blueHat[i];
    padTest.misc=i==0?2:(i==3?4:0);
    CHECK(readButtons(&padTest)==uint16_t(0xffff^(1U<<i)));
  }
  padTest.keys=1023;padTest.hat=15;padTest.misc=6;
  CHECK(readButtons(&padTest)==0);
  padTest.keys=0;padTest.hat=0;padTest.misc=1;CHECK(readButtons(&padTest)==0xffff);
  CHECK(axisByte(-10000)==0&&axisByte(-512)==0&&axisByte(0)==128);
  CHECK(axisByte(511)==255&&axisByte(512)==255&&axisByte(10000)==255);
  CHECK(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29b1);
}
void freshness(){
  connect();CHECK(reportSeq==1);
  uint32_t lastReport=reportAt;
  for(int i=0;i<8;i++){clockMs=lastReport+20*(i+1);loop();}
  CHECK(reportSeq==1&&wireSeq()==1&&sentValid);
  clockMs=lastReport+199;loop();CHECK(sentValid);
  clockMs=lastReport+200;loop();CHECK(!sentValid);
  CHECK(Serial2.tx[Serial2.tx.size()-11]==0);
  // Updates from another device are not fresh data from the chosen pad.
  padTest.fresh=false;BP32.nextUpdate=true;loop();CHECK(reportSeq==1&&!sentValid);
  padTest.fresh=true;BP32.nextUpdate=true;loop();CHECK(reportSeq==2&&sentValid);
  validWire();
}
void disconnect(){
  connect();padTest.connected=false;BP32.disconnectCallback(&padTest);
  loop();CHECK(!sentValid&&!gamepad&&!haveReport);
  padTest.connected=true;BP32.connectCallback(&padTest);BP32.nextUpdate=true;
  loop();CHECK(!sentValid);loop();CHECK(sentValid);
  Controller extra;BP32.connectCallback(&extra);CHECK(extra.disconnected&&gamepad==&padTest);
  BP32.disconnectCallback(&extra);CHECK(gamepad==&padTest);
  // Disconnect and reconnect before one loop still sends invalid first.
  BP32.disconnectCallback(&padTest);BP32.connectCallback(&padTest);BP32.nextUpdate=true;
  size_t n=Serial2.tx.size();loop();CHECK(Serial2.tx[n+3]==0);
  loop();CHECK(sentValid);
}
void delayedClassification(){
  padTest.gamepad=false;
  BP32.connectCallback(&padTest);loop();
  CHECK(gamepad==&padTest&&!padTest.disconnected&&!sentValid);
  padTest.gamepad=true;padTest.fresh=true;BP32.nextUpdate=true;loop();
  CHECK(sentValid&&reportSeq==1);
}
void staleRecovery(){
  padTest.delayedDisconnect=true;connect();const uint32_t start=reportAt;
  clockMs=start+199;loop();CHECK(sentValid&&padTest.disconnectCalls==0);
  clockMs=start+200;loop();CHECK(!sentValid&&padTest.connected&&padTest.disconnectCalls==0);
  clockMs=start+4999;loop();CHECK(!sentValid&&padTest.disconnectCalls==0);
  clockMs=start+5000;loop();CHECK(disconnectRequested&&padTest.disconnectCalls==1&&!sentValid);
  const uint16_t seq=reportSeq;
  for(int i=0;i<10;i++){
    clockMs+=20;BP32.nextUpdate=true;loop();
    CHECK(padTest.disconnectCalls==1&&reportSeq==seq&&!sentValid);
  }
  padTest.connected=false;BP32.disconnectCallback(&padTest);loop();
  CHECK(!gamepad&&!disconnectRequested&&disconnectCount==1);
  padTest.connected=true;connect();CHECK(sentValid&&reportSeq==uint16_t(seq+1));
  Controller extra;BP32.connectCallback(&extra);BP32.disconnectCallback(&extra);
  CHECK(extra.disconnectCalls==1&&disconnectCount==1&&gamepad==&padTest);
  padTest.connected=false;BP32.disconnectCallback(&padTest);
  Controller noData;noData.fresh=false;BP32.connectCallback(&noData);loop();
  const uint32_t connected=connectedAt;
  clockMs=connected+4999;loop();CHECK(noData.disconnectCalls==0&&!sentValid);
  clockMs=connected+5000;loop();CHECK(noData.disconnectCalls==1&&!sentValid);
  validWire();
}
void staleRollover(){
  clockMs=UINT32_MAX-2500;connect();const uint32_t start=reportAt;
  clockMs=start+4999;loop();CHECK(padTest.disconnectCalls==0&&!sentValid);
  clockMs=start+5000;loop();CHECK(padTest.disconnectCalls==1&&!sentValid);
  validWire();
}
void blockedSerial(){
  Serial2.capacity=0;BP32.connectCallback(&padTest);
  padTest.fresh=true;BP32.nextUpdate=true;loop();CHECK(Serial2.tx.empty()&&resetPending);
  clockMs+=250;Serial2.capacity=63;loop();CHECK(!sentValid&&!resetPending);
  CHECK(Serial2.tx.size()==14);
  BP32.nextUpdate=true;loop();CHECK(sentValid);
  Serial.capacity=0;clockMs+=200;loop();CHECK(!sentValid);
}
void wrap(){
  clockMs=UINT32_MAX-10;connect();
  reportSeq=65535;BP32.nextUpdate=true;loop();CHECK(reportSeq==0);
  clockMs+=20;loop();CHECK(sentValid&&wireSeq()==0);
  clockMs=reportAt+200;loop();CHECK(!sentValid);
}
void freshLog(){
  clockMs+=20;padTest.fresh=true;BP32.nextUpdate=true;loop();
}
void nextMonitorLine(){
  clockMs=statusAt+500;padTest.fresh=true;BP32.nextUpdate=true;loop();
}
size_t occurrences(const std::string &text,const std::string &part){
  size_t count=0,pos=0;
  while((pos=text.find(part,pos))!=std::string::npos){count++;pos+=part.size();}
  return count;
}
void monitorButtons(){
  Serial.capacity=127;connect();
  const uint16_t blueKeys[]={0,256,512,0,0,0,0,0,64,128,16,32,8,2,1,4,0};
  const uint8_t blueHat[]={0,0,0,0,1,4,2,8,0,0,0,0,0,0,0,0,0};
  const char *names[]={"SELECT","L3","R3","START","LEN","PHAI","XUONG","TRAI",
                       "L2","R2","L1","R1","TAM_GIAC","TRON","X","VUONG","PS"};
  for(int i=0;i<17;i++){
    Serial.log.clear();monitorJoy=false;pressedSincePrint=0;statusAt=clockMs;
    padTest.keys=blueKeys[i];padTest.hat=blueHat[i];
    padTest.misc=i==0?2:(i==3?4:(i==16?1:0));
    freshLog();CHECK(Serial.log.empty());nextMonitorLine();
    CHECK(Serial.log.find(std::string("GIU=")+names[i]+"\n")!=std::string::npos);
    Serial.log.clear();padTest.keys=0;padTest.hat=0;padTest.misc=0;
    freshLog();nextMonitorLine();
    CHECK(Serial.log.find(std::string("NHAN_1S=")+names[i]+"\n")!=std::string::npos);
    Serial.log.clear();nextMonitorLine();
    CHECK(Serial.log.find("GIU=KHONG\n")!=std::string::npos);
  }
  Serial.log.clear();monitorJoy=false;pressedSincePrint=0;
  padTest.keys=1023;padTest.hat=15;padTest.misc=7;nextMonitorLine();
  const std::string all="SELECT L3 R3 START LEN PHAI XUONG TRAI L2 R2 L1 R1 TAM_GIAC TRON X VUONG PS\n";
  CHECK(Serial.log.find("GIU="+all)!=std::string::npos);
  CHECK(Serial.log.size()<=127);
  Serial.log.clear();nextMonitorLine();
  CHECK(Serial.log.find("NHAN_1S="+all)!=std::string::npos);
  CHECK(Serial.log.size()<=127);
  validWire();
}
void monitorShortPress(){
  Serial.capacity=127;connect();Serial.log.clear();statusAt=clockMs;monitorJoy=true;
  padTest.keys=16;freshLog();padTest.keys=0;freshLog();
  CHECK(Serial.log.empty());nextMonitorLine();
  CHECK(Serial.log.find("NHAN_1S=L1\n")!=std::string::npos);
  Serial.log.clear();nextMonitorLine();CHECK(Serial.log.find("GIU=KHONG\n")!=std::string::npos);
  Serial.log.clear();nextMonitorLine();CHECK(Serial.log.find("NHAN_1S=KHONG\n")!=std::string::npos);
  validWire();
}
void monitorJoystick(){
  Serial.capacity=127;connect();Serial.log.clear();statusAt=clockMs;monitorJoy=true;
  padTest.lx=-512;padTest.ly=511;padTest.rx=0;padTest.ry=256;freshLog();
  CHECK(Serial.log.empty());nextMonitorLine();
  CHECK(Serial.log.find("JOY LX=0 LY=255 RX=128 RY=192 NHAN_1S=")!=std::string::npos);
  nextMonitorLine();Serial.log.clear();
  padTest.lx=padTest.ly=padTest.rx=padTest.ry=0;nextMonitorLine();
  CHECK(Serial.log.find("JOY LX=128 LY=128 RX=128 RY=128 NHAN_1S=")!=std::string::npos);
  nextMonitorLine();Serial.log.clear();padTest.rx=-512;padTest.ry=511;nextMonitorLine();
  CHECK(Serial.log.find("JOY LX=128 LY=128 RX=0 RY=255 NHAN_1S=")!=std::string::npos);
  validWire();
}
void monitorRate(){
  Serial.capacity=127;connect();Serial.log.clear();statusAt=clockMs;monitorJoy=true;
  size_t frames=Serial2.tx.size()/14;uint32_t start=clockMs;
  for(int i=1;i<=1000;i++){
    clockMs=start+i;padTest.lx=(i*4)%1024-512;
    padTest.keys=i%2?1023:0;padTest.hat=i%2?15:0;padTest.misc=i%2?7:0;
    padTest.fresh=true;BP32.nextUpdate=true;loop();
  }
  CHECK(occurrences(Serial.log,"\n")==2);
  CHECK(occurrences(Serial.log,"JOY LX=")==1);
  CHECK(occurrences(Serial.log,"PS3=1 SEQ=")==1);
  CHECK(Serial2.tx.size()/14-frames==50);
  Serial.log.clear();start=clockMs;frames=Serial2.tx.size()/14;
  for(int i=1;i<=1000;i++){clockMs=start+i;BP32.nextUpdate=true;loop();}
  CHECK(occurrences(Serial.log,"\n")==2&&Serial2.tx.size()/14-frames==50);
  validWire();
}
void monitorBackpressure(){
  Serial.capacity=127;connect();Serial.log.clear();Serial.capacity=0;statusAt=clockMs;
  padTest.ly=-512;padTest.keys=16;
  nextMonitorLine();CHECK(Serial.log.empty()&&sentValid);
  CHECK(wireSeq()==reportSeq);
  Serial.capacity=127;loop();
  CHECK(Serial.log.find("JOY LX=128 LY=0 RX=128 RY=128 NHAN_1S=L1")!=std::string::npos);
  size_t length=Serial.log.size();loop();CHECK(Serial.log.size()==length);
  Serial.log.clear();nextMonitorLine();
  CHECK(Serial.log.find("GIU=L1")!=std::string::npos);
  Serial.log.clear();Serial.capacity=0;clockMs=reportAt+200;loop();CHECK(!sentValid&&Serial.log.empty());
  clockMs=statusAt+500;loop();CHECK(Serial.log.empty());
  Serial.capacity=127;loop();
  CHECK(Serial.log=="JOY: KHONG XAC DINH (PS3=0)\n");
  CHECK(pressedSincePrint==0);
  validWire();
}
void monitorReconnect(){
  Serial.capacity=127;connect();Serial.log.clear();statusAt=clockMs;monitorJoy=false;
  padTest.keys=16;freshLog();BP32.disconnectCallback(&padTest);loop();
  CHECK(Serial.log.empty()&&!sentValid&&pressedSincePrint==0);
  clockMs=statusAt+500;loop();
  CHECK(Serial.log.find("PS3=0")!=std::string::npos);
  CHECK(Serial.log.find("GIU=KHONG_XAC_DINH")!=std::string::npos);
  Serial.log.clear();padTest.keys=0;connect();nextMonitorLine();
  CHECK(Serial.log.find("NHAN_1S=KHONG\n")!=std::string::npos);
  Serial.log.clear();statusAt=UINT32_MAX-250;
  clockMs=statusAt+499;BP32.nextUpdate=true;loop();CHECK(Serial.log.empty());
  clockMs=statusAt+500;BP32.nextUpdate=true;loop();CHECK(occurrences(Serial.log,"\n")==1);
  validWire();
}
void monitorLinkState(){
  Serial.capacity=127;connect();Serial.log.clear();monitorJoy=false;
  clockMs=reportAt+200;statusAt=clockMs-500;loop();
  CHECK(Serial.log.find("PS3=0")!=std::string::npos);
  CHECK(Serial.log.find("AGE=200 BT=1 DC=0")!=std::string::npos);
  CHECK(!sentValid&&padTest.disconnectCalls==0);
  padTest.connected=false;BP32.disconnectCallback(&padTest);loop();
  Serial.log.clear();monitorJoy=false;clockMs=statusAt+500;loop();
  CHECK(Serial.log.find("AGE=9999 BT=0 DC=1")!=std::string::npos);
  padTest.connected=true;connect();
  padTest.keys=1023;padTest.hat=15;padTest.misc=7;freshLog();
  reportSeq=disconnectCount=65535;clockMs=reportAt+199;statusAt=clockMs-500;
  Serial.log.clear();monitorJoy=false;loop();
  CHECK(Serial.log.find("SEQ=65535 AGE=199 BT=1 DC=65535")!=std::string::npos);
  CHECK(Serial.log.find("VUONG PS\n")!=std::string::npos&&Serial.log.size()<=127);
  validWire();
}
void resetDiagnostics(){
  CHECK(Serial.log.find("RESET_REASON=1 BROWNOUT=0")!=std::string::npos);
  Serial.log.clear();fakeResetReason=ESP_RST_BROWNOUT;setup();
  CHECK(Serial.log.find("RESET_REASON=9 BROWNOUT=1")!=std::string::npos);
}
void emit(const char *path){
  connect();padTest.keys=512;padTest.lx=-512;padTest.ly=511;padTest.rx=0;padTest.ry=256;
  clockMs+=20;BP32.nextUpdate=true;loop();
  CHECK(sentValid);
  std::ofstream out(path,std::ios::binary);CHECK(out.good());
  out.write(reinterpret_cast<const char*>(Serial2.tx.data()),std::streamsize(Serial2.tx.size()));
  CHECK(out.good());
}
int main(int argc,char **argv){
  try{
    CHECK(argc>=2);setup();std::string name=argv[1];
    if(name=="boot")boot();
    else if(name=="mapping")mapping();
    else if(name=="freshness")freshness();
    else if(name=="disconnect")disconnect();
    else if(name=="backpressure")blockedSerial();
    else if(name=="classification")delayedClassification();
    else if(name=="stale_recovery")staleRecovery();
    else if(name=="stale_rollover")staleRollover();
    else if(name=="rollover")wrap();
    else if(name=="monitor_buttons")monitorButtons();
    else if(name=="monitor_short_press")monitorShortPress();
    else if(name=="monitor_joystick")monitorJoystick();
    else if(name=="monitor_rate")monitorRate();
    else if(name=="monitor_backpressure")monitorBackpressure();
    else if(name=="monitor_reconnect")monitorReconnect();
    else if(name=="monitor_link_state")monitorLinkState();
    else if(name=="reset_diagnostics")resetDiagnostics();
    else if(name=="emit"){CHECK(argc==3);emit(argv[2]);}
    else throw std::runtime_error("Unknown test");
    std::printf("PASS esp32/%s\n",name.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}
}
