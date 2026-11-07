#define __AVR_ATmega2560__ 1
#include "Arduino.h"
#include <array>
#include <fstream>
#include <iterator>
#include <cmath>
#include "../MEGA_PS3_UART_V26_3/MEGA_PS3_UART_V26_3.ino"

using Frame=std::array<uint8_t,14>;
uint16_t sequence=0;
void seal(Frame &f){uint16_t c=crc16(f.data()+2,10);f[12]=uint8_t(c);f[13]=uint8_t(c>>8);}
Frame frame(uint16_t seq,uint16_t keys=0xffff,uint8_t lx=128,uint8_t ly=128,uint8_t rx=128,uint8_t ry=128,uint8_t valid=1){
  Frame f={0xa5,0x5a,1,valid,uint8_t(seq),uint8_t(seq>>8),uint8_t(keys),uint8_t(keys>>8),lx,ly,rx,ry,0,0};
  seal(f);return f;
}
void bytes(const uint8_t *data,size_t count){
  Serial2.rx.insert(Serial2.rx.end(),data,data+count);
  int budget=10000;
  do{loop();CHECK(--budget>0);}while(Serial2.available());
}
void inject(const Frame &f){bytes(f.data(),f.size());}
uint16_t held(uint8_t bit){return uint16_t(0xffff^(1U<<bit));}
void sample(uint16_t keys=0xffff,uint8_t lx=128,uint8_t ly=128,uint8_t rx=128,uint8_t ry=128,uint32_t step=20){
  clockMs+=step;inject(frame(++sequence,keys,lx,ly,rx,ry));
}
void arm(){
  for(int i=0;i<7;i++)sample();
  CHECK(armed);CHECK(uartOK);
}
void stopped(){
  CHECK(!armed&&!braking&&!autoLiftDirection&&!releaseOn);
  for(int i=0;i<4;i++){
    CHECK(command[i]==0&&pwmLevel[EN[i]]==255);
    CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);
  }
  CHECK(liftCommand==0&&slideCommand==0);
  CHECK(outputLevel[SUCTION]==LOW&&outputLevel[RELEASE_VALVE]==LOW);
}
void wheels(int a,int b,int c,int d){
  CHECK(command[0]==a&&command[1]==b&&command[2]==c&&command[3]==d);
}
void boot(){
  CHECK(Serial.baud==115200&&Serial2.baud==115200&&Serial3.baud==115200);
  CHECK(!armed&&outputLevel[28]==LOW&&outputLevel[40]==LOW);
  CHECK(VALVE_ON==HIGH&&VALVE_OFF==LOW);
  for(int pin:{18,19,26,27})CHECK(pinModes[pin]==INPUT_PULLUP);
  CHECK(!Hwt901b::valid&&!YawPid::ready);
  for(auto e:writes)CHECK(e.pin!=20&&e.pin!=21);
  CHECK(LIMIT_UP==18&&LIMIT_DOWN==19&&LIMIT_LEFT==26&&LIMIT_RIGHT==27);
  for(int pin:{28,40}){
    bool set=false;
    for(auto e:writes)if(e.pin==pin){
      if(e.kind=='D'){CHECK(e.value==LOW);set=true;}
      if(e.kind=='M'&&e.value==OUTPUT)CHECK(set);
    }
  }
  for(auto e:writes)CHECK(e.pin!=22&&e.pin!=23&&e.pin!=24&&e.pin!=25);
  for(int i=0;i<4;i++){CHECK(pwmLevel[EN[i]]==255);CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);}
}
void armButtons(){
  inject(frame(++sequence));CHECK(!armed);
  for(uint8_t bit:{uint8_t(4),uint8_t(5),uint8_t(6),uint8_t(7),uint8_t(11),uint8_t(14),uint8_t(15)}){
    for(int i=0;i<7;i++){sample(held(bit));}
    CHECK(!armed);
    for(int i=0;i<7;i++){sample();}
    CHECK(armed);
    sample(held(2));stopped();
  }
}
void protocol(){
  CHECK(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29b1);
  const uint8_t noise[]={0x42,0xa5,0xa5};
  bytes(noise,sizeof(noise));
  inject(frame(1));CHECK(rxCount==1);
  Frame f=frame(2);f[8]^=1;inject(f);CHECK(rxCount==1&&badCount>0);
  f=frame(2);f[2]=2;seal(f);inject(f);CHECK(rxCount==1);
  f=frame(2);f[3]=2;seal(f);inject(f);CHECK(rxCount==1);
  f=frame(2);bytes(f.data(),6);CHECK(rxCount==1);
  bytes(f.data()+6,8);CHECK(rxCount==2);
  f=frame(3);std::vector<uint8_t> broken(f.begin(),f.end());broken.erase(broken.begin()+8);
  bytes(broken.data(),broken.size());inject(frame(4));CHECK(lastSeq==4&&rxCount==3);
  f=frame(5);broken.assign(f.begin(),f.end());broken.insert(broken.begin()+7,0x77);
  bytes(broken.data(),broken.size());inject(frame(6));CHECK(lastSeq==6);
  f=frame(7);bytes(f.data(),5);clockMs+=30;loop();CHECK(uartLength==0);
  inject(frame(8));CHECK(lastSeq==8);
  // A header inside payload must not lose the following good frame.
  f=frame(9,0x5aa5,0xa5,0x5a);inject(f);CHECK(lastSeq==9&&pad.buttons==0x5aa5);
  Serial2.rx.assign(4096,0x11);loop();CHECK(Serial2.rx.size()==4096-64);
}
void sequences(){
  inject(frame(100));CHECK(rxCount==1&&!armed);
  clockMs=100;inject(frame(100));CHECK(rxAt==0&&rxCount==1&&!armed);
  inject(frame(99));CHECK(rxCount==1);
  inject(frame(uint16_t(100+0x8000)));CHECK(rxCount==1);
  inject(frame(101));CHECK(armed&&rxAt==100);
  clockMs=299;loop();CHECK(armed);
  clockMs=300;loop();stopped();CHECK(!sequenceSeen&&!uartOK);
  inject(frame(3,held(11)));CHECK(!armed&&outputLevel[28]==LOW);
  clockMs+=20;inject(frame(4));CHECK(!armed);
  clockMs+=100;loop();CHECK(!armed); // A single neutral sample cannot re-arm.
  inject(frame(5));CHECK(armed);
  invalidateLink();inject(frame(65535));inject(frame(0));CHECK(lastSeq==0&&uartOK);
  auto before=rxCount;inject(frame(65535));CHECK(rxCount==before);
  inject(frame(0,0xffff,128,128,128,128,0));stopped();CHECK(!uartOK);
}
void interlocks(){
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[40]==LOW);
  for(int i=0;i<20;i++){sample(held(11));}
  CHECK(suctionOn);
  sample();sample(held(11));CHECK(!suctionOn);
  CHECK(releaseOn&&outputLevel[40]==HIGH);
  sample();sample(held(11));CHECK(suctionOn&&!releaseOn&&outputLevel[40]==LOW);
  sample(held(2));stopped();CHECK(outputLevel[40]==LOW);
}
void suctionLeds(){
  CHECK(SUCTION_LED1==50 && SUCTION_LED2==52 && SUCTION==28);
  for(int pin:{50,52}){
    CHECK(pinModes[pin]==OUTPUT&&outputLevel[pin]==LOW);
    bool safeLow=false,configured=false;
    for(auto e:writes)if(e.pin==pin){
      if(e.kind=='D' && e.value==LOW && !configured)safeLow=true;
      if(e.kind=='M' && e.value==OUTPUT){CHECK(safeLow);configured=true;}
    }
    CHECK(configured);
  }
  arm();sample();CHECK(!suctionOn&&outputLevel[28]==LOW);
  CHECK(outputLevel[50]==LOW&&outputLevel[52]==LOW);
  sample(held(11));CHECK(suctionOn&&outputLevel[28]==HIGH);
  for(int i=0;i<20;i++)sample(held(11));
  CHECK(suctionOn&&outputLevel[50]==HIGH&&outputLevel[52]==HIGH);
  sample();sample(held(11));CHECK(!suctionOn&&releaseOn&&outputLevel[40]==HIGH&&outputLevel[28]==LOW);
  CHECK(outputLevel[50]==LOW&&outputLevel[52]==LOW);
  sample();sample(held(11));CHECK(suctionOn&&!releaseOn&&outputLevel[40]==LOW&&outputLevel[50]==HIGH&&outputLevel[52]==HIGH);
  sample(held(2));stopped();
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[50]==HIGH&&outputLevel[52]==HIGH);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  arm();sample(held(11));clockMs+=200;loop();stopped();CHECK(!uartOK);
}
void pumpEnable(){
  CHECK(SUCTION==28&&pinModes[28]==OUTPUT&&outputLevel[28]==LOW);
  for(int i=0;i<8;i++){sample(held(11));}
  CHECK(!armed&&outputLevel[28]==LOW);
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[28]==HIGH);
  for(int i=0;i<20;i++){sample(held(11));}
  CHECK(suctionOn&&outputLevel[28]==HIGH);
  sample();sample(held(11));CHECK(!suctionOn&&releaseOn&&outputLevel[28]==LOW&&outputLevel[40]==HIGH);
  sample();sample(held(11));CHECK(suctionOn&&!releaseOn&&outputLevel[28]==HIGH&&outputLevel[40]==LOW);
  sample(held(2));stopped();
  arm();sample(held(11));sample();CHECK(outputLevel[28]==HIGH);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[28]==HIGH);
  clockMs+=200;loop();stopped();
  for(auto e:writes)if(e.pin==28)CHECK(e.kind!='A');
}
void relayPulse(){
  CHECK(RELEASE_PULSE_MS==500);
  arm();sample(held(11));CHECK(suctionOn&&!releaseOn&&outputLevel[40]==LOW);
  sample();sample(held(11));CHECK(!suctionOn&&releaseOn&&outputLevel[40]==HIGH);
  const uint32_t started=releaseAt;
  Serial.log.clear();monitorPage=0;statusAt=clockMs-500;serialTask();
  CHECK(Serial.log.find("VAN=1")!=std::string::npos&&Serial.log.size()<=63);
  for(int i=0;i<24;i++){sample(held(11));}
  CHECK(uint32_t(clockMs-started)==480&&releaseOn&&outputLevel[40]==HIGH);
  sample(held(11),128,128,128,128,19);
  CHECK(uint32_t(clockMs-started)==499&&releaseOn&&outputLevel[40]==HIGH);
  sample(held(11),128,128,128,128,1);
  CHECK(uint32_t(clockMs-started)==500&&!releaseOn&&outputLevel[40]==LOW);
  for(int i=0;i<10;i++){sample(held(11));}
  CHECK(!suctionOn&&!releaseOn&&outputLevel[40]==LOW);
  sample();sample(held(11));CHECK(suctionOn&&!releaseOn&&outputLevel[40]==LOW);
  Serial.log.clear();monitorPage=0;statusAt=clockMs-500;serialTask();
  CHECK(Serial.log.find("VAN=0")!=std::string::npos&&Serial.log.size()<=63);
}
void relaySafety(){
  for(int i=0;i<8;i++){sample(held(11));}
  CHECK(!armed&&!releaseOn&&outputLevel[40]==LOW);
  arm();sample(held(11));sample();sample(held(11));CHECK(releaseOn&&outputLevel[40]==HIGH);
  sample(); // Isolate the valve state from the R1-held startup interlock.
  CHECK(!StartupPid::idleReady());
  sample(held(4),128,0);
  CHECK(releaseOn&&liftCommand==255&&command[1]!=0); // Release pulse does not block lift or wheels.
  inputLevel[LIMIT_UP]=HIGH;loop();CHECK(liftCommand==0&&releaseOn);
  inputLevel[LIMIT_UP]=LOW;
  sample(held(11));CHECK(suctionOn&&!releaseOn&&outputLevel[40]==LOW);
  sample();sample(held(11));CHECK(!suctionOn&&releaseOn&&outputLevel[40]==HIGH);
  sample(held(2));stopped();CHECK(outputLevel[40]==LOW);
  arm();CHECK(!releaseOn&&outputLevel[40]==LOW); // Re-arming never resumes a cancelled pulse.
  sample(held(11));sample();sample(held(11));CHECK(releaseOn);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  arm();CHECK(!releaseOn&&outputLevel[40]==LOW);
  sample(held(11));sample();sample(held(11));CHECK(releaseOn);
  clockMs+=200;loop();stopped();CHECK(outputLevel[40]==LOW);
  arm();CHECK(!releaseOn&&outputLevel[40]==LOW);
  bool pump=false,valve=false;
  for(const auto &event:writes)if(event.kind=='D'){
    if(event.pin==SUCTION)pump=event.value==HIGH;
    if(event.pin==RELEASE_VALVE)valve=event.value==HIGH;
    CHECK(!(pump&&valve)); // Verify the relay/pump output ordering never overlaps.
  }
}
void relayRollover(){
  clockMs=UINT32_MAX-200;arm();
  sample(held(11));sample();sample(held(11));
  const uint32_t started=releaseAt;
  CHECK(releaseOn&&started>UINT32_MAX-5);
  for(int i=0;i<24;i++){sample(held(11));}
  CHECK(uint32_t(clockMs-started)==480&&releaseOn);
  sample(held(11),128,128,128,128,19);CHECK(releaseOn);
  sample(held(11),128,128,128,128,1);
  CHECK(uint32_t(clockMs-started)==500&&!releaseOn&&outputLevel[40]==LOW);
  for(int i=0;i<10;i++){sample(held(11));}
  CHECK(!releaseOn&&outputLevel[40]==LOW);
}
void r3(){
  arm();sample(held(15));CHECK(liftCommand==255);
  sample(held(11),255,0);CHECK(suctionOn&&command[0]!=0);
  sample(held(2));stopped();
  for(int i=0;i<8;i++){sample(held(11),255);}
  stopped();
  arm();sample(held(11));CHECK(suctionOn);
  sample(0xffff,128,0);sample();CHECK(braking);
  sample(held(2));stopped();
}
void limits(){
  arm();
  const int pins[]={18,19,26,27};
  const uint8_t keys[]={4,6,7,5},opposite[]={6,4,5,7};
  for(int i=0;i<4;i++){
    sample(held(keys[i]));CHECK((i<2?liftCommand:slideCommand)!=0);
    CHECK(std::abs(i<2?liftCommand:slideCommand)==255);
    if(i<2)CHECK(autoLiftDirection==0);
    inputLevel[pins[i]]=HIGH;loop();CHECK((i<2?liftCommand:slideCommand)==0);
    inputLevel[pins[i]]=LOW;clockMs+=19;loop();CHECK((i<2?liftCommand:slideCommand)==0);
    clockMs++;loop();CHECK(i<2?liftCommand!=0:slideCommand!=0);
    inputLevel[pins[i]]=HIGH;loop();CHECK((i<2?liftCommand:slideCommand)==0);
    inputLevel[pins[i]]=LOW;clockMs+=19;loop();CHECK((i<2?liftCommand:slideCommand)==0);
    clockMs++;loop();CHECK(i<2?liftCommand!=0:slideCommand!=0);
    sample();sample(held(opposite[i]));CHECK((i<2?liftCommand:slideCommand)!=0);
    sample();inputLevel[18]=inputLevel[19]=inputLevel[26]=inputLevel[27]=LOW;
  }
  inputLevel[18]=inputLevel[19]=inputLevel[26]=inputLevel[27]=HIGH;
  sample(uint16_t(held(4)&held(7)));CHECK(liftCommand==0&&slideCommand==0);
  sample(uint16_t(held(6)&held(5)));CHECK(liftCommand==0&&slideCommand==0);
}
void manualLift(){
  arm();sample(held(15));CHECK(autoLiftDirection==1&&liftCommand==255);
  sample(uint16_t(held(15)&held(4)));
  CHECK(autoLiftDirection==0&&liftCommand==255); // Manual lift takes priority over auto-up.
  sample(held(15));CHECK(autoLiftDirection==0&&liftCommand==0); // No auto restart on release.
  sample(uint16_t(held(15)&held(4)));CHECK(liftCommand==255);
  inputLevel[LIMIT_UP]=HIGH;loop();CHECK(liftCommand==0);
  inputLevel[LIMIT_UP]=LOW;clockMs+=20;loop();CHECK(liftCommand==255); // Held manual control resumes after the NC switch closes.
  sample(uint16_t(held(4)&held(6)&held(15)&held(14)));
  CHECK(liftCommand==0&&autoLiftDirection==0); // Opposing manuals and autos cancel.
  sample(uint16_t(held(6)&held(15)));CHECK(liftCommand==-255&&autoLiftDirection==0);
  sample(held(15));CHECK(liftCommand==0&&autoLiftDirection==0); // Square cannot resume after manual override.
  sample();sample(held(15));CHECK(liftCommand==255&&autoLiftDirection==1); // New square edge starts auto mode.
  sample(uint16_t(held(15)&held(6)));
  CHECK(liftCommand==-255&&autoLiftDirection==0);
  sample(held(15));CHECK(liftCommand==0&&autoLiftDirection==0);
}
void automaticLift(){
  arm();
  for(int dir:{1,-1}){
    const uint8_t key=dir>0?15:14,opposite=dir>0?14:15;
    const int target=dir>0?18:19,other=dir>0?19:18;
    inputLevel[other]=HIGH;
    sample(held(key));CHECK(autoLiftDirection==dir&&liftCommand==dir*255);
    CHECK(pwmLevel[LIFT_EN]==255);
    Serial.log.clear();monitorPage=0;statusAt=clockMs-500;serialTask();
    CHECK(Serial.log.find(dir>0?"N=255":"N=-255")!=std::string::npos);
    CHECK(Serial.log.find(dir>0?"AUTO=1":"AUTO=-1")!=std::string::npos);
    inputLevel[target]=HIGH;sample(held(key));
    CHECK(autoLiftDirection==0&&liftCommand==0&&pwmLevel[LIFT_EN]==0);
    inputLevel[target]=LOW;clockMs+=10;loop();sample(held(key));
    CHECK(autoLiftDirection==0&&liftCommand==0); // Holding cannot restart after limit contact.
    inputLevel[target]=HIGH;loop();inputLevel[target]=LOW;clockMs+=19;loop();
    CHECK(autoLiftDirection==0&&liftCommand==0);
    clockMs++;loop();CHECK(autoLiftDirection==0&&liftCommand==0);
    inputLevel[other]=LOW;clockMs+=20;loop();
    sample();sample(held(key));CHECK(autoLiftDirection==dir&&liftCommand==dir*255);
    sample(uint16_t(held(key)&held(opposite)));
    CHECK(autoLiftDirection==0&&liftCommand==0);
    sample(held(opposite));CHECK(autoLiftDirection==0&&liftCommand==0);
    sample();sample(held(opposite));CHECK(autoLiftDirection==-dir&&liftCommand==-dir*255);
    sample(held(2));CHECK(autoLiftDirection==0&&liftCommand==0);
    inputLevel[other]=LOW;arm();
  }
}
void autoLiftSafety(){
  for(uint8_t key:{uint8_t(15),uint8_t(14)}){
    arm();sample(held(key));sample();CHECK(autoLiftDirection!=0);
    inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
    for(int i=0;i<8;i++){sample(held(key));}
    stopped();
    arm();sample(held(key));CHECK(autoLiftDirection!=0);
    clockMs+=200;loop();stopped();
    arm();sample(held(key));sample(held(2));stopped();
  }
}
void normalBrake(){
  CHECK(BRAKE_START_PWM==85&&NORMAL_BRAKE_PWM==170&&BRAKE_RAMP_MS==120);
  arm();sample();
  for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==NORMAL_BRAKE_PWM&&command[i]==0);
  sample(held(2),128,128,128,128,1);stopped(); // Must upgrade immediately inside the 20ms refresh window.
  arm();sample();
  for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  arm();sample();clockMs+=200;loop();stopped();
}
void movement(){
  arm();const int s=driveSpeed,t=turnSpeed,trim=(s*STRAFE_FORWARD_TRIM_PERCENT+50)/100,sb=s-trim;
  CHECK(driveSpeed==255&&turnSpeed==179);
  sample(0xffff,128,0);wheels(-s,s,-s,s);
  CHECK(outputLevel[30]==LOW&&outputLevel[31]==HIGH);
  CHECK(outputLevel[32]==HIGH&&outputLevel[33]==LOW);
  CHECK(outputLevel[34]==LOW&&outputLevel[35]==HIGH);
  CHECK(outputLevel[37]==LOW&&outputLevel[36]==HIGH);
  sample(0xffff,128,255);wheels(s,-s,s,-s);
  sample(0xffff,255,128);wheels(-sb-trim,-sb+trim,-sb-trim,-sb+trim);
  sample(0xffff,0,128);wheels(sb-trim,sb+trim,sb-trim,sb+trim);
  sample(0xffff,255,0);wheels(-s,s,-s,s);
  sample(0xffff,0,0);wheels(-s,s,-s,s);
  sample(0xffff,255,255);wheels(s,-s,s,-s);
  sample(0xffff,0,255);wheels(sb-trim,sb+trim,sb-trim,sb+trim);
  sample(0xffff,128,128,255);wheels(-t,t,t,-t);
  sample(0xffff,128,128,0);wheels(t,-t,-t,t);
  sample();CHECK(braking&&brakeDuration==BRAKE_RAMP_MS);wheels(0,0,0,0);
  clockMs+=BRAKE_RAMP_MS-1;loop();CHECK(braking);
  clockMs++;loop();CHECK(!braking);wheels(0,0,0,0);
  sample(0xffff,158);wheels(0,0,0,0);
  sample(0xffff,255,128);sample(0xffff,158);wheels(-sb-trim,-sb+trim,-sb-trim,-sb+trim);
  sample();wheels(0,0,0,0);
  sample(held(2));arm();sample(0xffff,128,0);sample();CHECK(braking&&brakeDuration==BRAKE_RAMP_MS);
  wheels(0,0,0,0);clockMs+=BRAKE_RAMP_MS-1;loop();CHECK(braking);
  clockMs++;loop();CHECK(!braking);wheels(0,0,0,0);
  for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
  sample(0xffff,255,128);sample();wheels(0,0,0,0);
}
void relayActiveHigh(){
  arm();
  for(uint8_t key:{uint8_t(8),uint8_t(9),uint8_t(10),uint8_t(13)}){
    sample(held(key));CHECK(!suctionOn&&slideCommand==0&&outputLevel[40]==LOW);sample();
  }
}
void softBrake(){
  const uint8_t positions[][3]={{128,0,128},{128,255,128},{0,128,128},
                                {255,128,128},{128,128,0},{128,128,255}};
  for(int speed:{1,26,100,235,255})for(const auto &p:positions){
    sample(held(2));arm();driveSpeed=turnSpeed=speed;
    sample(0xffff,p[0],p[1],p[2]);
    const int trim=(p[1]==128&&abs(int(p[0])-128)>35)?(speed*STRAFE_FORWARD_TRIM_PERCENT+50)/100:0;
    const int base=std::min(speed,255-trim),fast=base+trim,slow=base-trim;
    int expected[4]={speed,speed,speed,speed};
    if(trim){
      if(p[0]>128){expected[0]=expected[2]=fast;expected[1]=expected[3]=slow;}
      else {expected[0]=expected[2]=slow;expected[1]=expected[3]=fast;}
    }
    for(int i=0;i<4;i++)CHECK(abs(command[i])==expected[i]);
    sample();
    CHECK(braking && brakeDuration==BRAKE_RAMP_MS);
    for(int i=0;i<4;i++){
      CHECK(command[i]==0&&pwmLevel[EN[i]]==BRAKE_START_PWM);
      CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);
    }
    clockMs=brakeAt+BRAKE_RAMP_MS/2;loop();CHECK(braking);
    for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==127&&command[i]==0);
    clockMs=brakeAt+BRAKE_RAMP_MS;loop();CHECK(!braking);
    for(int i=0;i<4;i++){
      CHECK(command[i]==0&&pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
      CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);
    }
  }
}
void brakeInterrupt(){
  arm();const int s=driveSpeed;
  sample(0xffff,128,0);wheels(-s,s,-s,s);
  sample();CHECK(braking);sample(0xffff,128,255);CHECK(!braking);wheels(s,-s,s,-s);
  sample();CHECK(braking);sample(held(2));stopped();
}
void brakeNoReverse(){
  arm();sample(0xffff,128,0);wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
  const auto start=writes.size();sample();CHECK(braking);
  for(size_t i=start;i<writes.size();i++){
    const auto &w=writes[i];for(uint8_t m=0;m<4;m++)
      if((w.pin==A[m]||w.pin==B[m])&&w.kind=='D')CHECK(w.value==LOW);
  }
  for(int i=0;i<4;i++)CHECK(command[i]==0);
}
void fourDirections(){
  arm();const int speed=driveSpeed,trim=(speed*STRAFE_FORWARD_TRIM_PERCENT+50)/100,sb=speed-trim;
  struct Case {uint8_t x,y;int expected[4];};
  const Case cases[]={
    {128,0,{-speed,speed,-speed,speed}}, {128,255,{speed,-speed,speed,-speed}},
    {255,128,{-speed,-speed+2*trim,-speed,-speed+2*trim}},
    {0,128,{sb-trim,sb+trim,sb-trim,sb+trim}},
    {228,28,{-speed,speed,-speed,speed}}, {28,28,{-speed,speed,-speed,speed}},
    {228,228,{speed,-speed,speed,-speed}}, {28,228,{speed,-speed,speed,-speed}},
    {255,28,{-speed,-speed+2*trim,-speed,-speed+2*trim}},
    {255,228,{-speed,-speed+2*trim,-speed,-speed+2*trim}},
    {0,28,{sb-trim,sb+trim,sb-trim,sb+trim}}, {0,228,{sb-trim,sb+trim,sb-trim,sb+trim}},
    {228,0,{-speed,speed,-speed,speed}}, {28,0,{-speed,speed,-speed,speed}},
    {228,255,{speed,-speed,speed,-speed}}, {28,255,{speed,-speed,speed,-speed}}
  };
  for(const auto &c:cases){
    sample(0xffff,c.x,c.y);wheels(c.expected[0],c.expected[1],c.expected[2],c.expected[3]);
    for(int i=0;i<4;i++){
      CHECK(pwmLevel[EN[i]]==std::abs(c.expected[i]));
      const bool aHigh=(i==3)?c.expected[i]<0:c.expected[i]>0;
      CHECK(outputLevel[A[i]]==(aHigh?HIGH:LOW));CHECK(outputLevel[B[i]]==(aHigh?LOW:HIGH));
    }
  }
  CHECK(turnSpeed==179);
  sample(0xffff,228,28,255);wheels(-turnSpeed,turnSpeed,turnSpeed,-turnSpeed);
  sample(0xffff,28,228,0);wheels(turnSpeed,-turnSpeed,-turnSpeed,turnSpeed);
  sample(held(2));stopped();arm();sample(0xffff,228,28);clockMs+=200;loop();stopped();
}
void motorPolarity(){
  // Exact M1/M2/M3 direct and M4 reverse. All 4 outputs verified.
  CHECK(REVERSE[0]==false && REVERSE[1]==false &&
        REVERSE[2]==false && REVERSE[3]==true);
  arm();
  runWheels(100,100,100,100);
  for(int i=0;i<4;i++){
    const bool pinAHigh=(i!=3);
    CHECK(outputLevel[A[i]]==(pinAHigh?HIGH:LOW));
    CHECK(outputLevel[B[i]]==(pinAHigh?LOW:HIGH));
    CHECK(pwmLevel[EN[i]]==100);
  }
  runWheels(-100,-100,-100,-100);
  for(int i=0;i<4;i++){
    const bool pinAHigh=(i==3);
    CHECK(outputLevel[A[i]]==(pinAHigh?HIGH:LOW));
    CHECK(outputLevel[B[i]]==(pinAHigh?LOW:HIGH));
    CHECK(pwmLevel[EN[i]]==100);
  }
  sample(held(2));stopped();
}
void m2ForwardPolarity(){
  CHECK(!REVERSE[0]&&!REVERSE[1]&&!REVERSE[2]&&REVERSE[3]);
  arm();sample(0xffff,128,0);wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
  const uint8_t ea[4]={LOW,HIGH,LOW,LOW},eb[4]={HIGH,LOW,HIGH,HIGH};
  for(int i=0;i<4;i++){CHECK(outputLevel[A[i]]==ea[i]);CHECK(outputLevel[B[i]]==eb[i]);CHECK(pwmLevel[EN[i]]==driveSpeed);}
  sample(held(2));stopped();
}
void observedMapping(){
  const int forwardSign[4]={-1,1,-1,1};arm();
  for(int speed:{60,150,235,255}){
    driveSpeed=speed;const int trim=(speed*STRAFE_FORWARD_TRIM_PERCENT+50)/100;
    const int base=std::min(speed,255-trim);
    for(int x:{0,255}){
      sample(0xffff,uint8_t(x),128);
      int physical[4];for(int i=0;i<4;i++)physical[i]=command[i]*forwardSign[i];
      CHECK(physical[0]+physical[1]+physical[2]+physical[3]==4*trim);
      CHECK(physical[0]-physical[1]+physical[2]-physical[3]==(x>128?1:-1)*4*base);
      CHECK(physical[0]+physical[1]-physical[2]-physical[3]==0);
      for(int v:command)CHECK(std::abs(v)<=255);
    }
  }
}
void mecanumRotation(){
  // Motor longitudinal direction inferred from confirmed V19 forward travel.
  const int forwardSign[4]={-1,1,-1,1};
  const int oldRight[4]={1,1,-1,-1},reportedOldRight[4]={-1,1,1,-1};
  for(int i=0;i<4;i++)CHECK(oldRight[i]*forwardSign[i]==reportedOldRight[i]);
  arm();
  for(int speed:{60,180,255})for(int direction:{1,-1}){
    turnSpeed=speed;
    // Rotation still wins when the left stick is also displaced.
    sample(0xffff,255,0,direction>0?255:0);
    int physical[4];
    for(int i=0;i<4;i++){
      physical[i]=command[i]*forwardSign[i];
      CHECK(physical[i]==direction*(i<2?speed:-speed)); // Left side vs right side.
    }
    CHECK(physical[0]+physical[1]+physical[2]+physical[3]==0); // No forward component.
    CHECK(physical[0]-physical[1]+physical[2]-physical[3]==0); // No strafe component.
    CHECK(physical[0]+physical[1]-physical[2]-physical[3]==4*direction*speed);
    sample();CHECK(braking);
    for(int i=0;i<4;i++){
      CHECK(command[i]==0&&pwmLevel[EN[i]]==BRAKE_START_PWM);
      CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);
    }
    clockMs=brakeAt+brakeDuration;loop();CHECK(!braking);
    for(int i=0;i<4;i++)CHECK(command[i]==0&&pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
  }
  sample(0xffff,128,128,255);sample(held(2));stopped();
  arm();sample(0xffff,128,128,0);clockMs+=200;loop();stopped();
}
void speed(){
  arm();CHECK(driveSpeed==255&&turnSpeed==179&&slideSpeed==255);
  sample(held(8));CHECK(driveSpeed==255&&turnSpeed==179&&slideSpeed==255);
  sample(held(9));CHECK(driveSpeed==255&&turnSpeed==179&&slideSpeed==255);
  sample();sample(held(14));CHECK(liftCommand==-255&&autoLiftDirection==-1);
}
void rollover(){
  clockMs=UINT32_MAX-80;arm();sample(held(11));CHECK(suctionOn);
  sample();sample(held(11));CHECK(!suctionOn);
  clockMs+=200;loop();stopped();
  clockMs=UINT32_MAX-500;invalidateLink();arm();sample(held(15));CHECK(liftCommand==255);
}
void logBackpressure(){
  arm();Serial.capacity=0;sample(held(11));CHECK(suctionOn);
  sample(held(7));CHECK(slideCommand==-255);
  inputLevel[18]=HIGH;inputLevel[27]=HIGH;
  sample(uint16_t(held(15)&held(5)));CHECK(liftCommand==0&&slideCommand==0);
  clockMs+=200;loop();stopped();
}
void limitDiagnostics(){
  Serial.log.clear();inputLevel[18]=HIGH;clockMs=20;loop();
  CHECK(!armed&&upperBlocked&&!lowerBlocked&&!leftBlocked&&!rightBlocked);
  CHECK(Serial.log.empty());clockMs=500;loop();
  CHECK(Serial.log.find("HT=1000")!=std::string::npos);
  inputLevel[18]=LOW;clockMs=519;loop();CHECK(upperBlocked);
  clockMs=520;loop();CHECK(!upperBlocked);
  clockMs=1000;loop();clockMs=1500;loop();clockMs=2000;loop();
  CHECK(Serial.log.find("HT=0000")!=std::string::npos);
  inputLevel[26]=HIGH;loop();CHECK(leftBlocked);
  clockMs=2500;loop();clockMs=3000;loop();clockMs=3500;loop();
  CHECK(Serial.log.find("HT=0010")!=std::string::npos);
  stopped();
}
void monitorRate(){
  arm();Serial.log.clear();statusAt=clockMs;monitorPage=0;
  uint32_t received=rxCount;
  for(int i=0;i<1000;i++)sample(0xffff,uint8_t(i%256),128,128,128,1);
  CHECK(std::count(Serial.log.begin(),Serial.log.end(),'\n')==2);
  CHECK(Serial.log.find("HT=")!=std::string::npos&&Serial.log.find("UART_PS3=")!=std::string::npos);
  CHECK(rxCount-received==1000);
  Serial.log.clear();Serial.capacity=0;
  for(int i=0;i<1000;i++)sample(0xffff,128,128,128,128,1);
  CHECK(Serial.log.empty()&&armed);
  Serial.capacity=63;loop();CHECK(std::count(Serial.log.begin(),Serial.log.end(),'\n')==1);
  size_t n=Serial.log.size();loop();CHECK(Serial.log.size()==n);
  Serial.log.clear();monitorPage=1;rxCount=UINT32_MAX;
  for(int &v:command)v=-255;
  clockMs=statusAt+500;serialTask();
  CHECK(Serial.log.find("RX=4294967295 M=-255,-255,-255,-255")!=std::string::npos);
  CHECK(Serial.log.size()<=63);
  Serial.log.clear();statusAt=UINT32_MAX-250;
  clockMs=statusAt+499;serialTask();CHECK(Serial.log.empty());
  clockMs=statusAt+500;serialTask();CHECK(std::count(Serial.log.begin(),Serial.log.end(),'\n')==1);
}
using AngleFrame=std::array<uint8_t,11>;
AngleFrame angleFrame(float yawDeg,uint8_t type=0x53){
  const int16_t raw=int16_t(std::lround(yawDeg*32768.0f/180.0f));
  const uint16_t bits=uint16_t(raw);
  AngleFrame f={0x55,type,0,0,0,0,uint8_t(bits),uint8_t(bits>>8),0,0,0};
  uint8_t sum=0;for(uint8_t i=0;i<10;i++)sum=uint8_t(sum+f[i]);f[10]=sum;return f;
}
void queueAngle(float yawDeg){
  const auto f=angleFrame(yawDeg);Serial3.rx.insert(Serial3.rx.end(),f.begin(),f.end());
}
void yawSample(float yawDeg,uint16_t keys=0xffff,uint8_t lx=128,uint8_t ly=128,
               uint8_t rx=128,uint8_t ry=128,uint32_t step=20){
  queueAngle(yawDeg);sample(keys,lx,ly,rx,ry,step);
}
void readyYaw(float yawDeg=0.0f){arm();yawSample(yawDeg);yawSample(yawDeg);CHECK(YawPid::ready);}
void hwtLink(){
  readyYaw(45.0f);CHECK(Hwt901b::valid&&YawPid::enabled&&YawPid::automatic);
  CHECK(std::fabs(Hwt901b::yawDeg-45.0f)<0.01f);
  auto bad=angleFrame(80.0f);bad[10]^=1;
  Serial3.rx.insert(Serial3.rx.end(),bad.begin(),bad.end());sample();
  CHECK(Hwt901b::checksumErrors==1&&std::fabs(Hwt901b::yawDeg-45.0f)<0.01f);
  for(int i=0;i<10;i++)sample();
  CHECK(!Hwt901b::valid&&!YawPid::ready&&!YawPid::enabled);
  yawSample(0);yawSample(0);CHECK(YawPid::ready&&!YawPid::enabled);
  sample(held(2));stopped();
}
void hwtMonitor(){
  readyYaw(45);Serial.log.clear();statusAt=clockMs;monitorPage=0;
  for(int i=0;i<75;i++)yawSample(45);
  CHECK(std::count(Serial.log.begin(),Serial.log.end(),'\n')==3);
  CHECK(Serial.log.find("HT=")!=std::string::npos);
  CHECK(Serial.log.find("UART_PS3=")!=std::string::npos);
  CHECK(Serial.log.find("IMU=1 TYPE=53 YAW=45.0")!=std::string::npos);
  Serial.log.clear();monitorPage=2;statusAt=clockMs-500;Serial.capacity=0;serialTask();
  CHECK(Serial.log.empty()&&monitorPage==2);
  Serial.capacity=63;serialTask();CHECK(Serial.log.size()<=63&&monitorPage==0);
}
void hwtControl(){
  arm();sample(held(15));CHECK(liftCommand==255);
  Serial3.rx.assign(4096,0x11);inputLevel[LIMIT_UP]=HIGH;
  loop();CHECK(Serial3.rx.size()==4096-64&&liftCommand==0);
  sample(held(5));CHECK(slideCommand==255);
  inputLevel[LIMIT_RIGHT]=HIGH;loop();CHECK(slideCommand==0);
  sample(held(11));CHECK(suctionOn);
  clockMs+=200;loop();stopped();CHECK(!uartOK);
}

void bridge(const char *path){
  std::ifstream in(path,std::ios::binary);CHECK(in.good());
  std::vector<uint8_t> wire((std::istreambuf_iterator<char>(in)),{});
  CHECK(wire.size()>=14&&wire.size()%14==0);
  bytes(wire.data(),wire.size());
  CHECK(uartOK&&pad.buttons==held(2)&&pad.lx==0&&pad.ly==255&&pad.rx==128&&pad.ry==192);
  stopped();
}
int main(int argc,char **argv){
  try{
    CHECK(argc>=2);setup();std::string name=argv[1];
    if(name=="boot")boot();
    else if(name=="protocol")protocol();
    else if(name=="arm_buttons")armButtons();
    else if(name=="sequences")sequences();
    else if(name=="interlocks")interlocks();
    else if(name=="pump_enable")pumpEnable();
    else if(name=="suction_leds")suctionLeds();
    else if(name=="relay_pulse")relayPulse();
    else if(name=="relay_safety")relaySafety();
    else if(name=="relay_rollover")relayRollover();
    else if(name=="r3")r3();
    else if(name=="limits")limits();
    else if(name=="manual_lift")manualLift();
    else if(name=="square_cross")automaticLift();
    else if(name=="dpad_safety")autoLiftSafety();
    else if(name=="normal_brake")normalBrake();
    else if(name=="movement")movement();
    else if(name=="soft_brake")softBrake();
    else if(name=="brake_interrupt")brakeInterrupt();
    else if(name=="brake_no_reverse")brakeNoReverse();
    else if(name=="relay_active_high")relayActiveHigh();
    else if(name=="four_directions")fourDirections();
    else if(name=="observed_mapping")observedMapping();
    else if(name=="motor_polarity")motorPolarity();
    else if(name=="m2_forward_polarity")m2ForwardPolarity();
    else if(name=="mecanum_rotation")mecanumRotation();
    else if(name=="max_speed_unused_buttons")speed();
    else if(name=="rollover")rollover();
    else if(name=="log_backpressure")logBackpressure();
    else if(name=="limit_diagnostics")limitDiagnostics();
    else if(name=="monitor_rate")monitorRate();
    else if(name=="hwt_link")hwtLink();
    else if(name=="hwt_monitor")hwtMonitor();
    else if(name=="hwt_control")hwtControl();
    else if(name=="bridge"){CHECK(argc==3);bridge(argv[2]);}
    else throw std::runtime_error("Unknown test");
    std::printf("PASS mega/%s\n",name.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}
}
