#define __AVR_ATmega2560__ 1
#include "Arduino.h"
#include <array>
#include <fstream>
#include <iterator>
#include "../MEGA_PS3_UART_V25/MEGA_PS3_UART_V25.ino"

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
  CHECK(!armed&&!braking&&!autoLiftDirection);
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
  CHECK(Serial.baud==115200&&Serial2.baud==115200);
  CHECK(!armed&&outputLevel[28]==LOW&&outputLevel[40]==LOW);
  CHECK(VALVE_ON==HIGH&&VALVE_OFF==LOW);
  for(int pin:{18,19,26,27})CHECK(pinModes[pin]==INPUT_PULLUP);
  CHECK(Wire.begun&&Wire.hz==100000&&Wire.timeoutUs==3000&&Wire.resetTimeout);
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
  arm();sample(held(11));CHECK(suctionOn);
  sample(held(10));CHECK(releaseOn&&!suctionOn&&outputLevel[40]==HIGH);
  uint32_t begin=releaseAt;
  for(int i=0;i<49;i++)sample(held(10));
  CHECK(clockMs-begin==980&&releaseOn);
  sample(held(10));CHECK(!releaseOn&&outputLevel[40]==LOW);
  for(int i=0;i<8;i++)sample(held(10));
  CHECK(!releaseOn&&releaseAt==begin);
  sample();sample(held(10));begin=releaseAt;
  for(int i=0;i<10;i++)sample(held(10));
  sample();sample(held(10));CHECK(releaseAt>begin);
  begin=releaseAt;
  for(int i=0;i<49;i++)sample(held(10));
  CHECK(releaseOn);sample(held(10));CHECK(!releaseOn&&clockMs-begin==1000);
  sample();sample(uint16_t(held(10)&held(11)));
  CHECK(releaseOn&&!suctionOn);
  for(int i=0;i<51;i++)sample(held(11));
  CHECK(!releaseOn&&!suctionOn);
  sample();sample(held(11));CHECK(suctionOn);
  sample(held(10),128,0);CHECK(releaseOn&&command[0]==-driveSpeed);
  clockMs+=200;loop();stopped();CHECK(!uartOK);
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
  // During startup and neutral: no suction and no illuminated indicators.
  arm();sample(); CHECK(!suctionOn && outputLevel[28]==LOW);
  CHECK(outputLevel[50]==LOW && outputLevel[52]==LOW);
  // R1 activates suction: two outputs exactly mirror the active pump command.
  sample(held(11));
  CHECK(suctionOn && outputLevel[28]==HIGH);
  CHECK(outputLevel[50]==HIGH && outputLevel[52]==HIGH);
  sample(); CHECK(suctionOn && outputLevel[50]==HIGH && outputLevel[52]==HIGH);
  // L1 release turns pump off immediately; both indicators go dark.
  sample(held(10));
  CHECK(releaseOn && !suctionOn && outputLevel[28]==LOW);
  CHECK(outputLevel[50]==LOW && outputLevel[52]==LOW);
  for(int i=0;i<51;i++)sample(held(11));
  CHECK(!releaseOn && !suctionOn);
  CHECK(outputLevel[50]==LOW && outputLevel[52]==LOW);
  sample();sample(held(11));CHECK(suctionOn && outputLevel[28]==HIGH);
  CHECK(outputLevel[50]==HIGH && outputLevel[52]==HIGH);
  // Emergency R3 immediately turns off both indicators.
  sample(held(2));stopped();
  // Re-arm and turn on, then test link invalidation and timeout.
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[50]==HIGH&&outputLevel[52]==HIGH);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  for(int i=0;i<8;i++)sample(held(11));
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[50]==HIGH&&outputLevel[52]==HIGH);
  clockMs+=200;loop();stopped();CHECK(!uartOK);
}
void pumpEnable(){
  CHECK(SUCTION==28&&pinModes[28]==OUTPUT&&outputLevel[28]==LOW);
  for(int i=0;i<8;i++)sample(held(11));
  CHECK(!armed&&outputLevel[28]==LOW);
  arm();sample(held(11));CHECK(suctionOn&&outputLevel[28]==HIGH);
  sample();CHECK(suctionOn&&outputLevel[28]==HIGH); // R1 latches after release.
  sample(uint16_t(held(10)&held(11)));
  CHECK(!suctionOn&&outputLevel[28]==LOW&&releaseOn);
  for(int i=0;i<51;i++)sample(held(11));
  CHECK(!releaseOn&&!suctionOn&&outputLevel[28]==LOW);
  sample();sample(held(11));CHECK(outputLevel[28]==HIGH);
  sample(held(2));stopped();
  arm();sample(held(11));sample();CHECK(outputLevel[28]==HIGH);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  for(int i=0;i<8;i++)sample(held(11));
  CHECK(!armed&&outputLevel[28]==LOW);
  arm();sample(held(11));sample();CHECK(outputLevel[28]==HIGH);
  clockMs+=199;loop();CHECK(outputLevel[28]==HIGH);
  clockMs++;loop();stopped();
  for(auto e:writes)if(e.pin==28)CHECK(e.kind!='A'); // EN is on/off, never analogWrite on D28.
}
void r3(){
  arm();sample(held(5));CHECK(autoLiftDirection&&liftCommand==255);
  sample(held(10),255,0);CHECK(releaseOn&&command[0]!=0);
  sample(held(2));stopped();
  for(int i=0;i<8;i++)sample(held(11),255);
  stopped();
  arm();sample(held(11));CHECK(suctionOn);
  sample(0xffff,128,0);sample();CHECK(braking);
  sample(held(2));stopped();
}
void limits(){
  arm();
  const int pins[]={18,19,26,27};
  const uint8_t keys[]={4,6,15,13};
  const uint8_t opposite[]={6,4,13,15};
  for(int i=0;i<4;i++){
    sample(held(keys[i]));
    CHECK((i<2?liftCommand:slideCommand)!=0);
    inputLevel[pins[i]]=HIGH;loop();
    CHECK((i<2?liftCommand:slideCommand)==0);
    sample(held(opposite[i]));CHECK((i<2?liftCommand:slideCommand)!=0);
    sample(held(keys[i]));CHECK((i<2?liftCommand:slideCommand)==0);
    inputLevel[pins[i]]=LOW;clockMs+=19;loop();
    CHECK((i<2?liftCommand:slideCommand)==0);
    clockMs++;loop();CHECK((i<2?liftCommand:slideCommand)!=0);
    inputLevel[pins[i]]=HIGH;loop();CHECK((i<2?liftCommand:slideCommand)==0);
    inputLevel[pins[i]]=LOW;clockMs+=10;loop();
    inputLevel[pins[i]]=HIGH;loop();
    inputLevel[pins[i]]=LOW;clockMs+=19;loop();CHECK((i<2?liftCommand:slideCommand)==0);
    clockMs++;loop();CHECK((i<2?liftCommand:slideCommand)!=0);
    sample();
  }
  inputLevel[18]=inputLevel[19]=inputLevel[26]=inputLevel[27]=HIGH;
  sample(uint16_t(held(4)&held(13)));CHECK(liftCommand==0&&slideCommand==0);
  sample(uint16_t(held(6)&held(15)));CHECK(liftCommand==0&&slideCommand==0);
  for(int pin:pins)inputLevel[pin]=LOW;
  sample();sample(held(5));CHECK(autoLiftDirection==1);
  inputLevel[18]=HIGH;loop();CHECK(!autoLiftDirection&&liftCommand==0);
  inputLevel[18]=LOW;sample(held(5));sample(held(5));CHECK(!autoLiftDirection);
  sample();sample(held(5));CHECK(autoLiftDirection==1);
  sample(held(6));CHECK(!autoLiftDirection&&liftCommand==-255);
}
void automaticLift(){
  arm();
  for(int dir:{1,-1}){
    const int target=dir>0?18:19,other=dir>0?19:18;
    const uint8_t key=dir>0?5:7,manualOpposite=dir>0?6:4;
    inputLevel[other]=HIGH;
    sample(held(key));CHECK(autoLiftDirection==dir&&liftCommand==dir*255);
    CHECK(pwmLevel[LIFT_EN]==255);
    CHECK(outputLevel[LIFT_A]==(dir>0?HIGH:LOW));
    CHECK(outputLevel[LIFT_B]==(dir<0?HIGH:LOW));
    sample();CHECK(autoLiftDirection==dir&&liftCommand==dir*255);
    sample(held(key));inputLevel[target]=HIGH;loop();
    CHECK(autoLiftDirection==0&&liftCommand==0&&pwmLevel[LIFT_EN]==0);
    inputLevel[target]=LOW;sample(held(key));sample(held(key));
    CHECK(autoLiftDirection==0&&liftCommand==0); // Holding cannot restart after contact bounce.
    sample();sample(held(key));CHECK(autoLiftDirection==dir);
    inputLevel[other]=LOW;sample();sample(held(manualOpposite));
    CHECK(autoLiftDirection==0&&liftCommand==-dir*255);
    sample();CHECK(liftCommand==0);
    inputLevel[18]=inputLevel[19]=HIGH;
    sample(held(key));CHECK(autoLiftDirection==0&&liftCommand==0);
    inputLevel[18]=inputLevel[19]=LOW;sample();sample();
  }
  sample(held(7));CHECK(autoLiftDirection==-1);
  sample(uint16_t(held(5)&held(7)));CHECK(autoLiftDirection==0&&liftCommand==0);
  sample(held(5));CHECK(autoLiftDirection==0&&liftCommand==0);
  sample();sample(held(5));CHECK(autoLiftDirection==1);
  sample(uint16_t(held(4)&held(6)));CHECK(autoLiftDirection==0&&liftCommand==0);
  sample();sample(held(8));sample();sample(held(7));
  CHECK(autoLiftDirection==-1&&liftCommand==-255&&pwmLevel[LIFT_EN]==255);
  for(int i=0;i<25;i++)sample();
  Serial.log.clear();monitorPage=0;statusAt=clockMs-500;serialTask();
  CHECK(Serial.log.find("N=-255")!=std::string::npos&&Serial.log.find("AUTO=-1")!=std::string::npos);
  CHECK(Serial.log.size()<=63);
}
void autoLiftSafety(){
  for(uint8_t key:{uint8_t(5),uint8_t(7)}){
    arm();sample(held(key));sample();CHECK(autoLiftDirection!=0);
    inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
    for(int i=0;i<8;i++)sample(held(key));
    stopped();
    arm();sample(held(key));CHECK(autoLiftDirection!=0);
    clockMs+=200;loop();stopped();
    for(int i=0;i<8;i++)sample(held(key));
    stopped();
    arm();sample(held(key));sample(held(2));stopped();
  }
}
void normalBrake(){
  arm();sample();
  for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==NORMAL_BRAKE_PWM&&command[i]==0);
  sample(held(2),128,128,128,128,1);stopped(); // Must upgrade immediately inside the 20ms refresh window.
  arm();sample();
  for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
  inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  arm();sample();clockMs+=200;loop();stopped();
}
void movement(){
  arm();
  sample(0xffff,128,0);wheels(-235,235,-235,235);
  CHECK(outputLevel[30]==LOW&&outputLevel[31]==HIGH);
  CHECK(outputLevel[32]==HIGH&&outputLevel[33]==LOW);
  CHECK(outputLevel[34]==LOW&&outputLevel[35]==HIGH);
  CHECK(outputLevel[37]==LOW&&outputLevel[36]==HIGH);
  sample(0xffff,128,255);wheels(235,-235,235,-235);
  sample(0xffff,255);wheels(-235,-235,-235,-235);
  sample(0xffff,0);wheels(235,235,235,235);
  sample(0xffff,255,0);wheels(-235,235,-235,235);
  sample(0xffff,0,0);wheels(-235,235,-235,235);
  sample(0xffff,255,255);wheels(235,-235,235,-235);
  sample(0xffff,0,255);wheels(235,235,235,235);
  sample(0xffff,255,0,255);wheels(-180,180,180,-180);
  sample();CHECK(braking&&brakeDuration==BRAKE_RAMP_MS);wheels(0,0,0,0);
  clockMs+=BRAKE_RAMP_MS-1;loop();CHECK(braking);
  clockMs++;loop();CHECK(!braking);wheels(0,0,0,0);
  sample(0xffff,158);wheels(0,0,0,0);
  sample(0xffff,255);sample(0xffff,158);wheels(-235,-235,-235,-235);
  sample();wheels(0,0,0,0);
  sample(held(2));arm();
  sample(0xffff,128,0);sample();CHECK(braking&&brakeDuration==BRAKE_RAMP_MS);
  wheels(0,0,0,0);
  clockMs+=BRAKE_RAMP_MS-1;loop();CHECK(braking);
  clockMs++;loop();CHECK(!braking);wheels(0,0,0,0);
  for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
  sample(0xffff,255,0);sample();wheels(0,0,0,0);
}
void relayActiveHigh(){
  arm();
  for(int i=0;i<100;i++){
    sample(i%2?held(11):0xffff,128,i%2?0:255);
    CHECK(!releaseOn&&outputLevel[40]==LOW);
  }
  sample(held(10));CHECK(releaseOn&&outputLevel[40]==HIGH);
  for(int i=0;i<49;i++)sample(held(10));
  CHECK(releaseOn&&outputLevel[40]==HIGH);
  sample(held(10));CHECK(!releaseOn&&outputLevel[40]==LOW);
  for(int i=0;i<20;i++)sample(held(10));
  CHECK(!releaseOn&&outputLevel[40]==LOW);
  sample();sample(held(10));CHECK(releaseOn&&outputLevel[40]==HIGH);
  sample(held(2));stopped();CHECK(outputLevel[40]==LOW);
}
void softBrake(){
  const uint8_t positions[][3]={{128,0,128},{128,255,128},{0,128,128},
                                {255,128,128},{128,128,0},{128,128,255}};
  for(int speed:{1,26,100,235,255})for(const auto &p:positions){
    sample(held(2));arm();driveSpeed=turnSpeed=speed;
    sample(0xffff,p[0],p[1],p[2]);
    for(int i=0;i<4;i++)CHECK(abs(command[i])==speed);
    sample();
    CHECK(braking && brakeDuration==BRAKE_RAMP_MS);
    for(int i=0;i<4;i++){
      CHECK(command[i]==0&&pwmLevel[EN[i]]==BRAKE_START_PWM);
      CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);
    }
    clockMs=brakeAt+BRAKE_RAMP_MS/2;loop();CHECK(braking);
    for(int i=0;i<4;i++)CHECK(pwmLevel[EN[i]]==110&&command[i]==0);
    clockMs=brakeAt+BRAKE_RAMP_MS;loop();CHECK(!braking);
    for(int i=0;i<4;i++){
      CHECK(command[i]==0&&pwmLevel[EN[i]]==NORMAL_BRAKE_PWM);
      CHECK(outputLevel[A[i]]==LOW&&outputLevel[B[i]]==LOW);
    }
  }
}
void brakeInterrupt(){
  arm();
  sample(0xffff,128,0);wheels(-235,235,-235,235);
  sample();CHECK(braking);
  sample(0xffff,128,255);
  CHECK(!braking);
  wheels(235,-235,235,-235);
  sample();CHECK(braking);
  sample(held(2));stopped(); // R3 overrides ramp to EN 255 immediately.
}
void brakeNoReverse(){
  arm();sample(0xffff,128,0);wheels(-235,235,-235,235);
  const auto start=writes.size();
  sample();CHECK(braking);
  for(size_t i=start;i<writes.size();i++){
    const auto &w=writes[i];
    for(uint8_t motor=0;motor<4;motor++){
      if((w.pin==A[motor]||w.pin==B[motor])&&w.kind=='D')CHECK(w.value==LOW);
    }
  }
  for(int i=0;i<4;i++)CHECK(command[i]==0);
}
void fourDirections(){
  arm();
  struct Case { uint8_t x,y; int expected[4]; };
  const Case cases[]={
    {128,0,{-235,235,-235,235}}, {128,255,{235,-235,235,-235}},
    {255,128,{-235,-235,-235,-235}}, {0,128,{235,235,235,235}},
    {228,28,{-235,235,-235,235}}, {28,28,{-235,235,-235,235}},
    {228,228,{235,-235,235,-235}}, {28,228,{235,-235,235,-235}},
    {255,28,{-235,-235,-235,-235}}, {255,228,{-235,-235,-235,-235}},
    {0,28,{235,235,235,235}}, {0,228,{235,235,235,235}},
    {228,0,{-235,235,-235,235}}, {28,0,{-235,235,-235,235}},
    {228,255,{235,-235,235,-235}}, {28,255,{235,-235,235,-235}}
  };
  for(const auto &c:cases){
    sample(0xffff,c.x,c.y);
    wheels(c.expected[0],c.expected[1],c.expected[2],c.expected[3]);
    for(int i=0;i<4;i++){
      CHECK(pwmLevel[EN[i]]==235);
      const bool aHigh=(i==3)?c.expected[i]<0:c.expected[i]>0; // M1/M2/M3 direct; M4 reverse.
      CHECK(outputLevel[A[i]]==(aHigh?HIGH:LOW));
      CHECK(outputLevel[B[i]]==(aHigh?LOW:HIGH));
    }
  }
  sample(0xffff,228,28,255);wheels(-180,180,180,-180);
  const uint8_t forwardA[4]={30,32,34,37},forwardB[4]={31,33,35,36};
  for(int i=0;i<4;i++){
    CHECK(outputLevel[forwardA[i]]==(i>=1?HIGH:LOW));
    CHECK(outputLevel[forwardB[i]]==(i>=1?LOW:HIGH));
    CHECK(pwmLevel[EN[i]]==180);
  }
  sample(0xffff,28,228,0);wheels(180,-180,-180,180);
  for(int i=0;i<4;i++){
    CHECK(outputLevel[forwardA[i]]==(i>=1?LOW:HIGH));
    CHECK(outputLevel[forwardB[i]]==(i>=1?HIGH:LOW));
    CHECK(pwmLevel[EN[i]]==180);
  }
  sample(held(2));stopped();arm();
  sample(0xffff,228,28);clockMs+=200;loop();stopped();
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
  // Electrical pins for forward motion. M2 changed ONLY, others match V25.
  CHECK(REVERSE[0]==false && REVERSE[1]==false &&
        REVERSE[2]==false && REVERSE[3]==true);
  arm();sample(0xffff,128,0);wheels(-235,235,-235,235);
  const uint8_t expectedA[4]={LOW,HIGH,LOW,LOW};
  const uint8_t expectedB[4]={HIGH,LOW,HIGH,HIGH};
  for(int i=0;i<4;i++){
    CHECK(outputLevel[A[i]]==expectedA[i]);
    CHECK(outputLevel[B[i]]==expectedB[i]);
    CHECK(pwmLevel[EN[i]]==235);
  }
  sample(held(2));stopped();
}
void observedMapping(){
  struct Observation { int pattern[4]; int actualX,actualY; };
  const Observation observed[]={
    {{1,1,1,1},-1,0},      // V18 forward moved left.
    {{-1,-1,-1,-1},1,0},   // V18 reverse moved right.
    {{1,-1,1,-1},0,-1},    // V18 right moved backward.
    {{-1,1,-1,1},0,1}      // V18 left moved forward.
  };
  struct Request { uint8_t x,y; int expectedX,expectedY; };
  const Request requests[]={
    {128,0,0,1},{128,255,0,-1},{255,128,1,0},{0,128,-1,0}
  };
  arm();
  for(int speed:{60,150,235}){
    driveSpeed=speed;
    for(const auto &request:requests){
      sample(0xffff,request.x,request.y);
      int matches=0;
      for(const auto &observation:observed){
        bool match=true;
        for(int i=0;i<4;i++)if(command[i]!=observation.pattern[i]*speed)match=false;
        if(match){
          CHECK(observation.actualX==request.expectedX&&observation.actualY==request.expectedY);
          matches++;
        }
      }
      CHECK(matches==1);
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
  arm();driveSpeed=turnSpeed=slideSpeed=100;
  const int expected[]={120,144,115,138};
  for(int n:expected){
    sample(held(9));CHECK(driveSpeed==n&&turnSpeed==n&&slideSpeed==n);
    sample(held(9));CHECK(driveSpeed==n);sample();
  }
  CHECK(r2Step==0);sample(held(8));CHECK(driveSpeed==110);
  sample(held(8));CHECK(driveSpeed==110);sample();
  sample(uint16_t(held(8)&held(9)));CHECK(driveSpeed==110&&r2Step==0);
  sample();driveSpeed=255;sample(held(9));CHECK(driveSpeed==255);
  sample();driveSpeed=1;sample(held(8));CHECK(driveSpeed==1);
}
void rollover(){
  clockMs=UINT32_MAX-80;
  arm();CHECK(armed);
  sample(held(10));uint32_t t=releaseAt;
  for(int i=0;i<49;i++)sample(held(10));
  CHECK(releaseOn);sample(held(10));CHECK(!releaseOn&&uint32_t(clockMs-t)==1000);
  clockMs+=200;loop();stopped();
  clockMs=UINT32_MAX-500;invalidateLink();arm();sample(held(10));t=releaseAt;
  for(int i=0;i<50;i++)sample(held(10));
  CHECK(uint32_t(clockMs-t)==1000&&!releaseOn);
}
void logBackpressure(){
  arm();Serial.capacity=0;sample(held(11));CHECK(suctionOn);
  sample(held(10),128,0);CHECK(releaseOn);
  inputLevel[18]=HIGH;inputLevel[27]=HIGH;
  sample(uint16_t(held(4)&held(13)));CHECK(liftCommand==0&&slideCommand==0);
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
void mpuStep(){
  const uint32_t before=clockMs;
  const unsigned sent=Wire.endCalls,read=Wire.readCalls;
  Mpu6050::task();
  CHECK(clockMs==before); // No delay() or hidden software waits with a zero-delay mock.
  CHECK(Wire.endCalls-sent<=1&&Wire.readCalls-read<=1);
}
void sensorReady(){
  Wire=FakeWire{};Wire.present=true;Mpu6050::begin();
  for(int i=0;i<400&&!Mpu6050::valid;i++){clockMs++;mpuStep();}
  CHECK(Mpu6050::stage==Mpu6050::READY&&Mpu6050::valid);
}
void sensorWord(uint8_t reg,int16_t value){
  const uint16_t bits=uint16_t(value);
  Wire.registers[reg]=uint8_t(bits>>8);Wire.registers[reg+1]=uint8_t(bits);
}
void sensorRead(){
  clockMs+=20;Mpu6050::task();
}
void mpuConfig(){
  Wire=FakeWire{};Wire.present=true;Mpu6050::begin();
  clockMs+=99;mpuStep();CHECK(Wire.endCalls==0);
  clockMs++;mpuStep();CHECK(Mpu6050::stage==1);
  mpuStep();CHECK(Mpu6050::stage==2);
  const unsigned count=Wire.endCalls;
  clockMs+=99;mpuStep();CHECK(Wire.endCalls==count);
  clockMs++;mpuStep();CHECK(Mpu6050::stage==3);
  clockMs+=100;
  for(int i=0;i<7;i++)mpuStep();
  CHECK(Mpu6050::stage==Mpu6050::READY&&!Mpu6050::valid);
  clockMs+=19;mpuStep();CHECK(!Mpu6050::valid);
  clockMs++;mpuStep();CHECK(Mpu6050::valid);
  CHECK(Wire.registers[0x6B]==1&&Wire.registers[0x6C]==0);
  CHECK(Wire.registers[0x19]==9&&Wire.registers[0x1A]==3);
  CHECK(Wire.registers[0x1B]==0&&Wire.registers[0x1C]==0);
}
void mpuAxes(){
  uint8_t pair[2];
  for(int32_t value=-32768;value<=32767;value++){
    const uint16_t bits=uint16_t(value);pair[0]=uint8_t(bits>>8);pair[1]=uint8_t(bits);
    CHECK(Mpu6050::signedWord(pair)==value);
  }
  sensorReady();
  sensorWord(0x3B,16384);sensorWord(0x3D,-8192);sensorWord(0x3F,4096);
  sensorWord(0x41,32767);
  sensorWord(0x43,131);sensorWord(0x45,-262);sensorWord(0x47,1310);
  sensorRead();CHECK(Wire.lastReadReg==0x3B&&Wire.lastReadCount==14);
  const int expected[]={16384,-8192,4096,131,-262,1310};
  for(int i=0;i<6;i++)CHECK(Mpu6050::raw[i]==expected[i]);
  char line[64];Mpu6050::formatLine(line,sizeof(line));
  CHECK(std::string(line)=="MPU=1 A[g]=1.000,-0.500,0.250 G[dps]=1.0,-2.0,10.0\r\n");
  for(int16_t value:{int16_t(-32768),int16_t(32767)}){
    for(auto &raw:Mpu6050::raw)raw=value;
    Mpu6050::formatLine(line,sizeof(line));CHECK(strlen(line)<=63);
    CHECK(std::string(line).find("\r\n")!=std::string::npos);
  }
  clockMs=Mpu6050::sampleAt+200;Mpu6050::formatLine(line,sizeof(line));
  CHECK(std::string(line).find("DU_LIEU_CU")!=std::string::npos);
  CHECK(std::string(line).find("A[g]")==std::string::npos);
}
void mpuErrors(){
  char line[64];
  Wire=FakeWire{};Mpu6050::begin();clockMs+=100;Mpu6050::task();
  CHECK(Mpu6050::error==Mpu6050::I2C&&!Mpu6050::valid);
  const unsigned attempts=Wire.endCalls;
  clockMs+=999;Mpu6050::task();CHECK(Wire.endCalls==attempts);
  clockMs++;Mpu6050::task();CHECK(Wire.endCalls==attempts+1);
  for(int fault:{0,1,2,3,4}){
    sensorReady();
    if(fault==0)Wire.shortRead=13;
    if(fault==1)Wire.missingByte=true;
    if(fault==2)Wire.timeoutOnEnd=true;
    if(fault==3)Wire.timeoutOnRead=true;
    if(fault==4)Wire.present=false;
    sensorRead();CHECK(!Mpu6050::valid&&Mpu6050::stage==0);
    Mpu6050::formatLine(line,sizeof(line));
    CHECK(std::string(line).find("MPU=0 LOI=")==0&&std::string(line).find("A[g]")==std::string::npos);
  }
  Wire=FakeWire{};Wire.present=true;Wire.registers[0x75]=0x70;
  Mpu6050::begin();clockMs+=100;Mpu6050::task();CHECK(Mpu6050::error==Mpu6050::ID);
  for(int reg:{0x6B,0x6C,0x1A,0x19,0x1B,0x1C}){
    Wire=FakeWire{};Wire.present=true;Wire.nackRegister=reg;Mpu6050::begin();
    for(int i=0;i<500;i++){clockMs++;mpuStep();}
    CHECK(!Mpu6050::valid&&Mpu6050::error==Mpu6050::I2C);
  }
  Wire=FakeWire{};Wire.present=true;Wire.ignoreWriteRegister=0x19;Mpu6050::begin();
  for(int i=0;i<500;i++){clockMs++;mpuStep();}
  CHECK(!Mpu6050::valid&&Mpu6050::error==Mpu6050::CONFIG);
  Wire.ignoreWriteRegister=-1;
  for(int i=0;i<1500&&!Mpu6050::valid;i++){clockMs++;mpuStep();}
  CHECK(Mpu6050::valid&&Mpu6050::error==Mpu6050::NONE);
}
void mpuControl(){
  // Robot continues processing new reports throughout sensor reset/wake waits.
  Wire=FakeWire{};Wire.present=true;Mpu6050::begin();
  arm();const uint32_t firstRx=rxCount;
  for(int i=0;i<30;i++)sample(0xffff,128,0);
  CHECK(Mpu6050::valid&&armed&&rxCount==firstRx+30);
  wheels(-235,235,-235,235);
  Wire.timeoutOnEnd=true;Wire.endDelayMs=3;sample(0xffff,128,0);
  CHECK(armed&&!Mpu6050::valid);wheels(-235,235,-235,235);
  sample(held(2));stopped();
  sensorReady();arm();sample(0xffff,128,0);
  Mpu6050::waitMs=0;clockMs=rxAt+199;Wire.timeoutOnRead=true;Wire.readDelayMs=3;
  loop();stopped();CHECK(!uartOK); // Timeout crossed inside I2C, stop before this loop returns.
  sensorReady();arm();sample(held(4));CHECK(liftCommand==255);
  Wire.onRead=[](){inputLevel[18]=HIGH;};
  clockMs+=20;loop();CHECK(liftCommand==0&&upperBlocked);
  Wire.onRead={};inputLevel[18]=LOW;
  sensorReady();arm();sample(held(10));CHECK(releaseOn);
  while(clockMs-releaseAt<980)sample(held(10));
  Mpu6050::waitMs=0;clockMs=releaseAt+999;Wire.readDelayMs=3;
  loop();CHECK(!releaseOn&&outputLevel[40]==VALVE_OFF);
}
void mpuMonitor(){
  sensorReady();arm();Serial.log.clear();monitorPage=0;statusAt=clockMs;
  for(int i=0;i<1500;i++)sample(0xffff,128,128,128,128,1);
  CHECK(std::count(Serial.log.begin(),Serial.log.end(),'\n')==3);
  CHECK(Serial.log.find("HT=")!=std::string::npos&&Serial.log.find("UART_PS3=")!=std::string::npos);
  CHECK(Serial.log.find("MPU=1 A[g]=")!=std::string::npos);
  Serial.log.clear();monitorPage=2;statusAt=clockMs-500;Serial.capacity=0;
  const uint32_t previous=statusAt;serialTask();CHECK(Serial.log.empty()&&monitorPage==2&&statusAt==previous);
  Serial.capacity=63;for(auto &v:Mpu6050::raw)v=-32768;
  Mpu6050::sampleAt=clockMs;serialTask();
  CHECK(Serial.log.size()<=63&&std::count(Serial.log.begin(),Serial.log.end(),'\n')==1);
  CHECK(monitorPage==0);
  Serial.log.clear();Mpu6050::valid=false;Mpu6050::error=Mpu6050::TIMEOUT;
  monitorPage=2;statusAt=clockMs-500;serialTask();
  CHECK(Serial.log=="MPU=0 LOI=TIMEOUT A/G=NA\r\n");
}
void mpuRollover(){
  sensorReady();Mpu6050::stepAt=UINT32_MAX-10;clockMs=Mpu6050::stepAt+19;
  const unsigned requests=Wire.readCalls;
  CHECK(!Mpu6050::task()&&Wire.readCalls==requests);
  clockMs++;CHECK(Mpu6050::task()&&Mpu6050::valid);
  char line[64];Mpu6050::formatLine(line,sizeof(line));CHECK(std::string(line).find("MPU=1")==0);
  clockMs=Mpu6050::sampleAt+200;Mpu6050::formatLine(line,sizeof(line));
  CHECK(std::string(line).find("DU_LIEU_CU")!=std::string::npos);
  clockMs=UINT32_MAX-500;Mpu6050::error=Mpu6050::I2C;Mpu6050::failed();
  const unsigned attempts=Wire.endCalls;
  clockMs+=999;Mpu6050::task();CHECK(Wire.endCalls==attempts);
  clockMs++;Mpu6050::task();CHECK(Wire.endCalls==attempts+1);
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
    else if(name=="sequences")sequences();
    else if(name=="interlocks")interlocks();
    else if(name=="pump_enable")pumpEnable();
    else if(name=="suction_leds")suctionLeds();
    else if(name=="r3")r3();
    else if(name=="limits")limits();
    else if(name=="auto_lift")automaticLift();
    else if(name=="auto_lift_safety")autoLiftSafety();
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
    else if(name=="speed")speed();
    else if(name=="rollover")rollover();
    else if(name=="log_backpressure")logBackpressure();
    else if(name=="limit_diagnostics")limitDiagnostics();
    else if(name=="monitor_rate")monitorRate();
    else if(name=="mpu_config")mpuConfig();
    else if(name=="mpu_axes")mpuAxes();
    else if(name=="mpu_errors")mpuErrors();
    else if(name=="mpu_control")mpuControl();
    else if(name=="mpu_monitor")mpuMonitor();
    else if(name=="mpu_rollover")mpuRollover();
    else if(name=="bridge"){CHECK(argc==3);bridge(argv[2]);}
    else throw std::runtime_error("Unknown test");
    std::printf("PASS mega/%s\n",name.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}
}
