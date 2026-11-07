#include "stubs/Arduino.h"
#include "../MEGA_PS3_UART_V26_3/Hwt901b.h"
#include <array>
#include <cmath>

using Frame=std::array<uint8_t,11>;
void putWord(Frame &f,uint8_t at,int16_t value){
  const uint16_t bits=uint16_t(value);f[at]=uint8_t(bits);f[at+1]=uint8_t(bits>>8);
}
Frame frame(uint8_t type=0x53,int16_t roll=0,int16_t pitch=0,int16_t yaw=0){
  Frame f={0x55,type};putWord(f,2,roll);putWord(f,4,pitch);putWord(f,6,yaw);putWord(f,8,0);
  uint8_t sum=0;for(uint8_t i=0;i<10;i++)sum=uint8_t(sum+f[i]);f[10]=sum;return f;
}
bool near(float a,float b){return std::fabs(a-b)<0.002f;}
void feed(const Frame &f){for(uint8_t b:f)Hwt901b::ingest(b);}
void queue(const Frame &f){for(uint8_t b:f)Serial3.rx.push_back(b);}
void reset(){Serial3.rx.clear();clockMs=0;Hwt901b::begin();CHECK(Serial3.baud==Hwt901b::BAUD);}
void angles(){
  reset();CHECK(Hwt901b::FRAME_SIZE==11&&Hwt901b::FRESH_MS==200);
  feed(frame(0x53,-16384,4096,8192));
  CHECK(Hwt901b::valid&&near(Hwt901b::rollDeg,-90)&&near(Hwt901b::pitchDeg,22.5f));
  CHECK(near(Hwt901b::yawDeg,45)&&Hwt901b::sampleId==1&&Hwt901b::sampleAt==0);
  feed(frame(0x53,-32768,32767,-32768));
  CHECK(Hwt901b::sampleId==2&&Hwt901b::sampleAt==0&&near(Hwt901b::yawDeg,-180));
  clockMs=10;feed(frame(0x53,32767,0,32767));
  CHECK(near(Hwt901b::rollDeg,179.9945f)&&near(Hwt901b::pitchDeg,0));
  CHECK(near(Hwt901b::yawDeg,179.9945f)&&Hwt901b::sampleId==3&&Hwt901b::sampleAt==10);
}
void typesAndChecksum(){
  reset();auto f=frame(0x52,1,2,3);feed(f);
  CHECK(Hwt901b::received==1&&!Hwt901b::valid&&Hwt901b::sampleId==0);
  f=frame(0x5f,4,5,6);feed(f);CHECK(Hwt901b::received==2&&!Hwt901b::valid);
  f=frame(0x53,0,0,0x1234);f[10]^=0x80;feed(f);
  CHECK(!Hwt901b::valid&&Hwt901b::checksumErrors==1&&Hwt901b::badFrames==1);
  f=frame(0x53,0,0,100);feed(f);const uint32_t at=Hwt901b::sampleAt;
  clockMs=at+199;feed(frame(0x52));CHECK(Hwt901b::healthy(clockMs));
  clockMs=at+200;Hwt901b::task();CHECK(!Hwt901b::valid&&!Hwt901b::healthy(clockMs));
}
void fragmentedAndNoise(){
  reset();const auto f=frame(0x53,1,-2,1234);
  const uint8_t noise[]={0x11,0x55,0x00,0x42};
  for(uint8_t b:noise)Hwt901b::ingest(b);
  for(uint8_t i=0;i<5;i++)Hwt901b::ingest(f[i]);
  clockMs+=29;for(uint8_t i=5;i<11;i++)Hwt901b::ingest(f[i]);
  CHECK(Hwt901b::valid&&Hwt901b::sampleId==1);
  reset();for(uint8_t i=0;i<5;i++)Hwt901b::ingest(f[i]);
  clockMs+=30;Hwt901b::task();CHECK(Hwt901b::used==0&&!Hwt901b::valid);
  for(uint8_t i=5;i<11;i++){Hwt901b::ingest(f[i]);}
  CHECK(!Hwt901b::valid);
  feed(f);CHECK(Hwt901b::valid&&Hwt901b::sampleId==1);
}
void recovery(){
  reset();auto damaged=frame(0x53,0,0,77);auto good=frame(0x53,0,0,8192);
  std::vector<uint8_t> lost(damaged.begin(),damaged.end());lost.erase(lost.begin()+4);
  for(uint8_t b:lost){Hwt901b::ingest(b);}
  feed(good);
  CHECK(Hwt901b::valid&&near(Hwt901b::yawDeg,45));
  reset();std::vector<uint8_t> inserted(damaged.begin(),damaged.end());
  inserted.insert(inserted.begin()+5,0x33);
  for(uint8_t b:inserted){Hwt901b::ingest(b);}
  feed(good);
  CHECK(Hwt901b::valid&&near(Hwt901b::yawDeg,45));
  reset();const auto payloadHeader=frame(0x53,0x5555,0x2255,0x3355);
  feed(payloadHeader);CHECK(Hwt901b::valid&&Hwt901b::sampleId==1);
}
void taskBudgetAndFrameStop(){
  reset();auto other=frame(0x51),one=frame(0x53,0,0,8192),two=frame(0x53,0,0,-8192);
  for(int i=0;i<5;i++){queue(other);}
  queue(one);Hwt901b::task();
  CHECK(!Hwt901b::valid&&Serial3.rx.size()==2);
  Hwt901b::task();CHECK(Hwt901b::valid&&Hwt901b::sampleId==1);
  queue(two);Hwt901b::task();CHECK(Hwt901b::sampleId==2&&near(Hwt901b::yawDeg,-45));
  reset();for(int i=0;i<10;i++){queue(other);}
  Hwt901b::task();CHECK(Serial3.rx.size()==46&&Hwt901b::received==5);
}
void rolloverAndFormat(){
  reset();clockMs=UINT32_MAX-100;feed(frame(0x53,0,0,32767));
  CHECK(Hwt901b::healthy(clockMs));clockMs+=199;CHECK(Hwt901b::healthy(clockMs));
  char line[64];Hwt901b::formatLine(line,sizeof(line));CHECK(strlen(line)<=63&&strstr(line,"IMU=1 TYPE=53 YAW=")!=nullptr);
  clockMs++;Hwt901b::task();CHECK(!Hwt901b::valid);
  Hwt901b::formatLine(line,sizeof(line));CHECK(strlen(line)<=63&&strstr(line,"YAW=STALE")!=nullptr);
  reset();clockMs=UINT32_MAX-10;const auto f=frame();for(uint8_t i=0;i<3;i++)Hwt901b::ingest(f[i]);
  clockMs+=29;Hwt901b::task();CHECK(Hwt901b::used==3);
  clockMs++;Hwt901b::task();CHECK(Hwt901b::used==0);
}
void run(const char *name){
  if(strcmp(name,"angles")==0)angles();
  else if(strcmp(name,"types_checksum")==0)typesAndChecksum();
  else if(strcmp(name,"fragmented_noise")==0)fragmentedAndNoise();
  else if(strcmp(name,"recovery")==0)recovery();
  else if(strcmp(name,"task_budget")==0)taskBudgetAndFrameStop();
  else if(strcmp(name,"rollover_format")==0)rolloverAndFormat();
  else throw std::runtime_error("Unknown HWT test");
}
int main(int argc,char **argv){
  try{CHECK(argc==2);run(argv[1]);std::printf("PASS hwt/%s\n",argv[1]);return 0;}
  catch(const std::exception &e){std::fprintf(stderr,"FAIL hwt %s\n",e.what());return 1;}
}
