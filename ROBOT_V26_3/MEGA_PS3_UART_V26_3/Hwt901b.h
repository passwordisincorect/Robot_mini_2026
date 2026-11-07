#pragma once
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace Hwt901b {
const uint32_t BAUD=115200, FRESH_MS=200, INTERBYTE_MS=30;
const uint8_t FRAME_SIZE=11;
float rollDeg=0.0f,pitchDeg=0.0f,yawDeg=0.0f;
bool valid=false;
uint32_t sampleAt=0,sampleId=0,received=0,checksumErrors=0,badFrames=0;
uint8_t used=0,frame[FRAME_SIZE]={};
uint32_t lastByteAt=0;

bool healthy(uint32_t now){return valid&&uint32_t(now-sampleAt)<FRESH_MS;}
bool witType(uint8_t type){return type>=0x50&&type<=0x5f;}
int16_t signedWord(uint8_t low,uint8_t high){
  const uint16_t value=uint16_t(low)|(uint16_t(high)<<8);
  return int16_t(value<0x8000?int32_t(value):int32_t(value)-65536L);
}
void discardPrefix(uint8_t count){
  used=uint8_t(used-count);
  if(used)memmove(frame,frame+count,used);
}
void resync(){
  for(uint8_t i=1;i<used;i++){
    if(frame[i]!=0x55)continue;
    if(i+1==used){frame[0]=0x55;used=1;return;}
    if(witType(frame[i+1])){discardPrefix(i);return;}
  }
  if(used&&frame[used-1]==0x55){frame[0]=0x55;used=1;}
  else used=0;
}
bool ingest(uint8_t ch){
  lastByteAt=millis();
  if(!used){if(ch==0x55){frame[0]=ch;used=1;}return false;}
  frame[used++]=ch;
  if(used==2&&!witType(frame[1])){badFrames++;resync();return false;}
  if(used<FRAME_SIZE)return false;
  uint8_t sum=0;for(uint8_t i=0;i<10;i++)sum=uint8_t(sum+frame[i]);
  if(sum!=frame[10]){checksumErrors++;badFrames++;resync();return false;}
  received++;
  const uint8_t type=frame[1];
  if(type==0x53){
    rollDeg=float(signedWord(frame[2],frame[3]))*(180.0f/32768.0f);
    pitchDeg=float(signedWord(frame[4],frame[5]))*(180.0f/32768.0f);
    yawDeg=float(signedWord(frame[6],frame[7]))*(180.0f/32768.0f);
    sampleAt=millis();sampleId++;valid=true;
    used=0;return true;
  }
  used=0;return false;
}
void task(){
  const uint32_t now=millis();
  if(used&&uint32_t(now-lastByteAt)>=INTERBYTE_MS)used=0;
  if(valid&&!healthy(now))valid=false;
  uint8_t budget=64;
  while(budget--&&Serial3.available()){
    if(ingest(uint8_t(Serial3.read())))break;
  }
}
void begin(){
  Serial3.begin(BAUD,SERIAL_8N1);
  rollDeg=pitchDeg=yawDeg=0.0f;valid=false;sampleAt=sampleId=received=0;
  checksumErrors=badFrames=0;used=0;lastByteAt=0;memset(frame,0,sizeof(frame));
}
void formatLine(char *out,size_t size){
  const uint32_t now=millis();
  if(!healthy(now)){
    if(sampleId)snprintf(out,size,"IMU=0 TYPE=53 YAW=STALE AGE=%lu\r\n",
                         (unsigned long)uint32_t(now-sampleAt));
    else snprintf(out,size,"IMU=0 TYPE=-- YAW=STALE AGE=NA\r\n");
    return;
  }
  char angle[8];dtostrf(yawDeg,1,1,angle);
  snprintf(out,size,"IMU=1 TYPE=53 YAW=%s AGE=%lu\r\n",angle,
           (unsigned long)uint32_t(now-sampleAt));
}
} // namespace Hwt901b
