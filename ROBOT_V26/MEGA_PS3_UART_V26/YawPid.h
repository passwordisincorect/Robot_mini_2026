#pragma once
#include <Arduino.h>
#include <math.h>
#include "Hwt901b.h"

// Positive yaw is counterclockwise; positive mixer correction turns clockwise.
// Verify this sign on the robot before floor testing.
namespace YawPid {
const int8_t YAW_SIGN=1;
const float KP=1.8f, KI=0.25f, KD=0.25f;
const float HEADING_DEADBAND_DEG=0.25f, RATE_FILTER_TAU_MS=60.0f;
const float INTEGRAL_SHARE=0.30f, AUTO_SPIN_GUARD_DPS=90.0f;
const int MAX_PWM=25, AUTO_MAX_PWM=10, SLEW_PWM=4;
bool enabled=false,automatic=false,ready=false,targetSet=false;
bool seenSample=false,seenCalc=false;
float targetYawDeg=0.0f,headingErrorDeg=0.0f,filteredDps=0.0f;
float previousYawDeg=0.0f,integral=0.0f;
uint32_t lastSampleId=0,lastSampleAt=0,lastCalcId=0,lastCalcAt=0;
int appliedPwm=0;

float wrapDeg(float angle){
  if(angle>=180.0f)angle-=360.0f;
  if(angle< -180.0f)angle+=360.0f;
  return angle;
}
float clampFloat(float value,float minimum,float maximum){
  return value<minimum?minimum:(value>maximum?maximum:value);
}
bool healthy(uint32_t now){return Hwt901b::healthy(now);}
void resetControl(){
  targetSet=false;seenCalc=false;
  headingErrorDeg=0.0f;integral=0.0f;appliedPwm=0;
}
void disable(){enabled=false;automatic=false;resetControl();}
void reset(){
  disable();ready=false;seenSample=false;
  filteredDps=0.0f;lastSampleId=0;lastSampleAt=0;
}
void observe(){
  if(!healthy(millis())){reset();return;}
  const uint32_t id=Hwt901b::sampleId,at=Hwt901b::sampleAt;
  if(seenSample&&id==lastSampleId)return;
  const uint32_t interval=uint32_t(at-lastSampleAt);
  if(!seenSample||interval>=Hwt901b::FRESH_MS){
    reset();seenSample=true;lastSampleId=id;lastSampleAt=at;
    previousYawDeg=Hwt901b::yawDeg;return;
  }
  lastSampleId=id;
  // Queued frames may share a millisecond; wait for elapsed time before deriving rate.
  if(interval==0)return;
  const float rate=float(YAW_SIGN)*wrapDeg(Hwt901b::yawDeg-previousYawDeg)*1000.0f/float(interval);
  const float alpha=float(interval)/(RATE_FILTER_TAU_MS+float(interval));
  filteredDps+=alpha*(rate-filteredDps);
  previousYawDeg=Hwt901b::yawDeg;lastSampleAt=at;ready=true;
}
bool enableIfSafe(bool motionNeutral,bool bootAuto=false){
  if(!motionNeutral||!ready||!healthy(millis())||fabsf(filteredDps)>3.0f)return false;
  resetControl();enabled=true;automatic=bootAuto;return true;
}
int correction(int drivePwm){
  if(!enabled)return 0;
  if(!ready||!healthy(millis())){reset();return 0;}
  if(automatic&&fabsf(filteredDps)>AUTO_SPIN_GUARD_DPS){disable();return 0;}
  const uint32_t id=Hwt901b::sampleId,at=Hwt901b::sampleAt;
  if(!targetSet){
    targetYawDeg=Hwt901b::yawDeg;targetSet=true;seenCalc=true;
    lastCalcId=id;lastCalcAt=at;return 0;
  }
  if(seenCalc&&id==lastCalcId)return appliedPwm;
  const uint32_t elapsed=uint32_t(at-lastCalcAt);
  if(elapsed==0)return appliedPwm;
  if(elapsed>=Hwt901b::FRESH_MS){reset();return 0;}
  lastCalcId=id;lastCalcAt=at;
  const float dt=float(elapsed)*0.001f;
  headingErrorDeg=float(YAW_SIGN)*wrapDeg(Hwt901b::yawDeg-targetYawDeg);
  const float error=fabsf(headingErrorDeg)<HEADING_DEADBAND_DEG?0.0f:headingErrorDeg;
  int bound=automatic?AUTO_MAX_PWM:MAX_PWM;
  if(abs(drivePwm)/5<bound)bound=abs(drivePwm)/5;
  if(bound<=0){resetControl();return 0;}
  const float iMax=INTEGRAL_SHARE*float(bound)/KI;
  if(error==0.0f)integral*=1.0f/(1.0f+dt/1.2f);
  else{
    const float next=clampFloat(integral+error*dt,-iMax,iMax);
    const float candidate=KP*error+KI*next+KD*filteredDps;
    if(!((candidate>bound&&error>0.0f)||(candidate< -bound&&error<0.0f)))integral=next;
  }
  integral=clampFloat(integral,-iMax,iMax);
  const float wanted=clampFloat(KP*error+KI*integral+KD*filteredDps,-float(bound),float(bound));
  int target=int(wanted>=0.0f?wanted+0.5f:wanted-0.5f);
  if(target>appliedPwm+SLEW_PWM)target=appliedPwm+SLEW_PWM;
  if(target<appliedPwm-SLEW_PWM)target=appliedPwm-SLEW_PWM;
  appliedPwm=target;return appliedPwm;
}
} // namespace YawPid
