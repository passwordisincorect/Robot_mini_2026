// Regression tests for yaw-angle hold and the physical wheel mixer.
#define main v26_mega_previous_main
#include "test_mega.cpp"
#undef main
#include <cmath>

void pidCommand(const char *text){for(const char *p=text;*p;p++)Serial.rx.push_back(uint8_t(*p));loop();}
void readyAutoYaw(float yaw=0){
  arm();yawSample(yaw);yawSample(yaw);
  CHECK(YawPid::ready&&YawPid::enabled&&YawPid::automatic);
  CHECK(YawPid::headingErrorDeg==0&&YawPid::appliedPwm==0);
}
void readyManualYaw(float yaw=0){
  StartupPid::cancel();arm();yawSample(yaw);yawSample(yaw);
  CHECK(YawPid::ready&&!YawPid::enabled);
  pidCommand("PID ON\n");CHECK(YawPid::enabled&&!YawPid::automatic);
}
void headingHold(){
  readyAutoYaw();yawSample(0,0xffff,128,0);
  for(int i=1;i<=10;i++)yawSample(float(i),0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>9.9f&&YawPid::headingErrorDeg<10.1f);
  const float error=YawPid::headingErrorDeg;
  for(int i=0;i<12;i++)yawSample(10,0xffff,128,0);
  CHECK(std::fabs(YawPid::filteredDps)<2&&YawPid::headingErrorDeg==error);
  CHECK(YawPid::appliedPwm>0&&YawPid::appliedPwm<=YawPid::AUTO_MAX_PWM);
}
void oncePerSample(){
  readyManualYaw();yawSample(0,0xffff,128,0);yawSample(3,0xffff,128,0);
  const int pwm=YawPid::appliedPwm;const float error=YawPid::headingErrorDeg,integral=YawPid::integral;
  const uint32_t id=YawPid::lastCalcId;
  for(int i=0;i<100;i++)CHECK(YawPid::correction(driveSpeed)==pwm);
  CHECK(YawPid::lastCalcId==id&&YawPid::headingErrorDeg==error&&YawPid::integral==integral);
  queueAngle(4);loop();CHECK(YawPid::correction(driveSpeed)==pwm&&YawPid::lastCalcId==id);
  yawSample(4,0xffff,128,0);CHECK(YawPid::lastCalcId==Hwt901b::sampleId&&YawPid::appliedPwm!=0);
}
void antiWindup(){
  readyAutoYaw();yawSample(0,0xffff,128,0);
  for(int i=1;i<=12;i++)yawSample(float(i),0xffff,128,0);
  CHECK(YawPid::enabled&&YawPid::automatic&&YawPid::appliedPwm==YawPid::AUTO_MAX_PWM);
  CHECK(std::fabs(YawPid::integral)<=YawPid::INTEGRAL_SHARE*YawPid::AUTO_MAX_PWM/YawPid::KI+0.001f);
  CHECK(std::fabs(YawPid::integral)<1.0f);
  CHECK(YawPid::headingErrorDeg>11.9f&&YawPid::headingErrorDeg<12.1f);
}
void turnReset(){
  readyAutoYaw();yawSample(0,0xffff,128,0);yawSample(3,0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>2.9f&&YawPid::targetSet);
  yawSample(3,0xffff,128,128,255);
  CHECK(YawPid::enabled&&!YawPid::targetSet&&YawPid::appliedPwm==0&&YawPid::integral==0);
}
void neutralReset(){
  readyAutoYaw();yawSample(0,0xffff,128,0);
  for(int i=1;i<=5;i++)yawSample(float(i),0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>4.9f);
  yawSample(5);CHECK(braking&&!YawPid::targetSet);
  CHECK(YawPid::appliedPwm==0&&YawPid::integral==0);
}
void lowSpeedBound(){
  readyAutoYaw();driveSpeed=20;yawSample(0,0xffff,128,0);
  for(int i=1;i<=4;i++)yawSample(float(i),0xffff,128,0);
  CHECK(YawPid::enabled&&std::abs(YawPid::appliedPwm)<=4);
  for(int v:command)CHECK(std::abs(v)<=24);
}
void sampleGap(){
  readyAutoYaw();yawSample(0,0xffff,128,0);yawSample(1,0xffff,128,0);
  const uint32_t oldAt=Hwt901b::sampleAt;
  clockMs=oldAt+Hwt901b::FRESH_MS;queueAngle(1);sample(0xffff,128,0,128,128,1);
  CHECK(Hwt901b::valid&&!YawPid::ready&&!YawPid::enabled);
  yawSample(1);CHECK(YawPid::ready&&!YawPid::enabled);
}
void negativeSign(){
  readyAutoYaw(10);yawSample(10,0xffff,128,0);yawSample(7,0xffff,128,0);
  CHECK(YawPid::headingErrorDeg< -2.9f&&YawPid::appliedPwm<0);
  CHECK(YawPid::appliedPwm>=-YawPid::AUTO_MAX_PWM);
}
void manualMode(){
  readyManualYaw();yawSample(0,0xffff,128,0);yawSample(5,0xffff,128,0);
  CHECK(YawPid::appliedPwm>0&&YawPid::appliedPwm<=YawPid::MAX_PWM);
  CHECK(std::fabs(YawPid::integral)<=YawPid::INTEGRAL_SHARE*YawPid::MAX_PWM/YawPid::KI+0.001f);
  for(int v:command)CHECK(std::abs(v)<=255);
}
void timeRollover(){
  clockMs=UINT32_MAX-300;StartupPid::cancel();arm();
  yawSample(0);yawSample(0);pidCommand("PID ON\n");CHECK(YawPid::enabled);
  yawSample(0,0xffff,128,0);
  for(int i=0;i<10;i++)yawSample(1,0xffff,128,0);
  CHECK(clockMs<300&&YawPid::enabled&&YawPid::ready);
  CHECK(std::fabs(YawPid::headingErrorDeg-1.0f)<0.02f&&YawPid::appliedPwm>0);
}
void telemetry(){
  readyAutoYaw(45);Serial.log.clear();pidCommand("PID STATUS\n");
  CHECK(Serial.log.find("PID=1 AUTO=1 WAIT=0 YAW_OK=1")!=std::string::npos);
  CHECK(Serial.log.find("H10=")!=std::string::npos);
}
int main(int argc,char **argv){
  try{
    CHECK(argc==2);setup();const std::string test=argv[1];
    if(test=="heading_hold")headingHold();
    else if(test=="once_per_sample")oncePerSample();
    else if(test=="antiwindup")antiWindup();
    else if(test=="turn_reset")turnReset();
    else if(test=="neutral_reset")neutralReset();
    else if(test=="low_speed")lowSpeedBound();
    else if(test=="sample_gap")sampleGap();
    else if(test=="negative_sign")negativeSign();
    else if(test=="manual_mode")manualMode();
    else if(test=="telemetry")telemetry();
    else if(test=="time_rollover")timeRollover();
    else throw std::runtime_error("Unknown heading PID test");
    std::printf("PASS heading_pid/%s\n",test.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL heading_pid %s\n",e.what());return 1;}
}
