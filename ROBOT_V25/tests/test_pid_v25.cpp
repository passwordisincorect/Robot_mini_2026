// V25 PID improvements: uses actual V25 Mega sketch with Arduino/Wire host stubs.
#define main v25_mega_previous_main
#include "test_mega.cpp"
#undef main
#include <cmath>

void v25StationaryRegisters() {
  sensorWord(0x3B,0);sensorWord(0x3D,0);sensorWord(0x3F,16384);
  sensorWord(0x43,0);sensorWord(0x45,0);sensorWord(0x47,131); // 1 dps gyro offset.
}
void v25ReadyAuto() {
  sensorReady();v25StationaryRegisters();arm();
  for(int i=0;i<120;i++)sample();
  CHECK(YawPid::calibrated && YawPid::enabled && YawPid::automatic);
  CHECK(!StartupPid::pending);
  CHECK(YawPid::headingErrorDeg==0.0f && YawPid::appliedPwm==0);
}
void v25HeadingHold(){
  v25ReadyAuto();
  sample(0xffff,128,0);
  sensorWord(0x47,131*6); // +5 dps sau bias.
  for(int i=0;i<60;i++)sample(0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>1.0f);
  CHECK(YawPid::headingErrorDeg<=YawPid::HEADING_MAX_DEG);
  CHECK(YawPid::appliedPwm>0 && YawPid::appliedPwm<=YawPid::AUTO_MAX_PWM);
  sensorWord(0x47,131); // Gz=0 sau offset: gyro khong xoay, nhung lech goc van con.
  for(int i=0;i<22;i++)sample(0xffff,128,0);
  CHECK(std::fabs(YawPid::filteredDps)<0.5f);
  CHECK(YawPid::headingErrorDeg>0.5f);
  CHECK(YawPid::appliedPwm>0); // Outer heading control van yeu cau chinh.
}
void v25OneSampleOneUpdate(){
  v25ReadyAuto();sample(0xffff,128,0);
  sensorWord(0x47,131*5);
  for(int i=0;i<12;i++)sample(0xffff,128,0);
  const float angle=YawPid::headingErrorDeg, integ=YawPid::integral;
  const float derivative=YawPid::filteredDerivative;
  const int pwm=YawPid::appliedPwm;
  const uint32_t sampleId=YawPid::lastCalcSample;
  for(int i=0;i<100;i++){
    CHECK(YawPid::correction(driveSpeed)==pwm);
  }
  CHECK(YawPid::headingErrorDeg==angle);
  CHECK(YawPid::integral==integ);
  CHECK(YawPid::filteredDerivative==derivative);
  CHECK(YawPid::lastCalcSample==sampleId);
}
void v25AntiWindup(){
  v25ReadyAuto();sample(0xffff,128,0);
  sensorWord(0x47,131*13); // +12dps after bias; within auto 15dps guard.
  for(int i=0;i<125;i++)sample(0xffff,128,0);
  CHECK(YawPid::enabled && YawPid::automatic);
  CHECK(YawPid::appliedPwm==YawPid::AUTO_MAX_PWM);
  CHECK(std::fabs(YawPid::integral)<3.0f); // old PID would accumulate up to 8PWM of I
  CHECK(YawPid::headingErrorDeg<=YawPid::HEADING_MAX_DEG);
}
void v25ManualTurnResets(){
  v25ReadyAuto();sample(0xffff,128,0);
  sensorWord(0x47,131*7);
  for(int i=0;i<35;i++)sample(0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>0.5f);
  sample(0xffff,128,128,255);
  CHECK(YawPid::headingErrorDeg==0.0f);
  CHECK(YawPid::integral==0.0f);
  CHECK(YawPid::appliedPwm==0);
  CHECK(YawPid::enabled);
}
void v25NeutralResets(){
  v25ReadyAuto();sample(0xffff,128,0);
  sensorWord(0x47,131*7);
  for(int i=0;i<20;i++)sample(0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>0.1f);
  sample();CHECK(braking);
  CHECK(YawPid::headingErrorDeg==0.0f);
  CHECK(YawPid::appliedPwm==0);
  CHECK(YawPid::integral==0.0f);
}
void v25LowSpeed(){
  v25ReadyAuto();
  driveSpeed=20;
  sample(0xffff,128,0);
  sensorWord(0x47,131*11);
  for(int i=0;i<15;i++)sample(0xffff,128,0);
  CHECK(YawPid::enabled);
  CHECK(std::abs(YawPid::appliedPwm)<=4);
  for(int i=0;i<4;i++)CHECK(std::abs(command[i])<=24);
}
void v25SampleGap(){
  v25ReadyAuto();
  const uint32_t sample=YawPid::lastSample;
  clockMs=sample+101;Mpu6050::sampleAt=clockMs;
  YawPid::observe(true);
  CHECK(!YawPid::calibrated);
  CHECK(!YawPid::enabled);
  CHECK(!YawPid::automatic);
}
void v25NegativeSign(){
  v25ReadyAuto();sample(0xffff,128,0);
  sensorWord(0x47,-131*5);
  for(int i=0;i<20;i++)sample(0xffff,128,0);
  CHECK(YawPid::filteredDps<0.0f);
  CHECK(YawPid::headingErrorDeg<0.0f);
  CHECK(YawPid::appliedPwm<0);
  CHECK(YawPid::appliedPwm>=-YawPid::AUTO_MAX_PWM);
}
void v25ManualMode(){
  StartupPid::cancel();
  sensorReady();v25StationaryRegisters();arm();
  for(int i=0;i<120;i++)sample();
  CHECK(YawPid::calibrated && !YawPid::enabled);
  CHECK(YawPid::enableIfSafe(true));
  CHECK(YawPid::enabled && !YawPid::automatic);
  sample(0xffff,128,0);
  sensorWord(0x47,131*21);
  for(int i=0;i<70;i++)sample(0xffff,128,0);
  CHECK(YawPid::appliedPwm>0 && YawPid::appliedPwm<=YawPid::MAX_PWM);
  CHECK(std::fabs(YawPid::integral)<=YawPid::INTEGRAL_SHARE*float(YawPid::MAX_PWM)/YawPid::KI+0.001f);
}
void v25Telemetry(){
  v25ReadyAuto();
  sample(0xffff,128,0);
  sensorWord(0x47,131*6);
  for(int i=0;i<20;i++)sample(0xffff,128,0);
  for(const char *p="PID STATUS\n";*p;p++)Serial.rx.push_back(uint8_t(*p));
  loop();
  CHECK(Serial.log.find("PID=1 AUTO=1 WAIT=0 CAL=100/100")!=std::string::npos);
  CHECK(Serial.log.find("H10=")!=std::string::npos);
}
int main(int argc,char**argv) {
  try {
    CHECK(argc==2); setup();
    const std::string test=argv[1];
    if(test=="heading_hold")v25HeadingHold();
    else if(test=="once_per_sample")v25OneSampleOneUpdate();
    else if(test=="antiwindup")v25AntiWindup();
    else if(test=="turn_reset")v25ManualTurnResets();
    else if(test=="neutral_reset")v25NeutralResets();
    else if(test=="low_speed")v25LowSpeed();
    else if(test=="sample_gap")v25SampleGap();
    else if(test=="negative_sign")v25NegativeSign();
    else if(test=="manual_mode")v25ManualMode();
    else if(test=="telemetry")v25Telemetry();
    else throw std::runtime_error("Unknown V25 test");
    std::printf("PASS V25/%s\n",test.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL V25 %s\n",e.what());return 1;}
}
