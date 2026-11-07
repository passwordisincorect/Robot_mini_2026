// Host checks for the STM32 yaw angle PID, using the real Mega sketch.
#define main previous_test_main
#include "test_mega.cpp"
#undef main
#include <cmath>

void pidCommand(const char *text){for(const char *p=text;*p;p++)Serial.rx.push_back(uint8_t(*p));loop();}
void readyManual(float yaw=0){
  StartupPid::cancel();arm();yawSample(yaw);yawSample(yaw);
  CHECK(YawPid::ready&&!YawPid::enabled);
}
void yawReadiness(){
  arm();pidCommand("PID ON\n");CHECK(!YawPid::enabled);
  queueAngle(179);sample();CHECK(!YawPid::ready&&!YawPid::enabled);
  queueAngle(-179);loop();CHECK(!YawPid::ready); // Same-ms samples cannot form a rate.
  yawSample(-179);CHECK(YawPid::ready);
  for(int i=0;i<16;i++)yawSample(-179);
  pidCommand("PID ON\n");CHECK(YawPid::enabled&&!YawPid::automatic);
  clockMs=Hwt901b::sampleAt+Hwt901b::FRESH_MS;loop();
  CHECK(!Hwt901b::valid&&!YawPid::ready&&!YawPid::enabled);
}
void wrappedHeadingFeedback(){
  readyManual(179);pidCommand("PID ON\n");
  yawSample(179,0xffff,128,0);CHECK(YawPid::targetSet&&YawPid::targetYawDeg>178.9f);
  yawSample(-179,0xffff,128,0);
  CHECK(std::fabs(YawPid::headingErrorDeg-2.0f)<0.02f&&YawPid::appliedPwm>0);
  const float heldError=YawPid::headingErrorDeg;
  for(int i=0;i<12;i++)yawSample(-179,0xffff,128,0);
  CHECK(YawPid::headingErrorDeg==heldError); // Heading error is absolute, not accumulated.
  yawSample(177,0xffff,128,0);
  CHECK(std::fabs(YawPid::headingErrorDeg+2.0f)<0.02f);
  for(int i=0;i<10;i++)yawSample(177,0xffff,128,0);
  CHECK(YawPid::appliedPwm<0);
  for(int i=0;i<30;i++)yawSample(-179);
  yawSample(-179,0xffff,128,0);yawSample(179,0xffff,128,0);
  CHECK(std::fabs(YawPid::headingErrorDeg+2.0f)<0.02f);
  CHECK(YawPid::appliedPwm<0);
}
void fullSpeedMix(){
  readyManual();pidCommand("PID ON\n");
  const uint8_t moves[4][2]={{128,0},{128,255},{255,128},{0,128}};
  for(int speed:{20,128,255})for(const auto &move:moves)for(int errorSign:{1,-1}){
    driveSpeed=speed;
    for(int i=0;i<16;i++)yawSample(0);
    yawSample(0,0xffff,move[0],move[1]);
    yawSample(float(errorSign*2),0xffff,move[0],move[1]);
    const int c=YawPid::appliedPwm;
    const int trim=move[1]==128?(speed*STRAFE_FORWARD_TRIM_PERCENT+50)/100:0;
    const int base=std::min(speed,255-std::abs(c)-trim);
    CHECK(errorSign*c>0&&std::abs(c)<=std::min(YawPid::MAX_PWM,speed/5));
    int peak=0;for(int v:command){CHECK(std::abs(v)<=255);peak=std::max(peak,std::abs(v));}
    CHECK(peak==base+std::abs(c)+trim);
    const int p0=-command[0],p1=command[1],p2=-command[2],p3=command[3];
    CHECK(p0+p1-p2-p3==4*c); // Corrective yaw torque has the same sign in all drive directions.
    if(move[1]!=128){
      CHECK(p0+p1+p2+p3==(move[1]<128?1:-1)*4*base);
      CHECK(p0-p1+p2-p3==0);
    }else{
      CHECK(p0+p1+p2+p3==4*trim);
      CHECK(p0-p1+p2-p3==(move[0]>128?1:-1)*4*base);
    }
  }
}

void sampleCadence(){
  readyManual();pidCommand("PID ON\n");yawSample(0,0xffff,128,0);yawSample(3,0xffff,128,0);
  const int pwm=YawPid::appliedPwm;const float error=YawPid::headingErrorDeg,integral=YawPid::integral;
  for(int i=0;i<100;i++)CHECK(YawPid::correction(driveSpeed)==pwm);
  CHECK(YawPid::appliedPwm==pwm&&YawPid::headingErrorDeg==error&&YawPid::integral==integral);
  queueAngle(4);loop();CHECK(YawPid::ready&&Hwt901b::sampleAt==YawPid::lastCalcAt);
  CHECK(YawPid::correction(driveSpeed)==pwm); // A zero-ms angle cannot update the controller.
  yawSample(4,0xffff,128,0);CHECK(YawPid::lastCalcId==Hwt901b::sampleId);
}
void resetAndReacquire(){
  readyManual();pidCommand("PID ON\n");yawSample(0,0xffff,128,0);yawSample(3,0xffff,128,0);
  CHECK(YawPid::headingErrorDeg>2);
  yawSample(3,0xffff,128,128,255);
  CHECK(!YawPid::targetSet&&YawPid::appliedPwm==0&&YawPid::integral==0);
  yawSample(20);yawSample(20,0xffff,128,0);
  CHECK(YawPid::targetSet&&std::fabs(YawPid::targetYawDeg-20)<0.02f);
  CHECK(std::fabs(YawPid::headingErrorDeg)<0.02f);
}
void yawFaultsAndStatus(){
  readyManual();pidCommand("PID ON\n");yawSample(0,0xffff,128,0);
  auto bad=angleFrame(20);bad[10]^=1;Serial3.rx.insert(Serial3.rx.end(),bad.begin(),bad.end());sample();
  CHECK(YawPid::enabled&&Hwt901b::checksumErrors==1);
  const uint32_t sampleAt=Hwt901b::sampleAt;
  auto other=angleFrame(20,0x52);Serial3.rx.insert(Serial3.rx.end(),other.begin(),other.end());sample();
  CHECK(Hwt901b::sampleAt==sampleAt&&YawPid::enabled);
  Serial.log.clear();pidCommand("PID STATUS\n");
  CHECK(Serial.log.find("YAW_OK=1")!=std::string::npos&&Serial.log.find("H10=")!=std::string::npos);
  clockMs=sampleAt+Hwt901b::FRESH_MS;loop();
  CHECK(!YawPid::enabled&&!YawPid::ready&&!Hwt901b::valid);
  yawSample(10);yawSample(10);CHECK(YawPid::ready&&!YawPid::enabled); // No automatic restart.
}

int main(int argc,char **argv){
  try{
    CHECK(argc==2);setup();const std::string test=argv[1];
    if(test=="readiness")yawReadiness();
    else if(test=="wrapped_feedback")wrappedHeadingFeedback();
    else if(test=="full_speed_mix")fullSpeedMix();
    else if(test=="sample_cadence")sampleCadence();
    else if(test=="reset_reacquire")resetAndReacquire();
    else if(test=="faults_status")yawFaultsAndStatus();
    else throw std::runtime_error("Unknown yaw test");
    std::printf("PASS yaw/%s\n",test.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL yaw %s\n",e.what());return 1;}
}
