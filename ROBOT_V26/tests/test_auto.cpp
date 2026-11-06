// Automatic yaw-PID startup and failsafe checks using the real Mega sketch.
#define main mega_legacy_main
#include "test_mega.cpp"
#undef main

void pidCommand(const char *cmd){for(const char *p=cmd;*p;p++)Serial.rx.push_back(uint8_t(*p));loop();}
void readyAuto(){
  CHECK(StartupPid::pending&&!YawPid::enabled);arm();
  yawSample(0);CHECK(!YawPid::ready&&!YawPid::enabled);
  yawSample(0);CHECK(YawPid::ready&&YawPid::enabled&&YawPid::automatic);
  CHECK(!StartupPid::pending);
}
void automaticStartup(){
  readyAuto();CHECK(YawPid::appliedPwm==0);
  pidCommand("PID STATUS\n");
  CHECK(Serial.log.find("PID=1 AUTO=1 WAIT=0 YAW_OK=1")!=std::string::npos);
  yawSample(0,0xffff,128,0);CHECK(YawPid::targetSet&&command[0]==-255&&command[1]==255);
}
void waitUntilControllerReady(){
  for(int i=0;i<3;i++){queueAngle(0);clockMs+=20;loop();}
  CHECK(YawPid::ready&&!armed&&!YawPid::enabled&&StartupPid::pending);
  for(int i=0;i<7;i++)yawSample(0);
  CHECK(armed&&YawPid::enabled&&YawPid::automatic&&!StartupPid::pending);
}
void limitAutoPwm(){
  readyAuto();yawSample(0,0xffff,128,0);
  for(int i=1;i<=10;i++)yawSample(float(i),0xffff,128,0);
  CHECK(YawPid::enabled&&YawPid::automatic);
  CHECK(YawPid::appliedPwm>0&&YawPid::appliedPwm<=YawPid::AUTO_MAX_PWM);
  for(int v:command)CHECK(std::abs(v)<=255);
  yawSample(10,0xffff,128,128,255);
  CHECK(YawPid::enabled&&YawPid::appliedPwm==0&&!YawPid::targetSet);
}
void manualOffPersistence(){
  pidCommand("PID OFF\n");CHECK(!StartupPid::pending);
  arm();yawSample(0);yawSample(0);
  CHECK(YawPid::ready&&!YawPid::enabled);
  pidCommand("PID ON\n");CHECK(YawPid::enabled&&!YawPid::automatic&&!StartupPid::pending);
}
void failsafeEmergency(){
  readyAuto();yawSample(0,0xffff,128,0);yawSample(1,0xffff,128,0);
  sample(held(2));stopped();CHECK(!YawPid::enabled&&!StartupPid::pending);
  for(int i=0;i<9;i++){sample();}
  arm();yawSample(1);yawSample(1);
  CHECK(!YawPid::enabled); // R3 does not re-arm PID automatically.
}
void failsafeLink(){
  readyAuto();inject(frame(++sequence,0xffff,128,128,128,128,0));stopped();
  CHECK(!YawPid::enabled&&!StartupPid::pending);
}
void failsafeYaw(){
  readyAuto();yawSample(0,0xffff,128,0);
  clockMs=Hwt901b::sampleAt+Hwt901b::FRESH_MS;loop();
  CHECK(!YawPid::enabled&&!YawPid::ready&&!Hwt901b::valid);
  yawSample(1);yawSample(1);CHECK(YawPid::ready&&!YawPid::enabled);
}
void guardFastSpin(){
  readyAuto();yawSample(0,0xffff,128,0);yawSample(15,0xffff,128,0);
  CHECK(!YawPid::enabled&&!StartupPid::pending);
  for(int v:command)CHECK(std::abs(v)<=255);
}
void faultBeforeAuto(){
  arm();for(int i=0;i<120;i++)sample();
  CHECK(!YawPid::ready&&!YawPid::enabled&&StartupPid::pending);
}
int main(int argc,char **argv){
  try{
    CHECK(argc==2);setup();const std::string test=argv[1];
    if(test=="auto_start")automaticStartup();
    else if(test=="wait_controller")waitUntilControllerReady();
    else if(test=="limit_pwm")limitAutoPwm();
    else if(test=="off_persist")manualOffPersistence();
    else if(test=="r3")failsafeEmergency();
    else if(test=="uart")failsafeLink();
    else if(test=="yaw_stale")failsafeYaw();
    else if(test=="spin_guard")guardFastSpin();
    else if(test=="sensor_absent")faultBeforeAuto();
    else throw std::runtime_error("Unknown auto test");
    std::printf("PASS auto/%s\n",test.c_str());return 0;
  }catch(const std::exception &e){std::fprintf(stderr,"FAIL auto %s\n",e.what());return 1;}
}
