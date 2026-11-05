// V23 auto-start safety tests using the REAL V23 sketch and Arduino/Wire stubs.
#define main mega_legacy_main
#include "test_mega.cpp"
#undef main

void pidCommand(const char *cmd) {
  for(const char *p=cmd;*p;p++)Serial.rx.push_back(uint8_t(*p));
  loop();
}
void fillStationaryRegisters(){
  sensorWord(0x3B,0);sensorWord(0x3D,0);sensorWord(0x3F,16384);
  sensorWord(0x43,0);sensorWord(0x45,0);sensorWord(0x47,131); // gyro bias = +1dps
}
void readyAuto(){
  CHECK(StartupPid::pending && !YawPid::enabled);
  sensorReady();
  fillStationaryRegisters();
  arm();
  CHECK(!YawPid::enabled);
  for(int i=0;i<120;i++)sample();
  CHECK(YawPid::calibrated);
  CHECK(YawPid::enabled && YawPid::automatic);
  CHECK(!StartupPid::pending);
  CHECK(Serial.log.find("PID AUTO ON (LIMIT 10 PWM)")!=std::string::npos);
}
void automaticStartup(){
  readyAuto();
  CHECK(YawPid::appliedPwm==0);
  pidCommand("PID STATUS\n");
  CHECK(Serial.log.find("PID=1 AUTO=1 WAIT=0 CAL=100/100")!=std::string::npos);
  sample(0xffff,128,0);
  CHECK(command[0]==-driveSpeed && command[1]==driveSpeed);
}
void waitUntilControllerReady(){
  sensorReady(); fillStationaryRegisters();
  for(int i=0;i<112;i++){clockMs+=20;loop();}
  CHECK(YawPid::calibrated);
  CHECK(!armed && !uartOK);
  CHECK(!YawPid::enabled && StartupPid::pending);
  arm(); for(int i=0;i<3;i++)sample();
  CHECK(YawPid::enabled && YawPid::automatic && !StartupPid::pending);
}
void limitAutoPwm(){
  readyAuto();
  sensorWord(0x47,131*11); // ~+10dps after offset
  for(int i=0;i<15;i++)sample(0xffff,128,0);
  CHECK(YawPid::enabled && YawPid::automatic);
  CHECK(YawPid::appliedPwm>0);
  CHECK(YawPid::appliedPwm<=YawPid::AUTO_MAX_PWM);
  const int c=YawPid::appliedPwm;
  wheels(constrain(-driveSpeed-c,-255,255),constrain(driveSpeed+c,-255,255),
         constrain(-driveSpeed+c,-255,255),constrain(driveSpeed-c,-255,255));
  sample(0xffff,128,128,255);
  CHECK(YawPid::enabled && YawPid::automatic);
  CHECK(YawPid::appliedPwm==0);
  wheels(-turnSpeed,turnSpeed,turnSpeed,-turnSpeed);
}
void manualOffPersistence(){
  CHECK(StartupPid::pending && !YawPid::enabled);
  pidCommand("PID OFF\n");
  CHECK(!StartupPid::pending);
  sensorReady(); fillStationaryRegisters(); arm();
  for(int i=0;i<120;i++)sample();
  CHECK(YawPid::calibrated && !YawPid::enabled);
  pidCommand("PID ON\n");
  CHECK(YawPid::enabled && !YawPid::automatic);
}
void failsafeEmergency(){
  readyAuto();
  sample(held(2)); stopped();
  CHECK(!YawPid::enabled&&!StartupPid::pending);
  for(int i=0;i<9;i++)sample();
  arm();
  for(int i=0;i<8;i++)sample();
  CHECK(!YawPid::enabled); // no auto re-enable after R3
}
void failsafeLink(){
  readyAuto();
  clockMs+=200;loop();
  CHECK(!YawPid::enabled&&!StartupPid::pending);
  for(int i=0;i<9;i++)sample();
  arm();for(int i=0;i<6;i++)sample();
  CHECK(!YawPid::enabled);
}
void failsafeSensor(){
  readyAuto();
  Wire.present=false;
  sample(0xffff,128,0);
  CHECK(!YawPid::enabled&&!YawPid::calibrated);
  CHECK(!StartupPid::pending);
  sensorReady();fillStationaryRegisters();
  for(int i=0;i<120;i++)sample();
  CHECK(YawPid::calibrated && !YawPid::enabled);
}
void guardFastSpin(){
  readyAuto();
  sample(0xffff,128,0);
  sensorWord(0x47,131*41); // ~+40dps
  for(int i=0;i<8;i++)sample(0xffff,128,0);
  CHECK(!YawPid::enabled);
  CHECK(!StartupPid::pending);
  wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
}
void faultBeforeAuto(){
  arm();
  for(int i=0;i<120;i++)sample();
  CHECK(!YawPid::calibrated&&!YawPid::enabled&&StartupPid::pending);
}
int main(int argc,char **argv) {
  try {
    CHECK(argc==2); setup();
    const std::string test=argv[1];
    if(test=="auto_start")automaticStartup();
    else if(test=="wait_controller")waitUntilControllerReady();
    else if(test=="limit_pwm")limitAutoPwm();
    else if(test=="off_persist")manualOffPersistence();
    else if(test=="r3")failsafeEmergency();
    else if(test=="uart")failsafeLink();
    else if(test=="sensor")failsafeSensor();
    else if(test=="spin_guard")guardFastSpin();
    else if(test=="sensor_absent")faultBeforeAuto();
    else throw std::runtime_error("Unknown V23 test");
    std::printf("PASS V23/%s\n",test.c_str());
    return 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr,"FAIL V23 %s\n",e.what());
    return 1;
  }
}
