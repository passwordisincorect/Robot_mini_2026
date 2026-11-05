// Extra host tests for V22. Includes V21 regression harness (real V22 source).
#define main previous_test_main
#include "test_mega.cpp"
#undef main
#include <cmath>

void serialCommand(const char *s) {
  for (const char *p=s;*p;p++) Serial.rx.push_back(uint8_t(*p));
  loop();
}
void prepareYaw() {
  StartupPid::cancel(); // V22 manual-PID regression: suppress V23 auto-activation.
  CHECK(!YawPid::enabled && !YawPid::calibrated);
  sensorReady();
  sensorWord(0x3B,0);sensorWord(0x3D,0);sensorWord(0x3F,16384);
  sensorWord(0x43,0);sensorWord(0x45,0);sensorWord(0x47,131); // 1dps bias
  arm();
  for (int i=0;i<110;i++) sample();
  CHECK(YawPid::calibrated);
  CHECK(YawPid::calCount==YawPid::CAL_SAMPLES);
  CHECK(std::fabs(YawPid::biasDps - 1.0f)<0.02f);
  CHECK(!YawPid::enabled);
}
void pidCalibration() {
  prepareYaw();
  CHECK(!YawPid::enabled);
  serialCommand("PID STATUS\n");
  CHECK(Serial.log.find("CAL=100/100")!=std::string::npos);
}
void pidSignAndManual() {
  prepareYaw();
  serialCommand("PID ON\n");
  CHECK(YawPid::enabled);
  sample(0xffff,128,0);
  wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
  sensorWord(0x47,131*11); // +10dps beyond calibrated offset
  for (int i=0;i<9;i++)sample(0xffff,128,0);
  CHECK(YawPid::appliedPwm>0 && YawPid::appliedPwm<=YawPid::MAX_PWM);
  const int correction=YawPid::appliedPwm;
  wheels(-driveSpeed-correction,driveSpeed+correction,
         -driveSpeed+correction,driveSpeed-correction);
  sample(0xffff,128,128,255);
  wheels(-turnSpeed,turnSpeed,turnSpeed,-turnSpeed);
  CHECK(YawPid::appliedPwm==0);
  serialCommand("PID OFF\n");
  CHECK(!YawPid::enabled);
  sample(0xffff,128,0);
  wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
}
void pidModesAndBounds() {
  prepareYaw();serialCommand("PID ON\n");
  CHECK(YawPid::enabled);
  sensorWord(0x47,-131*14); // filtered negative yaw
  for(int i=0;i<20;i++)sample(0xffff,255,128); // drive strafe
  CHECK(YawPid::appliedPwm<0);
  CHECK(YawPid::appliedPwm>=-YawPid::MAX_PWM);
  const int c=YawPid::appliedPwm;
  // V21 clamps each wheel PWM to [-255,255]; saturation is expected.
  wheels(constrain(-driveSpeed-c,-255,255),constrain(-driveSpeed+c,-255,255),
         constrain(-driveSpeed+c,-255,255),constrain(-driveSpeed-c,-255,255));
  for(int i=0;i<4;i++)sample(); // Wait until V21 soft brake completes.
  CHECK(YawPid::appliedPwm==0);
  // Force low speed and prove correction cannot exceed 20% PWM.
  driveSpeed=20;sample(0xffff,128,0);
  sensorWord(0x47,131*40);
  for(int i=0;i<8;i++)sample(0xffff,128,0);
  CHECK(std::abs(YawPid::appliedPwm)<=4);
  for(int i=0;i<4;i++)CHECK(std::abs(command[i])<=24);
}
void pidFaultAndInterlock(){
  prepareYaw();
  sample(0xffff,128,0);
  serialCommand("PID ON\n");
  CHECK(!YawPid::enabled); // cannot opt in while driving
  for(int i=0;i<7;i++)sample(); // V24 phanh 120ms truoc khi PID ON
  serialCommand("PID ON\n");CHECK(YawPid::enabled);
  sample(0xffff,128,0);
  sensorWord(0x47,131*30);
  for(int i=0;i<5;i++)sample(0xffff,128,0);
  CHECK(YawPid::enabled);
  Wire.present=false;
  sample(0xffff,128,0);
  CHECK(!YawPid::enabled&&!YawPid::calibrated);
  wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
  sample(held(2));
  CHECK(!armed&&!YawPid::enabled);
}
void pidStale(){
  prepareYaw(); serialCommand("PID ON\n");
  CHECK(YawPid::enabled);
  sample(0xffff,128,0);
  Mpu6050::sampleAt=clockMs-81;
  controlTask();
  CHECK(!YawPid::enabled);
  wheels(-driveSpeed,driveSpeed,-driveSpeed,driveSpeed);
}
int main(int argc,char **argv){
  try {
    CHECK(argc==2);
    setup();
    std::string test=argv[1];
    if(test=="calibration")pidCalibration();
    else if(test=="manual")pidSignAndManual();
    else if(test=="modes")pidModesAndBounds();
    else if(test=="fault")pidFaultAndInterlock();
    else if(test=="stale")pidStale();
    else throw std::runtime_error("Unknown V22 test");
    std::printf("PASS V22/%s\n",test.c_str());
    return 0;
  }catch(const std::exception&e) {
    std::fprintf(stderr,"FAIL V22: %s\n",e.what());
    return 1;
  }
}
