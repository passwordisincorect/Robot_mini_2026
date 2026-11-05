/*
 MEGA_PS3_UART_V25 - Arduino Mega 2560 + MPU6050 + short-horizon heading PID + yaw-rate PID.
 MPU6050: SDA20/SCL21 qua chuyen muc I2C; auto PID sau hieu chuan+ARM.
 ESP32-WROOM GPIO17 TX -> Mega D17 RX2; GND chung. UART 115200 8N1.
 Xem HUONG_DAN.md: khong noi TX Mega 5V vao ESP32.
 DE: M1 FL EN5 A30 B31 dao; M2 RL EN6 A32 B33 dao;
     M3 RR EN7 A34 B35; M4 FR EN8 A37 B36.
 L298N truot: EN9 A43 B45; nang: EN10 A39 B41.
 Hanh trinh NC: COM=GND; tren18, duoi19, trai26, phai27.
 Bom L298N kenh A: D28->ENA (thao jumper); IN1=5V, IN2=GND.
 ENA keo xuong GND 10k; logic 5V rieng, bo jumper 5V-EN khi cap ngoai.
 Van: D40->IN relay 5V active HIGH; tai van 24V qua COM/NO.
 D40 can keo xuong GND bang 10k de giu OFF trong reset.
 Joy trai: 4 huong hieu chinh V19 theo huong xe do nguoi dung bao.
 Chon truc lech nhieu hon; bang nhau uu tien tien/lui.
 Joy phai X: xoay mecanum uu tien; phai = M1/M2 tien, M3/M4 lui.
 D-pad LEN/XUONG: giu de nang/ha; PHAI/TRAI: nhan de tu len/tu xuong het.
 TRON: truot phai; VUONG: truot trai. Kiem tra SLIDE_REVERSE khi ke tai.
 R1: giu bom ON. L1: tat bom + van ON 1000ms, khong delay.
 L2: -20%; R2: +20%, +20%, -20%, +20% lap lai (giu V11).
 R3: dung, huy lenh; tha nut va joy ve giua 100ms de mo khoa.
 Cac nut con lai chi log. Toc do giu V11; khong xung phanh nguoc, phanh dien 120ms.
 Khung moi hop le het han 200ms: dung banh/co cau, tat bom/van.
 R3 va firmware KHONG thay the nut cat nguon dong luc.
 Serial Monitor 115200. Chi nap sau khi doi dung day; khong tu nap.
*/
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "Mpu6050.h"
#include "YawPid.h"
#if !defined(__AVR_ATmega2560__)
#error Chon Arduino Mega 2560
#endif

// M1 truoc trai, M2 sau trai, M3 sau phai, M4 truoc phai.
const uint8_t EN[4]={5,6,7,8};
const uint8_t A[4]={30,32,34,37};
const uint8_t B[4]={31,33,35,36};
const bool REVERSE[4]={false,false,false,true}; // FIX M2 sau trai: dao so voi V25; M1/M3/M4 giu nguyen.
// Hanh trinh NC: COM -> GND, NC -> 18/19/26/27 (INPUT_PULLUP).
const uint8_t LIFT_EN=10, LIFT_A=39, LIFT_B=41;
const uint8_t SLIDE_EN=9, SLIDE_A=43, SLIDE_B=45;
const uint8_t LIMIT_UP=18, LIMIT_DOWN=19, LIMIT_LEFT=26, LIMIT_RIGHT=27;
const uint8_t SUCTION=28, RELEASE_VALVE=40;
const uint8_t SUCTION_LED1=50, SUCTION_LED2=52; // D50/D52 active-HIGH LED bao hut.
const bool LIFT_REVERSE=false, SLIDE_REVERSE=false;
const uint8_t VALVE_ON=HIGH, VALVE_OFF=LOW; // Module thuc te kich HIGH.
int driveSpeed=235,turnSpeed=180,slideSpeed=235;
uint16_t speedPrevious=0xffff;
uint8_t r2Step=0; // Tang, tang, GIAM, tang; sau do lap lai.
int liftCommand=0,slideCommand=0;
bool suctionOn=false,releaseOn=false;
int8_t autoLiftDirection=0; // +1 len D18, -1 xuong D19, 0 dung tu dong.
bool upperBlocked=true,lowerBlocked=true,leftBlocked=true,rightBlocked=true;
uint32_t upperOpenAt=0,lowerOpenAt=0,leftOpenAt=0,rightOpenAt=0;
uint16_t toolPrevious=0xffff;
uint32_t releaseAt=0;
const uint16_t BRAKE_RAMP_MS=120;   // Phanh dien tang dan, KHONG dao chieu dong co.
const uint8_t BRAKE_START_PWM=70;  // EN duty 27.5%, IN1=IN2=LOW.
const uint8_t NORMAL_BRAKE_PWM=150; // EN duty 58.8% sau phanh; KHONG phai giu vi tri.
bool braking=false;
uint8_t lastDriveKind=0; // 0=dung, 1=tien/lui, 2=xoay, 3=ngang
uint32_t brakeAt=0,brakeDuration=0;
// V24: khong phat lenh quay nguoc khi tha joystick.
const int CENTER=128, DEADZONE=25;
const uint32_t TIMEOUT_MS=200;

// UART: buttons active LOW theo thu tu nut cua V11.
struct Packet {
  uint16_t buttons;
  uint8_t lx,ly,rx,ry;
};
Packet pad={0xffff,128,128,128,128};
bool uartOK=false,got=false,padOK=false,armed=false,neutralWait=false;
bool sequenceSeen=false;
const uint8_t FRAME_SIZE=14;
uint8_t uartFrame[FRAME_SIZE]={},uartLength=0;
uint32_t uartByteAt=0;
uint32_t rxAt=0,lastSeq=0,neutralAt=0;
uint32_t rxCount=0,badCount=0,statusAt=0;
int command[4]={0,0,0,0};
const uint32_t IDLE_BRAKE_MS=20;
bool idleBrakeWritten=false;
uint8_t idleBrakePwm=255;
uint32_t idleBrakeAt=0;
uint32_t lastWheelWrite[4]={};
const uint32_t MONITOR_LINE_MS=500;
uint8_t monitorPage=0;
// Tu dong bat PID MOT LAN trong moi lan cap nguon: sau hieu chuan, ARM, nut/joy trung tinh.
// PID OFF, R3, hay mat UART sau khi da co ket noi se huy auto den reset tiep theo.
bool neutral(const Packet &p);
namespace StartupPid {
  bool pending = true;
  void cancel() { pending=false; }
  bool idleReady() {
    if(!armed||!uartOK||!got||!padOK||pad.buttons!=0xffff||
       !neutral(pad)||braking||lastDriveKind!=0||liftCommand||slideCommand||
       suctionOn||releaseOn) return false;
    for(uint8_t i=0;i<4;i++)if(command[i]!=0)return false;
    return true;
  }
  void task() {
    if(!pending || !YawPid::calibrated || !idleReady())return;
    if(YawPid::enableIfSafe(true,true)){
      pending=false;
      Serial.println(F("PID AUTO ON (LIMIT 10 PWM)"));
    }
  }
}

bool neutral(const Packet &p);
void controlTask();
void stopAll(uint8_t brakePwm=255);

bool neutral(const Packet &p){
  return abs(int(p.lx)-CENTER)<=DEADZONE&&abs(int(p.ly)-CENTER)<=DEADZONE&&
         abs(int(p.rx)-CENTER)<=DEADZONE&&abs(int(p.ry)-CENTER)<=DEADZONE;
}
void stopAll(uint8_t brakePwm){
  bool moving=false;
  for(uint8_t i=0;i<4;i++)if(command[i]!=0)moving=true;
  uint32_t now=millis();
  // Phanh ngay khi vua dung; sau do ap lai moi 20ms du khong co goi moi.
  if(!moving&&idleBrakeWritten&&brakePwm==idleBrakePwm&&now-idleBrakeAt<IDLE_BRAKE_MS)return;
  idleBrakeAt=now;idleBrakeWritten=true;idleBrakePwm=brakePwm;
  // Hai IN LOW: PWM EN giam phanh thuong; R3/mat lien lac dung EN=255.
  // Chi ngat EN khi chuyen tu chay sang phanh. Dung im khong cat EN dinh ky.
  if(moving)for(uint8_t i=0;i<4;i++) analogWrite(EN[i],0);
  for(uint8_t i=0;i<4;i++){
    digitalWrite(A[i],LOW);digitalWrite(B[i],LOW);
    command[i]=0;
    analogWrite(EN[i],brakePwm);
    lastWheelWrite[i]=now;
  }
}
void normalBrakeTask(uint32_t now){
  if(!braking)return;
  const uint32_t elapsed=uint32_t(now-brakeAt);
  if(elapsed>=brakeDuration){
    braking=false; lastDriveKind=0; stopAll(NORMAL_BRAKE_PWM); return;
  }
  const uint8_t pwm=uint8_t(BRAKE_START_PWM+
    (uint32_t(NORMAL_BRAKE_PWM-BRAKE_START_PWM)*elapsed)/brakeDuration);
  stopAll(pwm); // IN1=IN2=LOW; EN PWM: fast stop / coast theo datasheet L298.
}
bool pressed(uint8_t bit){return !(pad.buttons&(uint16_t(1U)<<bit));}
int percentSpeed(int value,int percent){
  return constrain((value*percent+50)/100,1,255);
}
void speedTask(){
  bool l2=pressed(8),r2=pressed(9);
  bool l2Edge=l2&&(speedPrevious&(1U<<8));
  bool r2Edge=r2&&(speedPrevious&(1U<<9));
  speedPrevious=pad.buttons;
  if(l2&&r2)return; // Hai nut cung nhan: khong doi toc do.
  int percent=100;
  if(l2Edge)percent=80;
  else if(r2Edge){
    percent=r2Step==2?80:120;
    r2Step=(r2Step+1)&3;
  }
  if(percent==100)return;
  driveSpeed=percentSpeed(driveSpeed,percent);
  turnSpeed=percentSpeed(turnSpeed,percent);
  slideSpeed=percentSpeed(slideSpeed,percent);
}
void writeTool(uint8_t en,uint8_t a,uint8_t b,int v,bool reverse){
  analogWrite(en,0);
  int out=reverse?-v:v;
  digitalWrite(a,out>0?HIGH:LOW);digitalWrite(b,out<0?HIGH:LOW);
  analogWrite(en,abs(out));
}
void stopTools(){
  analogWrite(LIFT_EN,0);analogWrite(SLIDE_EN,0);
  digitalWrite(LIFT_A,LOW);digitalWrite(LIFT_B,LOW);
  digitalWrite(SLIDE_A,LOW);digitalWrite(SLIDE_B,LOW);
  digitalWrite(SUCTION,LOW);
  digitalWrite(SUCTION_LED1,LOW);digitalWrite(SUCTION_LED2,LOW);
  digitalWrite(RELEASE_VALVE,VALVE_OFF);
  liftCommand=slideCommand=0; suctionOn=releaseOn=false;
  toolPrevious=0xffff;autoLiftDirection=0;
}
void readLimits(){
  uint32_t now=millis();
  // NC mo = HIGH: chan ngay. Chi cho chay lai khi NC dong on dinh 20ms.
  if(digitalRead(LIMIT_UP)==HIGH){upperOpenAt=now;upperBlocked=true;}
  else if(now-upperOpenAt>=20)upperBlocked=false;
  if(digitalRead(LIMIT_DOWN)==HIGH){lowerOpenAt=now;lowerBlocked=true;}
  else if(now-lowerOpenAt>=20)lowerBlocked=false;
  if(digitalRead(LIMIT_LEFT)==HIGH){leftOpenAt=now;leftBlocked=true;}
  else if(now-leftOpenAt>=20)leftBlocked=false;
  if(digitalRead(LIMIT_RIGHT)==HIGH){rightOpenAt=now;rightBlocked=true;}
  else if(now-rightOpenAt>=20)rightBlocked=false;
}
void toolsTask(){
  int speed=slideSpeed;
  bool manualUp=pressed(4),manualDown=pressed(6);
  const bool autoUp=pressed(5),autoDown=pressed(7);
  if(manualUp||manualDown||(autoUp&&autoDown))autoLiftDirection=0;
  else if(autoUp&&(toolPrevious&(1U<<5)))autoLiftDirection=1;
  else if(autoDown&&(toolPrevious&(1U<<7)))autoLiftDirection=-1;
  if((autoLiftDirection>0&&upperBlocked)||(autoLiftDirection<0&&lowerBlocked))autoLiftDirection=0;
  int lift=((manualUp||manualDown)?int(manualUp)-int(manualDown):int(autoLiftDirection))*255;
  int slide=(int(pressed(13))-int(pressed(15)))*speed;
  if((lift>0&&upperBlocked)||(lift<0&&lowerBlocked))lift=0;
  if((slide>0&&rightBlocked)||(slide<0&&leftBlocked))slide=0;
  if(lift!=liftCommand){writeTool(LIFT_EN,LIFT_A,LIFT_B,lift,LIFT_REVERSE);liftCommand=lift;}
  if(slide!=slideCommand){writeTool(SLIDE_EN,SLIDE_A,SLIDE_B,slide,SLIDE_REVERSE);slideCommand=slide;}
  if(pressed(10)&&(toolPrevious&(1U<<10))){releaseOn=true;releaseAt=millis();}
  if(releaseOn&&millis()-releaseAt>=1000)releaseOn=false;
  if(pressed(10)||releaseOn)suctionOn=false;
  else if(pressed(11)&&(toolPrevious&(1U<<11)))suctionOn=true;
  toolPrevious=pad.buttons;
  digitalWrite(SUCTION,suctionOn?HIGH:LOW); // D28 -> ENA L298N: HIGH bat, LOW ngat cau.
  digitalWrite(SUCTION_LED1,suctionOn?HIGH:LOW);
  digitalWrite(SUCTION_LED2,suctionOn?HIGH:LOW); // LED mirror trang thai hut.
  digitalWrite(RELEASE_VALVE,releaseOn?VALVE_ON:VALVE_OFF);
}
void lockStop(){if(got&&(!uartOK||!padOK||!(pad.buttons&(1U<<2))))StartupPid::cancel();YawPid::disable();speedPrevious=0xffff;armed=false;neutralWait=false;braking=false;lastDriveKind=0;stopAll();stopTools();}
void runWheels(int a,int b,int c,int d){
  if(!armed||!uartOK||!got||!padOK||millis()-rxAt>=TIMEOUT_MS||!(pad.buttons&(1U<<2))){lockStop();return;}
  int requested[4]={a,b,c,d};
  for(uint8_t i=0;i<4;i++){
    int v=constrain(requested[i],-255,255);
    bool changed=v!=command[i];
    uint32_t now=millis();
    if(!changed&&now-lastWheelWrite[i]<20)continue;
    // Ghi lai 20ms, khong cat EN neu chieu/PWM khong doi.
    if(changed)analogWrite(EN[i],0);
    int out=REVERSE[i]?-v:v;
    digitalWrite(A[i],out>0?HIGH:LOW);
    digitalWrite(B[i],out<0?HIGH:LOW);
    analogWrite(EN[i],abs(out));
    command[i]=v;lastWheelWrite[i]=now;
  }
}
uint16_t crc16(const uint8_t *data,uint8_t length){
  uint16_t crc=0xffff;
  for(uint8_t i=0;i<length;i++){
    crc^=uint16_t(data[i])<<8;
    for(uint8_t bit=0;bit<8;bit++)
      crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
  }
  return crc;
}
void invalidateLink(){
  uartOK=false;padOK=false;sequenceSeen=false;
  lockStop();
}
void acceptFrame(){
  if(uartFrame[3]==0){invalidateLink();return;}
  const uint16_t seq=uint16_t(uartFrame[4])|(uint16_t(uartFrame[5])<<8);
  if(sequenceSeen){
    const uint16_t delta=uint16_t(seq-uint16_t(lastSeq));
    if(delta==0||delta>=0x8000)return;
  }
  if(!uartOK)lockStop();
  sequenceSeen=true;lastSeq=seq;
  pad.buttons=uint16_t(uartFrame[6])|(uint16_t(uartFrame[7])<<8);
  pad.lx=uartFrame[8];pad.ly=uartFrame[9];
  pad.rx=uartFrame[10];pad.ry=uartFrame[11];
  got=true;uartOK=true;padOK=true;rxAt=millis();rxCount++;
  controlTask();
}
void receiveByte(uint8_t value){
  if(uartLength==0&&value!=0xa5)return;
  if(uartLength==1&&value!=0x5a){
    uartLength=value==0xa5?1:0;return;
  }
  uartFrame[uartLength++]=value;
  if(uartLength<FRAME_SIZE)return;
  const uint16_t checksum=uint16_t(uartFrame[12])|(uint16_t(uartFrame[13])<<8);
  if(uartFrame[2]==1&&uartFrame[3]<=1&&crc16(uartFrame+2,10)==checksum){
    acceptFrame();uartLength=0;return;
  }
  badCount++;
  // Giu lai dau khung tiep theo, ke ca khi mat byte giua hai khung.
  uint8_t next=1;
  while(next<FRAME_SIZE&&
        !(uartFrame[next]==0xa5&&(next==FRAME_SIZE-1||uartFrame[next+1]==0x5a)))next++;
  uartLength=FRAME_SIZE-next;
  memmove(uartFrame,uartFrame+next,uartLength);
}
void uartTask(){
  const uint32_t now=millis();
  if(uartLength&&uint32_t(now-uartByteAt)>=30){uartLength=0;badCount++;}
  // Gioi han moi vong de nhieu UART khong lam tre hanh trinh/timer.
  uint8_t budget=64;
  while(budget--&&Serial2.available()){
    uartByteAt=millis();
    if(uartOK&&uint32_t(uartByteAt-rxAt)>=TIMEOUT_MS)invalidateLink();
    receiveByte(uint8_t(Serial2.read()));
  }
}
void controlTask(){
  readLimits();
  uint32_t now=millis();
  if(!uartOK||!got||!padOK||now-rxAt>=TIMEOUT_MS||!(pad.buttons&(1U<<2))){lockStop();return;}
  if(!armed){
    stopAll();stopTools();
    const uint16_t motionButtons=(1U<<4)|(1U<<5)|(1U<<6)|(1U<<7)|(1U<<8)|(1U<<9)|(1U<<10)|(1U<<11)|(1U<<13)|(1U<<15);
    if((pad.buttons&motionButtons)!=motionButtons){neutralWait=false;return;}
    if(!neutral(pad)){neutralWait=false;return;}
    if(!neutralWait){neutralWait=true;neutralAt=rxAt;}
    // Phai co bao cao MOI xuyen suot 100ms, khong mo khoa bang khung lap.
    if(uint32_t(rxAt-neutralAt)>=100){armed=true;neutralWait=false;}
    return;
  }
  speedTask();
  toolsTask();
  int speed=driveSpeed,turn=turnSpeed;
  int x=int(pad.lx)-CENTER,y=int(pad.ly)-CENTER,r=int(pad.rx)-CENTER;
  // Joystick moi ngat phanh mem ngay; khong khoa tay cam suot 120ms.
  if(braking){
    YawPid::resetControl();
    if(abs(x)<=35&&abs(y)<=35&&abs(r)<=35){
      normalBrakeTask(now);
      return;
    }
    braking=false;
  }
  const int moveThreshold=lastDriveKind?DEADZONE:35;
  if(abs(r)>moveThreshold){
    YawPid::resetControl();
    braking=false;lastDriveKind=2;
    int v=r>0?turn:-turn;runWheels(-v,v,v,-v); // Dung chieu tien tung motor da xac nhan.
  }else if(abs(y)>moveThreshold&&abs(y)>=abs(x)){
    braking=false;lastDriveKind=1;
    int v=y<0?speed:-speed;int c=YawPid::correction(speed);runWheels(-v-c,v+c,-v+c,v-c); // V23 bu yaw.
  }else if(abs(x)>moveThreshold){
    braking=false;lastDriveKind=3;
    int v=x>0?speed:-speed;int c=YawPid::correction(speed);runWheels(-v-c,-v+c,-v+c,-v-c); // V23 bu yaw.
  }else{
    YawPid::resetControl();
    // Phanh dien 120ms, khong phat lenh quay nguoc khi tha joystick.
    if(!braking&&lastDriveKind!=0){
      brakeDuration=BRAKE_RAMP_MS;
      brakeAt=now;braking=true;lastDriveKind=0;
    }
    if(braking)normalBrakeTask(now);
    else stopAll(NORMAL_BRAKE_PWM);
  }
}
bool printLine(const char *s){
  if(Serial.availableForWrite()<int(strlen(s)))return false;
  Serial.print(s);return true;
}
void serialTask(){
  const uint32_t now=millis();
  if(uint32_t(now-statusAt)<MONITOR_LINE_MS)return;
  char line[64];
  if(monitorPage==0)
    snprintf(line,sizeof(line),"N=%d T=%d BOM=%u VAN=%u HT=%u%u%u%u AUTO=%d\r\n",
      liftCommand,slideCommand,unsigned(suctionOn),unsigned(releaseOn),
      unsigned(upperBlocked),unsigned(lowerBlocked),unsigned(leftBlocked),unsigned(rightBlocked),int(autoLiftDirection));
  else if(monitorPage==1)
    snprintf(line,sizeof(line),"UART_PS3=%u ARM=%u RX=%lu M=%d,%d,%d,%d\r\n",
      unsigned(uartOK),unsigned(armed),(unsigned long)rxCount,command[0],command[1],command[2],command[3]);
  else Mpu6050::formatLine(line,sizeof(line));
  if(!printLine(line))return;
  statusAt=now;monitorPage=(monitorPage+1)%3;
}
void pidSerialTask(){
  static char line[14]={}; static uint8_t used=0;
  uint8_t budget=16;
  while(budget-- && Serial.available()){
    const int ch=Serial.read();
    if(ch<0)break;
    if(ch==10 || ch==13){
      line[used]=0;
      if(strcmp(line,"PID OFF")==0){
        StartupPid::cancel();YawPid::disable();Serial.println(F("PID OFF"));
      }else if(strcmp(line,"PID ON")==0){
        const bool safe=armed&&uartOK&&neutral(pad)&&!braking&&lastDriveKind==0;
        if(YawPid::enableIfSafe(safe)){StartupPid::cancel();Serial.println(F("PID ON"));}
        else Serial.println(F("PID DENIED: IDLE/CAL/SENSOR"));
      }else if(strcmp(line,"PID STATUS")==0){
        char out[80];
        snprintf(out,sizeof(out),"PID=%u AUTO=%u WAIT=%u CAL=%u/%u C=%d H10=%d\r\n",
          unsigned(YawPid::enabled),unsigned(YawPid::automatic),unsigned(StartupPid::pending),unsigned(YawPid::calCount),
          unsigned(YawPid::CAL_SAMPLES),
          YawPid::appliedPwm,int(YawPid::headingErrorDeg*10.0f));
        if(Serial.availableForWrite()>=int(strlen(out)))Serial.print(out);
      }
      used=0;
    }else if(ch>=32 && ch<=126){
      if(used<sizeof(line)-1)line[used++]=char(ch);
      else used=0;
    }
  }
}
void setup(){
  digitalWrite(RELEASE_VALVE,VALVE_OFF);pinMode(RELEASE_VALVE,OUTPUT);
  const uint8_t toolPins[]={LIFT_EN,SLIDE_EN,LIFT_A,LIFT_B,SLIDE_A,SLIDE_B,SUCTION,SUCTION_LED1,SUCTION_LED2};
  for(uint8_t pin:toolPins){digitalWrite(pin,LOW);pinMode(pin,OUTPUT);}
  pinMode(LIMIT_UP,INPUT_PULLUP);pinMode(LIMIT_DOWN,INPUT_PULLUP);
  pinMode(LIMIT_LEFT,INPUT_PULLUP);pinMode(LIMIT_RIGHT,INPUT_PULLUP);
  stopTools();readLimits();
  for(uint8_t i=0;i<4;i++){digitalWrite(EN[i],LOW);pinMode(EN[i],OUTPUT);}
  for(uint8_t i=0;i<4;i++){
    digitalWrite(A[i],LOW);digitalWrite(B[i],LOW);pinMode(A[i],OUTPUT);pinMode(B[i],OUTPUT);
  }
  stopAll();YawPid::resetCalibration();StartupPid::pending=true;Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1);
  Serial.println(F("MEGA V25 PS3 UART2 RX=D17 - AUTO HEADING+YAW PID"));
  Serial.println(F("PHANH DIEN 70->150/120MS; R3=EN255; KHONG DAO CHIEU"));
  Serial.println(F("PHAI=TU LEN, TRAI=TU XUONG"));
  Serial.println(F("NANG/HA PWM255 TOI DA; VAN D40 ACTIVE HIGH, L1=1S"));
  Serial.println(F("BOM L298N: D28=ENA; R1=HUT L1=XA; LED D50/D52"));
  Serial.println(F("THA JOYSTICK/NUT DE MO KHOA. R3=DUNG"));
  Serial.println(F("MPU6050 SDA20 SCL21; PID AUTO HEADING+YAW SAU CAL+ARM"));
  Serial.println(F("MONITOR 2 DONG/GIAY: CO CAU, BANH, MPU"));
  Mpu6050::begin();
  statusAt=millis();
}
void loop(){
  if(uartOK&&uint32_t(millis()-rxAt)>=TIMEOUT_MS)invalidateLink();
  uartTask();controlTask();
  if(Mpu6050::task()){
    const bool stationary=neutral(pad)&&!braking&&
       command[0]==0&&command[1]==0&&command[2]==0&&command[3]==0;
    YawPid::observe(stationary);
    if(uartOK&&uint32_t(millis()-rxAt)>=TIMEOUT_MS)invalidateLink();
    controlTask(); // Kiem lai hanh trinh/timer ngay sau giao dich I2C co timeout.
    StartupPid::task(); // Auto chi khi banh dung, PS3 ARM, sensor da hieu chuan.
  }
  pidSerialTask();
  serialTask();
}
