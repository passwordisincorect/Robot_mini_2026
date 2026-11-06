/*
 ESP32_PS3_BRIDGE_V11 - ESP32-WROOM-32 + esp32-bluepad32 core 4.1.0.
 GPIO17 TX -> Mega D17 RX2; GND chung. Khong noi TX Mega vao ESP32.
 Serial2 115200 8N1, chi gui. Serial USB 115200 de xem MAC va PS3.
 PS3 can ghep USB voi dia chi Bluetooth in ra; xem HUONG_DAN.md.
 Giu pairing qua reset; chi mot gamepad duoc dieu khien robot.
*/
#include <Arduino.h>
#include <Bluepad32.h>
#include <esp_system.h>
#include <stdio.h>
#include <string.h>
#if !defined(ARDUINO_ARCH_ESP32) || !defined(CONFIG_IDF_TARGET_ESP32)
#error Chon ESP32 Dev Module trong esp32_bluepad32 (ESP32 doi goc, khong C3/S3)
#endif

ControllerPtr gamepad=nullptr;
const uint8_t UART_TX=17,FRAME_SIZE=14;
const uint32_t SEND_MS=20,REPORT_TIMEOUT_MS=200;
const uint32_t STALE_LINK_MS=5000;
const uint32_t MONITOR_LINE_MS=500;
uint16_t reportSeq=0;
uint16_t buttons=0xffff;
uint8_t axes[4]={128,128,128,128};
uint32_t reportAt=0,sendAt=0,statusAt=0;
uint32_t connectedAt=0;
uint16_t disconnectCount=0;
bool disconnectRequested=false;
bool haveReport=false,resetPending=true,sentOnce=false,sentValid=false;
bool systemPressed=false;
const char *const BUTTON_NAMES[17]={
  "SELECT","L3","R3","START","LEN","PHAI","XUONG","TRAI",
  "L2","R2","L1","R1","TAM_GIAC","TRON","X","VUONG","PS"
};
bool monitorJoy=true;
uint32_t pressedSincePrint=0;

uint16_t crc16(const uint8_t *data,uint8_t length){
  uint16_t crc=0xffff;
  for(uint8_t i=0;i<length;i++){
    crc^=uint16_t(data[i])<<8;
    for(uint8_t bit=0;bit<8;bit++)
      crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
  }
  return crc;
}
uint8_t axisByte(int32_t value){
  return uint8_t((constrain(value,-512,511)+512)/4);
}
uint16_t readButtons(ControllerPtr ctl){
  const uint8_t dpad=ctl->dpad();
  const bool down[16]={
    ctl->miscSelect(),ctl->thumbL(),ctl->thumbR(),ctl->miscStart(),
    bool(dpad&DPAD_UP),bool(dpad&DPAD_RIGHT),bool(dpad&DPAD_DOWN),bool(dpad&DPAD_LEFT),
    ctl->l2(),ctl->r2(),ctl->l1(),ctl->r1(),
    ctl->y(),ctl->b(),ctl->a(),ctl->x()
  };
  uint16_t result=0xffff;
  for(uint8_t i=0;i<16;i++)if(down[i])result&=uint16_t(~(uint16_t(1U)<<i));
  return result;
}
void onConnectedController(ControllerPtr ctl){
  if(gamepad){ctl->disconnect();return;}
  // Kiem tra isGamepad khi co report, khong phu thuoc du lieu ngay luc connect.
  gamepad=ctl;haveReport=false;resetPending=true;pressedSincePrint=0;
  connectedAt=millis();disconnectRequested=false;
}
void onDisconnectedController(ControllerPtr ctl){
  if(ctl!=gamepad)return;
  gamepad=nullptr;haveReport=false;resetPending=true;pressedSincePrint=0;
  disconnectRequested=false;disconnectCount++;
}
bool sendFrame(bool valid){
  if(Serial2.availableForWrite()<FRAME_SIZE)return false;
  uint8_t frame[FRAME_SIZE]={0xa5,0x5a,1,uint8_t(valid)};
  frame[4]=uint8_t(reportSeq);frame[5]=uint8_t(reportSeq>>8);
  const uint16_t bits=valid?buttons:0xffff;
  frame[6]=uint8_t(bits);frame[7]=uint8_t(bits>>8);
  for(uint8_t i=0;i<4;i++)frame[8+i]=valid?axes[i]:128;
  const uint16_t checksum=crc16(frame+2,10);
  frame[12]=uint8_t(checksum);frame[13]=uint8_t(checksum>>8);
  return Serial2.write(frame,sizeof(frame))==sizeof(frame);
}
void monitorTask(bool valid,uint32_t now){
  if(!valid)pressedSincePrint=0;
  if(uint32_t(now-statusAt)<MONITOR_LINE_MS)return;
  char line[128];
  size_t length;
  if(monitorJoy&&!valid){
    length=snprintf(line,sizeof(line),"JOY: KHONG XAC DINH (PS3=0)");
  }else{
    if(monitorJoy)
      length=snprintf(line,sizeof(line),"JOY LX=%u LY=%u RX=%u RY=%u NHAN_1S=",
        unsigned(axes[0]),unsigned(axes[1]),unsigned(axes[2]),unsigned(axes[3]));
    else
      length=snprintf(line,sizeof(line),"PS3=%u SEQ=%u AGE=%lu BT=%u DC=%u GIU=",
        unsigned(valid),unsigned(reportSeq),(unsigned long)(haveReport?uint32_t(now-reportAt):9999),
        unsigned(gamepad&&gamepad->isConnected()),unsigned(disconnectCount));
    const uint32_t held=(uint32_t(~buttons)&0xffffUL)|(systemPressed?(1UL<<16):0);
    const uint32_t shown=monitorJoy?pressedSincePrint:held;
    bool any=false;
    if(valid)for(uint8_t i=0;i<17;i++){
      if(!(shown&(1UL<<i)))continue;
      length+=snprintf(line+length,sizeof(line)-length,"%s%s",any?" ":"",BUTTON_NAMES[i]);
      any=true;
    }
    if(!any)length+=snprintf(line+length,sizeof(line)-length,"%s",valid?"KHONG":"KHONG_XAC_DINH");
  }
  snprintf(line+length,sizeof(line)-length,"\n");
  if(Serial.availableForWrite()<int(strlen(line)))return;
  Serial.print(line);statusAt=now;
  if(monitorJoy)pressedSincePrint=0;
  monitorJoy=!monitorJoy;
}
void setup(){
  Serial.begin(115200);
  const esp_reset_reason_t resetReason=esp_reset_reason();
  Serial.printf("RESET_REASON=%u BROWNOUT=%u\n",unsigned(resetReason),unsigned(resetReason==ESP_RST_BROWNOUT));
  Serial2.begin(115200,SERIAL_8N1,-1,UART_TX);
  BP32.setup(&onConnectedController,&onDisconnectedController);
  BP32.enableVirtualDevice(false);
  const uint8_t *addr=BP32.localBdAddress();
  Serial.printf("Bluetooth MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
  Serial.printf("Bluepad32: %s\n",BP32.firmwareVersion());
  Serial.println("TX GPIO17 -> MEGA D17; UART 115200. PS3=0");
  Serial.println("MONITOR 2 DONG/GIAY; JOY 0..255 GIUA=128");
  statusAt=millis();
}
void loop(){
  const bool updated=BP32.update();
  const uint32_t now=millis();
  if(updated&&!disconnectRequested&&gamepad&&gamepad->isConnected()&&gamepad->isGamepad()&&gamepad->hasData()){
    buttons=readButtons(gamepad);
    systemPressed=gamepad->miscSystem();
    // shortcut: gop nut da nhan giua hai dong JOY; khong giu thu tu/tung canh THA.
    pressedSincePrint|=(uint32_t(~buttons)&0xffffUL)|(systemPressed?(1UL<<16):0);
    axes[0]=axisByte(gamepad->axisX());axes[1]=axisByte(gamepad->axisY());
    axes[2]=axisByte(gamepad->axisRX());axes[3]=axisByte(gamepad->axisRY());
    reportAt=now;haveReport=true;reportSeq++;
  }
  // Dong phien treo de noi lai; robot van dung tu moc 200ms, khong doi den 5s.
  if(gamepad&&gamepad->isConnected()&&!disconnectRequested&&
     uint32_t(now-(haveReport?reportAt:connectedAt))>=STALE_LINK_MS){
    disconnectRequested=true;resetPending=true;gamepad->disconnect();
  }
  const bool valid=!resetPending&&!disconnectRequested&&gamepad&&gamepad->isConnected()&&haveReport&&
                   uint32_t(now-reportAt)<REPORT_TIMEOUT_MS;
  if(!sentOnce||valid!=sentValid||resetPending||uint32_t(now-sendAt)>=SEND_MS){
    if(sendFrame(valid)){
      sendAt=now;sentOnce=true;sentValid=valid;resetPending=false;
    }
  }
  monitorTask(valid,now);
  delay(1); // Nhuong CPU cho Bluetooth/watchdog, khong cho 20ms bang delay.
}
