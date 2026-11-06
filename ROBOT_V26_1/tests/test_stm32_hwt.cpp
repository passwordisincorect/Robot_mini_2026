#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>
extern "C" {
#include "../STM32_HWT901B_BRIDGE/hwt901b_bridge.h"
}

static UART_HandleTypeDef sensor{}, mega{}, other{};
uint32_t fakeTick=0, fakePrimask=0;

extern "C" uint32_t HAL_GetTick(void){return fakeTick;}
extern "C" HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                                   uint8_t *buffer,uint16_t size){
  uart->receiveCalls++;uart->rxData=buffer;uart->rxSize=size;return uart->receiveResult;
}
extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,
                                                 const uint8_t *data,uint16_t size,
                                                 uint32_t timeout){
  uart->transmitCalls++;uart->lastTimeout=timeout;
  if(uart->transmitResult!=HAL_OK)return uart->transmitResult;
  assert(uint32_t(uart->outputSize)+size<=sizeof(uart->output));
  for(uint16_t i=0;i<size;i++)uart->output[uart->outputSize++]=data[i];
  return HAL_OK;
}
extern "C" uint32_t __get_PRIMASK(void){return fakePrimask;}
extern "C" void __disable_irq(void){fakePrimask=1;}
extern "C" void __set_PRIMASK(uint32_t v){fakePrimask=v;}
extern "C" void __DMB(void){}

static std::vector<uint8_t> frame(uint8_t type, int16_t yaw = 0) {
  std::vector<uint8_t> f={0x55,type,0,0,0,0,
    uint8_t(yaw),uint8_t(uint16_t(yaw)>>8),0,0,0};
  uint8_t sum=0;
  for(unsigned i=0;i<10;i++)sum=uint8_t(sum+f[i]);
  f[10]=sum;
  return f;
}

static void byte(uint8_t b) {
  assert(sensor.rxData && sensor.rxSize==1);
  *sensor.rxData=b;
  HWT901B_Bridge_RxCpltCallback(&sensor);
}

static void feed(const std::vector<uint8_t>& f) {
  for(uint8_t b:f)byte(b);
}

static void checkForwarded(const std::vector<uint8_t>& f) {
  const auto n=mega.outputSize;
  for(uint8_t b:f)byte(b);
  HWT901B_Bridge_Task();
  assert(mega.outputSize==n+11);
  for(unsigned i=0;i<11;i++){
    if(mega.output[n+i]!=f[i]){
      std::fprintf(stderr,"frame mismatch #%u byte %u: got %02x want %02x\n",
                   unsigned(mega.transmitCalls),i,mega.output[n+i],f[i]);
      assert(false);
    }
  }
  assert(mega.lastTimeout==3);
}

int main() {
  sensor.id=1;mega.id=2;other.id=3;
  assert(HWT901B_Bridge_Init(nullptr,&mega)==HAL_ERROR);
  assert(HWT901B_Bridge_Init(&sensor,&mega)==HAL_OK);
  assert(sensor.receiveCalls==1 && sensor.rxSize==1);
  const auto initialReceives=sensor.receiveCalls;
  sensor.receiveResult=HAL_ERROR;
  assert(HWT901B_Bridge_Init(&sensor,&mega)==HAL_ERROR);
  sensor.receiveResult=HAL_OK;HWT901B_Bridge_Task();
  assert(sensor.receiveCalls==initialReceives+2); // Init failure is reported and retried.

  auto angle=frame(0x53,8192); // 45 degrees
  auto acceleration=frame(0x51,1234);
  feed(acceleration);HWT901B_Bridge_Task();
  assert(mega.outputSize==0);
  checkForwarded(angle);

  auto bad=angle;bad[4]^=1;
  feed(bad);feed(angle);HWT901B_Bridge_Task();
  assert(mega.outputSize==22); // bad checksum discarded, following frame recovered

  auto missingByte=angle;missingByte.erase(missingByte.begin()+5);
  auto insertedByte=angle;insertedByte.insert(insertedByte.begin()+5,0x31);
  feed(missingByte);feed(insertedByte);feed(angle);
  HWT901B_Bridge_Task();HWT901B_Bridge_Task();
  assert(mega.outputSize==33); // Lost and inserted bytes resync at the next header.

  feed(std::vector<uint8_t>(angle.begin(),angle.begin()+6));
  fakeTick+=31;HWT901B_Bridge_Task();
  feed(std::vector<uint8_t>(angle.begin()+6,angle.end()));
  feed(angle);HWT901B_Bridge_Task();
  assert(mega.outputSize==44); // stale fragment never forwarded

  auto minus180=frame(0x53,INT16_MIN);
  checkForwarded(minus180);
  const auto n=mega.outputSize;
  auto angle55=frame(0x53,0x5555);
  checkForwarded(angle55); // payload 0x55 bytes do not break framing
  assert(mega.outputSize==n+11);

  const auto rxCalls=sensor.receiveCalls;
  HWT901B_Bridge_RxCpltCallback(&other);
  HWT901B_Bridge_ErrorCallback(&other);
  assert(sensor.receiveCalls==rxCalls);

  // More than the 127-byte SPSC capacity causes the queued partial data to be discarded.
  for(unsigned i=0;i<200;i++)byte(0x33);
  HWT901B_Bridge_Task();
  const auto before=mega.outputSize;
  checkForwarded(angle);
  assert(mega.outputSize==before+11);

  mega.transmitResult=HAL_ERROR;
  feed(angle);HWT901B_Bridge_Task();
  assert(mega.outputSize==before+11);
  mega.transmitResult=HAL_OK;
  checkForwarded(angle);

  const auto transmitted=mega.transmitCalls;
  const auto tick=fakeTick;
  feed(angle);feed(angle);feed(angle);feed(angle);feed(angle);
  HWT901B_Bridge_Task();
  assert(mega.transmitCalls==transmitted+1); // at most one angle sample per task
  assert(fakeTick==tick); // no software waits or blocking retry loops
  HWT901B_Bridge_Task();
  assert(mega.transmitCalls==transmitted+2);
  HWT901B_Bridge_Task();
  assert(mega.transmitCalls==transmitted+3);
  HWT901B_Bridge_Task();
  assert(mega.transmitCalls==transmitted+4);
  HWT901B_Bridge_Task();
  assert(mega.transmitCalls==transmitted+5);

  HWT901B_Bridge_ErrorCallback(&sensor);
  HWT901B_Bridge_Task(); // error discards queued data and restarts cleanly
  const auto attempts=sensor.receiveCalls;
  sensor.receiveResult=HAL_ERROR;
  byte(0x55); // RX-complete rearm failure must schedule a main-loop retry.
  assert(sensor.receiveCalls==attempts+1);
  HWT901B_Bridge_Task(); // failed restart stays pending for a later main-loop pass
  assert(sensor.receiveCalls==attempts+2);
  sensor.receiveResult=HAL_OK;
  HWT901B_Bridge_Task(); // bounded retry rearms reception
  assert(sensor.receiveCalls==attempts+3);
  checkForwarded(angle);
  std::puts("PASS STM32 HWT901B bridge parser, recovery, overflow, callbacks and bounded tasks");
}
