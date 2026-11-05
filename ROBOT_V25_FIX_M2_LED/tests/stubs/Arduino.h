#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <deque>
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define SERIAL_8N1 6
#define F(x) x
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
inline uint32_t clockMs=0;
inline int inputLevel[80]={},outputLevel[80]={},pinModes[80]={},pwmLevel[80]={};
struct WriteEvent { char kind; uint8_t pin; int value; };
inline std::vector<WriteEvent> writes;
inline uint32_t millis(){return clockMs;}
inline void delay(uint32_t ms){clockMs+=ms;}
inline void digitalWrite(uint8_t pin,int v){outputLevel[pin]=v;writes.push_back({'D',pin,v});}
inline void analogWrite(uint8_t pin,int v){pwmLevel[pin]=v;writes.push_back({'A',pin,v});}
inline int digitalRead(uint8_t pin){return inputLevel[pin];}
inline void pinMode(uint8_t pin,int v){pinModes[pin]=v;writes.push_back({'M',pin,v});}
struct FakeSerial {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  std::string log;
  unsigned long baud=0;
  int capacity=63,rxPin=-1,txPin=-1;
  void begin(unsigned long speed,int=SERIAL_8N1,int rx=-1,int tx=-1){
    baud=speed;rxPin=rx;txPin=tx;
  }
  int available(){return int(rx.size());}
  int read(){if(rx.empty())return -1;int b=rx.front();rx.pop_front();return b;}
  int availableForWrite(){return capacity;}
  size_t write(const uint8_t *data,size_t count){tx.insert(tx.end(),data,data+count);return count;}
  void print(const char *s){log+=s;}
  void println(const char *s){log+=s;log+='\n';}
  void printf(const char *format,...){
    char b[256];va_list args;va_start(args,format);vsnprintf(b,sizeof(b),format,args);va_end(args);log+=b;
  }
};
inline FakeSerial Serial,Serial2;
#define CHECK(x) do { if(!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+": " #x); } while(0)

inline char *dtostrf(double value, signed char width, unsigned char precision, char *out){
  std::snprintf(out,8,"%*.*f",int(width),int(precision),value);return out;
}
