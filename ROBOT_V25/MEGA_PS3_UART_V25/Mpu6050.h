#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>

namespace Mpu6050 {
const uint8_t ADDRESS = 0x68; // AD0=GND; use 0x69 when AD0 is sensor logic HIGH.
const uint8_t READY = 10;
const uint32_t SAMPLE_MS = 20, STALE_MS = 200, RETRY_MS = 1000;
enum Error : uint8_t { NONE, I2C, TIMEOUT, ID, CONFIG };
uint8_t stage = 0;
Error error = NONE;
bool valid = false;
int16_t raw[6] = {};
uint32_t stepAt = 0, waitMs = 100, sampleAt = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg); Wire.write(value);
  const uint8_t result = Wire.endTransmission(true);
  if (Wire.getWireTimeoutFlag()) { error = TIMEOUT; return false; }
  if (result != 0) { error = I2C; return false; }
  return true;
}
bool readRegisters(uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  const uint8_t result = Wire.endTransmission(false);
  if (Wire.getWireTimeoutFlag()) { error = TIMEOUT; return false; }
  if (result != 0) { error = I2C; return false; }
  const uint8_t received = Wire.requestFrom(ADDRESS, count, uint8_t(true));
  if (Wire.getWireTimeoutFlag()) { error = TIMEOUT; return false; }
  if (received != count) { error = I2C; return false; }
  for (uint8_t i = 0; i < count; i++) {
    const int value = Wire.read();
    if (value < 0) { error = I2C; return false; }
    data[i] = uint8_t(value);
  }
  return true;
}
int16_t signedWord(const uint8_t *data) {
  const uint16_t value = (uint16_t(data[0]) << 8) | data[1];
  return int16_t(value >= 0x8000 ? int32_t(value) - 65536L : int32_t(value));
}
void failed() {
  valid = false; stage = 0;
  stepAt = millis(); waitMs = RETRY_MS;
}
void begin() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  stage = 0; error = NONE; valid = false;
  stepAt = millis(); waitMs = 100;
}
bool task() {
  if (uint32_t(millis() - stepAt) < waitMs) return false;
  waitMs = 0;
  // At most one register transaction per call; reset/wake delays use millis().
  static const uint8_t settings[][2] = {
    {0x6B,0x80}, {0x6B,0x01}, {0x6C,0x00}, {0x1A,0x03},
    {0x19,0x09}, {0x1B,0x00}, {0x1C,0x00}
  };
  uint8_t data[14];
  if (stage == 0) {
    if (!readRegisters(0x75, data, 1)) { failed(); return true; }
    if (data[0] != 0x68) { error = ID; failed(); return true; }
    error = NONE; stage = 1;
  } else if (stage <= 7) {
    if (!writeRegister(settings[stage-1][0], settings[stage-1][1])) { failed(); return true; }
    if (stage == 1 || stage == 2) waitMs = 100;
    stage++;
  } else if (stage == 8) {
    if (!readRegisters(0x19, data, 4)) { failed(); return true; }
    if (data[0] != 9 || data[1] != 3 || data[2] != 0 || data[3] != 0) {
      error = CONFIG; failed(); return true;
    }
    stage = 9;
  } else if (stage == 9) {
    if (!readRegisters(0x6B, data, 2)) { failed(); return true; }
    if (data[0] != 1 || data[1] != 0) { error = CONFIG; failed(); return true; }
    stage = READY; waitMs = SAMPLE_MS;
  } else {
    if (!readRegisters(0x3B, data, sizeof(data))) { failed(); return true; }
    for (uint8_t i = 0; i < 3; i++) {
      raw[i] = signedWord(data + 2*i);
      raw[i+3] = signedWord(data + 8 + 2*i); // Skip temperature bytes 6/7.
    }
    sampleAt = millis(); valid = true; error = NONE; waitMs = SAMPLE_MS;
  }
  stepAt = millis();
  return true;
}
void formatLine(char *line, size_t size) {
  if (error != NONE) {
    const char *label = error == I2C ? "I2C" : error == TIMEOUT ? "TIMEOUT" :
                        error == ID ? "ID" : "CONFIG";
    snprintf(line, size, "MPU=0 LOI=%s A/G=NA\r\n", label);
  } else if (!valid) {
    snprintf(line, size, "MPU=0 DANG_KHOI_TAO A/G=NA\r\n");
  } else if (uint32_t(millis() - sampleAt) >= STALE_MS) {
    snprintf(line, size, "MPU=0 DU_LIEU_CU A/G=NA\r\n");
  } else {
    char value[6][8];
    for (uint8_t i = 0; i < 6; i++)
      dtostrf(raw[i] / (i < 3 ? 16384.0f : 131.0f), 1, i < 3 ? 3 : 1, value[i]);
    snprintf(line, size, "MPU=1 A[g]=%s,%s,%s G[dps]=%s,%s,%s\r\n",
             value[0],value[1],value[2],value[3],value[4],value[5]);
  }
}
} // namespace Mpu6050
