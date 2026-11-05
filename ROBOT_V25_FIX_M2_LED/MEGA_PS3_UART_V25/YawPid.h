#pragma once
#include <Arduino.h>
#include <math.h>
#include "Mpu6050.h"

// V25: gyro-Z cascaded short-horizon heading hold + yaw-rate PID.
// MPU Z phai huong len, Gz quay phai du kien AM. CHUA XAC MINH tren robot that.
// Neu dau Gz / chieu bu PWM sai, closed-loop co the tang lech -> thu tren gia do.
namespace YawPid {
const int8_t GZ_SIGN = 1;
const uint16_t CAL_SAMPLES = 100;          // Khoang 2s khi bat dong.
const uint32_t FRESH_MS = 80;
const float KP = 1.8f, KI = 0.25f, KD = 0.015f;
const float HEADING_KP = 0.6f;              // Deg/s mong muon tren 1 deg lech.
const float HEADING_MAX_DEG = 6.0f;         // Khong cho tich luy yaw vo han.
const float HEADING_LEAK_S = 8.0f;          // Giam drift khi gyro khong co magnetometer.
const float RATE_FILTER_TAU_MS = 60.0f;
const float DERIV_FILTER_TAU_S = 0.10f;
const float RATE_DEADBAND_DPS = 0.6f;
const float HEADING_DEADBAND_DEG = 0.25f;
const float INTEGRAL_SHARE = 0.30f;         // I dong gop <= 30% gioi han PWM.
const int MAX_PWM = 25, AUTO_MAX_PWM = 10, SLEW_PWM = 4;
const float RATE_LIMIT_DPS = 90.0f;
const float AUTO_SPIN_GUARD_DPS = 15.0f;

bool enabled = false;                     // Boot OFF; auto ON sau MPU CAL va PS3 ARM.
bool automatic = false;
bool calibrated = false;
uint16_t calCount = 0;
float calSum = 0.0f, calSum2 = 0.0f, biasDps = 0.0f;
float filteredDps = 0.0f;
float headingErrorDeg = 0.0f;              // Goc lech tu luc bat dau di chuyen.
float integral = 0.0f;
float filteredDerivative = 0.0f, lastRate = 0.0f;
uint32_t lastSample = 0, lastCalcSample = 0;
bool seenSample = false, seenCalc = false;
int appliedPwm = 0;

bool healthy(uint32_t now) {
  return Mpu6050::valid && Mpu6050::error == Mpu6050::NONE &&
         uint32_t(now - Mpu6050::sampleAt) < FRESH_MS;
}
void resetControl() {
  appliedPwm = 0;
  headingErrorDeg = 0.0f;
  integral = 0.0f;
  filteredDerivative = 0.0f;
  lastRate = 0.0f;
  lastCalcSample = 0;
  seenCalc = false;
}
void disable() {
  enabled = false;
  automatic = false;
  resetControl();
}
void resetCalibration() {
  disable();
  calibrated = false;
  calCount = 0; calSum = 0.0f; calSum2 = 0.0f; biasDps = 0.0f;
  filteredDps = 0.0f;
  seenSample = false; lastSample = 0;
}
float clampFloat(float x, float minimum, float maximum) {
  if(x<minimum) return minimum;
  if(x>maximum) return maximum;
  return x;
}
void observe(bool stationary) {
  const uint32_t now = millis();
  if (!healthy(now)) { resetCalibration(); return; }
  const uint32_t sample = Mpu6050::sampleAt;
  if (seenSample && sample == lastSample) return;
  const uint32_t interval = seenSample ? uint32_t(sample-lastSample) : 20;
  if (seenSample && interval > FRESH_MS) {
    // Mất mẫu liên tiếp hoặc I2C tắc: không dùng góc/tích phân đã cũ.
    resetCalibration(); return;
  }
  lastSample = sample; seenSample = true;

  const float gx = float(Mpu6050::raw[3]) / 131.0f;
  const float gy = float(Mpu6050::raw[4]) / 131.0f;
  const float gz = float(Mpu6050::raw[5]) / 131.0f;
  if (!calibrated) {
    const float ax = float(Mpu6050::raw[0]) / 16384.0f;
    const float ay = float(Mpu6050::raw[1]) / 16384.0f;
    const float az = float(Mpu6050::raw[2]) / 16384.0f;
    const float norm2 = ax*ax + ay*ay + az*az;
    if (!stationary || norm2 < 0.64f || norm2 > 1.44f ||
        fabsf(gx)>4.0f || fabsf(gy)>4.0f || fabsf(gz)>4.0f) {
      calCount=0;calSum=0.0f;calSum2=0.0f;return;
    }
    ++calCount; calSum += gz; calSum2 += gz*gz;
    if (calCount>=CAL_SAMPLES) {
      const float mean=calSum/float(calCount);
      const float variance=calSum2/float(calCount)-mean*mean;
      if (variance<=0.64f && fabsf(mean)<=4.0f) {
        biasDps=mean;calibrated=true;filteredDps=0.0f;
      } else {calCount=0;calSum=0.0f;calSum2=0.0f;}
    }
    return;
  }
  const float rate=float(GZ_SIGN)*(gz-biasDps);
  if (fabsf(rate)>RATE_LIMIT_DPS) { disable();return; }
  const float alpha=float(interval)/(RATE_FILTER_TAU_MS+float(interval));
  filteredDps += alpha*(rate-filteredDps);
}
bool enableIfSafe(bool motionNeutral, bool bootAuto=false) {
  if (!motionNeutral || !calibrated || !healthy(millis()) ||
      fabsf(filteredDps)>3.0f) return false;
  resetControl();automatic=bootAuto;enabled=true;return true;
}
int correction(int drivePwm) {
  const uint32_t now=millis();
  if (!enabled) return 0;
  if (!calibrated || !healthy(now)) {resetCalibration();return 0;}
  if (automatic && fabsf(filteredDps)>AUTO_SPIN_GUARD_DPS) {
    disable();return 0;
  }
  const uint32_t currentSample=Mpu6050::sampleAt;
  if (!seenCalc) {
    lastCalcSample=currentSample;
    lastRate=filteredDps;
    seenCalc=true;
    return 0;
  }
  if (currentSample==lastCalcSample) return appliedPwm; // Khong tinh PID lap lai cung 1 mau.
  const uint32_t deltaMs=uint32_t(currentSample-lastCalcSample);
  if(deltaMs>FRESH_MS){disable();return 0;}
  if(deltaMs<10) return appliedPwm;
  lastCalcSample=currentSample;
  const float dt=float(deltaMs)*0.001f;

  // Outer loop: yaw heading trong thoi gian ngan. Reset khi dung/phanh/xoay.
  // Leak gioi han drift vi MPU6050 KHONG co heading tuyet doi.
  const float rateForAngle=fabsf(filteredDps)<0.25f?0.0f:filteredDps;
  headingErrorDeg=(headingErrorDeg+rateForAngle*dt)*
                  (HEADING_LEAK_S/(HEADING_LEAK_S+dt));
  headingErrorDeg=clampFloat(headingErrorDeg,-HEADING_MAX_DEG,HEADING_MAX_DEG);

  // Inner loop: rate error = measured yaw rate - desired yaw rate.
  // desiredRate = -HEADING_KP*headingErrorDeg.
  const float measuredRate=fabsf(filteredDps)<RATE_DEADBAND_DPS?0.0f:filteredDps;
  const float error=measuredRate+HEADING_KP*headingErrorDeg;

  // D tren PHEP DO (gyro rate), khong tren setpoint/error -> giam derivative kick.
  const float derivative=(filteredDps-lastRate)/dt;
  lastRate=filteredDps;
  const float dAlpha=dt/(DERIV_FILTER_TAU_S+dt);
  filteredDerivative+=dAlpha*(derivative-filteredDerivative);

  int bound=automatic?AUTO_MAX_PWM:MAX_PWM;
  const int speedBound=abs(drivePwm)/5;
  if(speedBound<bound)bound=speedBound;
  if(bound<=0){resetControl();return 0;}

  // Anti-windup: giam I o bias nho, va chi tich luy khi khong day them vao bao hoa.
  const float iMax=INTEGRAL_SHARE*float(bound)/KI;
  if(fabsf(measuredRate)<RATE_DEADBAND_DPS &&
     fabsf(headingErrorDeg)<HEADING_DEADBAND_DEG) {
    integral*=1.0f/(1.0f+dt/1.2f);
  } else {
    const float nextI=clampFloat(integral+error*dt,-iMax,iMax);
    const float candidate=KP*error+KI*nextI+KD*filteredDerivative;
    if (!((candidate>float(bound)&&error>0.0f) ||
          (candidate<-float(bound)&&error<0.0f))) integral=nextI;
  }
  integral=clampFloat(integral,-iMax,iMax);
  float wanted=KP*error+KI*integral+KD*filteredDerivative;
  wanted=clampFloat(wanted,-float(bound),float(bound));
  int target=int(wanted>=0.0f?wanted+0.5f:wanted-0.5f);
  if(target>appliedPwm+SLEW_PWM)target=appliedPwm+SLEW_PWM;
  if(target<appliedPwm-SLEW_PWM)target=appliedPwm-SLEW_PWM;
  appliedPwm=target;
  return appliedPwm;
}
} // namespace YawPid
