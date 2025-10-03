/*
   s3servo.cpp - Library for control servo motor on esp32s3.
   Created by HA.S, October 10, 2022.
   Public domain.
   NOTE: This file is adapted to work with both Arduino-ESP32 v2.x and v3.x.
*/

#include <Arduino.h>
#include "s3servo.h"

// --- 常量（按常见舵机设置） ---
static constexpr uint32_t kServoHz = 50;     // 舵机频率 50Hz
// MAX_BIT_NUM、CHANNEL_MAX_NUM 应在你的 s3servo.h 里定义

// 将“微秒脉宽”换算为 LEDC 的占空计数（0..(2^bits-1)）
static inline uint32_t usToDuty(uint32_t us, uint8_t bits, uint32_t freqHz) {
  // duty = us * freq * (2^bits - 1) / 1e6
  const uint32_t maxDuty = (1UL << bits) - 1UL;
  // 使用 64 位避免溢出
  uint64_t num = (uint64_t)us * (uint64_t)freqHz * (uint64_t)maxDuty;
  return (uint32_t)(num / 1000000ULL);
}

s3servo::s3servo() {}
s3servo::~s3servo() { detach(); }

void s3servo::detach() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // v3.x
  ledcDetach(_pin);
#else
  // v2.x
  ledcDetachPin(_pin);
#endif
}

void s3servo::_setAngleRange(int min, int max) {
  _minAngle = min;
  _maxAngle = max;
}
void s3servo::_setPulseRange(int min, int max) {
  _minPulseWidth = min;   // 单位：微秒
  _maxPulseWidth = max;   // 单位：微秒
}

int8_t s3servo::attach(int pin, int channel, int min_angle, int max_angle,
                       int min_pulse, int max_pulse) {
  if (channel < 0 || channel > CHANNEL_MAX_NUM) {
    return -1;
  }
  _pin = pin;
  _channel = channel;
  _setAngleRange(min_angle, max_angle);
  _setPulseRange(min_pulse, max_pulse);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // v3.x：一次性完成配置与绑定
  if (!ledcAttachChannel(_pin, kServoHz, MAX_BIT_NUM, _channel)) {
    return -1;
  }
  // 初始写 0（或写居中脉宽均可）
  ledcWriteChannel(_channel, 0);
#else
  // v2.x：先 setup，再 attachPin
  ledcSetup(_channel, kServoHz, MAX_BIT_NUM);
  ledcAttachPin(_pin, _channel);
  ledcWrite(_channel, 0);
#endif
  return 0;
}

void s3servo::write(int angle) {
  // 角度限制
  if (angle < _minAngle) angle = _minAngle;
  if (angle > _maxAngle) angle = _maxAngle;

  // 角度 → 脉宽(µs)
  long us = map(angle, _minAngle, _maxAngle, _minPulseWidth, _maxPulseWidth);
  if (us < _minPulseWidth) us = _minPulseWidth;
  if (us > _maxPulseWidth) us = _maxPulseWidth;

  // 脉宽(µs) → LEDC 占空计数
  uint32_t duty = usToDuty((uint32_t)us, MAX_BIT_NUM, kServoHz);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(_channel, duty);
#else
  ledcWrite(_channel, duty);
#endif
}

void s3servo::writeDuty(int duty) {
  // 保持原有语义：这里的 duty 视为“LEDC 计数值”，不是微秒。
  if (duty < 0) duty = 0;
  const uint32_t maxDuty = (1UL << MAX_BIT_NUM) - 1UL;
  if ((uint32_t)duty > maxDuty) duty = (int)maxDuty;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(_channel, (uint32_t)duty);
#else
  ledcWrite(_channel, (uint32_t)duty);
#endif
}
