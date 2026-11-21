#pragma once
#include "config.h"

// ============================================================
//  PID cho 1 trục
//  - I được giới hạn (chống tích lũy quá mức) và có thể tạm dừng
//  - D tính trên giá trị ĐO (không phải sai số) -> không bị giật khi đổi setpoint
//  - D qua bộ lọc thông thấp vì đạo hàm khuếch đại nhiễu rung
// ============================================================
class Pid {
public:
  float kp = 0, ki = 0, kd = 0;
  float iLimit = 0.15f;
  float outLimit = 0.4f;
  float dCutoffHz = 30.0f;

  // các thành phần lần gần nhất (để in debug khi chỉnh PID)
  float pTerm = 0, iTerm = 0, dTerm = 0;

  float update(float setpoint, float measured, float dt, bool integrate) {
    float err = setpoint - measured;
    pTerm = kp * err;

    if (integrate) iTerm = clampf(iTerm + ki * err * dt, -iLimit, iLimit);

    float dRaw = first_ ? 0.0f : -(measured - prevMeas_) / dt;
    prevMeas_ = measured;
    first_ = false;
    float rc = 1.0f / (2.0f * PI * dCutoffHz);
    dFilt_ += (dt / (rc + dt)) * (dRaw - dFilt_);
    dTerm = kd * dFilt_;

    return clampf(pTerm + iTerm + dTerm, -outLimit, outLimit);
  }

  void reset() { iTerm = 0; dFilt_ = 0; first_ = true; pTerm = dTerm = 0; }

private:
  float prevMeas_ = 0, dFilt_ = 0;
  bool first_ = true;
};
