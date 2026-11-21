#pragma once
#include "config.h"

// ============================================================
//  Đo pin qua chia áp 2 × 100 kΩ (điểm giữa -> D10)
// ============================================================
class Battery {
public:
  void begin() {
    analogReadResolution(12);
    update();
  }

  // Gọi ~10 lần/giây
  void update() {
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) sum += analogReadMilliVolts(cfg::PIN_BAT);
    float v = (sum / 4.0f) * 2.0f / 1000.0f;
    if (first_) { volts_ = v; first_ = false; }
    else volts_ += 0.2f * (v - volts_);   // làm mượt
  }

  float volts() const { return volts_; }
  bool low() const { return volts_ < cfg::VBAT_LOW; }

private:
  float volts_ = 0;
  bool first_ = true;
};
