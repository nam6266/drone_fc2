#pragma once
#include "config.h"

// ============================================================
//  4 kênh PWM -> Gate MOSFET. Giá trị 0..1 (0 = tắt, 1 = hết ga)
// ============================================================
class Motors {
public:
  float out[4] = {0, 0, 0, 0};   // giá trị đang xuất (để in debug / telemetry)

  void begin() {
    for (int i = 0; i < 4; i++) {
      ledcAttach(cfg::PIN_MOTOR[i], cfg::PWM_FREQ, cfg::PWM_BITS);
      ledcWrite(cfg::PIN_MOTOR[i], 0);
      out[i] = 0;
    }
  }

  void write(const float v[4]) {
    for (int i = 0; i < 4; i++) {
      out[i] = clampf(v[i], 0.0f, 1.0f);
      ledcWrite(cfg::PIN_MOTOR[i], (uint32_t)(out[i] * cfg::PWM_MAX + 0.5f));
    }
  }

  void stop() {
    const float zero[4] = {0, 0, 0, 0};
    write(zero);
  }
};
