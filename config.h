#pragma once
#include <Arduino.h>

// ============================================================
//  CẤU HÌNH CHUNG – mọi hằng số chỉnh ở đây
// ============================================================
namespace cfg {

// ---------- Chân XIAO ESP32-S3 ----------
// M1 trước-phải (CCW), M2 sau-phải (CW), M3 sau-trái (CCW), M4 trước-trái (CW)
const int PIN_MOTOR[4] = { D0, D1, D2, D3 };
const int PIN_SDA = D4;
const int PIN_SCL = D5;
const int PIN_BAT = D10;
const int PIN_LED = 21;          // LED vàng trên XIAO, sáng khi LOW

// ---------- Vòng lặp điều khiển ----------
const uint32_t LOOP_US = 2000;   // 500 Hz

// ---------- PWM motor ----------
const uint32_t PWM_FREQ = 20000; // 20 kHz (trên ngưỡng tai nghe)
const uint8_t  PWM_BITS = 10;    // độ phân giải 0..1023
const uint32_t PWM_MAX  = (1u << PWM_BITS) - 1;

// ---------- WiFi / UDP ----------
// XIAO tự phát WiFi (Access Point). Điện thoại kết nối vào, IP của XIAO = 192.168.4.1
const char* const WIFI_SSID = "XIAO-Drone";
const char* const WIFI_PASS = "drone1234";   // tối thiểu 8 ký tự
const uint8_t  WIFI_CHANNEL = 6;
const uint16_t UDP_PORT     = 4210;

// ---------- An toàn ----------
const uint32_t LINK_TIMEOUT_MS     = 300;   // không nhận gói quá lâu -> mất kết nối
const uint32_t FAILSAFE_DESCENT_MS = 2000;  // mất kết nối: giảm ga về 0 trong 2 s rồi tắt
const float ARM_MAX_THROTTLE = 0.05f;       // chỉ cho arm khi ga < 5 %
const float ARM_MAX_TILT_DEG = 15.0f;       // chỉ cho arm khi drone gần như nằm phẳng
const float CRASH_TILT_DEG   = 60.0f;       // nghiêng quá mức -> coi như rơi, tắt motor
const float ARM_MIN_VBAT     = 3.5f;        // pin dưới mức này không cho arm
                                            // (đặt 0 nếu muốn test arm khi chỉ cắm USB)
const float VBAT_LOW         = 3.3f;        // cảnh báo pin yếu (gửi lên app)

// ---------- Giới hạn bay ----------
const float MAX_ANGLE_DEG     = 20.0f;   // nghiêng tối đa khi gạt hết cần
const float MAX_RATE_DPS      = 250.0f;  // tốc độ xoay roll/pitch tối đa tầng ngoài được đòi
const float MAX_YAW_RATE_DPS  = 150.0f;  // tốc độ xoay yaw tối đa
const float THROTTLE_MIN_ACTIVE = 0.05f; // ga dưới mức này: motor tắt, PID không chạy
const float THROTTLE_I_ENABLE   = 0.20f; // ga trên mức này mới tích phân (tránh tích lũy khi còn nằm đất)
const float PID_I_LIMIT   = 0.15f;       // giới hạn phần I (đơn vị: tỉ lệ ga 0..1)
const float PID_OUT_LIMIT = 0.40f;       // giới hạn tổng đầu ra mỗi trục
const float MIX_YAW_SIGN  = 1.0f;        // đổi thành -1 nếu kiểm tra thấy yaw bị ngược

} // namespace cfg

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
