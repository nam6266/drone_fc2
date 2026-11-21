// ============================================================
//  Firmware drone mini – Seeed XIAO ESP32-S3
//  Arduino IDE: board XIAO_ESP32S3, USB CDC On Boot: Enabled
//
//  Cấu trúc (mỗi file là 1 tab trong Arduino IDE):
//    config.h            – chân, hằng số, giới hạn an toàn
//    Imu.h               – MPU6500 + bộ lọc bù
//    Motors.h            – 4 kênh PWM
//    Pid.h               – bộ PID 1 trục
//    Mixer.h             – trộn lực cho quad X
//    Battery.h           – đo pin
//    Radio.h             – WiFi AP + giao thức UDP
//    FlightController.h  – vòng lặp chính, máy trạng thái, lệnh Serial
// ============================================================
#include "FlightController.h"

FlightController fc;

void setup() { fc.begin(); }
void loop()  { fc.loop(); }
