#pragma once
#include <Wire.h>
#include "config.h"

// ============================================================
//  MPU6500 (module GY-521) – đọc thanh ghi trực tiếp + bộ lọc bù
//  Quy ước dấu (mũi tên X của module chỉ về mũi drone, chip ngửa lên):
//    roll  > 0 : nghiêng PHẢI (cánh phải chúi xuống)
//    pitch > 0 : CHÚI MŨI xuống
//    gx    > 0 : roll đang tăng,  gy > 0 : pitch đang tăng
//    gz    > 0 : mũi quay sang TRÁI (ngược chiều kim đồng hồ nhìn từ trên)
// ============================================================
class Imu {
public:
  float roll = 0, pitch = 0;                    // góc đã lọc (độ)
  float rollByGravity = 0, pitchByGravity = 0;  // góc chỉ từ trọng lực (độ)
  float gx = 0, gy = 0, gz = 0;                 // tốc độ xoay (độ/giây)
  float tau = 0.5f;                             // hằng số thời gian bộ lọc bù (giây)

  bool begin() {
    Wire.begin(cfg::PIN_SDA, cfg::PIN_SCL);
    Wire.setClock(400000);
    writeReg(0x6B, 0x00);   // đánh thức
    delay(100);
    uint8_t who = readReg(0x75);
    Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);   // MPU6500 = 0x70, MPU6050 = 0x68
    writeReg(0x1A, 0x03);   // lọc gyro ~41 Hz
    writeReg(0x1B, 0x08);   // gyro ±500 dps -> 65.5 / (độ/giây)
    writeReg(0x1C, 0x08);   // gia tốc ±4 g -> 8192 / g
    writeReg(0x1D, 0x03);   // lọc gia tốc ~41 Hz
    return readRaw();
  }

  // Giữ YÊN board khi gọi. Trả về false nếu board bị rung/di chuyển trong lúc đo.
  bool calibrate(int samples = 1000) {
    double s[3] = {0, 0, 0}, ss[3] = {0, 0, 0};
    for (int i = 0; i < samples; i++) {
      if (!readRaw()) return false;
      const int16_t r[3] = { rgx, rgy, rgz };
      for (int k = 0; k < 3; k++) { s[k] += r[k]; ss[k] += (double)r[k] * r[k]; }
      delay(2);
    }
    for (int k = 0; k < 3; k++) {
      double mean = s[k] / samples;
      double stdDps = sqrt(fmax(0.0, ss[k] / samples - mean * mean)) / 65.5;
      if (stdDps > 2.0) return false;     // dao động > 2 độ/giây -> đang bị cầm/rung
    }
    off[0] = s[0] / samples; off[1] = s[1] / samples; off[2] = s[2] / samples;
    readRaw();
    anglesByGravity();
    roll = rollByGravity;
    pitch = pitchByGravity;
    return true;
  }

  bool update(float dt) {
    if (!readRaw()) return false;
    gx = (rgx - off[0]) / 65.5f;
    gy = (rgy - off[1]) / 65.5f;
    gz = (rgz - off[2]) / 65.5f;
    anglesByGravity();
    float alpha = tau / (tau + dt);
    roll  = alpha * (roll  + gx * dt) + (1 - alpha) * rollByGravity;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitchByGravity;
    return true;
  }

private:
  static const uint8_t ADDR = 0x68;
  int16_t rax = 0, ray = 0, raz = 0, rgx = 0, rgy = 0, rgz = 0;
  float off[3] = {0, 0, 0};

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
  }
  uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(ADDR); Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom(ADDR, (uint8_t)1) != 1) return 0xFF;
    return Wire.read();
  }
  bool readRaw() {
    Wire.beginTransmission(ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDR, (uint8_t)14) != 14) return false;
    rax = (Wire.read() << 8) | Wire.read();
    ray = (Wire.read() << 8) | Wire.read();
    raz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();   // nhiệt độ (bỏ qua)
    rgx = (Wire.read() << 8) | Wire.read();
    rgy = (Wire.read() << 8) | Wire.read();
    rgz = (Wire.read() << 8) | Wire.read();
    return true;
  }
  void anglesByGravity() {
    float ax = rax / 8192.0f, ay = ray / 8192.0f, az = raz / 8192.0f;
    rollByGravity  = atan2f(ay, az) * 57.2958f;
    pitchByGravity = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  }
};
