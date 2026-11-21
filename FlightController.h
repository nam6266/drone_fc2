#pragma once
#include "config.h"
#include "Imu.h"
#include "Motors.h"
#include "Pid.h"
#include "Mixer.h"
#include "Battery.h"
#include "Radio.h"

enum class State : uint8_t { Disarmed = 0, Armed = 1, Failsafe = 2, Error = 3 };

inline const char* stateName(State s) {
  switch (s) {
    case State::Disarmed: return "DISARMED";
    case State::Armed:    return "ARMED";
    case State::Failsafe: return "FAILSAFE";
    default:              return "ERROR";
  }
}

// Hệ số PID – giá trị KHỞI ĐẦU thận trọng, sẽ chỉnh khi bay thử (lệnh "set" qua Serial)
struct Gains {
  float angleKp = 4.0f;      // tầng ngoài: 1 độ sai -> đòi xoay 4 độ/giây
  float rateKp  = 0.0030f;   // tầng trong roll/pitch: 1 độ/giây sai -> 0.3 % ga
  float rateKi  = 0.0040f;
  float rateKd  = 0.00003f;
  float yawKp   = 0.0020f;   // yaw: chỉ điều khiển tốc độ xoay
  float yawKi   = 0.0005f;
};

// ============================================================
//  Bộ điều khiển bay
//  Mỗi vòng 500 Hz: đọc IMU -> nhận lệnh UDP -> máy trạng thái -> PID 2 tầng -> mixer -> PWM
// ============================================================
class FlightController {
public:
  void begin() {
    motors_.begin();                        // việc đầu tiên: ép 4 motor về 0
    pinMode(cfg::PIN_LED, OUTPUT);
    digitalWrite(cfg::PIN_LED, HIGH);
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== Drone FC – XIAO ESP32-S3 ===");

    battery_.begin();

    if (!imu_.begin()) { enterError("Khong doc duoc MPU6500 - kiem tra day I2C"); return; }
    Serial.println("Hieu chinh gyro: GIU YEN drone ~2 giay...");
    bool ok = false;
    for (int i = 0; i < 10 && !ok; i++) {
      ok = imu_.calibrate();
      if (!ok) Serial.println("  Board dang rung/di chuyen, do lai...");
    }
    if (!ok) { enterError("Khong hieu chinh duoc gyro"); return; }
    Serial.println("  Xong.");

    if (!radio_.begin()) { enterError("Khong bat duoc WiFi AP"); return; }
    Serial.printf("WiFi: \"%s\"  mat khau \"%s\"  IP %s  UDP %u\n",
                  cfg::WIFI_SSID, cfg::WIFI_PASS, radio_.ip().toString().c_str(), cfg::UDP_PORT);

    applyGains();
    Serial.println("Go 'help' de xem lenh.");
    lastUs_ = micros();
  }

  void loop() {
    handleSerial();

    uint32_t nowUs = micros();
    if (nowUs - lastUs_ < cfg::LOOP_US) return;
    float dt = (nowUs - lastUs_) * 1e-6f;
    lastUs_ = nowUs;
    if (dt > 0.01f) dt = 0.01f;
    uint32_t nowMs = millis();
    loopCount_++;
    measureLoopRate(nowMs);

    if (state_ == State::Error) { motors_.stop(); updateLed(nowMs); return; }

    if (imu_.update(dt)) imuErrors_ = 0;
    else if (++imuErrors_ >= 10) { enterError("Mat ket noi MPU6500 giua chung"); return; }

    radio_.poll(nowMs);
    if (loopCount_ % 50 == 0) battery_.update();          // 10 Hz

    updateState(nowMs);
    runControl(dt, nowMs);

    if (loopCount_ % 50 == 25) sendTelemetry();          // 10 Hz
    if (logOn_ && loopCount_ % 25 == 0) printLog();       // 20 Hz
    updateLed(nowMs);
  }

private:
  Imu imu_;
  Motors motors_;
  Battery battery_;
  Radio radio_;
  Pid pidRoll_, pidPitch_, pidYaw_;
  Gains g_;

  State state_ = State::Disarmed;
  const char* reason_ = "khoi dong";
  uint32_t lastUs_ = 0, loopCount_ = 0;
  uint32_t rateWindowMs_ = 0, rateLoops_ = 0, loopHz_ = 0;
  int imuErrors_ = 0;

  bool prevArmBit_ = true;     // true: app phải gửi arm=0 trước rồi mới 0->1 để arm
  float throttle_ = 0;         // ga đang dùng (0..1)
  float fsStartThrottle_ = 0;
  uint32_t fsStartMs_ = 0;
  float mixOut_[4] = {0, 0, 0, 0};

  // Test trên bàn (chỉ khi DISARMED)
  int testMotor_ = -1;  float testDuty_ = 0;  uint32_t testStartMs_ = 0;
  float dryThrottle_ = 0;      // >0: chạy PID+mixer nhưng KHÔNG xuất PWM

  bool logOn_ = false;
  char line_[64];
  size_t lineLen_ = 0;

  // ---------------- Máy trạng thái ----------------
  void updateState(uint32_t nowMs) {
    const proto::Command& c = radio_.cmd();
    bool link = radio_.linkOk(nowMs);
    bool armBit  = link && (c.flags & proto::FLAG_ARM);
    bool stopBit = link && (c.flags & proto::FLAG_STOP);
    bool armEdge = false;
    if (link) { armEdge = armBit && !prevArmBit_; prevArmBit_ = armBit; }
    float tilt = fmaxf(fabsf(imu_.roll), fabsf(imu_.pitch));

    switch (state_) {
      case State::Disarmed:
        if (armEdge) tryArm(c, tilt);
        break;

      case State::Armed:
        if (stopBit)                         disarm("STOP khan cap");
        else if (!link) {
          state_ = State::Failsafe;
          fsStartMs_ = nowMs;
          fsStartThrottle_ = throttle_;
          Serial.println("!! Mat ket noi -> FAILSAFE: ha ga dan ve 0");
        }
        else if (!armBit)                    disarm("app tat arm");
        else if (tilt > cfg::CRASH_TILT_DEG) disarm("nghieng qua lon (roi/va cham)");
        break;

      case State::Failsafe:
        if (tilt > cfg::CRASH_TILT_DEG)      disarm("nghieng qua lon (roi/va cham)");
        else if (link && (stopBit || !armBit)) disarm("app tat arm / STOP");
        else if (link && armBit) { state_ = State::Armed; Serial.println("Co lai ket noi -> ARMED"); }
        break;

      case State::Error:
        break;
    }
  }

  void tryArm(const proto::Command& c, float tilt) {
    const char* why = nullptr;
    if (c.throttle > cfg::ARM_MAX_THROTTLE * 1000)  why = "ga chua ve 0";
    else if (tilt > cfg::ARM_MAX_TILT_DEG)          why = "drone khong nam phang";
    else if (battery_.volts() < cfg::ARM_MIN_VBAT)  why = "pin yeu / chua cam pin";
    if (why) { Serial.printf("Tu choi ARM: %s\n", why); return; }

    testMotor_ = -1;
    dryThrottle_ = 0;
    resetPids();
    state_ = State::Armed;
    reason_ = "";
    Serial.println(">> ARMED");
  }

  void disarm(const char* why) {
    motors_.stop();
    resetPids();
    throttle_ = 0;
    state_ = State::Disarmed;
    reason_ = why;
    Serial.printf(">> DISARMED: %s\n", why);
  }

  void enterError(const char* why) {
    motors_.stop();
    state_ = State::Error;
    reason_ = why;
    Serial.printf("!! LOI: %s\n", why);
  }

  // ---------------- Điều khiển ----------------
  void runControl(float dt, uint32_t nowMs) {
    const proto::Command& c = radio_.cmd();
    float thr = 0, sRoll = 0, sPitch = 0, sYaw = 0;   // cần: -1..1, ga: 0..1

    switch (state_) {
      case State::Armed:
        thr    = clampf(c.throttle / 1000.0f, 0, 1);
        sRoll  = clampf(c.roll  / 1000.0f, -1, 1);
        sPitch = clampf(c.pitch / 1000.0f, -1, 1);
        sYaw   = clampf(c.yaw   / 1000.0f, -1, 1);
        break;

      case State::Failsafe: {
        float k = 1.0f - (float)(nowMs - fsStartMs_) / cfg::FAILSAFE_DESCENT_MS;
        if (k <= 0) { disarm("failsafe: da ha xong"); return; }
        thr = fsStartThrottle_ * k;          // giữ thăng bằng (cần = 0), ga giảm dần
        break;
      }

      case State::Disarmed:
        runBenchTests(dt, nowMs);
        return;

      default:
        motors_.stop();
        return;
    }

    throttle_ = thr;
    if (thr < cfg::THROTTLE_MIN_ACTIVE) { motors_.stop(); resetPids(); return; }
    computeMix(dt, thr, sRoll, sPitch, sYaw);
    motors_.write(mixOut_);
  }

  // PID 2 tầng + mixer, kết quả để trong mixOut_
  void computeMix(float dt, float thr, float sRoll, float sPitch, float sYaw) {
    // Tầng ngoài (góc): sai số góc -> tốc độ xoay mong muốn
    float rollSet  = sRoll  * cfg::MAX_ANGLE_DEG;
    float pitchSet = sPitch * cfg::MAX_ANGLE_DEG;
    float rollRateSet  = clampf(g_.angleKp * (rollSet  - imu_.roll),  -cfg::MAX_RATE_DPS, cfg::MAX_RATE_DPS);
    float pitchRateSet = clampf(g_.angleKp * (pitchSet - imu_.pitch), -cfg::MAX_RATE_DPS, cfg::MAX_RATE_DPS);
    // Yaw: cần + = xoay phải = gz âm
    float yawRateSet = -sYaw * cfg::MAX_YAW_RATE_DPS;

    // Tầng trong (tốc độ xoay): sai số tốc độ -> chênh lệch lực motor
    bool integ = thr > cfg::THROTTLE_I_ENABLE;
    float r = pidRoll_.update(rollRateSet,  imu_.gx, dt, integ);
    float p = pidPitch_.update(pitchRateSet, imu_.gy, dt, integ);
    float y = pidYaw_.update(yawRateSet,    imu_.gz, dt, integ);

    mixQuadX(thr, r, p, y, mixOut_);
  }

  void runBenchTests(float dt, uint32_t nowMs) {
    if (testMotor_ >= 0) {
      if (nowMs - testStartMs_ > 10000) {
        testMotor_ = -1;
        Serial.println("Test motor: het 10 s, tu tat.");
      } else {
        float v[4] = {0, 0, 0, 0};
        v[testMotor_] = testDuty_;
        motors_.write(v);
        return;
      }
    }
    motors_.stop();
    if (dryThrottle_ > 0) computeMix(dt, dryThrottle_, 0, 0, 0);   // chỉ tính, không xuất PWM
    else resetPids();
  }

  void resetPids() { pidRoll_.reset(); pidPitch_.reset(); pidYaw_.reset(); }

  void applyGains() {
    Pid* rp[2] = { &pidRoll_, &pidPitch_ };
    for (Pid* q : rp) { q->kp = g_.rateKp; q->ki = g_.rateKi; q->kd = g_.rateKd; }
    pidYaw_.kp = g_.yawKp; pidYaw_.ki = g_.yawKi; pidYaw_.kd = 0;
    for (Pid* q : { &pidRoll_, &pidPitch_, &pidYaw_ }) {
      q->iLimit = cfg::PID_I_LIMIT;
      q->outLimit = cfg::PID_OUT_LIMIT;
    }
  }

  // ---------------- Telemetry / LED / đo tần số ----------------
  void sendTelemetry() {
    proto::Telemetry t{};
    t.state   = (uint8_t)state_;
    t.roll10  = (int16_t)(imu_.roll * 10);
    t.pitch10 = (int16_t)(imu_.pitch * 10);
    t.vbatMv  = (uint16_t)(battery_.volts() * 1000);
    t.flags   = battery_.low() ? proto::TEL_FLAG_LOW_BAT : 0;
    radio_.sendTelemetry(t);
  }

  void updateLed(uint32_t nowMs) {
    bool on;
    switch (state_) {
      case State::Armed:    on = true; break;                          // sáng liên tục
      case State::Failsafe: on = (nowMs / 100) % 2; break;             // nháy nhanh 5 Hz
      case State::Error:    on = (nowMs / 50) % 2; break;              // nháy rất nhanh
      default:
        on = radio_.linkOk(nowMs) ? (nowMs % 1000) < 100              // có app: chớp ngắn
                                  : (nowMs % 1000) < 500;             // chưa có app: nháy 1 Hz
    }
    digitalWrite(cfg::PIN_LED, on ? LOW : HIGH);
  }

  void measureLoopRate(uint32_t nowMs) {
    rateLoops_++;
    if (nowMs - rateWindowMs_ >= 1000) { loopHz_ = rateLoops_; rateLoops_ = 0; rateWindowMs_ = nowMs; }
  }

  // ---------------- Lệnh Serial ----------------
  void handleSerial() {
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\r') continue;
      if (ch == '\n') {
        line_[lineLen_] = 0;
        if (lineLen_) runCommand(line_);
        lineLen_ = 0;
      } else if (lineLen_ < sizeof(line_) - 1) {
        line_[lineLen_++] = ch;
      }
    }
  }

  struct Param { const char* name; float* value; };

  void runCommand(char* s) {
    char* cmd = strtok(s, " ");
    char* a1 = strtok(nullptr, " ");
    char* a2 = strtok(nullptr, " ");
    if (!cmd) return;

    if (!strcmp(cmd, "help")) {
      Serial.println(
        "Lenh:\n"
        "  status            - trang thai, goc, pin, ket noi, motor\n"
        "  log               - bat/tat in lien tuc (dung Serial Plotter)\n"
        "  gains             - in he so PID\n"
        "  set <ten> <gia>   - doi he so, vd: set rateKp 0.004\n"
        "  motor <1-4> <%>   - quay 1 motor (chi khi DISARMED, tu tat sau 10 s). THAO CANH!\n"
        "  motor off         - tat test motor\n"
        "  dry <%>           - chay PID+mixer o muc ga <%> nhung KHONG xuat PWM (kiem tra dau)\n"
        "  dry off\n"
        "  stop              - tat het motor, disarm");
    }
    else if (!strcmp(cmd, "status")) printStatus();
    else if (!strcmp(cmd, "log"))  { logOn_ = !logOn_; Serial.printf("log %s\n", logOn_ ? "ON" : "OFF"); }
    else if (!strcmp(cmd, "gains")) printGains();
    else if (!strcmp(cmd, "set")) {
      if (!a1 || !a2) { Serial.println("Cu phap: set <ten> <gia tri>"); return; }
      Param ps[] = {
        {"angleKp", &g_.angleKp}, {"rateKp", &g_.rateKp}, {"rateKi", &g_.rateKi},
        {"rateKd", &g_.rateKd},   {"yawKp", &g_.yawKp},   {"yawKi", &g_.yawKi},
      };
      for (auto& p : ps) {
        if (!strcmp(a1, p.name)) { *p.value = atof(a2); applyGains(); printGains(); return; }
      }
      Serial.println("Khong co he so nay. Go 'gains' de xem ten.");
    }
    else if (!strcmp(cmd, "motor")) {
      if (a1 && !strcmp(a1, "off")) { testMotor_ = -1; motors_.stop(); Serial.println("Test motor: tat"); return; }
      if (state_ != State::Disarmed) { Serial.println("Chi test motor khi DISARMED"); return; }
      int n = a1 ? atoi(a1) : 0;
      float pct = a2 ? atof(a2) : -1;
      if (n < 1 || n > 4 || pct < 0 || pct > 100) { Serial.println("Cu phap: motor <1-4> <0-100>"); return; }
      dryThrottle_ = 0;
      testMotor_ = n - 1; testDuty_ = pct / 100.0f; testStartMs_ = millis();
      Serial.printf("Test motor M%d = %.0f%% (tu tat sau 10 s)\n", n, pct);
    }
    else if (!strcmp(cmd, "dry")) {
      if (a1 && !strcmp(a1, "off")) { dryThrottle_ = 0; Serial.println("dry: tat"); return; }
      if (state_ != State::Disarmed) { Serial.println("Chi dung dry khi DISARMED"); return; }
      float pct = a1 ? atof(a1) : 0;
      dryThrottle_ = clampf(pct / 100.0f, 0, 1);
      testMotor_ = -1;
      Serial.printf("dry: ga %.0f%% (motor KHONG quay). Go 'log' de xem m1..m4\n", pct);
    }
    else if (!strcmp(cmd, "stop")) {
      testMotor_ = -1; dryThrottle_ = 0;
      if (state_ == State::Armed || state_ == State::Failsafe) disarm("lenh stop tu Serial");
      motors_.stop();
      Serial.println("Da tat motor.");
    }
    else Serial.println("Lenh khong ro. Go 'help'.");
  }

  void printGains() {
    Serial.printf("angleKp=%.3f rateKp=%.5f rateKi=%.5f rateKd=%.6f yawKp=%.5f yawKi=%.5f\n",
                  g_.angleKp, g_.rateKp, g_.rateKi, g_.rateKd, g_.yawKp, g_.yawKi);
  }

  void printStatus() {
    uint32_t nowMs = millis();
    Serial.printf("State: %s %s%s\n", stateName(state_), *reason_ ? "- " : "", reason_);
    Serial.printf("Vong lap: %lu Hz | Pin: %.2f V%s\n",
                  (unsigned long)loopHz_, battery_.volts(), battery_.low() ? " (YEU)" : "");
    Serial.printf("Goc: roll %.1f  pitch %.1f | gyro: %.1f %.1f %.1f dps\n",
                  imu_.roll, imu_.pitch, imu_.gx, imu_.gy, imu_.gz);
    const proto::Command& c = radio_.cmd();
    Serial.printf("Ket noi: %s | goi OK %lu, loi %lu | lenh: flags=%u thr=%u r=%d p=%d y=%d\n",
                  radio_.linkOk(nowMs) ? "CO" : "KHONG",
                  (unsigned long)radio_.rxCount, (unsigned long)radio_.badCount,
                  c.flags, c.throttle, c.roll, c.pitch, c.yaw);
    Serial.printf("Motor (%%): M1 %.0f  M2 %.0f  M3 %.0f  M4 %.0f\n",
                  motors_.out[0] * 100, motors_.out[1] * 100, motors_.out[2] * 100, motors_.out[3] * 100);
  }

  // Định dạng "ten:gia,ten:gia" cho Serial Plotter của Arduino IDE
  void printLog() {
    const float* m = (dryThrottle_ > 0) ? mixOut_ : motors_.out;
    Serial.printf("roll:%.1f,pitch:%.1f,gz:%.0f,m1:%.0f,m2:%.0f,m3:%.0f,m4:%.0f\n",
                  imu_.roll, imu_.pitch, imu_.gz, m[0] * 100, m[1] * 100, m[2] * 100, m[3] * 100);
  }
};
