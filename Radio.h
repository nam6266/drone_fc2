#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

// ============================================================
//  Giao thức UDP giữa app và drone (little-endian, không padding)
//  App -> drone: Command, 13 byte, gửi đều 50 lần/giây
//  Drone -> app: Telemetry, 12 byte, 10 lần/giây, gửi về đúng IP:port đã gửi Command
// ============================================================
namespace proto {

const uint8_t CMD_MAGIC = 0xA5;
const uint8_t TEL_MAGIC = 0x5A;

const uint8_t FLAG_ARM  = 0x01;   // 1 = muốn arm. Phải chuyển 0 -> 1 mới arm được
const uint8_t FLAG_STOP = 0x02;   // 1 = dừng khẩn cấp, tắt motor ngay

const uint8_t TEL_FLAG_LOW_BAT = 0x01;

#pragma pack(push, 1)
struct Command {
  uint8_t  magic;      // = CMD_MAGIC
  uint8_t  flags;      // FLAG_ARM | FLAG_STOP
  uint16_t seq;        // tăng 1 mỗi gói
  int16_t  roll;       // -1000..1000, + = bay sang PHẢI
  int16_t  pitch;      // -1000..1000, + = bay TỚI
  int16_t  yaw;        // -1000..1000, + = xoay PHẢI (theo chiều kim đồng hồ nhìn từ trên)
  uint16_t throttle;   // 0..1000
  uint8_t  checksum;   // XOR của 12 byte phía trước
};

struct Telemetry {
  uint8_t  magic;      // = TEL_MAGIC
  uint8_t  state;      // 0 DISARMED, 1 ARMED, 2 FAILSAFE, 3 ERROR
  uint16_t seq;
  int16_t  roll10;     // góc roll × 10 (độ)
  int16_t  pitch10;    // góc pitch × 10 (độ)
  uint16_t vbatMv;     // điện áp pin (mV)
  uint8_t  flags;      // TEL_FLAG_LOW_BAT
  uint8_t  checksum;   // XOR của 11 byte phía trước
};
#pragma pack(pop)

static_assert(sizeof(Command) == 13, "Command phai 13 byte");
static_assert(sizeof(Telemetry) == 12, "Telemetry phai 12 byte");

inline uint8_t xorSum(const uint8_t* p, size_t n) {
  uint8_t s = 0;
  while (n--) s ^= *p++;
  return s;
}

} // namespace proto

class Radio {
public:
  bool begin() {
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(cfg::WIFI_SSID, cfg::WIFI_PASS, cfg::WIFI_CHANNEL, 0, 2)) return false;
    WiFi.setSleep(false);
    return udp_.begin(cfg::UDP_PORT);
  }

  IPAddress ip() const { return WiFi.softAPIP(); }

  // Gọi mỗi vòng lặp: đọc hết các gói đang chờ, giữ lại gói hợp lệ mới nhất
  void poll(uint32_t nowMs) {
    while (udp_.parsePacket() > 0) {
      proto::Command c;
      int n = udp_.read((uint8_t*)&c, sizeof(c));
      bool sizeOk = (n == (int)sizeof(c)) && (udp_.available() == 0);
      IPAddress ip = udp_.remoteIP();
      uint16_t port = udp_.remotePort();
      udp_.flush();   // bỏ phần còn lại của gói, sẵn sàng cho gói sau

      if (!sizeOk || c.magic != proto::CMD_MAGIC ||
          c.checksum != proto::xorSum((const uint8_t*)&c, sizeof(c) - 1)) {
        badCount++;
        continue;
      }
      // Bỏ gói cũ đến muộn (UDP có thể đảo thứ tự). Khi đã mất kết nối thì nhận lại từ đầu.
      if (linkOk(nowMs) && (int16_t)(c.seq - cmd_.seq) <= 0) continue;

      cmd_ = c;
      lastRxMs_ = nowMs;
      hasRx_ = true;
      remoteIp_ = ip;
      remotePort_ = port;
      rxCount++;
    }
  }

  bool linkOk(uint32_t nowMs) const { return hasRx_ && (nowMs - lastRxMs_) < cfg::LINK_TIMEOUT_MS; }
  const proto::Command& cmd() const { return cmd_; }

  void sendTelemetry(proto::Telemetry t) {
    if (!hasRx_) return;
    t.magic = proto::TEL_MAGIC;
    t.seq = telSeq_++;
    t.checksum = proto::xorSum((const uint8_t*)&t, sizeof(t) - 1);
    udp_.beginPacket(remoteIp_, remotePort_);
    udp_.write((const uint8_t*)&t, sizeof(t));
    udp_.endPacket();
  }

  uint32_t rxCount = 0, badCount = 0;

private:
  WiFiUDP udp_;
  proto::Command cmd_{};
  uint32_t lastRxMs_ = 0;
  bool hasRx_ = false;
  IPAddress remoteIp_;
  uint16_t remotePort_ = 0;
  uint16_t telSeq_ = 0;
};
