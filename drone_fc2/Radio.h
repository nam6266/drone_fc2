#ifndef DRONE_RADIO_H
#define DRONE_RADIO_H

#include <WiFi.h>
#include <WiFiUdp.h>
#include <array>
#include <cstdint>

#include "Protocol.h"
#include "config.h"

// ============================================================
//  WiFi Access Point + UDP: receives Command, sends Telemetry
// ============================================================
class Radio
{
public:
    bool begin();
    IPAddress ip() const;
    float txPowerDbm() const; // current WiFi transmit power (dBm)

    // Call every loop: reads pending packets, keeps the newest valid one
    void poll(uint32_t nowMs);

    bool linkOk(uint32_t nowMs) const;
    const proto::Command& command() const;
    void sendTelemetry(proto::Telemetry telemetry);

    uint32_t rxCount() const;
    uint32_t badCount() const;

private:
    WiFiUDP m_udp;
    proto::Command m_command{};
    IPAddress m_remoteIp;
    uint32_t m_lastRxMs = 0U;
    uint32_t m_rxCount = 0U;
    uint32_t m_badCount = 0U;
    uint16_t m_remotePort = 0U;
    uint16_t m_telemetrySeq = 0U;
    bool m_hasRx = false;
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace radio_detail
{

constexpr float QUARTER_DBM_PER_DBM = 4.0f;

} // namespace radio_detail

inline bool Radio::begin()
{
    static_cast<void>(WiFi.mode(WIFI_AP));
    //create network "XIAO-Drone", password "drone1234", channel "6", hidden "0", max clien "2"
    bool ok = WiFi.softAP(cfg::WIFI_SSID, cfg::WIFI_PASS, cfg::WIFI_CHANNEL, cfg::WIFI_HIDDEN,
                          cfg::WIFI_MAX_CLIENTS);
    if (ok)
    {
        // Power must be set AFTER the AP is up, otherwise it has no effect
        if (!WiFi.setTxPower(static_cast<wifi_power_t>(cfg::WIFI_TX_POWER_QUARTER_DBM)))
        {
            Serial.println("Warning: could not set WiFi power, using default");
        }
        // Keep WiFi awake: lower command latency (at the cost of more power)
        static_cast<void>(WiFi.setSleep(false));
        ok = (m_udp.begin(cfg::UDP_PORT) != 0U);
    }
    return ok;
}

inline IPAddress Radio::ip() const
{
    return WiFi.softAPIP();
}

inline float Radio::txPowerDbm() const
{
    const int quarterDbm = static_cast<int>(WiFi.getTxPower());
    return static_cast<float>(quarterDbm) / radio_detail::QUARTER_DBM_PER_DBM;
}

inline void Radio::poll(const uint32_t nowMs)
{
    // Read 1 extra byte to detect oversized packets
    std::array<uint8_t, proto::CMD_SIZE + 1U> buffer{};
    bool more = true;

    for (uint32_t i = 0U; (i < cfg::MAX_PACKETS_PER_POLL) && more; ++i)
    {
        more = (m_udp.parsePacket() > 0);
        if (more)
        {
            const int length = m_udp.read(buffer.data(), buffer.size());
            const IPAddress ip = m_udp.remoteIP();
            const uint16_t port = m_udp.remotePort();
            m_udp.flush(); // drop the rest of the packet, ready for the next one

            proto::Command command{};
            const bool valid = (length == static_cast<int>(proto::CMD_SIZE)) &&
                               proto::decodeCommand(buffer.data(), proto::CMD_SIZE, command);
            // UDP may reorder: drop late old packets. After a link loss, accept from scratch.
            const bool fresh = (!linkOk(nowMs)) || proto::isNewerSeq(command.seq, m_command.seq);

            if (!valid)
            {
                ++m_badCount;
            }
            else if (fresh)
            {
                m_command = command;
                m_lastRxMs = nowMs;
                m_hasRx = true;
                m_remoteIp = ip;
                m_remotePort = port;
                ++m_rxCount;
            }
            else
            {
                // old packet: ignore
            }
        }
    }
}

inline bool Radio::linkOk(const uint32_t nowMs) const
{
    return m_hasRx && ((nowMs - m_lastRxMs) < cfg::LINK_TIMEOUT_MS);
}

inline const proto::Command& Radio::command() const
{
    return m_command;
}

inline void Radio::sendTelemetry(proto::Telemetry telemetry)
{
    if (m_hasRx)
    {
        telemetry.seq = m_telemetrySeq;
        ++m_telemetrySeq;
        std::array<uint8_t, proto::TEL_SIZE> buffer{};
        const size_t length = proto::encodeTelemetry(telemetry, buffer.data(), buffer.size());
        if (length > 0U)
        {
            static_cast<void>(m_udp.beginPacket(m_remoteIp, m_remotePort));
            static_cast<void>(m_udp.write(buffer.data(), length));
            static_cast<void>(m_udp.endPacket());
        }
    }
}

inline uint32_t Radio::rxCount() const
{
    return m_rxCount;
}

inline uint32_t Radio::badCount() const
{
    return m_badCount;
}

#endif // DRONE_RADIO_H
