#ifndef DRONE_BATTERY_H
#define DRONE_BATTERY_H

#include "config.h"

// ============================================================
//  Battery voltage via a 2 × 100 kΩ divider (midpoint -> D10)
// ============================================================
class Battery
{
public:
    void begin();
    void update(); // call ~10 times per second
    float volts() const;
    bool isLow() const;

private:
    float m_volts = 0.0f;
    bool m_first = true;
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace battery_detail
{

constexpr uint8_t ADC_RESOLUTION_BITS = 12U;
constexpr uint32_t SAMPLES = 4U;
constexpr float MV_PER_V = 1000.0f;
constexpr float SMOOTHING = 0.2f;

} // namespace battery_detail

inline void Battery::begin()
{
    analogReadResolution(battery_detail::ADC_RESOLUTION_BITS);
    update();
}

inline void Battery::update()
{
    uint32_t sumMv = 0U;
    for (uint32_t i = 0U; i < battery_detail::SAMPLES; ++i)
    {
        sumMv += static_cast<uint32_t>(analogReadMilliVolts(cfg::PIN_BAT));
    }
    const float midMv = static_cast<float>(sumMv) / static_cast<float>(battery_detail::SAMPLES);
    const float volts = (midMv * cfg::BAT_DIVIDER_RATIO) / battery_detail::MV_PER_V;
    if (m_first)
    {
        m_volts = volts;
        m_first = false;
    }
    else
    {
        m_volts += battery_detail::SMOOTHING * (volts - m_volts);
    }
}

inline float Battery::volts() const
{
    return m_volts;
}

inline bool Battery::isLow() const
{
    return m_volts < cfg::VBAT_LOW;
}

#endif // DRONE_BATTERY_H
