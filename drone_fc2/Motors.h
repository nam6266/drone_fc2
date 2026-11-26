#ifndef DRONE_MOTORS_H
#define DRONE_MOTORS_H

#include <cmath>

#include "MathUtil.h"
#include "config.h"

// ============================================================
//  4 PWM channels -> MOSFET gates. Values 0..1 (0 = off, 1 = full power)
// ============================================================
class Motors
{
public:
    bool begin();
    void write(const MotorOutputs& values);
    void stop();
    const MotorOutputs& outputs() const; // values currently output (debug / telemetry)

private:
    MotorOutputs m_out{};
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
inline bool Motors::begin()
{
    bool ok = true;
    for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
    {
        if (!ledcAttach(cfg::PIN_MOTOR[i], cfg::PWM_FREQ_HZ, cfg::PWM_BITS))
        //        e.g.  ledcAttach(D0,  20000,          10);
        {
            ok = false;
        }
    }
    stop();
    return ok;
}

inline void Motors::write(const MotorOutputs& values)
{
    for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
    {
        m_out[i] = mathutil::clamp(values[i], 0.0f, 1.0f);
        const uint32_t duty = static_cast<uint32_t>(std::lround(m_out[i] * static_cast<float>(cfg::PWM_MAX)));
        static_cast<void>(ledcWrite(cfg::PIN_MOTOR[i], duty));
    }
}

inline void Motors::stop()
{
    const MotorOutputs zero{};
    write(zero);
}

inline const MotorOutputs& Motors::outputs() const
{
    return m_out;
}

#endif // DRONE_MOTORS_H
