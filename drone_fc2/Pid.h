#ifndef DRONE_PID_H
#define DRONE_PID_H

#include "MathUtil.h"
#include "config.h"

// ============================================================
//  Single-axis PID
//  - I is clamped (anti-windup) and can be paused
//  - D acts on the MEASUREMENT (not the error) -> no kick when the setpoint changes
//  - D is low-pass filtered because differentiation amplifies vibration noise
// ============================================================
class Pid
{
public:
    void setGains(float kp, float ki, float kd);
    void setLimits(float iLimit, float outLimit);

    float update(float setpoint, float measured, float dt, bool integrate);
    void reset();

    // Last computed terms (for debugging while tuning)
    float pTerm() const;
    float iTerm() const;
    float dTerm() const;

private:
    float m_kp = 0.0f;
    float m_ki = 0.0f;
    float m_kd = 0.0f;
    float m_iLimit = 0.0f;
    float m_outLimit = 0.0f;
    float m_pTerm = 0.0f;
    float m_iTerm = 0.0f;
    float m_dTerm = 0.0f;
    float m_prevMeasured = 0.0f;
    float m_dFiltered = 0.0f;
    bool m_first = true;
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
inline void Pid::setGains(const float kp, const float ki, const float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

inline void Pid::setLimits(const float iLimit, const float outLimit)
{
    m_iLimit = iLimit;
    m_outLimit = outLimit;
}

inline float Pid::update(const float setpoint, const float measured, const float dt, const bool integrate)
{
    const float error = setpoint - measured;
    m_pTerm = m_kp * error;

    if (integrate)
    {
        m_iTerm = mathutil::clamp(m_iTerm + (m_ki * error * dt), -m_iLimit, m_iLimit);
    }

    float dRaw = 0.0f;
    if (!m_first)
    {
        dRaw = -(measured - m_prevMeasured) / dt;
    }
    m_prevMeasured = measured;
    m_first = false;

    const float rc = 1.0f / (2.0f * mathutil::PI_F * cfg::PID_D_CUTOFF_HZ);
    m_dFiltered += (dt / (rc + dt)) * (dRaw - m_dFiltered);
    m_dTerm = m_kd * m_dFiltered;

    return mathutil::clamp(m_pTerm + m_iTerm + m_dTerm, -m_outLimit, m_outLimit);
}

inline void Pid::reset()
{
    m_pTerm = 0.0f;
    m_iTerm = 0.0f;
    m_dTerm = 0.0f;
    m_dFiltered = 0.0f;
    m_first = true;
}

inline float Pid::pTerm() const
{
    return m_pTerm;
}

inline float Pid::iTerm() const
{
    return m_iTerm;
}

inline float Pid::dTerm() const
{
    return m_dTerm;
}

#endif // DRONE_PID_H
