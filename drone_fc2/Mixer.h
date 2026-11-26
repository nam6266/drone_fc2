#ifndef DRONE_MIXER_H
#define DRONE_MIXER_H

#include "MathUtil.h"
#include "config.h"

// ============================================================
//  Mixer quad X
//
//            NOSE
//   M4 (CW)  o      o  M1 (CCW)
//              \    /
//               [XX]
//              /    \ .
//   M3 (CCW) o      o  M2 (CW)
//
//  roll  > 0 : want roll to increase (bank right) -> lower right motors, raise left motors
//  pitch > 0 : want pitch to increase (nose down) -> lower front motors, raise rear motors
//  yaw   > 0 : want yawRate to increase (nose left) -> raise CW motors (the reaction torque
//                                                 of CW props turns the body CCW)
// ============================================================
namespace mixer
{

struct AxisCommand
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

MotorOutputs mixQuadX(float throttle, const AxisCommand& axis);

} // namespace mixer

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace mixer
{

inline MotorOutputs mixQuadX(const float throttle, const AxisCommand& axis)
{
    const float r = axis.roll;
    const float p = axis.pitch;
    const float y = axis.yaw * cfg::MIX_YAW_SIGN;

    MotorOutputs out{};
    out[0] = throttle - r - p - y; // M1 front-right CCW
    out[1] = throttle - r + p + y; // M2 rear-right  CW
    out[2] = throttle + r + p - y; // M3 rear-left   CCW
    out[3] = throttle + r - p + y; // M4 front-left  CW

    // If any motor exceeds 100 %: lower all 4 equally to keep the differences (keeps attitude)
    float highest = out[0];
    for (size_t i = 1U; i < cfg::MOTOR_COUNT; ++i)
    {
        highest = mathutil::maxf(highest, out[i]);
    }
    if (highest > 1.0f)
    {
        const float excess = highest - 1.0f;
        for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
        {
            out[i] -= excess;
        }
    }

    for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
    {
        out[i] = mathutil::clamp(out[i], 0.0f, 1.0f);
    }
    return out;
}

} // namespace mixer

#endif // DRONE_MIXER_H
