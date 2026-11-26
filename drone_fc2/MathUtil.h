#ifndef DRONE_MATH_UTIL_H
#define DRONE_MATH_UTIL_H

// Small math helpers called very often in the 500 Hz loop -> kept inline in the header.
namespace mathutil
{

constexpr float PI_F = 3.14159265f;
// _F suffix: Arduino.h already defines PI and RAD_TO_DEG macros; reusing those names breaks the build
constexpr float RAD_TO_DEG_F = 57.2957795f;

inline float clamp(const float value, const float lo, const float hi)
{
    float result = value;
    if (result < lo)
    {
        result = lo;
    }
    else if (result > hi)
    {
        result = hi;
    }
    else
    {
        // within range: keep as is
    }
    return result;
}

inline float absf(const float value)
{
    return (value < 0.0f) ? -value : value;
}

inline float minf(const float a, const float b)
{
    return (a < b) ? a : b;
}

inline float maxf(const float a, const float b)
{
    return (a > b) ? a : b;
}

} // namespace mathutil

#endif // DRONE_MATH_UTIL_H
