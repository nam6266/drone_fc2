#ifndef DRONE_IMU_H
#define DRONE_IMU_H

#include <Wire.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "MathUtil.h"
#include "config.h"

// ============================================================
//  MPU6500 (GY-521 module) – direct register access + complementary filter
//  Sign convention (module X arrow points to the nose, chip facing up):
//    roll  > 0 : banked RIGHT (right side down)
//    pitch > 0 : NOSE DOWN
//    rollRate > 0 : roll increasing,  pitchRate > 0 : pitch increasing
//    yawRate  > 0 : nose turning LEFT (counter-clockwise seen from above)
// ============================================================
class Imu
{
public:
    // Result of one gyro-calibration step (see stepCalibration)
    enum class CalibStep : uint8_t
    {
        Running = 0U,  // still collecting samples
        Done = 1U,     // offsets stored, IMU ready to fly
        Moving = 2U,   // board moved/vibrated: call startCalibration() again
        ReadError = 3U // I2C read failed: sensor lost
    };

    // Start the I2C bus. Call once at startup.
    void initBus();

    // Wake up and configure the sensor. Returns false if it does not answer.
    // Safe to call again and again (the flight controller retries every second).
    bool detect();

    // Non-blocking gyro calibration: call startCalibration() once, then stepCalibration()
    // once per control loop (one sample each). Keep the board STILL for ~2 s.
    void startCalibration();
    CalibStep stepCalibration();

    bool update(float dt);

    float roll() const;      // degrees, filtered
    float pitch() const;     // degrees, filtered
    float rollRate() const;  // deg/s
    float pitchRate() const; // deg/s
    float yawRate() const;   // deg/s

private:
    static constexpr size_t AXES = 3U;

    bool readRaw();
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void computeGravityAngles();

    std::array<int64_t, AXES> m_calibSum{};
    std::array<int64_t, AXES> m_calibSumSq{};
    uint32_t m_calibCount = 0U;

    std::array<int16_t, AXES> m_rawAcc{};
    std::array<int16_t, AXES> m_rawGyro{};
    std::array<float, AXES> m_gyroOffset{};
    std::array<float, AXES> m_rate{};
    float m_roll = 0.0f;
    float m_pitch = 0.0f;
    float m_rollByGravity = 0.0f;
    float m_pitchByGravity = 0.0f;
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace imu_detail
{

constexpr uint8_t MPU_I2C_ADDR = 0x68U;

// MPU6500 registers
constexpr uint8_t MPU_REG_CONFIG = 0x1AU;        // gyro filter
constexpr uint8_t MPU_REG_GYRO_CONFIG = 0x1BU;   // gyro range
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1CU;  // accel range
constexpr uint8_t MPU_REG_ACCEL_CONFIG2 = 0x1DU; // accel filter
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3BU;  // start of the 14-byte data block
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6BU;
constexpr uint8_t MPU_REG_WHO_AM_I = 0x75U;

constexpr uint8_t VAL_WAKE_UP = 0x00U;
constexpr uint8_t INVALID_REGISTER = 0xFFU;
constexpr uint8_t VAL_DLPF_41HZ = 0x03U;
constexpr uint8_t VAL_GYRO_500DPS = 0x08U;
constexpr uint8_t VAL_ACCEL_4G = 0x08U;

constexpr float GYRO_LSB_PER_DPS = 65.5f; // ±500 dps
constexpr float ACC_LSB_PER_G = 8192.0f;  // ±4 g

constexpr uint8_t DATA_BLOCK_SIZE = 14U;
constexpr uint32_t WAKE_UP_DELAY_MS = 100U;
constexpr uint32_t CALIB_SAMPLES = 1000U; // one per 2 ms control loop -> ~2 s
constexpr double CALIB_MAX_STD_DPS = 2.0; // more jitter -> being held/vibrating

// Combine 2 bytes (high byte first) into a signed 16-bit value
constexpr uint32_t BYTE_MASK = 0xFFU;
constexpr uint32_t BITS_PER_BYTE = 8U;
constexpr uint32_t I16_SIGN_BIT = 0x8000U;
constexpr int32_t U16_RANGE = 0x10000;

inline int16_t toInt16(const int high, const int low)
{
    const uint32_t raw = ((static_cast<uint32_t>(high) & BYTE_MASK) << BITS_PER_BYTE) |
                         (static_cast<uint32_t>(low) & BYTE_MASK);
    const int32_t value =
        (raw >= I16_SIGN_BIT) ? (static_cast<int32_t>(raw) - U16_RANGE) : static_cast<int32_t>(raw);
    return static_cast<int16_t>(value);
}

} // namespace imu_detail

inline void Imu::initBus()
{
    static_cast<void>(Wire.begin(static_cast<int>(cfg::PIN_SDA), static_cast<int>(cfg::PIN_SCL)));
    static_cast<void>(Wire.setClock(cfg::I2C_CLOCK_HZ));
    Wire.setTimeOut(cfg::I2C_TIMEOUT_MS); // a missing/stuck sensor must not freeze the loop
}

inline bool Imu::detect()
{
    bool found = false;
    writeRegister(imu_detail::MPU_REG_PWR_MGMT_1, imu_detail::VAL_WAKE_UP);
    const uint8_t who = readRegister(imu_detail::MPU_REG_WHO_AM_I);
    if (who != imu_detail::INVALID_REGISTER)
    {
        delay(imu_detail::WAKE_UP_DELAY_MS);
        Serial.printf("MPU WHO_AM_I = 0x%02X\n", static_cast<unsigned>(who)); // MPU6500 = 0x70
        writeRegister(imu_detail::MPU_REG_CONFIG, imu_detail::VAL_DLPF_41HZ);
        writeRegister(imu_detail::MPU_REG_GYRO_CONFIG, imu_detail::VAL_GYRO_500DPS);
        writeRegister(imu_detail::MPU_REG_ACCEL_CONFIG, imu_detail::VAL_ACCEL_4G);
        writeRegister(imu_detail::MPU_REG_ACCEL_CONFIG2, imu_detail::VAL_DLPF_41HZ);
        found = readRaw();
    }
    return found;
}

inline void Imu::startCalibration()
{
    m_calibSum = {};
    m_calibSumSq = {};
    m_calibCount = 0U;
}

inline Imu::CalibStep Imu::stepCalibration()
{
    CalibStep result = CalibStep::Running;
    if (!readRaw())
    {
        result = CalibStep::ReadError;
    }
    else
    {
        for (size_t k = 0U; k < AXES; ++k)
        {
            const int64_t v = static_cast<int64_t>(m_rawGyro[k]);
            m_calibSum[k] += v;
            m_calibSumSq[k] += v * v;
        }
        ++m_calibCount;
    }

    if ((result == CalibStep::Running) && (m_calibCount >= imu_detail::CALIB_SAMPLES))
    {
        const double n = static_cast<double>(m_calibCount);
        std::array<float, AXES> offset{};
        bool still = true;
        for (size_t k = 0U; (k < AXES) && still; ++k)
        {
            const double mean = static_cast<double>(m_calibSum[k]) / n;
            const double variance = (static_cast<double>(m_calibSumSq[k]) / n) - (mean * mean);
            const double stdDps = std::sqrt((variance > 0.0) ? variance : 0.0) /
                                  static_cast<double>(imu_detail::GYRO_LSB_PER_DPS);
            if (stdDps > imu_detail::CALIB_MAX_STD_DPS)
            {
                still = false;
            }
            else
            {
                offset[k] = static_cast<float>(mean);
            }
        }

        if (still)
        {
            m_gyroOffset = offset;
            m_rate = {};
            computeGravityAngles(); // start the filter from the current gravity angles
            m_roll = m_rollByGravity;
            m_pitch = m_pitchByGravity;
            result = CalibStep::Done;
        }
        else
        {
            result = CalibStep::Moving;
        }
    }
    return result;
}

inline bool Imu::update(const float dt)
{
    const bool ok = readRaw();
    if (ok)
    {
        for (size_t k = 0U; k < AXES; ++k)
        {
            m_rate[k] = (static_cast<float>(m_rawGyro[k]) - m_gyroOffset[k]) / imu_detail::GYRO_LSB_PER_DPS;
        }
        computeGravityAngles();
        // Complementary filter: trust the gyro short-term, gravity long-term
        const float alpha = cfg::IMU_TAU_S / (cfg::IMU_TAU_S + dt);
        m_roll = (alpha * (m_roll + (m_rate[0] * dt))) + ((1.0f - alpha) * m_rollByGravity);
        m_pitch = (alpha * (m_pitch + (m_rate[1] * dt))) + ((1.0f - alpha) * m_pitchByGravity);
    }
    return ok;
}

inline float Imu::roll() const
{
    return m_roll;
}

inline float Imu::pitch() const
{
    return m_pitch;
}

inline float Imu::rollRate() const
{
    return m_rate[0];
}

inline float Imu::pitchRate() const
{
    return m_rate[1];
}

inline float Imu::yawRate() const
{
    return m_rate[2];
}

inline void Imu::writeRegister(const uint8_t reg, const uint8_t value)
{
    Wire.beginTransmission(imu_detail::MPU_I2C_ADDR);
    static_cast<void>(Wire.write(reg));
    static_cast<void>(Wire.write(value));
    static_cast<void>(Wire.endTransmission());
}

inline uint8_t Imu::readRegister(const uint8_t reg)
{
    uint8_t value = imu_detail::INVALID_REGISTER;
    Wire.beginTransmission(imu_detail::MPU_I2C_ADDR);
    static_cast<void>(Wire.write(reg));
    if (Wire.endTransmission(false) == 0U)
    {
        if (Wire.requestFrom(imu_detail::MPU_I2C_ADDR, static_cast<uint8_t>(1U)) == 1U)
        {
            value = static_cast<uint8_t>(Wire.read());
        }
    }
    return value;
}

inline bool Imu::readRaw()
{
    bool ok = false;
    Wire.beginTransmission(imu_detail::MPU_I2C_ADDR);
    static_cast<void>(Wire.write(imu_detail::MPU_REG_ACCEL_XOUT_H));
    if (Wire.endTransmission(false) == 0U)
    {
        if (Wire.requestFrom(imu_detail::MPU_I2C_ADDR, imu_detail::DATA_BLOCK_SIZE) ==
            imu_detail::DATA_BLOCK_SIZE)
        {
            for (size_t k = 0U; k < AXES; ++k)
            {
                const int high = Wire.read();
                const int low = Wire.read();
                m_rawAcc[k] = imu_detail::toInt16(high, low);
            }
            static_cast<void>(Wire.read()); // temperature (ignored)
            static_cast<void>(Wire.read());
            for (size_t k = 0U; k < AXES; ++k)
            {
                const int high = Wire.read();
                const int low = Wire.read();
                m_rawGyro[k] = imu_detail::toInt16(high, low);
            }
            ok = true;
        }
    }
    return ok;
}

inline void Imu::computeGravityAngles()
{
    const float ax = static_cast<float>(m_rawAcc[0]) / imu_detail::ACC_LSB_PER_G;
    const float ay = static_cast<float>(m_rawAcc[1]) / imu_detail::ACC_LSB_PER_G;
    const float az = static_cast<float>(m_rawAcc[2]) / imu_detail::ACC_LSB_PER_G;
    m_rollByGravity = std::atan2(ay, az) * mathutil::RAD_TO_DEG_F;
    m_pitchByGravity = std::atan2(-ax, std::sqrt((ay * ay) + (az * az))) * mathutil::RAD_TO_DEG_F;
}

#endif // DRONE_IMU_H
