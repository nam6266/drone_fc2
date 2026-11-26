#ifndef DRONE_PROTOCOL_H
#define DRONE_PROTOCOL_H

#include <cstddef>
#include <cstdint>

// ============================================================
//  UDP protocol between app and drone (little-endian)
//  App -> drone: Command, 13 bytes, sent steadily 50 times/s
//  Drone -> app: Telemetry, 14 bytes, 10 times/s, sent back to the IP:port the Command came from
//
//  Command:   [0] magic 0xA5  [1] flags (bit0 ARM, bit1 STOP, bit2 MOTOR TEST,
//             bits4-7 motor-test mask M1..M4)  [2-3] seq u16
//             [4-5] roll i16  [6-7] pitch i16  [8-9] yaw i16   [10-11] throttle u16
//             [12] XOR of bytes 0..11
//  Telemetry: [0] magic 0x5A  [1] state  [2-3] seq u16  [4-5] roll×10 i16
//             [6-7] pitch×10 i16  [8-9] battery mV u16  [10] flags (bit0 low battery,
//             bit1 MPU not found, bit2 gyro calibrating, bit3 motor test running)
//             [11-12] chip temperature °C×10 i16 (-32768 = unknown)  [13] XOR of bytes 0..12
//
//  Each field is encoded/decoded by hand (no struct-to-bytes casts),
//  so the result does not depend on compiler padding or endianness.
// ============================================================
namespace proto
{

constexpr size_t CMD_SIZE = 13U;
constexpr size_t TEL_SIZE = 14U;
constexpr int16_t TEL_TEMP_UNKNOWN = -32768; // temperature not read yet

struct Command
{
    bool arm = false;       // request to arm. Must go false -> true to arm
    bool stop = false;      // emergency stop
    uint16_t seq = 0U;      // +1 per packet
    int16_t roll = 0;       // -1000..1000, + = move RIGHT
    int16_t pitch = 0;      // -1000..1000, + = move FORWARD
    int16_t yaw = 0;        // -1000..1000, + = turn RIGHT
    uint16_t throttle = 0U; // 0..1000 (also the power of a motor test)
    bool motorTest = false; // ground test from the app: spin motors without IMU / arming
    uint8_t testMask = 0U;  // bit0..3 = M1..M4 to spin during the motor test
};

struct Telemetry
{
    uint8_t state = 0U; // 0 DISARMED, 1 ARMED, 2 FAILSAFE, 3 ERROR
    uint16_t seq = 0U;
    int16_t roll10 = 0;  // roll angle × 10 (deg)
    int16_t pitch10 = 0; // pitch angle × 10 (deg)
    uint16_t vbatMv = 0U;
    bool lowBattery = false;
    bool imuMissing = false;               // MPU6500 not found (arming blocked)
    bool imuCalibrating = false;           // gyro calibrating (arming blocked)
    bool motorTest = false;                // app motor test is spinning motors
    int16_t chipTemp10 = TEL_TEMP_UNKNOWN; // chip temperature × 10 (°C)
};

// Returns true if [buffer] (exactly CMD_SIZE bytes) is a valid packet.
bool decodeCommand(const uint8_t* buffer, size_t length, Command& out);

// Writes the packet into [buffer]; returns bytes written (0 if the buffer is too small).
size_t encodeTelemetry(const Telemetry& telemetry, uint8_t* buffer, size_t capacity);

// true if sequence [candidate] is newer than [reference] (handles 65535 -> 0 wrap).
bool isNewerSeq(uint16_t candidate, uint16_t reference);

uint8_t xorChecksum(const uint8_t* data, size_t count);

} // namespace proto

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace proto
{

namespace detail
{

constexpr uint8_t CMD_MAGIC = 0xA5U;
constexpr uint8_t TEL_MAGIC = 0x5AU;
constexpr uint8_t FLAG_ARM = 0x01U;
constexpr uint8_t FLAG_STOP = 0x02U;
constexpr uint8_t FLAG_MOTOR_TEST = 0x04U;
constexpr uint32_t TEST_MASK_SHIFT = 4U;
constexpr uint32_t TEST_MASK_BITS = 0x0FU;
constexpr uint8_t TEL_FLAG_LOW_BAT = 0x01U;
constexpr uint8_t TEL_FLAG_IMU_MISSING = 0x02U;
constexpr uint8_t TEL_FLAG_IMU_CALIBRATING = 0x04U;
constexpr uint8_t TEL_FLAG_MOTOR_TEST = 0x08U;
constexpr uint32_t SEQ_HALF_RANGE = 0x8000U;
constexpr uint32_t BYTE_MASK = 0xFFU;
constexpr uint32_t BITS_PER_BYTE = 8U;
constexpr uint32_t U16_MASK = 0xFFFFU;
constexpr uint32_t I16_SIGN_BIT = 0x8000U;
constexpr int32_t U16_RANGE = 0x10000;

// Field offsets in the Command packet
constexpr size_t CMD_OFS_MAGIC = 0U;
constexpr size_t CMD_OFS_FLAGS = 1U;
constexpr size_t CMD_OFS_SEQ = 2U;
constexpr size_t CMD_OFS_ROLL = 4U;
constexpr size_t CMD_OFS_PITCH = 6U;
constexpr size_t CMD_OFS_YAW = 8U;
constexpr size_t CMD_OFS_THROTTLE = 10U;
constexpr size_t CMD_OFS_CHECKSUM = CMD_SIZE - 1U;

// Field offsets in the Telemetry packet
constexpr size_t TEL_OFS_MAGIC = 0U;
constexpr size_t TEL_OFS_STATE = 1U;
constexpr size_t TEL_OFS_SEQ = 2U;
constexpr size_t TEL_OFS_ROLL = 4U;
constexpr size_t TEL_OFS_PITCH = 6U;
constexpr size_t TEL_OFS_VBAT = 8U;
constexpr size_t TEL_OFS_FLAGS = 10U;
constexpr size_t TEL_OFS_TEMP = 11U;
constexpr size_t TEL_OFS_CHECKSUM = TEL_SIZE - 1U;

inline uint16_t readU16(const uint8_t* buffer, const size_t offset)
{
    const uint32_t lo = static_cast<uint32_t>(buffer[offset]);
    const uint32_t hi = static_cast<uint32_t>(buffer[offset + 1U]);
    return static_cast<uint16_t>(lo | (hi << BITS_PER_BYTE));
}

inline int16_t readI16(const uint8_t* buffer, const size_t offset)
{
    const uint32_t raw = static_cast<uint32_t>(readU16(buffer, offset));
    const int32_t value =
        (raw >= I16_SIGN_BIT) ? (static_cast<int32_t>(raw) - U16_RANGE) : static_cast<int32_t>(raw);
    return static_cast<int16_t>(value);
}

inline void writeU16(uint8_t* buffer, const size_t offset, const uint16_t value)
{
    const uint32_t v = static_cast<uint32_t>(value);
    buffer[offset] = static_cast<uint8_t>(v & BYTE_MASK);
    buffer[offset + 1U] = static_cast<uint8_t>((v >> BITS_PER_BYTE) & BYTE_MASK);
}

inline void writeI16(uint8_t* buffer, const size_t offset, const int16_t value)
{
    // Signed -> unsigned 16-bit (two's complement), well defined by the C++ standard
    writeU16(buffer, offset, static_cast<uint16_t>(value));
}

} // namespace detail

inline uint8_t xorChecksum(const uint8_t* data, const size_t count)
{
    uint8_t sum = 0U;
    for (size_t i = 0U; i < count; ++i)
    {
        sum = static_cast<uint8_t>(sum ^ data[i]);
    }
    return sum;
}

inline bool decodeCommand(const uint8_t* buffer, const size_t length, Command& out)
{
    bool ok = (buffer != nullptr) && (length == CMD_SIZE);
    if (ok)
    {
        ok = (buffer[detail::CMD_OFS_MAGIC] == detail::CMD_MAGIC) &&
             (buffer[detail::CMD_OFS_CHECKSUM] == xorChecksum(buffer, detail::CMD_OFS_CHECKSUM));
    }
    if (ok)
    {
        const uint8_t flags = buffer[detail::CMD_OFS_FLAGS];
        out.arm = (flags & detail::FLAG_ARM) != 0U;
        out.stop = (flags & detail::FLAG_STOP) != 0U;
        out.motorTest = (flags & detail::FLAG_MOTOR_TEST) != 0U;
        out.testMask = static_cast<uint8_t>((static_cast<uint32_t>(flags) >> detail::TEST_MASK_SHIFT) &
                                            detail::TEST_MASK_BITS);
        out.seq = detail::readU16(buffer, detail::CMD_OFS_SEQ);
        out.roll = detail::readI16(buffer, detail::CMD_OFS_ROLL);
        out.pitch = detail::readI16(buffer, detail::CMD_OFS_PITCH);
        out.yaw = detail::readI16(buffer, detail::CMD_OFS_YAW);
        out.throttle = detail::readU16(buffer, detail::CMD_OFS_THROTTLE);
    }
    return ok;
}

inline size_t encodeTelemetry(const Telemetry& telemetry, uint8_t* buffer, const size_t capacity)
{
    size_t written = 0U;
    if ((buffer != nullptr) && (capacity >= TEL_SIZE))
    {
        buffer[detail::TEL_OFS_MAGIC] = detail::TEL_MAGIC;
        buffer[detail::TEL_OFS_STATE] = telemetry.state;
        detail::writeU16(buffer, detail::TEL_OFS_SEQ, telemetry.seq);
        detail::writeI16(buffer, detail::TEL_OFS_ROLL, telemetry.roll10);
        detail::writeI16(buffer, detail::TEL_OFS_PITCH, telemetry.pitch10);
        detail::writeU16(buffer, detail::TEL_OFS_VBAT, telemetry.vbatMv);
        uint8_t flags = 0U;
        if (telemetry.lowBattery)
        {
            flags |= detail::TEL_FLAG_LOW_BAT;
        }
        if (telemetry.imuMissing)
        {
            flags |= detail::TEL_FLAG_IMU_MISSING;
        }
        if (telemetry.imuCalibrating)
        {
            flags |= detail::TEL_FLAG_IMU_CALIBRATING;
        }
        if (telemetry.motorTest)
        {
            flags |= detail::TEL_FLAG_MOTOR_TEST;
        }
        buffer[detail::TEL_OFS_FLAGS] = flags;
        detail::writeI16(buffer, detail::TEL_OFS_TEMP, telemetry.chipTemp10);
        buffer[detail::TEL_OFS_CHECKSUM] = xorChecksum(buffer, detail::TEL_OFS_CHECKSUM);
        written = TEL_SIZE;
    }
    return written;
}

inline bool isNewerSeq(const uint16_t candidate, const uint16_t reference)
{
    const uint32_t diff =
        (static_cast<uint32_t>(candidate) - static_cast<uint32_t>(reference)) & detail::U16_MASK;
    return (diff != 0U) && (diff < detail::SEQ_HALF_RANGE);
}

} // namespace proto

#endif // DRONE_PROTOCOL_H
