#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

#include <Arduino.h>
#include <array>
#include <cstddef>
#include <cstdint>

// ============================================================
//  GLOBAL CONFIGURATION – tune every constant here
// ============================================================
namespace cfg
{

// ---------- XIAO ESP32-S3 pins ----------
constexpr size_t MOTOR_COUNT = 4U;
// M1 front-right (CCW), M2 rear-right (CW), M3 rear-left (CCW), M4 front-left (CW)
const std::array<uint8_t, MOTOR_COUNT> PIN_MOTOR = {D0, D1, D2, D3};
const uint8_t PIN_SDA = D4;
const uint8_t PIN_SCL = D5;
const uint8_t PIN_BAT = D10;
constexpr uint8_t PIN_LED = 21U;          // yellow on-board LED, lit when LOW
constexpr uint8_t PIN_BOOT_BUTTON = 0U;   // BOOT button on the XIAO, reads LOW when pressed
constexpr float BAT_DIVIDER_RATIO = 2.0f; // battery divider: two equal 100 kΩ resistors

// ---------- Startup ----------
constexpr uint32_t SERIAL_BAUD = 115200U;
constexpr uint32_t STARTUP_DELAY_MS = 1500U;

// ---------- Control loop ----------
constexpr uint32_t LOOP_US = 2000U;               // 500 Hz
constexpr float MAX_DT_S = 0.01f;                 // clamp larger dt (avoids PID kicks)
constexpr uint32_t BATTERY_EVERY_N_LOOPS = 50U;   // 10 Hz
constexpr uint32_t TELEMETRY_EVERY_N_LOOPS = 50U; // 10 Hz
constexpr uint32_t TELEMETRY_PHASE = 25U;         // offset from the battery reading
constexpr uint32_t LOG_EVERY_N_LOOPS = 25U;       // 20 Hz
constexpr uint32_t TEMP_EVERY_N_LOOPS = 1000U;    // read chip temperature every 2 s
constexpr uint32_t TEMP_PHASE = 10U;              // offset from battery / telemetry
// If one temperature read takes longer than this (µs), do NOT read while flying,
// so the 2 ms control loop is not delayed. Actual read time: see the "status" command.
constexpr uint32_t TEMP_READ_MAX_IN_FLIGHT_US = 1000U;
constexpr uint32_t IMU_MAX_CONSECUTIVE_ERRORS = 10U;

// ---------- IMU ----------
constexpr uint32_t I2C_CLOCK_HZ = 400000U;
constexpr uint16_t I2C_TIMEOUT_MS = 5U;  // max wait for one I2C transfer
constexpr float IMU_TAU_S = 0.5f;        // complementary filter time constant
constexpr uint32_t IMU_RETRY_MS = 1000U; // MPU not found: try again every second

// ---------- Motor PWM ----------
constexpr uint32_t PWM_FREQ_HZ = 20000U; // 20 kHz (above audible range)
constexpr uint8_t PWM_BITS = 10U;        // resolution 0..1023
constexpr uint32_t PWM_MAX = (1U << PWM_BITS) - 1U;

// ---------- WiFi / UDP ----------
// The XIAO runs its own WiFi (Access Point). The phone joins it; XIAO IP = 192.168.4.1
constexpr const char* WIFI_SSID = "XIAO-Drone";
constexpr const char* WIFI_PASS = "drone1234"; // at least 8 characters
constexpr int WIFI_CHANNEL = 6;
constexpr int WIFI_HIDDEN = 0;
constexpr int WIFI_MAX_CLIENTS = 2;
constexpr uint16_t UDP_PORT = 4210U;
constexpr uint32_t MAX_PACKETS_PER_POLL = 8U; // max packets read per loop

// WiFi transmit power in 0.25 dBm units (ESP32 wifi_power_t enum).
// Lower power -> less heat, less battery drain, smaller voltage dips. A few metres indoors is enough.
// Valid levels: 78 (19.5 dBm, default), 68 (17), 60 (15), 52 (13), 44 (11),
//               34 (8.5), 28 (7), 20 (5), 8 (2)
// If the app often reports signal loss at a distance -> raise to 44 or 52.
constexpr int WIFI_TX_POWER_QUARTER_DBM = 34; // 8.5 dBm

// ---------- Safety ----------
constexpr uint32_t LINK_TIMEOUT_MS = 300U;      // no packet for this long -> link lost
constexpr uint32_t FAILSAFE_DESCENT_MS = 2000U; // link lost: ramp power to 0 over 2 s, then disarm
constexpr float ARM_MAX_THROTTLE = 0.05f;       // arm only when power < 5 %
constexpr float ARM_MAX_TILT_DEG = 15.0f;       // arm only when the drone is nearly level
constexpr float CRASH_TILT_DEG = 60.0f;         // excessive tilt -> treat as crash, stop motors
constexpr float ARM_MIN_VBAT = 3.5f;            // refuse to arm below this battery voltage
                                                // (set 0 to test arming on USB power only)
constexpr float VBAT_LOW = 3.3f;                // low-battery warning (sent to the app)

// ---------- Flight limits ----------
constexpr float MAX_ANGLE_DEG = 20.0f;       // max tilt at full stick
constexpr float MAX_RATE_DPS = 250.0f;       // max roll/pitch rate the outer loop may request
constexpr float MAX_YAW_RATE_DPS = 150.0f;   // max yaw rate
constexpr float THROTTLE_MIN_ACTIVE = 0.05f; // below this power: motors off, PID idle
constexpr float THROTTLE_I_ENABLE = 0.20f;   // integrate only above this power
constexpr float PID_I_LIMIT = 0.15f;         // I-term limit (fraction of power 0..1)
constexpr float PID_OUT_LIMIT = 0.40f;       // total output limit per axis
constexpr float PID_D_CUTOFF_HZ = 30.0f;     // low-pass cutoff for the D-term
constexpr float MIX_YAW_SIGN = 1.0f;         // set to -1 if yaw turns out to be reversed

// ---------- Bench tests ----------
constexpr uint32_t MOTOR_TEST_TIMEOUT_MS = 10000U;
// Motor test from the app (works without IMU / arming, DISARMED only, needs a live link)
constexpr float APP_TEST_RAMP_PER_S = 2.0f;  // power change limit: 0 -> 100 % in 0.5 s
constexpr uint32_t APP_TEST_MAX_MS = 15000U; // stop after 15 s; the app must release and press again

// ---------- Power test ("ptest" commands, BOOT button) – REMOVE THE PROPS ----------
// Hold the BOOT button for PTEST_BUTTON_HOLD_MS while DISARMED = automatic power test.
// Set PTEST_BUTTON_ENABLED = false to disable the button (Serial "ptest" commands still work).
constexpr bool PTEST_BUTTON_ENABLED = true;
constexpr uint32_t PTEST_BUTTON_HOLD_MS = 3000U;
constexpr uint32_t PTEST_BUTTON_DEBOUNCE_MS = 50U;
constexpr float PTEST_RAMP_PER_S = 2.0f;          // 0 -> 100 % in 0.5 s
constexpr uint32_t PTEST_IDLE_MEASURE_MS = 2000U; // resting voltage, motors off
constexpr uint32_t PTEST_SETTLE_MS = 300U;        // ignore samples right after a power change
constexpr uint32_t PTEST_DEFAULT_HOLD_MS = 1500U; // measuring time per step
constexpr uint32_t PTEST_MIN_HOLD_MS = 500U;
constexpr uint32_t PTEST_MAX_HOLD_MS = 10000U; // never run one step longer than 10 s
constexpr uint32_t PTEST_MAX_TEST_MS = 60000U; // whole test limit
constexpr size_t PTEST_MAX_STEPS = 20U;
constexpr int32_t PTEST_MIN_PERCENT = 1;
constexpr int32_t PTEST_MAX_PERCENT = 100;
constexpr uint8_t PTEST_AUTO_MASK = 0x0FU; // auto test: all motors, 10 -> 60 % in 10 % steps
constexpr int32_t PTEST_AUTO_FROM_PERCENT = 10;
constexpr int32_t PTEST_AUTO_TO_PERCENT = 60;
constexpr int32_t PTEST_AUTO_STEP_PERCENT = 10;
constexpr float PTEST_DEFAULT_ABORT_AVG_V = 3.3f; // a step AVERAGE below this stops the test
constexpr float PTEST_MIN_ABORT_AVG_V = 2.8f;
constexpr float PTEST_MAX_ABORT_AVG_V = 3.8f;
constexpr float PTEST_VERDICT_LOW_AVG_V = 3.5f;      // hint: average below = real voltage drop
constexpr float PTEST_VERDICT_NOISE_SPREAD_V = 0.3f; // hint: max - min above = electrical noise

} // namespace cfg

// Output of the 4 motors, 0..1
using MotorOutputs = std::array<float, cfg::MOTOR_COUNT>;

#endif // DRONE_CONFIG_H
