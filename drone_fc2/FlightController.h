#ifndef DRONE_FLIGHT_CONTROLLER_H
#define DRONE_FLIGHT_CONTROLLER_H

#include <array>
#include <cmath>
#include <cstdint>
#include <esp_system.h>

#include "Battery.h"
#include "Imu.h"
#include "Mixer.h"
#include "Motors.h"
#include "Pid.h"
#include "PowerTest.h"
#include "Radio.h"
#include "MathUtil.h"
#include "config.h"

// ============================================================
//  Flight controller
//  Every 500 Hz loop: read IMU -> receive UDP command -> state machine -> cascaded PID -> mixer -> PWM
//  While DISARMED, bench tests can drive the motors instead: power test ("ptest", BOOT button),
//  Serial "motor" test, or the app motor test (TEST button).
//
//  FlightController.cpp    : loop, state machine, control
//  FlightControllerCli.cpp : commands typed in the Serial Monitor
// ============================================================
class FlightController
{
public:
    void begin();
    void loop();

private:
    enum class State : uint8_t
    {
        Disarmed = 0U,
        Armed = 1U,
        Failsafe = 2U,
        Error = 3U
    };

    // MPU6500 status. The XIAO keeps running (WiFi, app, Serial) in every status,
    // but arming is only allowed when Ready.
    enum class ImuStatus : uint8_t
    {
        NotFound = 0U,    // no answer on I2C -> retried every IMU_RETRY_MS
        Calibrating = 1U, // found, measuring gyro offsets (keep still ~2 s)
        Ready = 2U
    };

    // PID gains – conservative STARTING values, tune in test flights ("set" command)
    struct Gains
    {
        float angleKp = 4.0f;   // outer loop: 1 deg error -> request 4 deg/s
        float rateKp = 0.0030f; // inner roll/pitch loop: 1 deg/s error -> 0.3 % power
        float rateKi = 0.0040f;
        float rateKd = 0.00003f;
        float yawKp = 0.0020f; // yaw: rate control only
        float yawKi = 0.0005f;
    };

    // Normalised command: power 0..1, sticks -1..1
    struct Sticks
    {
        float throttle = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
    };

    static constexpr size_t CLI_LINE_SIZE = 64U;
    static constexpr int NO_TEST_MOTOR = -1;

    // ---- FlightController.cpp ----
    void serviceImu(float dt, uint32_t nowMs);
    void loseImu(const char* reason, uint32_t nowMs);
    bool imuReady() const;
    static const char* imuStatusName(ImuStatus status);
    void updateState(uint32_t nowMs);
    void tryArm(const proto::Command& command, float tiltDeg);
    void disarm(const char* reason);
    void enterError(const char* reason);
    void runControl(float dt, uint32_t nowMs);
    void computeMix(float dt, const Sticks& sticks);
    void runBenchTests(float dt, uint32_t nowMs);
    bool runAppMotorTest(float dt, uint32_t nowMs);
    bool runPowerTest(float dt, uint32_t nowMs);
    bool canStartPowerTest() const;
    void handleBootButton(uint32_t nowMs);
    void resetPids();
    void applyGains();
    void sendTelemetry();
    void readChipTemperature();
    void updateLed(uint32_t nowMs) const;
    void measureLoopRate(uint32_t nowMs);
    float tiltDeg() const;
    static const char* stateName(State state);

    // ---- FlightControllerCli.cpp ----
    void handleSerial();
    void runCommand(char* line);
    void cmdSet(const char* name, const char* value);
    void cmdMotor(const char* arg1, const char* arg2);
    void cmdDry(const char* arg1);
    void cmdStop();
    void cmdPowerTest(const text::Tokens& tokens, size_t count);
    static void printHelp();
    void printGains() const;
    void printStatus() const;
    void printLog() const;

    Imu m_imu;
    Motors m_motors;
    Battery m_battery;
    Radio m_radio;
    Pid m_pidRoll;
    Pid m_pidPitch;
    Pid m_pidYaw;
    Gains m_gains{};
    PowerTest m_powerTest;

    State m_state = State::Disarmed;
    ImuStatus m_imuStatus = ImuStatus::NotFound;
    uint32_t m_imuRetryMs = 0U;        // time of the last detection attempt
    bool m_imuMissingReported = false; // print "not found" once, not every second
    const char* m_reason = "startup";

    uint32_t m_lastUs = 0U;
    uint32_t m_loopCount = 0U;
    uint32_t m_rateWindowMs = 0U;
    uint32_t m_rateLoops = 0U;
    uint32_t m_loopHz = 0U;
    uint32_t m_imuErrors = 0U;

    // Chip temperature
    float m_chipTempC = 0.0f;
    bool m_hasChipTemp = false;
    uint32_t m_tempReadUs = 0U;  // duration of one read (µs)
    bool m_tempReadSlow = false; // true: reads are slow -> skip while flying

    bool m_prevArmBit = true; // true: the app must send arm=0 first, then 0->1 to arm
    float m_throttle = 0.0f;  // power in use (0..1)
    float m_failsafeStartThrottle = 0.0f;
    uint32_t m_failsafeStartMs = 0U;
    MotorOutputs m_mixOut{};

    // Bench tests (only while DISARMED)
    int m_testMotor = NO_TEST_MOTOR;
    float m_testDuty = 0.0f;
    uint32_t m_testStartMs = 0U;
    float m_dryThrottle = 0.0f; // > 0: run PID + mixer but do NOT output PWM

    // Motor test requested by the app
    bool m_appTestActive = false;
    bool m_appTestBlocked = false; // timed out: wait until the app releases the test
    float m_appTestLevel = 0.0f;   // current (ramped) power 0..1
    uint32_t m_appTestStartMs = 0U;

    // BOOT button (power test)
    bool m_buttonRaw = false;
    bool m_buttonStable = false;
    bool m_buttonHandled = false; // this press already started / stopped a power test
    uint32_t m_buttonChangeMs = 0U;

    bool m_logOn = false;
    std::array<char, CLI_LINE_SIZE> m_line{};
    size_t m_lineLength = 0U;
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace fc_detail
{

constexpr float US_TO_S = 1.0e-6f;
constexpr float CMD_SCALE = 1000.0f; // UDP command: -1000..1000 / 0..1000
constexpr float TELEMETRY_ANGLE_SCALE = 10.0f;
constexpr float V_TO_MV = 1000.0f;
constexpr uint32_t RATE_WINDOW_MS = 1000U;
constexpr float TEMP_SCALE = 10.0f;
constexpr float TEMP_MIN_C = -100.0f; // limit so that ×10 still fits in int16
constexpr float TEMP_MAX_C = 200.0f;

// LED blink timing (ms)
constexpr uint32_t LED_FAILSAFE_HALF_MS = 100U;
constexpr uint32_t LED_ERROR_HALF_MS = 50U;
constexpr uint32_t LED_IDLE_PERIOD_MS = 1000U;
constexpr uint32_t LED_LINKED_ON_MS = 100U;
constexpr uint32_t LED_UNLINKED_ON_MS = 500U;
constexpr uint32_t LED_DOUBLE_FLASH_MS = 100U;
constexpr uint32_t LED_POWER_TEST_HALF_MS =
    250U; // power test running: 2 Hz blink // IMU not ready: on-off-on, 100 ms each

inline bool blinkPhase(const uint32_t nowMs, const uint32_t halfPeriodMs)
{
    return ((nowMs / halfPeriodMs) % 2U) == 1U;
}

} // namespace fc_detail

// ============================================================
//  Startup
// ============================================================
inline void FlightController::begin()
{
    const bool motorsOk = m_motors.begin(); // first thing: force all 4 motors to 0
    const uint32_t motorsReadyMs = millis();
    pinMode(cfg::PIN_LED, OUTPUT);
    digitalWrite(cfg::PIN_LED, HIGH);
    Serial.begin(cfg::SERIAL_BAUD);
    delay(cfg::STARTUP_DELAY_MS);
    Serial.println("\n=== Drone FC – XIAO ESP32-S3 ===");
    Serial.printf("Motor pins: LOW before setup() (EarlyInit), PWM ready %lu ms after app start\n",
                  static_cast<unsigned long>(motorsReadyMs));
    Serial.printf("Last reset: %s\n", resetlog::reasonText(esp_reset_reason()));
    m_powerTest.begin(); // reports a power test cut short by this reset
    pinMode(cfg::PIN_BOOT_BUTTON, INPUT_PULLUP);

    if (!motorsOk)
    {
        enterError("Could not initialise motor PWM");
    }
    else
    {
        m_battery.begin();
        // WiFi first: the app can connect even if the MPU is missing or still calibrating
        if (m_radio.begin())
        {
            Serial.printf("WiFi: \"%s\"  password \"%s\"  IP %s  UDP %u\n", cfg::WIFI_SSID, cfg::WIFI_PASS,
                          m_radio.ip().toString().c_str(), static_cast<unsigned>(cfg::UDP_PORT));
        }
        else
        {
            enterError("Could not start WiFi AP");
        }
    }

    if (m_state != State::Error)
    {
        m_imu.initBus();
        m_imuStatus = ImuStatus::NotFound;
        m_imuRetryMs = millis() - cfg::IMU_RETRY_MS; // try to find the MPU on the first loop
    }

    applyGains();
    Serial.println("Type 'help' for commands.");
    m_lastUs = micros();
}

// ============================================================
//  IMU: detect -> calibrate -> ready (never blocks the loop for long)
// ============================================================
inline void FlightController::serviceImu(const float dt, const uint32_t nowMs)
{
    switch (m_imuStatus)
    {
        case ImuStatus::NotFound:
            if ((nowMs - m_imuRetryMs) >= cfg::IMU_RETRY_MS)
            {
                m_imuRetryMs = nowMs;
                if (m_imu.detect())
                {
                    m_imu.startCalibration();
                    m_imuStatus = ImuStatus::Calibrating;
                    m_imuMissingReported = false;
                    Serial.println("MPU6500 found. Calibrating gyro: keep the drone STILL for ~2 s...");
                }
                else if (!m_imuMissingReported)
                {
                    m_imuMissingReported = true;
                    Serial.println("!! MPU6500 not found - check I2C wiring. Retrying every second; "
                                   "flying is blocked until it answers.");
                }
                else
                {
                    // keep retrying quietly
                }
            }
            break;

        case ImuStatus::Calibrating:
            switch (m_imu.stepCalibration())
            {
                case Imu::CalibStep::Done:
                    m_imuStatus = ImuStatus::Ready;
                    m_imuErrors = 0U;
                    Serial.println("  Gyro calibrated. IMU ready.");
                    break;
                case Imu::CalibStep::Moving:
                    Serial.println("  Board is moving/vibrating, calibration restarted...");
                    m_imu.startCalibration();
                    break;
                case Imu::CalibStep::ReadError:
                    loseImu("MPU6500 stopped answering during calibration", nowMs);
                    break;
                case Imu::CalibStep::Running:
                default:
                    break;
            }
            break;

        case ImuStatus::Ready:
        default:
            if (m_imu.update(dt))
            {
                m_imuErrors = 0U;
            }
            else
            {
                ++m_imuErrors;
                if (m_imuErrors >= cfg::IMU_MAX_CONSECUTIVE_ERRORS)
                {
                    loseImu("Lost MPU6500 connection", nowMs);
                }
            }
            break;
    }
}

inline void FlightController::loseImu(const char* reason, const uint32_t nowMs)
{
    Serial.printf("!! %s\n", reason);
    if ((m_state == State::Armed) || (m_state == State::Failsafe))
    {
        disarm(reason); // no attitude data -> motors off
    }
    m_dryThrottle = 0.0f;
    m_imuStatus = ImuStatus::NotFound;
    m_imuErrors = 0U;
    m_imuRetryMs = nowMs;
    m_imuMissingReported = true; // the reason was just printed
}

inline bool FlightController::imuReady() const
{
    return m_imuStatus == ImuStatus::Ready;
}

inline const char* FlightController::imuStatusName(const ImuStatus status)
{
    const char* name = "READY";
    switch (status)
    {
        case ImuStatus::NotFound:
            name = "NOT FOUND (retrying)";
            break;
        case ImuStatus::Calibrating:
            name = "CALIBRATING (keep still)";
            break;
        case ImuStatus::Ready:
        default:
            break;
    }
    return name;
}

// ============================================================
//  Main loop
// ============================================================
inline void FlightController::loop()
{
    handleSerial();

    const uint32_t nowUs = micros();
    if ((nowUs - m_lastUs) >= cfg::LOOP_US)
    {
        const float dt =
            mathutil::clamp(static_cast<float>(nowUs - m_lastUs) * fc_detail::US_TO_S, 0.0f, cfg::MAX_DT_S);
        m_lastUs = nowUs;
        const uint32_t nowMs = millis();
        ++m_loopCount;
        measureLoopRate(nowMs);

        if (m_state == State::Error)
        {
            m_motors.stop();
        }
        else
        {
            serviceImu(dt, nowMs);
        }

        if (m_state != State::Error)
        {
            m_radio.poll(nowMs);
            if ((m_loopCount % cfg::BATTERY_EVERY_N_LOOPS) == 0U)
            {
                m_battery.update();
            }
            if ((m_loopCount % cfg::TEMP_EVERY_N_LOOPS) == cfg::TEMP_PHASE)
            {
                readChipTemperature();
            }

            handleBootButton(nowMs);
            updateState(nowMs);
            runControl(dt, nowMs);

            if ((m_loopCount % cfg::TELEMETRY_EVERY_N_LOOPS) == cfg::TELEMETRY_PHASE)
            {
                sendTelemetry();
            }
            if (m_logOn && ((m_loopCount % cfg::LOG_EVERY_N_LOOPS) == 0U))
            {
                printLog();
            }
        }
        updateLed(nowMs);
    }
}

// ============================================================
//  State machine
// ============================================================
inline void FlightController::updateState(const uint32_t nowMs)
{
    const proto::Command& command = m_radio.command();
    const bool link = m_radio.linkOk(nowMs);
    const bool armBit = link && command.arm;
    const bool stopBit = link && command.stop;
    bool armEdge = false;
    if (link)
    {
        armEdge = armBit && (!m_prevArmBit);
        m_prevArmBit = armBit;
    }
    const float tilt = tiltDeg();

    switch (m_state)
    {
        case State::Disarmed:
            if (armEdge)
            {
                tryArm(command, tilt);
            }
            break;

        case State::Armed:
            if (stopBit)
            {
                disarm("emergency STOP");
            }
            else if (!link)
            {
                m_state = State::Failsafe;
                m_failsafeStartMs = nowMs;
                m_failsafeStartThrottle = m_throttle;
                Serial.println("!! Link lost -> FAILSAFE: ramping power down to 0");
            }
            else if (!armBit)
            {
                disarm("app released arm");
            }
            else if (tilt > cfg::CRASH_TILT_DEG)
            {
                disarm("excessive tilt (fall/crash)");
            }
            else
            {
                // normal flight
            }
            break;

        case State::Failsafe:
            if (tilt > cfg::CRASH_TILT_DEG)
            {
                disarm("excessive tilt (fall/crash)");
            }
            else if (link && (stopBit || (!armBit)))
            {
                disarm("app released arm / STOP");
            }
            else if (link && armBit)
            {
                m_state = State::Armed;
                Serial.println("Link restored -> ARMED");
            }
            else
            {
                // still no link: keep descending
            }
            break;

        case State::Error:
        default:
            break;
    }
}

inline void FlightController::tryArm(const proto::Command& command, const float tiltDeg)
{
    const char* why = nullptr;
    if (command.motorTest)
    {
        why = "motor test is running";
    }
    else if (m_powerTest.isRunning())
    {
        why = "power test is running";
    }
    else if (m_imuStatus == ImuStatus::NotFound)
    {
        why = "MPU6500 not found - check I2C wiring";
    }
    else if (m_imuStatus == ImuStatus::Calibrating)
    {
        why = "gyro still calibrating - keep the drone still";
    }
    else if ((static_cast<float>(command.throttle) / fc_detail::CMD_SCALE) > cfg::ARM_MAX_THROTTLE)
    {
        why = "power not at 0";
    }
    else if (tiltDeg > cfg::ARM_MAX_TILT_DEG)
    {
        why = "drone is not level";
    }
    else if (m_battery.volts() < cfg::ARM_MIN_VBAT)
    {
        why = "battery low / not connected";
    }
    else
    {
        // all conditions met
    }

    if (why != nullptr)
    {
        Serial.printf("ARM refused: %s\n", why);
    }
    else
    {
        m_testMotor = NO_TEST_MOTOR;
        m_dryThrottle = 0.0f;
        resetPids();
        m_state = State::Armed;
        m_reason = "";
        Serial.println(">> ARMED");
    }
}

inline void FlightController::disarm(const char* reason)
{
    m_motors.stop();
    resetPids();
    m_throttle = 0.0f;
    m_state = State::Disarmed;
    m_reason = reason;
    Serial.printf(">> DISARMED: %s\n", reason);
}

inline void FlightController::enterError(const char* reason)
{
    m_powerTest.stop("flight controller error");
    m_motors.stop();
    m_state = State::Error;
    m_reason = reason;
    Serial.printf("!! ERROR: %s\n", reason);
}

// ============================================================
//  Control
// ============================================================
inline void FlightController::runControl(const float dt, const uint32_t nowMs)
{
    const proto::Command& command = m_radio.command();
    Sticks sticks{};
    bool fly = false;

    switch (m_state)
    {
        case State::Armed:
            sticks.throttle =
                mathutil::clamp(static_cast<float>(command.throttle) / fc_detail::CMD_SCALE, 0.0f, 1.0f);
            sticks.roll =
                mathutil::clamp(static_cast<float>(command.roll) / fc_detail::CMD_SCALE, -1.0f, 1.0f);
            sticks.pitch =
                mathutil::clamp(static_cast<float>(command.pitch) / fc_detail::CMD_SCALE, -1.0f, 1.0f);
            sticks.yaw = mathutil::clamp(static_cast<float>(command.yaw) / fc_detail::CMD_SCALE, -1.0f, 1.0f);
            fly = true;
            break;

        case State::Failsafe:
        {
            const float elapsed = static_cast<float>(nowMs - m_failsafeStartMs);
            const float remaining = 1.0f - (elapsed / static_cast<float>(cfg::FAILSAFE_DESCENT_MS));
            if (remaining <= 0.0f)
            {
                disarm("failsafe: descent finished");
            }
            else
            {
                sticks.throttle = m_failsafeStartThrottle * remaining; // stay level, power ramps down
                fly = true;
            }
            break;
        }

        case State::Disarmed:
            runBenchTests(dt, nowMs);
            break;

        case State::Error:
        default:
            m_motors.stop();
            break;
    }

    if (fly)
    {
        m_throttle = sticks.throttle;
        if (sticks.throttle < cfg::THROTTLE_MIN_ACTIVE)
        {
            m_motors.stop();
            resetPids();
        }
        else
        {
            computeMix(dt, sticks);
            m_motors.write(m_mixOut);
        }
    }
}

// Cascaded PID + mixer, result stored in m_mixOut
inline void FlightController::computeMix(const float dt, const Sticks& sticks)
{
    // Outer loop (angle): angle error -> desired rotation rate
    const float rollSet = sticks.roll * cfg::MAX_ANGLE_DEG;
    const float pitchSet = sticks.pitch * cfg::MAX_ANGLE_DEG;
    const float rollRateSet =
        mathutil::clamp(m_gains.angleKp * (rollSet - m_imu.roll()), -cfg::MAX_RATE_DPS, cfg::MAX_RATE_DPS);
    const float pitchRateSet =
        mathutil::clamp(m_gains.angleKp * (pitchSet - m_imu.pitch()), -cfg::MAX_RATE_DPS, cfg::MAX_RATE_DPS);
    // Yaw: stick + = turn right = negative yawRate
    const float yawRateSet = -sticks.yaw * cfg::MAX_YAW_RATE_DPS;

    // Inner loop (rate): rate error -> motor thrust difference
    const bool integrate = sticks.throttle > cfg::THROTTLE_I_ENABLE;
    mixer::AxisCommand axis{};
    axis.roll = m_pidRoll.update(rollRateSet, m_imu.rollRate(), dt, integrate);
    axis.pitch = m_pidPitch.update(pitchRateSet, m_imu.pitchRate(), dt, integrate);
    axis.yaw = m_pidYaw.update(yawRateSet, m_imu.yawRate(), dt, integrate);

    m_mixOut = mixer::mixQuadX(sticks.throttle, axis);
}

inline void FlightController::runBenchTests(const float dt, const uint32_t nowMs)
{
    bool testing = false;
    if (m_powerTest.isRunning())
    {
        testing = runPowerTest(dt, nowMs);
    }
    else if ((m_testMotor == NO_TEST_MOTOR) && runAppMotorTest(dt, nowMs))
    {
        testing = true;
    }
    else if (m_testMotor != NO_TEST_MOTOR)
    {
        if ((nowMs - m_testStartMs) > cfg::MOTOR_TEST_TIMEOUT_MS)
        {
            m_testMotor = NO_TEST_MOTOR;
            Serial.println("Motor test: 10 s elapsed, stopped.");
        }
        else
        {
            MotorOutputs values{};
            values[static_cast<size_t>(m_testMotor)] = m_testDuty;
            m_motors.write(values);
            testing = true;
        }
    }

    if (!testing)
    {
        m_motors.stop();
        if ((m_dryThrottle > 0.0f) && imuReady())
        {
            Sticks sticks{};
            sticks.throttle = m_dryThrottle;
            computeMix(dt, sticks); // compute only, no PWM output
        }
        else
        {
            resetPids();
        }
    }
}

// Motor test from the app: spins the selected motors at the requested power.
// No IMU, no arming, no PID – like the Serial "motor" command, but works on battery only.
// Stops as soon as the app releases the button, the link drops, STOP is pressed or after 15 s.
inline bool FlightController::runAppMotorTest(const float dt, const uint32_t nowMs)
{
    const proto::Command& command = m_radio.command();
    const bool requested =
        m_radio.linkOk(nowMs) && command.motorTest && (!command.stop) && (command.testMask != 0U);

    if (!requested)
    {
        m_appTestBlocked = false;
    }
    bool run = requested && (!m_appTestBlocked);

    if (run && (!m_appTestActive))
    {
        m_appTestActive = true;
        m_appTestLevel = 0.0f;
        m_appTestStartMs = nowMs;
        Serial.println("App motor test: START");
    }
    if (run && ((nowMs - m_appTestStartMs) > cfg::APP_TEST_MAX_MS))
    {
        m_appTestBlocked = true;
        run = false;
        Serial.println("App motor test: 15 s limit reached, release the button to test again");
    }
    if ((!run) && m_appTestActive)
    {
        m_appTestActive = false;
        m_appTestLevel = 0.0f;
        Serial.println("App motor test: STOP");
    }

    if (run)
    {
        const float target =
            mathutil::clamp(static_cast<float>(command.throttle) / fc_detail::CMD_SCALE, 0.0f, 1.0f);
        const float maxStep = cfg::APP_TEST_RAMP_PER_S * dt;
        m_appTestLevel += mathutil::clamp(target - m_appTestLevel, -maxStep, maxStep);

        MotorOutputs values{};
        for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
        {
            if (((static_cast<uint32_t>(command.testMask) >> i) & 1U) != 0U)
            {
                values[i] = m_appTestLevel;
            }
        }
        m_motors.write(values);
    }
    return run;
}

// Power test (see PowerTest.h). STOP in the app also stops it.
inline bool FlightController::runPowerTest(const float dt, const uint32_t nowMs)
{
    const proto::Command& command = m_radio.command();
    if (m_radio.linkOk(nowMs) && command.stop)
    {
        m_powerTest.stop("STOP from the app");
    }
    MotorOutputs values{};
    m_powerTest.update(dt, nowMs, values);
    m_motors.write(values);
    return true;
}

// A power test may start only when nothing else is driving the motors
inline bool FlightController::canStartPowerTest() const
{
    return (m_state == State::Disarmed) && (!m_appTestActive) && (m_testMotor == NO_TEST_MOTOR) &&
           (m_dryThrottle <= 0.0f);
}

// BOOT button: hold PTEST_BUTTON_HOLD_MS while DISARMED = start the auto power test;
// any press while a power test runs = stop it.
inline void FlightController::handleBootButton(const uint32_t nowMs)
{
    if (cfg::PTEST_BUTTON_ENABLED)
    {
        const bool pressed = (digitalRead(cfg::PIN_BOOT_BUTTON) == LOW);
        if (pressed != m_buttonRaw)
        {
            m_buttonRaw = pressed;
            m_buttonChangeMs = nowMs;
        }
        else if (((nowMs - m_buttonChangeMs) >= cfg::PTEST_BUTTON_DEBOUNCE_MS) && (pressed != m_buttonStable))
        {
            m_buttonStable = pressed;
            m_buttonHandled = false;
            if (pressed && m_powerTest.isRunning())
            {
                m_powerTest.stop("stopped with the BOOT button");
                m_buttonHandled = true;
            }
        }
        else if (m_buttonStable && (!m_buttonHandled) &&
                 ((nowMs - m_buttonChangeMs) >= cfg::PTEST_BUTTON_HOLD_MS))
        {
            m_buttonHandled = true; // one action per press
            if (canStartPowerTest())
            {
                static_cast<void>(m_powerTest.startAuto(nowMs));
            }
            else
            {
                Serial.println("BOOT held: power test only while DISARMED with no other motor test running.");
            }
        }
        else
        {
            // no change, still bouncing, or waiting for the hold time
        }
    }
}

inline void FlightController::resetPids()
{
    m_pidRoll.reset();
    m_pidPitch.reset();
    m_pidYaw.reset();
}

inline void FlightController::applyGains()
{
    m_pidRoll.setGains(m_gains.rateKp, m_gains.rateKi, m_gains.rateKd);
    m_pidPitch.setGains(m_gains.rateKp, m_gains.rateKi, m_gains.rateKd);
    m_pidYaw.setGains(m_gains.yawKp, m_gains.yawKi, 0.0f);
    m_pidRoll.setLimits(cfg::PID_I_LIMIT, cfg::PID_OUT_LIMIT);
    m_pidPitch.setLimits(cfg::PID_I_LIMIT, cfg::PID_OUT_LIMIT);
    m_pidYaw.setLimits(cfg::PID_I_LIMIT, cfg::PID_OUT_LIMIT);
}

// ============================================================
//  Telemetry / LED / loop rate
// ============================================================
inline void FlightController::sendTelemetry()
{
    proto::Telemetry telemetry{};
    telemetry.state = static_cast<uint8_t>(m_state);
    if (imuReady())
    {
        telemetry.roll10 = static_cast<int16_t>(m_imu.roll() * fc_detail::TELEMETRY_ANGLE_SCALE);
        telemetry.pitch10 = static_cast<int16_t>(m_imu.pitch() * fc_detail::TELEMETRY_ANGLE_SCALE);
    }
    telemetry.imuMissing = (m_imuStatus == ImuStatus::NotFound);
    telemetry.imuCalibrating = (m_imuStatus == ImuStatus::Calibrating);
    telemetry.motorTest = m_appTestActive || m_powerTest.isRunning(); // app shows "motors running"
    telemetry.vbatMv = static_cast<uint16_t>(m_battery.volts() * fc_detail::V_TO_MV);
    telemetry.lowBattery = m_battery.isLow();
    if (m_hasChipTemp)
    {
        const float celsius = mathutil::clamp(m_chipTempC, fc_detail::TEMP_MIN_C, fc_detail::TEMP_MAX_C);
        telemetry.chipTemp10 = static_cast<int16_t>(celsius * fc_detail::TEMP_SCALE);
    }
    m_radio.sendTelemetry(telemetry);
}

inline void FlightController::readChipTemperature()
{
    const bool flying = (m_state == State::Armed) || (m_state == State::Failsafe);
    // On the ground always read (to re-measure timing). In flight read only if known to be fast.
    if ((!flying) || (!m_tempReadSlow))
    {
        const uint32_t startUs = micros();
        const float celsius = temperatureRead();
        m_tempReadUs = micros() - startUs;
        m_tempReadSlow = (m_tempReadUs > cfg::TEMP_READ_MAX_IN_FLIGHT_US);
        if (std::isfinite(celsius))
        {
            m_chipTempC = celsius;
            m_hasChipTemp = true;
        }
    }
}

inline void FlightController::updateLed(const uint32_t nowMs) const
{
    bool on = false;
    switch (m_state)
    {
        case State::Armed:
            on = true; // solid on
            break;
        case State::Failsafe:
            on = fc_detail::blinkPhase(nowMs, fc_detail::LED_FAILSAFE_HALF_MS); // fast blink 5 Hz
            break;
        case State::Error:
            on = fc_detail::blinkPhase(nowMs, fc_detail::LED_ERROR_HALF_MS); // very fast blink
            break;
        case State::Disarmed:
        default:
        {
            const uint32_t phaseMs = nowMs % fc_detail::LED_IDLE_PERIOD_MS;
            if (m_powerTest.isRunning())
            {
                on = fc_detail::blinkPhase(nowMs, fc_detail::LED_POWER_TEST_HALF_MS); // 2 Hz blink
            }
            else if (!imuReady())
            {
                // MPU missing / calibrating: double flash each second
                on = (phaseMs < fc_detail::LED_DOUBLE_FLASH_MS) ||
                     ((phaseMs >= (2U * fc_detail::LED_DOUBLE_FLASH_MS)) &&
                      (phaseMs < (3U * fc_detail::LED_DOUBLE_FLASH_MS)));
            }
            else
            {
                // app connected: short flash each second; no app: 1 Hz blink
                const uint32_t onMs =
                    m_radio.linkOk(nowMs) ? fc_detail::LED_LINKED_ON_MS : fc_detail::LED_UNLINKED_ON_MS;
                on = phaseMs < onMs;
            }
            break;
        }
    }
    digitalWrite(cfg::PIN_LED, on ? LOW : HIGH);
}

inline void FlightController::measureLoopRate(const uint32_t nowMs)
{
    ++m_rateLoops;
    if ((nowMs - m_rateWindowMs) >= fc_detail::RATE_WINDOW_MS)
    {
        m_loopHz = m_rateLoops;
        m_rateLoops = 0U;
        m_rateWindowMs = nowMs;
    }
}

inline float FlightController::tiltDeg() const
{
    return mathutil::maxf(mathutil::absf(m_imu.roll()), mathutil::absf(m_imu.pitch()));
}

inline const char* FlightController::stateName(const State state)
{
    const char* name = "ERROR";
    switch (state)
    {
        case State::Disarmed:
            name = "DISARMED";
            break;
        case State::Armed:
            name = "ARMED";
            break;
        case State::Failsafe:
            name = "FAILSAFE";
            break;
        case State::Error:
        default:
            break;
    }
    return name;
}

// Serial commands: the rest of FlightController's member functions
#include "FlightControllerCli.h"

#endif // DRONE_FLIGHT_CONTROLLER_H
