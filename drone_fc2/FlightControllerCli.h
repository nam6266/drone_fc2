#ifndef DRONE_FLIGHT_CONTROLLER_CLI_H
#define DRONE_FLIGHT_CONTROLLER_CLI_H

// Commands typed in the Serial Monitor (115200 baud, line ending "New Line").
// Part of FlightController: included at the end of FlightController.h.

#include "FlightController.h"
#include "MathUtil.h"
#include "TextUtil.h"

namespace fccli_detail
{

constexpr float PERCENT = 100.0f;
constexpr int32_t MOTOR_NUMBER_MIN = 1;
constexpr int32_t MOTOR_NUMBER_MAX = 4;

struct GainParam
{
    const char* name;
    float* value;
};

} // namespace fccli_detail

inline void FlightController::handleSerial()
{
    bool more = (Serial.available() > 0);
    while (more)
    {
        const int c = Serial.read();
        if (c < 0)
        {
            more = false;
        }
        else
        {
            const char ch = static_cast<char>(c);
            if (ch == '\n')
            {
                m_line[m_lineLength] = '\0';
                if (m_lineLength > 0U)
                {
                    runCommand(m_line.data());
                }
                m_lineLength = 0U;
            }
            else if ((ch != '\r') && (m_lineLength < (CLI_LINE_SIZE - 1U)))
            {
                m_line[m_lineLength] = ch;
                ++m_lineLength;
            }
            else
            {
                // drop '\r' or characters beyond the line length
            }
            more = (Serial.available() > 0);
        }
    }
}

inline void FlightController::runCommand(char* line)
{
    text::Tokens tokens{};
    const size_t count = text::split(line, CLI_LINE_SIZE, tokens);
    const char* const cmd = tokens[0];
    const char* const arg1 = (count > 1U) ? tokens[1] : nullptr;
    const char* const arg2 = (count > 2U) ? tokens[2] : nullptr;

    if (count == 0U)
    {
        // empty line
    }
    else if (text::equals(cmd, "help"))
    {
        printHelp();
    }
    else if (text::equals(cmd, "status"))
    {
        printStatus();
    }
    else if (text::equals(cmd, "log"))
    {
        m_logOn = !m_logOn;
        Serial.printf("log %s\n", m_logOn ? "ON" : "OFF");
    }
    else if (text::equals(cmd, "gains"))
    {
        printGains();
    }
    else if (text::equals(cmd, "set"))
    {
        cmdSet(arg1, arg2);
    }
    else if (text::equals(cmd, "motor"))
    {
        cmdMotor(arg1, arg2);
    }
    else if (text::equals(cmd, "dry"))
    {
        cmdDry(arg1);
    }
    else if (text::equals(cmd, "stop"))
    {
        cmdStop();
    }
    else if (text::equals(cmd, "ptest"))
    {
        cmdPowerTest(tokens, count);
    }
    else
    {
        Serial.println("Unknown command. Type 'help'.");
    }
}

inline void FlightController::cmdSet(const char* name, const char* value)
{
    float number = 0.0f;
    if ((name == nullptr) || (!text::parseFloat(value, number)))
    {
        Serial.println("Usage: set <name> <value>, e.g. set rateKp 0.004");
    }
    else
    {
        const std::array<fccli_detail::GainParam, 6U> params = {{
            {"angleKp", &m_gains.angleKp},
            {"rateKp", &m_gains.rateKp},
            {"rateKi", &m_gains.rateKi},
            {"rateKd", &m_gains.rateKd},
            {"yawKp", &m_gains.yawKp},
            {"yawKi", &m_gains.yawKi},
        }};
        bool found = false;
        for (const fccli_detail::GainParam& param : params)
        {
            if ((!found) && text::equals(name, param.name))
            {
                *param.value = number;
                found = true;
            }
        }
        if (found)
        {
            applyGains();
            printGains();
        }
        else
        {
            Serial.println("No such gain. Type 'gains' to list names.");
        }
    }
}

inline void FlightController::cmdMotor(const char* arg1, const char* arg2)
{
    int32_t number = 0;
    float percent = -1.0f;

    if (text::equals(arg1, "off"))
    {
        m_testMotor = NO_TEST_MOTOR;
        m_motors.stop();
        Serial.println("Motor test: off");
    }
    else if (m_state != State::Disarmed)
    {
        Serial.println("Motor test only while DISARMED");
    }
    else if (m_powerTest.isRunning())
    {
        Serial.println("Power test is running. Type 'ptest stop' first.");
    }
    else if ((!text::parseInt(arg1, number)) || (!text::parseFloat(arg2, percent)) ||
             (number < fccli_detail::MOTOR_NUMBER_MIN) || (number > fccli_detail::MOTOR_NUMBER_MAX) ||
             (percent < 0.0f) || (percent > fccli_detail::PERCENT))
    {
        Serial.println("Usage: motor <1-4> <0-100>");
    }
    else
    {
        m_dryThrottle = 0.0f;
        m_testMotor = static_cast<int>(number - fccli_detail::MOTOR_NUMBER_MIN);
        m_testDuty = percent / fccli_detail::PERCENT;
        m_testStartMs = millis();
        Serial.printf("Motor test M%d = %.0f%% (auto-off after 10 s)\n", static_cast<int>(number),
                      static_cast<double>(percent));
    }
}

inline void FlightController::cmdDry(const char* arg1)
{
    float percent = 0.0f;
    if (text::equals(arg1, "off"))
    {
        m_dryThrottle = 0.0f;
        Serial.println("dry: off");
    }
    else if (m_state != State::Disarmed)
    {
        Serial.println("dry only while DISARMED");
    }
    else if (m_powerTest.isRunning())
    {
        Serial.println("Power test is running. Type 'ptest stop' first.");
    }
    else if (!imuReady())
    {
        Serial.printf("dry needs the IMU (status: %s)\n", imuStatusName(m_imuStatus));
    }
    else if (!text::parseFloat(arg1, percent))
    {
        Serial.println("Usage: dry <0-100> or dry off");
    }
    else
    {
        m_dryThrottle = mathutil::clamp(percent / fccli_detail::PERCENT, 0.0f, 1.0f);
        m_testMotor = NO_TEST_MOTOR;
        Serial.printf("dry: power %.0f%% (motors do NOT spin). Type 'log' to watch m1..m4\n",
                      static_cast<double>(percent));
    }
}

inline void FlightController::cmdStop()
{
    m_powerTest.stop("stop command from Serial");
    m_testMotor = NO_TEST_MOTOR;
    m_dryThrottle = 0.0f;
    if ((m_state == State::Armed) || (m_state == State::Failsafe))
    {
        disarm("stop command from Serial");
    }
    m_motors.stop();
    Serial.println("Motors stopped.");
}

inline void FlightController::cmdPowerTest(const text::Tokens& tokens, const size_t count)
{
    m_powerTest.runCommand(tokens, count, millis(), canStartPowerTest());
}

inline void FlightController::printHelp()
{
    Serial.println(
        "Commands:\n"
        "  status            - state, angles, battery, link, motors\n"
        "  log               - toggle continuous output (use Serial Plotter)\n"
        "  gains             - print PID gains\n"
        "  set <name> <val>  - change a gain, e.g. set rateKp 0.004\n"
        "  motor <1-4> <%>   - spin one motor (DISARMED only, auto-off after 10 s). REMOVE PROPS!\n"
        "  motor off         - stop the motor test\n"
        "  dry <%>           - run PID+mixer at <%> power but do NOT output PWM\n"
        "  dry off\n"
        "  stop              - stop all motors (and a power test), disarm\n"
        "  ptest ...         - power / motor bench test, type 'ptest' for its commands");
}

inline void FlightController::printGains() const
{
    Serial.printf("angleKp=%.3f rateKp=%.5f rateKi=%.5f rateKd=%.6f yawKp=%.5f yawKi=%.5f\n",
                  static_cast<double>(m_gains.angleKp), static_cast<double>(m_gains.rateKp),
                  static_cast<double>(m_gains.rateKi), static_cast<double>(m_gains.rateKd),
                  static_cast<double>(m_gains.yawKp), static_cast<double>(m_gains.yawKi));
}

inline void FlightController::printStatus() const
{
    const uint32_t nowMs = millis();
    const proto::Command& command = m_radio.command();
    const MotorOutputs& out = m_motors.outputs();

    Serial.printf("State: %s %s\n", stateName(m_state), m_reason);
    Serial.printf("IMU: %s\n", imuStatusName(m_imuStatus));
    Serial.printf("Loop: %lu Hz | Battery: %.2f V%s\n", static_cast<unsigned long>(m_loopHz),
                  static_cast<double>(m_battery.volts()), m_battery.isLow() ? " (LOW)" : "");
    if (m_hasChipTemp)
    {
        Serial.printf("Chip temperature: %.1f C (read took %lu us%s) | WiFi TX: %.1f dBm\n",
                      static_cast<double>(m_chipTempC), static_cast<unsigned long>(m_tempReadUs),
                      m_tempReadSlow ? ", slow -> skipped in flight" : "",
                      static_cast<double>(m_radio.txPowerDbm()));
    }
    else
    {
        Serial.printf("Chip temperature: not read yet | WiFi TX: %.1f dBm\n",
                      static_cast<double>(m_radio.txPowerDbm()));
    }
    Serial.printf("Angles: roll %.1f  pitch %.1f | gyro: %.1f %.1f %.1f dps\n",
                  static_cast<double>(m_imu.roll()), static_cast<double>(m_imu.pitch()),
                  static_cast<double>(m_imu.rollRate()), static_cast<double>(m_imu.pitchRate()),
                  static_cast<double>(m_imu.yawRate()));
    Serial.printf("Link: %s | packets OK %lu, bad %lu | command: arm=%d stop=%d thr=%u r=%d p=%d y=%d\n",
                  m_radio.linkOk(nowMs) ? "YES" : "NO", static_cast<unsigned long>(m_radio.rxCount()),
                  static_cast<unsigned long>(m_radio.badCount()), command.arm ? 1 : 0, command.stop ? 1 : 0,
                  static_cast<unsigned>(command.throttle), static_cast<int>(command.roll),
                  static_cast<int>(command.pitch), static_cast<int>(command.yaw));
    Serial.printf("Motor (%%): M1 %.0f  M2 %.0f  M3 %.0f  M4 %.0f\n",
                  static_cast<double>(out[0] * fccli_detail::PERCENT),
                  static_cast<double>(out[1] * fccli_detail::PERCENT),
                  static_cast<double>(out[2] * fccli_detail::PERCENT),
                  static_cast<double>(out[3] * fccli_detail::PERCENT));
    m_powerTest.printStatusLine();
}

// "name:value,name:value" format for the Arduino IDE Serial Plotter
inline void FlightController::printLog() const
{
    const MotorOutputs& m = (m_dryThrottle > 0.0f) ? m_mixOut : m_motors.outputs();
    Serial.printf(
        "roll:%.1f,pitch:%.1f,gz:%.0f,m1:%.0f,m2:%.0f,m3:%.0f,m4:%.0f\n", static_cast<double>(m_imu.roll()),
        static_cast<double>(m_imu.pitch()), static_cast<double>(m_imu.yawRate()),
        static_cast<double>(m[0] * fccli_detail::PERCENT), static_cast<double>(m[1] * fccli_detail::PERCENT),
        static_cast<double>(m[2] * fccli_detail::PERCENT), static_cast<double>(m[3] * fccli_detail::PERCENT));
}

#endif // DRONE_FLIGHT_CONTROLLER_CLI_H
