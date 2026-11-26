#ifndef DRONE_POWER_TEST_H
#define DRONE_POWER_TEST_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "MathUtil.h"
#include "ResetRecord.h"
#include "TextUtil.h"
#include "config.h"

// ============================================================
//  Power / motor bench test.          !!! REMOVE THE PROPS !!!
//
//  Finds out how far the battery voltage drops while the motors run, and at which power the XIAO resets.
//  A test = 1 idle row (motors OFF) + up to PTEST_MAX_STEPS power steps.
//  Each step: ramp to the power -> wait PTEST_SETTLE_MS -> measure the battery every control loop (500 Hz).
//  Per step it records the average (real voltage), minimum and maximum (spikes / noise).
//
//  Runs only while DISARMED. The FlightController owns the motors: it calls update() every loop
//  while isRunning() and writes the returned outputs. Started by:
//    - Serial: "ptest step / hold / auto / idle"
//    - holding the BOOT button for PTEST_BUTTON_HOLD_MS (battery only, no USB needed)
//  Results stay in RAM until the next reset ("ptest report").
//  If the XIAO resets during a test, the next boot reports the step that was running (ResetRecord.h).
// ============================================================

// Voltage statistics over a time window
class VoltageWindow
{
public:
    void reset();
    void add(float volts);
    float average() const;
    float minimum() const;
    float maximum() const;
    uint32_t count() const;

private:
    uint32_t m_count = 0U;
    double m_sumV = 0.0;
    float m_minV = 0.0f;
    float m_maxV = 0.0f;
};

class PowerTest
{
public:
    // Call once in setup(), after Serial is running: reads the reset record.
    void begin();

    bool isRunning() const;

    // Every control loop while isRunning(): advances the test, fills [out] with the motor outputs.
    void update(float dt, uint32_t nowMs, MotorOutputs& out);

    // Stops at once (the FlightController writes 0 to the motors on its next loop).
    void stop(const char* reason);

    // Serial "ptest ..." (tokens[0] = "ptest"). [canStart] = false: no new test may start
    // (not DISARMED, other motor test running...); report / cutoff / stop still work.
    void runCommand(const text::Tokens& tokens, size_t count, uint32_t nowMs, bool canStart);

    // Automatic test (BOOT button). Returns true if it started.
    bool startAuto(uint32_t nowMs);

    void printStatusLine() const;
    void printBootInfo() const; // "previous test interrupted" message, if any
    static void printHelp();

private:
    enum class Phase : uint8_t
    {
        Off = 0U,
        IdleMeasure = 1U, // motors off, measuring the resting voltage
        Ramp = 2U,        // power moving towards the step target
        Settle = 3U,      // target reached, waiting before measuring
        Measure = 4U      // collecting samples
    };

    enum class Outcome : uint8_t
    {
        None = 0U,
        Running = 1U,
        Finished = 2U,
        Aborted = 3U,
        Stopped = 4U
    };

    struct Step
    {
        uint8_t mask = 0U; // bit0 = M1 ... bit3 = M4
        uint8_t percent = 0U;
        uint32_t holdMs = 0U;
    };

    struct Result
    {
        uint8_t mask = 0U; // 0 = idle row (motors off)
        uint8_t percent = 0U;
        float avgV = 0.0f;
        float minV = 0.0f;
        float maxV = 0.0f;
        uint32_t samples = 0U;
    };

    static constexpr size_t MAX_RESULTS = cfg::PTEST_MAX_STEPS + 1U;
    static constexpr float PERCENT = 100.0f;
    static constexpr float MV_PER_V = 1000.0f;
    static constexpr float AT_TARGET_EPS = 0.001f;
    static constexpr uint32_t LIVE_CHECK_MIN_SAMPLES = 100U; // ~0.2 s at 500 Hz before a live abort
    static constexpr uint32_t ONE_BIT = 1U;
    static constexpr uint8_t ALL_MOTORS_MASK = 0x0FU;
    static constexpr uint32_t MS_PER_S = 1000U;
    static constexpr int32_t MIN_HOLD_S = 1;
    static constexpr int32_t MAX_HOLD_S = 10;
    static constexpr size_t MASK_COLUMN_WIDTH = 6U;

    void startTest(uint32_t nowMs);
    void startStep(size_t index, uint32_t nowMs);
    void finishStep(uint32_t nowMs);
    void finish(Outcome outcome, const char* reason);
    void enterPhase(Phase phase, uint32_t nowMs);
    void storeResult(uint8_t mask, uint8_t percent);
    void rampOutputs(float dt);
    bool atTarget() const;
    bool buildRamp(uint8_t mask, int32_t fromPct, int32_t toPct, int32_t incPct, uint32_t holdMs);

    void cmdStep(const text::Tokens& tokens, size_t count, uint32_t nowMs);
    void cmdHold(const text::Tokens& tokens, size_t count, uint32_t nowMs);
    void cmdCutoff(const char* arg);

    void printReport() const;
    void printResultRow(size_t index) const;
    void printVerdict() const;
    static void printTableHeader();
    static void printMask(uint8_t mask, bool padded);
    static uint8_t parseMask(const char* s);
    static bool isSelected(uint8_t mask, size_t index);
    static float readBatteryVolts();
    static const char* outcomeName(Outcome outcome);
    static const char* phaseName(Phase phase);

    Phase m_phase = Phase::Off;
    Outcome m_outcome = Outcome::None;
    const char* m_reason = "";
    uint32_t m_testStartMs = 0U;
    uint32_t m_phaseStartMs = 0U;
    float m_abortAvgV = cfg::PTEST_DEFAULT_ABORT_AVG_V;

    MotorOutputs m_target{};
    MotorOutputs m_level{};
    VoltageWindow m_window;

    std::array<Step, cfg::PTEST_MAX_STEPS> m_steps{};
    size_t m_stepCount = 0U;
    size_t m_stepIndex = 0U;
    std::array<Result, MAX_RESULTS> m_results{};
    size_t m_resultCount = 0U;

    bool m_interrupted = false;
    resetlog::Record m_interruptedRecord{};
};

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================

// ---------- VoltageWindow ----------
inline void VoltageWindow::reset()
{
    m_count = 0U;
    m_sumV = 0.0;
    m_minV = 0.0f;
    m_maxV = 0.0f;
}

inline void VoltageWindow::add(const float volts)
{
    if (m_count == 0U)
    {
        m_minV = volts;
        m_maxV = volts;
    }
    else
    {
        m_minV = mathutil::minf(m_minV, volts);
        m_maxV = mathutil::maxf(m_maxV, volts);
    }
    m_sumV += static_cast<double>(volts);
    ++m_count;
}

inline float VoltageWindow::average() const
{
    float result = 0.0f;
    if (m_count > 0U)
    {
        result = static_cast<float>(m_sumV / static_cast<double>(m_count));
    }
    return result;
}

inline float VoltageWindow::minimum() const
{
    return m_minV;
}

inline float VoltageWindow::maximum() const
{
    return m_maxV;
}

inline uint32_t VoltageWindow::count() const
{
    return m_count;
}

// ---------- PowerTest: public ----------
inline void PowerTest::begin()
{
    m_interrupted = resetlog::takeInterrupted(m_interruptedRecord);
    printBootInfo();
}

inline bool PowerTest::isRunning() const
{
    return m_phase != Phase::Off;
}

inline void PowerTest::update(const float dt, const uint32_t nowMs, MotorOutputs& out)
{
    if (m_phase != Phase::Off)
    {
        rampOutputs(dt);
        const uint32_t phaseMs = nowMs - m_phaseStartMs;

        if ((nowMs - m_testStartMs) >= cfg::PTEST_MAX_TEST_MS)
        {
            finish(Outcome::Aborted, "whole-test time limit reached");
        }
        else
        {
            switch (m_phase)
            {
                case Phase::IdleMeasure:
                    m_window.add(readBatteryVolts());
                    if (phaseMs >= cfg::PTEST_IDLE_MEASURE_MS)
                    {
                        storeResult(0U, 0U);
                        printResultRow(m_resultCount - 1U);
                        startStep(0U, nowMs);
                    }
                    break;
                case Phase::Ramp:
                    if (atTarget())
                    {
                        enterPhase(Phase::Settle, nowMs);
                    }
                    break;
                case Phase::Settle:
                    if (phaseMs >= cfg::PTEST_SETTLE_MS)
                    {
                        m_window.reset();
                        enterPhase(Phase::Measure, nowMs);
                    }
                    break;
                case Phase::Measure:
                    m_window.add(readBatteryVolts());
                    if ((m_window.count() >= LIVE_CHECK_MIN_SAMPLES) && (m_window.average() < m_abortAvgV))
                    {
                        // do not wait for the end of a long step when the voltage is already too low
                        finishStep(nowMs);
                    }
                    else if (phaseMs >= m_steps[m_stepIndex].holdMs)
                    {
                        finishStep(nowMs);
                    }
                    else
                    {
                        // keep measuring
                    }
                    break;
                case Phase::Off:
                default:
                    break;
            }
        }
    }
    out = m_level; // all zero once the test has finished
}

inline void PowerTest::stop(const char* reason)
{
    if (m_phase != Phase::Off)
    {
        finish(Outcome::Stopped, reason);
    }
}

inline void PowerTest::runCommand(const text::Tokens& tokens, const size_t count, const uint32_t nowMs,
                                  const bool canStart)
{
    const char* const sub = (count > 1U) ? tokens[1] : nullptr;

    if ((sub == nullptr) || text::equals(sub, "help"))
    {
        printHelp();
    }
    else if (text::equals(sub, "report"))
    {
        printReport();
    }
    else if (text::equals(sub, "stop"))
    {
        stop("stopped by user");
    }
    else if (text::equals(sub, "cutoff"))
    {
        cmdCutoff((count > 2U) ? tokens[2] : nullptr);
    }
    else if (isRunning())
    {
        Serial.println("Power test already running. Type 'ptest stop' first.");
    }
    else if (!canStart)
    {
        Serial.println("Power test only while DISARMED, with no other motor test / dry run active.");
    }
    else if (text::equals(sub, "step"))
    {
        cmdStep(tokens, count, nowMs);
    }
    else if (text::equals(sub, "hold"))
    {
        cmdHold(tokens, count, nowMs);
    }
    else if (text::equals(sub, "auto"))
    {
        static_cast<void>(startAuto(nowMs));
    }
    else if (text::equals(sub, "idle"))
    {
        m_stepCount = 0U; // idle row only
        startTest(nowMs);
    }
    else
    {
        Serial.println("Unknown ptest command. Type 'ptest help'.");
    }
}

inline bool PowerTest::startAuto(const uint32_t nowMs)
{
    bool started = false;
    if ((!isRunning()) &&
        buildRamp(cfg::PTEST_AUTO_MASK, cfg::PTEST_AUTO_FROM_PERCENT, cfg::PTEST_AUTO_TO_PERCENT,
                  cfg::PTEST_AUTO_STEP_PERCENT, cfg::PTEST_DEFAULT_HOLD_MS))
    {
        Serial.println("\nPower test AUTO (BOOT button or 'ptest auto').");
        startTest(nowMs);
        started = true;
    }
    return started;
}

inline void PowerTest::printStatusLine() const
{
    Serial.printf("Power test: %s | phase: %s | step %u/%u | abort limit %.2f V | results kept: %u\n",
                  outcomeName(m_outcome), phaseName(m_phase),
                  static_cast<unsigned>(isRunning() ? (m_stepIndex + 1U) : 0U),
                  static_cast<unsigned>(m_stepCount), static_cast<double>(m_abortAvgV),
                  static_cast<unsigned>(m_resultCount));
}

inline void PowerTest::printBootInfo() const
{
    if (m_interrupted)
    {
        Serial.printf("!! The previous POWER TEST was INTERRUPTED by this reset at step %lu (motors ",
                      static_cast<unsigned long>(m_interruptedRecord.step));
        printMask(static_cast<uint8_t>(m_interruptedRecord.mask), false);
        Serial.printf(", %lu %%).\n", static_cast<unsigned long>(m_interruptedRecord.percent));
    }
}

inline void PowerTest::printHelp()
{
    Serial.println(
        "Power test (DISARMED only, REMOVE THE PROPS):\n"
        "  ptest step <motors> <from%> <to%> <inc%> [holdMs]  power steps, e.g. ptest step 1234 10 60 10\n"
        "  ptest hold <motors> <power%> <seconds>              one level 1-10 s, e.g. ptest hold 1 20 10\n"
        "  ptest auto        all motors 10..60 % (= hold the BOOT button 3 s)\n"
        "  ptest idle        resting voltage only (motors off)\n"
        "  ptest cutoff <V>  abort when a step average falls below V (default 3.3)\n"
        "  ptest report      results of the last power test\n"
        "  ptest stop        stop the power test (also: 'stop', BOOT button, STOP in the app)\n"
        "  motors: digits 1-4 (1234 = all, 13 = M1 + M3) or 'all'");
}

// ---------- PowerTest: test engine ----------
inline void PowerTest::startTest(const uint32_t nowMs)
{
    m_resultCount = 0U;
    m_stepIndex = 0U;
    m_interrupted = false; // a new test replaces the old "interrupted" information
    m_outcome = Outcome::Running;
    m_reason = "";
    m_testStartMs = nowMs;
    m_target.fill(0.0f);
    m_level.fill(0.0f);
    m_window.reset();
    Serial.printf(
        "\nPower test: %u power step(s), abort if a step average < %.2f V. Type 'stop' to cancel.\n",
        static_cast<unsigned>(m_stepCount), static_cast<double>(m_abortAvgV));
    printTableHeader();
    enterPhase(Phase::IdleMeasure, nowMs);
}

inline void PowerTest::startStep(const size_t index, const uint32_t nowMs)
{
    if (index >= m_stepCount)
    {
        finish(Outcome::Finished, "all steps done");
    }
    else
    {
        m_stepIndex = index;
        const Step& step = m_steps[index];
        resetlog::markRunning(step.mask, step.percent, static_cast<uint32_t>(index + 1U));
        const float level = static_cast<float>(step.percent) / PERCENT;
        for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
        {
            m_target[i] = isSelected(step.mask, i) ? level : 0.0f;
        }
        enterPhase(Phase::Ramp, nowMs);
    }
}

inline void PowerTest::finishStep(const uint32_t nowMs)
{
    const Step& step = m_steps[m_stepIndex];
    storeResult(step.mask, step.percent);
    printResultRow(m_resultCount - 1U);
    if (m_results[m_resultCount - 1U].avgV < m_abortAvgV)
    {
        finish(Outcome::Aborted, "step average voltage below the abort limit");
    }
    else
    {
        startStep(m_stepIndex + 1U, nowMs);
    }
}

inline void PowerTest::finish(const Outcome outcome, const char* reason)
{
    m_target.fill(0.0f);
    m_level.fill(0.0f);
    m_phase = Phase::Off;
    m_outcome = outcome;
    m_reason = reason;
    resetlog::markFinished();
    Serial.printf("Motors OFF. Power test %s: %s\n", outcomeName(outcome), reason);
    printVerdict();
}

inline void PowerTest::enterPhase(const Phase phase, const uint32_t nowMs)
{
    m_phase = phase;
    m_phaseStartMs = nowMs;
}

inline void PowerTest::storeResult(const uint8_t mask, const uint8_t percent)
{
    if (m_resultCount < MAX_RESULTS)
    {
        Result& result = m_results[m_resultCount];
        result.mask = mask;
        result.percent = percent;
        result.avgV = m_window.average();
        result.minV = m_window.minimum();
        result.maxV = m_window.maximum();
        result.samples = m_window.count();
        ++m_resultCount;
    }
}

inline void PowerTest::rampOutputs(const float dt)
{
    const float maxStep = cfg::PTEST_RAMP_PER_S * dt;
    for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
    {
        m_level[i] += mathutil::clamp(m_target[i] - m_level[i], -maxStep, maxStep);
    }
}

inline bool PowerTest::atTarget() const
{
    bool reached = true;
    for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
    {
        if (mathutil::absf(m_target[i] - m_level[i]) > AT_TARGET_EPS)
        {
            reached = false;
        }
    }
    return reached;
}

inline bool PowerTest::buildRamp(const uint8_t mask, const int32_t fromPct, const int32_t toPct,
                                 const int32_t incPct, const uint32_t holdMs)
{
    bool ok = (fromPct >= cfg::PTEST_MIN_PERCENT) && (toPct <= cfg::PTEST_MAX_PERCENT) &&
              (fromPct <= toPct) && (incPct > 0);
    size_t count = 0U;
    if (ok)
    {
        for (int32_t pct = fromPct; (pct <= toPct) && ok; pct += incPct)
        {
            if (count >= cfg::PTEST_MAX_STEPS)
            {
                ok = false;
            }
            else
            {
                m_steps[count].mask = mask;
                m_steps[count].percent = static_cast<uint8_t>(pct);
                m_steps[count].holdMs = holdMs;
                ++count;
            }
        }
    }
    m_stepCount = ok ? count : 0U;
    return ok;
}

// ---------- PowerTest: commands ----------
// ptest step <motors> <from%> <to%> <inc%> [holdMs]
inline void PowerTest::cmdStep(const text::Tokens& tokens, const size_t count, const uint32_t nowMs)
{
    int32_t fromPct = 0;
    int32_t toPct = 0;
    int32_t incPct = 0;
    int32_t holdMs = static_cast<int32_t>(cfg::PTEST_DEFAULT_HOLD_MS);
    const uint8_t mask = (count > 2U) ? parseMask(tokens[2]) : 0U;

    bool ok = (count >= 6U) && (mask != 0U) && text::parseInt(tokens[3], fromPct) &&
              text::parseInt(tokens[4], toPct) && text::parseInt(tokens[5], incPct);
    if (ok && (count > 6U))
    {
        ok = text::parseInt(tokens[6], holdMs) && (holdMs >= static_cast<int32_t>(cfg::PTEST_MIN_HOLD_MS)) &&
             (holdMs <= static_cast<int32_t>(cfg::PTEST_MAX_HOLD_MS));
    }
    if (ok)
    {
        ok = buildRamp(mask, fromPct, toPct, incPct, static_cast<uint32_t>(holdMs));
    }

    if (ok)
    {
        startTest(nowMs);
    }
    else
    {
        Serial.printf(
            "Usage: ptest step <motors> <from%%> <to%%> <inc%%> [holdMs]   e.g. ptest step 1234 10 60 10\n"
            "  motors: digits 1-4 or 'all' · power 1..100 %% · at most %u steps · holdMs %lu..%lu\n",
            static_cast<unsigned>(cfg::PTEST_MAX_STEPS), static_cast<unsigned long>(cfg::PTEST_MIN_HOLD_MS),
            static_cast<unsigned long>(cfg::PTEST_MAX_HOLD_MS));
    }
}

// ptest hold <motors> <power%> <seconds>
inline void PowerTest::cmdHold(const text::Tokens& tokens, const size_t count, const uint32_t nowMs)
{
    int32_t pct = 0;
    int32_t seconds = 0;
    const uint8_t mask = (count > 2U) ? parseMask(tokens[2]) : 0U;

    bool ok = (count >= 5U) && (mask != 0U) && text::parseInt(tokens[3], pct) &&
              text::parseInt(tokens[4], seconds) && (seconds >= MIN_HOLD_S) && (seconds <= MAX_HOLD_S);
    if (ok)
    {
        ok = buildRamp(mask, pct, pct, 1, static_cast<uint32_t>(seconds) * MS_PER_S);
    }

    if (ok)
    {
        startTest(nowMs);
    }
    else
    {
        Serial.printf("Usage: ptest hold <motors> <power%%> <seconds 1..%ld>   e.g. ptest hold 1 20 10\n",
                      static_cast<long>(MAX_HOLD_S));
    }
}

inline void PowerTest::cmdCutoff(const char* arg)
{
    float value = 0.0f;
    if ((arg != nullptr) && text::parseFloat(arg, value) && (value >= cfg::PTEST_MIN_ABORT_AVG_V) &&
        (value <= cfg::PTEST_MAX_ABORT_AVG_V))
    {
        m_abortAvgV = value;
    }
    else
    {
        Serial.printf("Usage: ptest cutoff <volts %.1f..%.1f>\n",
                      static_cast<double>(cfg::PTEST_MIN_ABORT_AVG_V),
                      static_cast<double>(cfg::PTEST_MAX_ABORT_AVG_V));
    }
    Serial.printf("Power test abort limit: a step average below %.2f V stops the test.\n",
                  static_cast<double>(m_abortAvgV));
}

// ---------- PowerTest: output ----------
inline void PowerTest::printReport() const
{
    Serial.println("\n===== POWER TEST REPORT =====");
    printBootInfo();
    if (m_resultCount == 0U)
    {
        Serial.println("No power test results yet (results are kept until the next reset).");
    }
    else
    {
        printTableHeader();
        for (size_t i = 0U; i < m_resultCount; ++i)
        {
            printResultRow(i);
        }
        Serial.printf("Power test %s: %s\n", outcomeName(m_outcome), m_reason);
        printVerdict();
    }
}

inline void PowerTest::printTableHeader()
{
    Serial.println(" #  motors  power   avg V   min V   max V   sag V  spread V  samples");
}

inline void PowerTest::printResultRow(const size_t index) const
{
    const Result& r = m_results[index];
    const float idleAvg = (m_resultCount > 0U) ? m_results[0].avgV : r.avgV;
    Serial.printf("%2u  ", static_cast<unsigned>(index));
    printMask(r.mask, true);
    Serial.printf("  %3u %%   %5.2f   %5.2f   %5.2f   %5.2f   %5.2f    %5lu\n",
                  static_cast<unsigned>(r.percent), static_cast<double>(r.avgV), static_cast<double>(r.minV),
                  static_cast<double>(r.maxV), static_cast<double>(idleAvg - r.avgV),
                  static_cast<double>(r.maxV - r.minV), static_cast<unsigned long>(r.samples));
}

// Plain-language hints from the numbers
inline void PowerTest::printVerdict() const
{
    if (m_resultCount > 1U)
    {
        size_t lowestAvg = 1U;
        size_t widestSpread = 1U;
        for (size_t i = 1U; i < m_resultCount; ++i)
        {
            if (m_results[i].avgV < m_results[lowestAvg].avgV)
            {
                lowestAvg = i;
            }
            if ((m_results[i].maxV - m_results[i].minV) >
                (m_results[widestSpread].maxV - m_results[widestSpread].minV))
            {
                widestSpread = i;
            }
        }

        const Result& low = m_results[lowestAvg];
        const Result& noisy = m_results[widestSpread];
        const float spread = noisy.maxV - noisy.minV;

        Serial.println("--- What the numbers say ---");
        if (low.avgV < cfg::PTEST_VERDICT_LOW_AVG_V)
        {
            Serial.printf("* REAL voltage drop: the average fell to %.2f V at %u %% (%.2f V below idle).\n"
                          "  -> battery / connector / wires / joints. Use 'ptest hold' and measure with the\n"
                          "     multimeter at the battery (A) and at the board rail (B).\n",
                          static_cast<double>(low.avgV), static_cast<unsigned>(low.percent),
                          static_cast<double>(m_results[0].avgV - low.avgV));
        }
        else
        {
            Serial.printf("* Average voltage stayed OK (lowest %.2f V at %u %%).\n",
                          static_cast<double>(low.avgV), static_cast<unsigned>(low.percent));
        }
        if (spread > cfg::PTEST_VERDICT_NOISE_SPREAD_V)
        {
            Serial.printf("* Large spread (max - min = %.2f V at %u %%): electrical NOISE / spikes.\n"
                          "  -> check the flyback diodes and the GND wiring.\n",
                          static_cast<double>(spread), static_cast<unsigned>(noisy.percent));
        }
        else
        {
            Serial.printf("* Noise looks small (largest spread %.2f V).\n", static_cast<double>(spread));
        }
    }
}

// "idle", "1234", "13". [padded] = fill up to 6 characters (table column)
inline void PowerTest::printMask(const uint8_t mask, const bool padded)
{
    size_t printed = 0U;
    if (mask == 0U)
    {
        Serial.print("idle");
        printed = 4U;
    }
    else
    {
        for (size_t i = 0U; i < cfg::MOTOR_COUNT; ++i)
        {
            if (isSelected(mask, i))
            {
                Serial.print(static_cast<char>('1' + static_cast<char>(i)));
                ++printed;
            }
        }
    }
    for (size_t i = printed; padded && (i < MASK_COLUMN_WIDTH); ++i)
    {
        Serial.print(' ');
    }
}

// "1234", "13", "2" or "all" -> bit mask (bit0 = M1). 0 = invalid.
inline uint8_t PowerTest::parseMask(const char* s)
{
    uint32_t mask = 0U;
    bool ok = (s != nullptr);
    if (ok && text::equals(s, "all"))
    {
        mask = ALL_MOTORS_MASK;
    }
    else
    {
        for (size_t i = 0U; ok && (i <= cfg::MOTOR_COUNT) && (s[i] != '\0'); ++i)
        {
            const int32_t number = static_cast<int32_t>(s[i]) - static_cast<int32_t>('0');
            if ((i == cfg::MOTOR_COUNT) || (number < 1) || (number > static_cast<int32_t>(cfg::MOTOR_COUNT)))
            {
                ok = false; // too long or not a motor number 1..4
            }
            else
            {
                mask |= (ONE_BIT << static_cast<uint32_t>(number - 1));
            }
        }
    }
    return ok ? static_cast<uint8_t>(mask) : 0U;
}

inline bool PowerTest::isSelected(const uint8_t mask, const size_t index)
{
    const uint32_t bit = ONE_BIT << static_cast<uint32_t>(index);
    return (static_cast<uint32_t>(mask) & bit) != 0U;
}

inline float PowerTest::readBatteryVolts()
{
    const float midMv = static_cast<float>(analogReadMilliVolts(cfg::PIN_BAT));
    return (midMv * cfg::BAT_DIVIDER_RATIO) / MV_PER_V;
}

inline const char* PowerTest::outcomeName(const Outcome outcome)
{
    const char* name = "?";
    switch (outcome)
    {
        case Outcome::None:
            name = "none";
            break;
        case Outcome::Running:
            name = "RUNNING";
            break;
        case Outcome::Finished:
            name = "FINISHED";
            break;
        case Outcome::Aborted:
            name = "ABORTED";
            break;
        case Outcome::Stopped:
            name = "STOPPED";
            break;
        default:
            break;
    }
    return name;
}

inline const char* PowerTest::phaseName(const Phase phase)
{
    const char* name = "?";
    switch (phase)
    {
        case Phase::Off:
            name = "off";
            break;
        case Phase::IdleMeasure:
            name = "idle measure";
            break;
        case Phase::Ramp:
            name = "ramp";
            break;
        case Phase::Settle:
            name = "settle";
            break;
        case Phase::Measure:
            name = "measure";
            break;
        default:
            break;
    }
    return name;
}

#endif // DRONE_POWER_TEST_H
