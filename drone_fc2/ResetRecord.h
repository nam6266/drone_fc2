#ifndef DRONE_RESET_RECORD_H
#define DRONE_RESET_RECORD_H

#include <esp_attr.h>
#include <esp_system.h>

#include <cstdint>

// ============================================================
//  Remembers which test step was running when the chip reset.
//
//  The record lives in RTC memory marked RTC_NOINIT: it is NOT cleared by a reset
//  (brownout, watchdog, crash, reset button), only by losing power completely.
//  So when the XIAO resets in the middle of a test on battery, the next boot can say
//  "the previous power test was interrupted at motors 1234, 30 %".
//
//  Header-only: include this file exactly ONCE (through PowerTest.h).
// ============================================================
namespace resetlog
{

// Deviation from "every variable is initialised": this struct must NOT be initialised,
// otherwise its contents would be wiped at every boot. The magic number tells valid data from garbage.
struct Record
{
    uint32_t magic;
    uint32_t running; // 1 = a test step was in progress
    uint32_t mask;    // motors of that step (bit0 = M1)
    uint32_t percent; // power of that step
    uint32_t step;    // step number (1 = first power step)
};

constexpr uint32_t RECORD_MAGIC = 0x50575254U; // "PWRT"
constexpr uint32_t RUNNING_YES = 1U;
constexpr uint32_t RUNNING_NO = 0U;

namespace detail
{

RTC_NOINIT_ATTR Record g_rtcRecord;

} // namespace detail

inline void markRunning(const uint8_t mask, const uint8_t percent, const uint32_t step)
{
    detail::g_rtcRecord.magic = RECORD_MAGIC;
    detail::g_rtcRecord.running = RUNNING_YES;
    detail::g_rtcRecord.mask = static_cast<uint32_t>(mask);
    detail::g_rtcRecord.percent = static_cast<uint32_t>(percent);
    detail::g_rtcRecord.step = step;
}

inline void markFinished()
{
    detail::g_rtcRecord.magic = RECORD_MAGIC;
    detail::g_rtcRecord.running = RUNNING_NO;
}

// Call once at boot. Returns true (and fills [out]) if the previous test was cut short by a reset.
inline bool takeInterrupted(Record& out)
{
    const bool interrupted =
        (detail::g_rtcRecord.magic == RECORD_MAGIC) && (detail::g_rtcRecord.running == RUNNING_YES);
    if (interrupted)
    {
        out = detail::g_rtcRecord;
    }
    markFinished();
    return interrupted;
}

// Why the chip (re)started
inline const char* reasonText(const esp_reset_reason_t reason)
{
    const char* text = "other";
    switch (reason)
    {
        case ESP_RST_POWERON:
            text = "power-on";
            break;
        case ESP_RST_EXT:
            text = "reset pin";
            break;
        case ESP_RST_SW:
            text = "software restart / upload";
            break;
        case ESP_RST_BROWNOUT:
            text = "BROWNOUT - supply voltage dipped (motor current / weak battery / bad joint / noise)";
            break;
        case ESP_RST_PANIC:
            text = "crash (panic)";
            break;
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            text = "watchdog";
            break;
        case ESP_RST_USB:
            text = "USB (reset button / IDE)";
            break;
        default:
            break;
    }
    return text;
}

} // namespace resetlog

#endif // DRONE_RESET_RECORD_H
