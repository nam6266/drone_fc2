// ============================================================
//  Mini drone firmware – Seeed XIAO ESP32-S3   (header-only version of drone_fc)
//  Arduino IDE: board XIAO_ESP32S3, USB CDC On Boot: Enabled, ESP32 core 3.x
//  The sketch FOLDER must be named "drone_fc2" (same as this .ino file).
//
//  Every module is ONE .h file (declarations first, inline definitions below); each file is an IDE tab:
//    config.h                – pins, constants, safety limits, power-test settings
//    EarlyInit.h             – drives the motor pins LOW before setup() runs
//    MathUtil.h              – clamp/abs/min/max (inline)
//    Imu.h                   – MPU6500 + complementary filter
//    Motors.h                – 4 PWM channels
//    Pid.h                   – single-axis PID
//    Mixer.h                 – quad-X thrust mixer
//    Battery.h               – battery voltage (smoothed, 10 Hz)   
//    Protocol.h              – UDP packet encode / decode (same bytes as the Android app)
//    Radio.h                 – WiFi AP + UDP
//    TextUtil.h              – string splitting / number parsing for Serial commands
//    ResetRecord.h           – remembers a power-test step across a reset (RTC memory)
//    PowerTest.h             – power / motor bench test ("ptest" commands, BOOT button)
//    FlightController.h      – main loop, state machine
//    FlightControllerCli.h   – Serial commands
//
//  Coding rules: CODING_STYLE.md. Formatting: Tools -> Auto Format (Ctrl+T).
// ============================================================
#include "EarlyInit.h"
#include "FlightController.h"

namespace
{
FlightController g_flightController;
} // namespace

void setup()
{
    g_flightController.begin();
}

void loop()
{
    g_flightController.loop();
}
