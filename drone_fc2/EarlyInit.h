#ifndef DRONE_EARLY_INIT_H
#define DRONE_EARLY_INIT_H

// ============================================================
//  Force all motor gates LOW as early as the firmware can (header-only version of drone_fc/EarlyInit.cpp).
//
//  A function marked "constructor" runs during C++ static initialisation, i.e. before the
//  Arduino core and before setup(). It drives D0..D3 LOW and enables the internal pull-downs.
//  Header-only: include this file exactly ONCE (from drone_fc2.ino).
// ============================================================
#include <driver/gpio.h>

#include "config.h"

// Anonymous namespace (not "inline"): the compiler may drop an unused inline function,
// but this one must always be emitted so that its pointer lands in .init_array.
namespace
{

constexpr uint32_t LEVEL_LOW = 0U;
constexpr uint64_t ONE_BIT = 1U;

__attribute__((constructor)) void forceMotorsOffEarly()
{
    uint64_t mask = 0U;
    for (const uint8_t pin : cfg::PIN_MOTOR)
    {
        static_cast<void>(gpio_set_level(static_cast<gpio_num_t>(pin), LEVEL_LOW)); // latch LOW first
        mask |= (ONE_BIT << pin);
    }

    gpio_config_t config{};
    config.pin_bit_mask = mask;
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    static_cast<void>(gpio_config(&config));

    for (const uint8_t pin : cfg::PIN_MOTOR)
    {
        static_cast<void>(gpio_set_level(static_cast<gpio_num_t>(pin), LEVEL_LOW));
    }
}

} // namespace

#endif // DRONE_EARLY_INIT_H
