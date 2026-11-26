# Firmware coding rules

These rules follow the **spirit of MISRA C++** (the rule set for safety-related embedded software in cars and aircraft). The goal is code that is easy to read, has few hidden pitfalls and behaves predictably.

> This is **not** a MISRA compliance claim. Claiming compliance requires a dedicated checker (PC-lint Plus, Helix QAC, Parasoft, Cppcheck Premium…). The Arduino/ESP32 libraries underneath are not MISRA-compliant either; see "Deviations".

## 1. Formatting (automatic)
The `.clang-format` file lives in the sketch folder. In Arduino IDE 2: **Tools → Auto Format** (`Ctrl + T`) formats according to it.

- `{` **always on its own line** (Allman style), 4-space indent, no tabs, max 110 characters per line.
- **Always use `{ }`** for `if`, `else`, `for`, `while`, even for a single statement:
  ```cpp
  // WRONG
  if (link) armEdge = true;

  // RIGHT
  if (link)
  {
      armEdge = true;
  }
  ```
- One statement per line, one declaration per line.

## 2. Control flow
- An `if … else if …` chain **always ends with `else`**. If there is nothing to do, say why in a comment:
  ```cpp
  else
  {
      // within range: keep as is
  }
  ```
- `switch`: every `case` ends with `break`, no fall-through, **always a `default`**.
- Loops must be **bounded**. E.g. `Radio::poll` reads at most `MAX_PACKETS_PER_POLL` packets per loop; `TextUtil` never reads beyond `MAX_STRING_LENGTH` characters.
- No recursion, no `goto`.
- A single `return` point is not mandatory, but mid-function returns are kept to a minimum. The code uses an `ok` / `result` variable and returns at the end.

## 3. Types and conversions
- Use fixed-width types (`uint8_t`, `int16_t`, `uint32_t`…) where the size matters.
- **No C-style casts** `(int)x`. Use `static_cast<int>(x)`.
- **No implicit lossy conversions**: float ↔ int, signed ↔ unsigned, 32 bit → 16 bit. Checked with `-Wconversion -Wsign-conversion`.
- Float constants carry the `f` suffix (`0.5f`) so they are not computed as `double`. For `%f` arguments of `printf`, cast explicitly with `static_cast<double>(x)`.
- Unsigned constants carry the `U` suffix (`10U`). Bitwise operators (`&`, `|`, `<<`) are used on **unsigned** types only.
- `enum class` instead of `enum`. `nullptr` instead of `NULL`/`0`.
- **No struct-to-byte-array casts** (`reinterpret_cast`, `#pragma pack`). UDP packets are written/read field by field in `Protocol.cpp`, so they do not depend on padding or endianness.

## 4. Variables and constants
- Every variable is **initialised at declaration**, including members (`float m_roll = 0.0f;`).
- **No magic numbers**: meaningful values get a name (`MPU_REG_PWR_MGMT_1`, `CMD_OFS_THROTTLE`, `LED_ERROR_HALF_MS`). Reasonable exceptions are `0`, `1`, `0.0f`, `1.0f` and obvious array indices.
- `constexpr` instead of `#define`. Configuration constants live in `config.h`; module-internal constants live in an anonymous `namespace { }` at the top of the `.cpp` file.
- **Do not reuse Arduino macro names** such as `PI`, `RAD_TO_DEG`, `DEG_TO_RAD`, `DEFAULT`, `HIGH`, `LOW`, `INPUT`, `OUTPUT`, `CHANGE` or `B0`, `B1`… Macros ignore namespaces, so `mathutil::RAD_TO_DEG` breaks the build. Use a module prefix or suffix (`MPU_`, `CMD_`, `_F`).
- Parameters that are not modified are `const`. Member functions that do not modify the object are `const`.
- `std::array` instead of C arrays.

## 5. File organisation
- Each module is a pair: **`.h` = declarations**, **`.cpp` = implementation**. Exceptions: `config.h` (constants only) and `MathUtil.h` (small `inline` functions called constantly in the 500 Hz loop).
- Headers use **include guards** `#ifndef DRONE_X_H` rather than `#pragma once`, which is not standard C++.
- Functions and constants used by one `.cpp` file only go into an **anonymous namespace** `namespace { }`.
- No `using namespace` in headers.
- Naming:
  - Classes: `PascalCase`. Functions and variables: `camelCase`. Constants: `UPPER_CASE`.
  - Class members: **`m_`** prefix (as in Qt). Globals: `g_` prefix.

## 6. Libraries
- **Do not use** `atof`, `atoi`, `strtok`, `strcmp`, `malloc`/`new` in our own code. `TextUtil` implements tokenising, string comparison and number parsing.
- Function return values must be used. When ignoring one on purpose, say so with `static_cast<void>(...)`:
  ```cpp
  static_cast<void>(Wire.endTransmission());
  ```

## 7. Intentional deviations
| # | Deviation | Reason |
|---|---|---|
| D1 | `Serial.printf` is used (variadic function based on `<cstdio>`) | Serial debugging only. Format strings are checked by the compiler with `-Wformat` |
| D2 | Return values of `Serial.print*` are not checked | A failed debug print does not affect flight safety |
| D3 | The Arduino / ESP-IDF / WiFi libraries use dynamic allocation and other non-MISRA constructs | Outside our code, cannot be replaced |
| D4 | `parseFloat` does not support exponent form (`1e-5`) | Sufficient for the `set` command. Type `0.00001` |

## 8. How it was checked
- Every file compiles with `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wold-style-cast -Wshadow -Wdouble-promotion -Wformat=2 -Wswitch-default -Wcast-qual -Werror`: **0 warnings** (against Arduino stub headers that include the Arduino macros).
- `clang-tidy` with the groups: braces, C-style casts, uninitialised variables, `bugprone-*`, bitwise ops on signed types, missing switch branches, implicit bool conversion, magic numbers, narrowing conversions: **0 warnings**.
- `clang-format --dry-run --Werror`: **clean**.
- Host unit tests for `Protocol`, `TextUtil`, `Mixer` and a closed-loop simulation using the real `Pid` + `Mixer`: **all pass**.
