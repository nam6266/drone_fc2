#ifndef DRONE_TEXT_UTIL_H
#define DRONE_TEXT_UTIL_H

#include <array>
#include <cstddef>
#include <cstdint>

// ============================================================
//  String helpers for Serial commands.
//  Hand-written replacements for strtok / strcmp / atof / atoi (not allowed by MISRA).
// ============================================================
namespace text
{

constexpr size_t MAX_TOKENS = 8U; // "ptest step 1234 10 60 10 1500" = 7 tokens
using Tokens = std::array<const char*, MAX_TOKENS>;

// Splits [line] on spaces (in place: spaces become '\0').
// [capacity] = buffer size. Returns the number of tokens found (at most MAX_TOKENS).
size_t split(char* line, size_t capacity, Tokens& tokens);

// Compares two '\0'-terminated strings. nullptr never equals anything.
bool equals(const char* a, const char* b);

// Decimal number [-]123.456 (no exponent form like 1e-5). false on bad format.
bool parseFloat(const char* s, float& out);

// Integer [-]123. false on bad format.
bool parseInt(const char* s, int32_t& out);

} // namespace text

// ============================================================
//  Implementation (header-only: inline definitions)
// ============================================================
namespace text
{

namespace detail
{

constexpr size_t MAX_STRING_LENGTH = 64U; // loop bound: no command string is longer
constexpr size_t MAX_DIGITS = 9U;         // avoids overflow while parsing
constexpr int32_t DECIMAL_BASE = 10;
constexpr double FIRST_DECIMAL_SCALE = 0.1;

inline bool isDigit(const char c)
{
    return (c >= '0') && (c <= '9');
}

inline int32_t digitValue(const char c)
{
    return static_cast<int32_t>(c) - static_cast<int32_t>('0');
}

} // namespace detail

inline size_t split(char* line, const size_t capacity, Tokens& tokens)
{
    size_t count = 0U;
    bool inToken = false;
    tokens.fill(nullptr);

    if (line != nullptr)
    {
        for (size_t i = 0U; (i < capacity) && (line[i] != '\0'); ++i)
        {
            if (line[i] == ' ')
            {
                line[i] = '\0';
                inToken = false;
            }
            else if ((!inToken) && (count < MAX_TOKENS))
            {
                tokens[count] = &line[i];
                ++count;
                inToken = true;
            }
            else
            {
                // inside a token, or token limit reached
            }
        }
    }
    return count;
}

inline bool equals(const char* a, const char* b)
{
    bool same = (a != nullptr) && (b != nullptr);
    bool done = !same;
    for (size_t i = 0U; (i < detail::MAX_STRING_LENGTH) && (!done); ++i)
    {
        if (a[i] != b[i])
        {
            same = false;
            done = true;
        }
        else if (a[i] == '\0')
        {
            done = true;
        }
        else
        {
            // same character, continue
        }
    }
    return same;
}

inline bool parseFloat(const char* s, float& out)
{
    bool ok = (s != nullptr);
    size_t i = 0U;
    bool negative = false;
    double value = 0.0;
    size_t digits = 0U;

    if (ok)
    {
        if (s[0] == '-')
        {
            negative = true;
            i = 1U;
        }
        else if (s[0] == '+')
        {
            i = 1U;
        }
        else
        {
            // no sign
        }

        while ((i < detail::MAX_STRING_LENGTH) && detail::isDigit(s[i]))
        {
            value = (value * static_cast<double>(detail::DECIMAL_BASE)) +
                    static_cast<double>(detail::digitValue(s[i]));
            ++digits;
            ++i;
        }
        if ((i < detail::MAX_STRING_LENGTH) && (s[i] == '.'))
        {
            ++i;
            double scale = detail::FIRST_DECIMAL_SCALE;
            while ((i < detail::MAX_STRING_LENGTH) && detail::isDigit(s[i]))
            {
                value += static_cast<double>(detail::digitValue(s[i])) * scale;
                scale *= detail::FIRST_DECIMAL_SCALE;
                ++digits;
                ++i;
            }
        }
        ok = (digits > 0U) && (i < detail::MAX_STRING_LENGTH) && (s[i] == '\0');
    }
    if (ok)
    {
        out = static_cast<float>(negative ? -value : value);
    }
    return ok;
}

inline bool parseInt(const char* s, int32_t& out)
{
    bool ok = (s != nullptr);
    size_t i = 0U;
    bool negative = false;
    int32_t value = 0;
    size_t digits = 0U;

    if (ok)
    {
        if (s[0] == '-')
        {
            negative = true;
            i = 1U;
        }
        while ((i < detail::MAX_STRING_LENGTH) && detail::isDigit(s[i]) && (digits < detail::MAX_DIGITS))
        {
            value = (value * detail::DECIMAL_BASE) + detail::digitValue(s[i]);
            ++digits;
            ++i;
        }
        ok = (digits > 0U) && (i < detail::MAX_STRING_LENGTH) && (s[i] == '\0');
    }
    if (ok)
    {
        out = negative ? -value : value;
    }
    return ok;
}

} // namespace text

#endif // DRONE_TEXT_UTIL_H
