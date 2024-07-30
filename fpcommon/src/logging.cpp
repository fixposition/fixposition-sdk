/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Logging
 */

/* LIBC/STL */
#include <cctype>
#include <cstdio>
#include <cstring>
#include <mutex>

/* EXTERNAL */
#include <unistd.h>

/* PACKAGE */
#include "fpcommon/logging.hpp"

namespace fp {
namespace common {
namespace logging {
/* ****************************************************************************************************************** */

static LoggingColour g_colour_init = LoggingColour::AUTO;
static LoggingParams g_params;
std::mutex g_mutex;
char g_line[0xffff];

// ---------------------------------------------------------------------------------------------------------------------

const char* LoggingLevelStr(const LoggingLevel level)
{
    switch (level) {  // clang-format off
        case LoggingLevel::TRACE:   return "TRACE";
        case LoggingLevel::DEBUG:   return "DEBUG";
        case LoggingLevel::INFO:    return "INFO";
        case LoggingLevel::NOTICE:  return "NOTICE";
        case LoggingLevel::WARNING: return "WARNING";
        case LoggingLevel::ERROR:   return "ERROR";
        case LoggingLevel::FATAL:   return "FATAL";  // clang-format on
    }
    return "?";
}

LoggingLevel& operator++(LoggingLevel& level)
{
    switch (level) {  // clang-format off
        case LoggingLevel::TRACE:                                  break;
        case LoggingLevel::DEBUG:   level = LoggingLevel::TRACE;   break;
        case LoggingLevel::INFO:    level = LoggingLevel::DEBUG;   break;
        case LoggingLevel::NOTICE:  level = LoggingLevel::INFO;    break;
        case LoggingLevel::WARNING: level = LoggingLevel::NOTICE;  break;
        case LoggingLevel::ERROR:   level = LoggingLevel::WARNING; break;
        case LoggingLevel::FATAL:   level = LoggingLevel::ERROR;   break;  // clang-format on
    }
    return level;
}

LoggingLevel operator++(LoggingLevel& level, int)
{
    const LoggingLevel old_level = level;
    ++level;
    return old_level;
}

LoggingLevel& operator--(LoggingLevel& level)
{
    switch (level) {
            // clang-format off
        case LoggingLevel::TRACE:   level = LoggingLevel::DEBUG;   break;
        case LoggingLevel::DEBUG:   level = LoggingLevel::INFO;    break;
        case LoggingLevel::INFO:    level = LoggingLevel::NOTICE;  break;
        case LoggingLevel::NOTICE:  level = LoggingLevel::WARNING; break;
        case LoggingLevel::WARNING: level = LoggingLevel::ERROR;   break;
        case LoggingLevel::ERROR:   level = LoggingLevel::FATAL;   break;
        case LoggingLevel::FATAL:                                  break;
            // clang-format on
    }
    return level;
}

LoggingLevel operator--(LoggingLevel& level, int)
{
    const LoggingLevel old_level = level;
    --level;
    return old_level;
}

bool LoggingIsLevel(const LoggingLevel level)
{
    return (level <= g_params.level_);
}

// ---------------------------------------------------------------------------------------------------------------------

static void LoggingDefaultFn(const LoggingParams& params, const LoggingLevel level, const char* str)
{
    const char* prefix = NULL;
    const char* suffix = NULL;

    switch (params.colour_) {
        case LoggingColour::YES:
            switch (level) {  // clang-format off
                case LoggingLevel::TRACE:   prefix = "\033[0;35m"; suffix = "\033[m\n"; break;
                case LoggingLevel::DEBUG:   prefix = "\033[0;36m"; suffix = "\033[m\n"; break;
                case LoggingLevel::INFO:    prefix = NULL;         suffix = "\n";       break;
                case LoggingLevel::NOTICE:  prefix = "\033[1m";    suffix = "\033[m\n"; break;
                case LoggingLevel::WARNING: prefix = "\033[1;33m"; suffix = "\033[m\n"; break;
                case LoggingLevel::ERROR:   prefix = "\033[1;31m"; suffix = "\033[m\n"; break;
                case LoggingLevel::FATAL:   prefix = "\033[1;35m"; suffix = "\033[m\n"; break;
            }  // clang-format on
            break;
        case LoggingColour::JOURNAL:
            switch (params.level_) {  // clang-format off
                case LoggingLevel::TRACE:   prefix = "<7>"; suffix = "\n"; break;
                case LoggingLevel::DEBUG:   prefix = "<7>"; suffix = "\n"; break;
                case LoggingLevel::INFO:    prefix = "<6>"; suffix = "\n"; break;
                case LoggingLevel::NOTICE:  prefix = "<5>"; suffix = "\n"; break;
                case LoggingLevel::WARNING: prefix = "<4>"; suffix = "\n"; break;
                case LoggingLevel::ERROR:   prefix = "<3>"; suffix = "\n"; break;
                case LoggingLevel::FATAL:   prefix = "<2>"; suffix = "\n"; break;
            }  // clang-format on
            break;
        case LoggingColour::AUTO:
        case LoggingColour::NO:
            prefix = NULL;
            suffix = "\n";
            break;
    }

    if (prefix != NULL) {
        std::fputs(prefix, stderr);
    }
    std::fputs(str, stderr);
    if (suffix != NULL) {
        std::fputs(suffix, stderr);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

LoggingParams::LoggingParams(const LoggingLevel level, const LoggingColour colour) /* clang-format off */ :
    level_    { level },
    colour_   { colour },
    fn_       { LoggingDefaultFn }  // clang-format on
{
    // User wants us to decide...
    if (colour_ == LoggingColour::AUTO) {
        // Determine this once on first call
        if (g_colour_init == LoggingColour::AUTO) {
            g_colour_init = (isatty(fileno(stderr)) == 1 ? LoggingColour::YES : LoggingColour::NO);
        }
        colour_ = g_colour_init;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

LoggingParams LoggingSetParams(const LoggingParams& params)
{
    std::unique_lock<std::mutex> lock(g_mutex);
    g_params = params;
    return g_params;
}

// ---------------------------------------------------------------------------------------------------------------------

LoggingParams LoggingGetParams()
{
    return g_params;
}

// ---------------------------------------------------------------------------------------------------------------------

void LoggingPrint(const LoggingLevel level, const char* fmt, ...)
{
    if (!LoggingIsLevel(level)) {
        return;
    }
    std::unique_lock<std::mutex> lock(g_mutex);

    va_list args;
    va_start(args, fmt);
    std::vsnprintf(g_line, sizeof(g_line), fmt, args);
    va_end(args);

    g_params.fn_(g_params, level, g_line);
}

// ---------------------------------------------------------------------------------------------------------------------

void LoggingHexdump(
    const LoggingLevel level, const uint8_t* data, const uint64_t size, const char* prefix, const char* fmt, ...)
{
    if (!LoggingIsLevel(level)) {
        return;
    }
    std::unique_lock<std::mutex> lock(g_mutex);

    if (fmt != NULL) {
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(g_line, sizeof(g_line), fmt, args);
        va_end(args);
        g_params.fn_(g_params, level, g_line);
    }

    const char i2hex[] = "0123456789abcdef";
    const uint8_t* pData = data;
    for (uint64_t ix = 0; ix < size;) {
        char str[70];
        std::memset(str, ' ', sizeof(str));
        str[50] = '|';
        str[67] = '|';
        str[68] = '\0';
        for (int ix2 = 0; (ix2 < 16) && ((ix + ix2) < size); ix2++) {
            //           1         2         3         4         5         6
            // 012345678901234567890123456789012345678901234567890123456789012345678
            // xx xx xx xx xx xx xx xx  xx xx xx xx xx xx xx xx  |................|\0
            // 0  1  2  3  4  5  6  7   8  9  10 11 12 13 14 15
            const uint8_t c = pData[ix + ix2];
            int pos1 = 3 * ix2;
            int pos2 = 51 + ix2;
            if (ix2 > 7) {
                pos1++;
            }
            // clang-format off
            str[pos1    ] = i2hex[ (c >> 4) & 0xf ];
            str[pos1 + 1] = i2hex[  c       & 0xf ];
            // clang-format on
            str[pos2] = isprint((int)c) ? c : '.';
        }
        std::snprintf(
            g_line, sizeof(g_line), "%s0x%04" PRIx64 " %05" PRIu64 "  %s", prefix != NULL ? prefix : "", ix, ix, str);
        g_params.fn_(g_params, level, g_line);

        ix += 16;
    }
}

/* ****************************************************************************************************************** */
}  // namespace logging
}  // namespace common
}  // namespace fp
