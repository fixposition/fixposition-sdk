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
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>

/* EXTERNAL */
#include <unistd.h>

/* PACKAGE */
#include "fpsdk_common/logging.hpp"

namespace fpsdk {
namespace common {
namespace logging {
/* ****************************************************************************************************************** */

static LoggingParams g_params;
std::mutex g_mutex;
static char g_line[0x1fff];
static struct timespec g_time0 = { 0, 0 };

// ---------------------------------------------------------------------------------------------------------------------

const char* LoggingLevelStr(const LoggingLevel level)
{
    switch (level) { /* clang-format off */
        case LoggingLevel::TRACE:   return "TRACE";
        case LoggingLevel::DEBUG:   return "DEBUG";
        case LoggingLevel::INFO:    return "INFO";
        case LoggingLevel::NOTICE:  return "NOTICE";
        case LoggingLevel::WARNING: return "WARNING";
        case LoggingLevel::ERROR:   return "ERROR";
        case LoggingLevel::FATAL:   return "FATAL";
    }  // clang-format on
    return "?";
}

LoggingLevel& operator++(LoggingLevel& level)
{
    switch (level) { /* clang-format off */
        case LoggingLevel::TRACE:                                  break;
        case LoggingLevel::DEBUG:   level = LoggingLevel::TRACE;   break;
        case LoggingLevel::INFO:    level = LoggingLevel::DEBUG;   break;
        case LoggingLevel::NOTICE:  level = LoggingLevel::INFO;    break;
        case LoggingLevel::WARNING: level = LoggingLevel::NOTICE;  break;
        case LoggingLevel::ERROR:   level = LoggingLevel::WARNING; break;
        case LoggingLevel::FATAL:   level = LoggingLevel::ERROR;   break;
    }  // clang-format on
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
    switch (level) { /* clang-format off */
        case LoggingLevel::TRACE:   level = LoggingLevel::DEBUG;   break;
        case LoggingLevel::DEBUG:   level = LoggingLevel::INFO;    break;
        case LoggingLevel::INFO:    level = LoggingLevel::NOTICE;  break;
        case LoggingLevel::NOTICE:  level = LoggingLevel::WARNING; break;
        case LoggingLevel::WARNING: level = LoggingLevel::ERROR;   break;
        case LoggingLevel::ERROR:   level = LoggingLevel::FATAL;   break;
        case LoggingLevel::FATAL:                                  break;
    }  // clang-format on
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

const char* LoggingColourStr(const LoggingColour colour)
{
    switch (colour) { /* clang-format off */
        case LoggingColour::AUTO:      return "AUTO";
        case LoggingColour::YES:       return "YES";
        case LoggingColour::NO:        return "NO";
        case LoggingColour::JOURNAL:   return "JOURNAL";
    }  // clang-format on
    return "?";
}

const char* LoggingTimestampsStr(const LoggingTimestamps timestamps)
{
    switch (timestamps) { /* clang-format off */
        case LoggingTimestamps::NONE:       return "NONE";
        case LoggingTimestamps::RELATIVE:   return "RELATIVE";
        case LoggingTimestamps::ABSOLUTE:   return "ABSOLUTE";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

void LoggingDefaultWriteFn(const LoggingParams& params, const LoggingLevel level, const char* str)
{
    const char* prefix = NULL;
    const char* suffix = NULL;
    const char* tscol = NULL;

    switch (params.colour_) {
        case LoggingColour::YES:
            switch (level) {  // clang-format off
                case LoggingLevel::TRACE:   prefix = "\033[0;35m"; suffix = "\033[m\n"; break;
                case LoggingLevel::DEBUG:   prefix = "\033[0;36m"; suffix = "\033[m\n"; break;
                case LoggingLevel::INFO:    prefix = "\033[m";     suffix = "\n";       break;
                case LoggingLevel::NOTICE:  prefix = "\033[1;37m"; suffix = "\033[m\n"; break;
                case LoggingLevel::WARNING: prefix = "\033[1;33m"; suffix = "\033[m\n"; break;
                case LoggingLevel::ERROR:   prefix = "\033[1;31m"; suffix = "\033[m\n"; break;
                case LoggingLevel::FATAL:   prefix = "\033[1;35m"; suffix = "\033[m\n"; break;
            }  // clang-format on
            tscol = "\033[0;34m";
            break;
        case LoggingColour::JOURNAL:
            switch (level) {  // clang-format off
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
            suffix = "\n";
            break;
    }

    // We want to output the entire line at once or it may interleave with stdout output
    static char output[0x1fff];
    std::size_t len = 0;

    // Journal marker must always come first.
    if ((params.colour_ == LoggingColour::JOURNAL) && (prefix != NULL)) {
        len += std::snprintf(&output[len], sizeof(output) - len, "%s", prefix);
    }

    // Timestamps. Note: deliberately *not* using fpsdk::common::time
    if (params.timestamps_ != LoggingTimestamps::NONE) {
        if (g_time0.tv_sec == 0) {
            clock_gettime(CLOCK_REALTIME, &g_time0);
        }
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);

        if (tscol != NULL) {
            len += std::snprintf(&output[len], sizeof(output) - len, "%s", tscol);
        }
        if (params.timestamps_ == LoggingTimestamps::RELATIVE) {
            now.tv_sec -= g_time0.tv_sec;
            now.tv_nsec -= g_time0.tv_nsec;
            len += std::snprintf(
                &output[len], sizeof(output) - len, "%09.3f ", (double)now.tv_sec + ((double)now.tv_nsec * 1e-9));
        } else {
            struct tm tm;
            localtime_r(&now.tv_sec, &tm);
            len += std::strftime(&output[len], sizeof(output) - len, "%Y-%m-%d %H:%M:%S", &tm);
            len += std::snprintf(&output[len], sizeof(output) - len, ".%03d ", (int)(now.tv_nsec / 1000000));
        }
    }

    if ((params.colour_ != LoggingColour::JOURNAL) && (prefix != NULL)) {
        len += std::snprintf(&output[len], sizeof(output) - len, "%s", prefix);
    }
    len += std::snprintf(&output[len], sizeof(output) - len, "%s", str);
    if (suffix != NULL) {
        // Truncate to accommodate suffix
        if (len >= sizeof(output)) {
            len = sizeof(output) - 10;
        }
        /* len += */ std::snprintf(&output[len], sizeof(output) - len, "%s", suffix);
    }

    // Put the entire output line at once (see comment above)
    std::fputs(output, stderr);
}

// ---------------------------------------------------------------------------------------------------------------------

LoggingParams::LoggingParams(
    const LoggingLevel level, const LoggingColour colour, const LoggingTimestamps timestamps) /* clang-format off */ :
    level_        { level },
    colour_       { colour },
    timestamps_   { timestamps },
    fn_           { LoggingDefaultWriteFn }  // clang-format on
{
    // Change defaults from environment variables when g_params ctor is called, before any user calls to
    // LoggingParams()/LoggingSetParams(). Do this once only.
    static bool s_defaults_init = false;
    if (!s_defaults_init) {
        const char* env_logging = std::getenv("FPSDK_LOGGING");
        if ((env_logging != nullptr) && (env_logging[0] != '\0')) {
            // Copy string and lower-case it
            char words[1000];
            std::snprintf(words, sizeof(words), "%s,", env_logging);
            std::transform(words, &words[std::strlen(words)], words, [](const char c) { return std::tolower(c); });
            for (char* word = std::strtok(words, ","); word != nullptr; word = std::strtok(nullptr, ",")) {
                // clang-format off
                if      (std::strcmp(word, "trace")    == 0) { level_      = LoggingLevel::TRACE;         }
                else if (std::strcmp(word, "debug")    == 0) { level_      = LoggingLevel::DEBUG;         }
                else if (std::strcmp(word, "info")     == 0) { level_      = LoggingLevel::INFO;          }
                else if (std::strcmp(word, "notice")   == 0) { level_      = LoggingLevel::NOTICE;        }
                else if (std::strcmp(word, "warning")  == 0) { level_      = LoggingLevel::WARNING;       }
                else if (std::strcmp(word, "error")    == 0) { level_      = LoggingLevel::ERROR;         }
                else if (std::strcmp(word, "fatal")    == 0) { level_      = LoggingLevel::FATAL;         }
                else if (std::strcmp(word, "auto")     == 0) { colour_     = LoggingColour::AUTO;         }
                else if (std::strcmp(word, "yes")      == 0) { colour_     = LoggingColour::YES;          }
                else if (std::strcmp(word, "no")       == 0) { colour_     = LoggingColour::NO;           }
                else if (std::strcmp(word, "journal")  == 0) { colour_     = LoggingColour::JOURNAL;      }
                else if (std::strcmp(word, "none")     == 0) { timestamps_ = LoggingTimestamps::NONE;     }
                else if (std::strcmp(word, "relative") == 0) { timestamps_ = LoggingTimestamps::RELATIVE; }
                else if (std::strcmp(word, "absolute") == 0) { timestamps_ = LoggingTimestamps::ABSOLUTE; }
                // clang-format on
            }
        }
        s_defaults_init = true;
    }

    // User wants us to decide...
    if (colour_ == LoggingColour::AUTO) {
        colour_ = (isatty(fileno(stderr)) == 1 ? LoggingColour::YES : LoggingColour::NO);
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

void LoggingPrint(const LoggingLevel level, const std::size_t repeat, const char* fmt, ...)
{
    if (!LoggingIsLevel(level)) {
        return;
    }
    std::unique_lock<std::mutex> lock(g_mutex);

    va_list args;
    va_start(args, fmt);
    int len = std::vsnprintf(g_line, sizeof(g_line), fmt, args);
    va_end(args);

    if (repeat > 0) {
        if (len > (int)(sizeof(g_line) - 10)) {
            len -= 10;
        }
        std::snprintf(&g_line[len], sizeof(g_line) - len, " [%" PRIuMAX "x]", repeat);
    }

    g_params.fn_(g_params, level, g_line);
}

// ---------------------------------------------------------------------------------------------------------------------

void LoggingHexdump(
    const LoggingLevel level, const uint8_t* data, const std::size_t size, const char* prefix, const char* fmt, ...)
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
    for (std::size_t ix = 0; ix < size;) {
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
}  // namespace fpsdk
