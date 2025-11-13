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
 *
 * @page FPSDK_COMMON_LOGGING Logging
 *
 * **API**: fpsdk_common/logging.hpp and fpsdk::common::logging
 *
 * @section FPSDK_COMMON_LOGGIN_CONFIG Logging configuration
 *
 * The defaults are:
 *
 * - fpsdk::common::logging::LoggingLevel::INFO
 * - fpsdk::common::logging::LoggingColour::AUTO
 * - fpsdk::common::logging::LoggingTimestamps::NONE
 *
 * The defaults can be changed by setting the FPSDK_LOGGING environment variable to a comma-separated list of words that
 * correspond to the above parameters. The parameters are case-insensitive and they are applied in the order of
 * appearance. Unknown words are silently ignored. Examples:
 *
 * @code{sh}
 * export FPSDK_LOGGING=DEBUG                   # Changes the level to DEBUG
 * export FPSDK_LOGGING=Warning,JOURNAL         # Changes the level to WARNING and the "colour" to JOURNAL
 * export FPSDK_LOGGING=Warning,relative,debug  # Changes the level to DEBUG and the timestamps to RELATIVE
 * export FPSDK_LOGGING=Warning,retalive,debgu  # Changes the level to WARNING, and nothing else
 * @endcode
 *
 * Apps can configure the logging using the fpsdk::common::logging::LoggingSetParams(). See also the implementation of
 * fpsdk::common::app::ProgramOptions().
 */
#ifndef __FPSDK_COMMON_LOGGING_HPP__
#define __FPSDK_COMMON_LOGGING_HPP__

/* LIBC/STL */
#include <cinttypes>  // PRI.. macros, often used with formats
#include <cstdarg>
#include <cstdint>
#include <sstream>

/* EXTERNAL */

/* PACKAGE */
#include "string.hpp"
#include "time.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief Logging
 */
namespace logging {
/* ****************************************************************************************************************** */

/**
 * @name Printf() style logging
 *
 * For example: `INFO("Hello world, the number is %d", 42);`
 *
 * @{
 */
// clang-format off
/**
 * @brief Print a fatal message                                                                                         @hideinitializer
 */
#define FATAL(...)   _FPSDK_LOGGING_LOG(FATAL,   "Fatal: "    __VA_ARGS__)
/**
 * @brief Print a error message                                                                                         @hideinitializer
 */
#define ERROR(...)   _FPSDK_LOGGING_LOG(ERROR,   "Error: "    __VA_ARGS__)
/**
 * @brief Print a warning message                                                                                       @hideinitializer
 */
#define WARNING(...) _FPSDK_LOGGING_LOG(WARNING, "Warning: "  __VA_ARGS__)
/**
 * @brief Print a notice message                                                                                        @hideinitializer
 */
#define NOTICE(...)  _FPSDK_LOGGING_LOG(NOTICE,               __VA_ARGS__)
/**
 * @brief Print a info message                                                                                          @hideinitializer
 */
#define INFO(...)    _FPSDK_LOGGING_LOG(INFO,                 __VA_ARGS__)
/**
 * @brief Print a debug message                                                                                         @hideinitializer
 */
#define DEBUG(...)   _FPSDK_LOGGING_LOG(DEBUG,                __VA_ARGS__)
/**
 * @brief Print a debug hexdump                                                                                         @hideinitializer
 */
#define DEBUG_HEXDUMP(data, size, prefix, ...) _FPSDK_LOGGING_HEX(DEBUG, data, size, prefix, __VA_ARGS__)
#if !defined(NDEBUG) || defined(_DOXYGEN_) // Only for non-Release builds

/**
 * @brief Print a trace message (only debug builds, compiled out in release builds)                                     @hideinitializer
 */
#  define TRACE(...) _FPSDK_LOGGING_LOG(TRACE,                __VA_ARGS__)
/**
 * @brief Print a trace hexdump (only debug builds, compiled out in release builds)                                     @hideinitializer
 */
#  define TRACE_HEXDUMP(data, size, prefix, ...) _FPSDK_LOGGING_HEX(TRACE, data, size, prefix, __VA_ARGS__)
#else
#  define TRACE(...)         /* nothing */
#  define TRACE_HEXDUMP(...) /* nothing */
#endif

/**
 * @brief Print a error message (throttled)                                                                             @hideinitializer
 */
#define ERROR_THR(millis, ...)   _FPSDK_LOGGING_THR(ERROR,   millis, __VA_ARGS__)
/**
 * @brief Print a warning message (throttled)                                                                           @hideinitializer
 */
#define WARNING_THR(millis, ...) _FPSDK_LOGGING_THR(WARNING, millis, __VA_ARGS__)
/**
 * @brief Print a notice message (throttled)                                                                            @hideinitializer
 */
#define NOTICE_THR(millis, ...)  _FPSDK_LOGGING_THR(NOTICE,  millis, __VA_ARGS__)
/**
 * @brief Print a info message (throttled)                                                                              @hideinitializer
 */
#define INFO_THR(millis, ...)    _FPSDK_LOGGING_THR(INFO,    millis, __VA_ARGS__)
/**
 * @brief Print a debug message (throttled)                                                                             @hideinitializer
 */
#define DEBUG_THR(millis, ...)   _FPSDK_LOGGING_THR(DEBUG,   millis, __VA_ARGS__)
// clang-format on
///@}

/**
 * @name C++ style logging
 *
 * For example, `INFO_S("Hello world, the number is " << 42);`
 *
 * @{
 */
// clang-format off
/**
 * @brief Print a fatal message                                                                                         @hideinitializer
 */
#define FATAL_S(expr)   _FPSDK_LOGGING_STR(FATAL,   expr)
/**
 * @brief Print a error message                                                                                         @hideinitializer
 */
#define ERROR_S(expr)   _FPSDK_LOGGING_STR(ERROR,   expr)
/**
 * @brief Print a warning message                                                                                       @hideinitializer
 */
#define WARNING_S(expr) _FPSDK_LOGGING_STR(WARNING, expr)
/**
 * @brief Print a debug message                                                                                         @hideinitializer
 */
#define DEBUG_S(expr)   _FPSDK_LOGGING_STR(DEBUG,   expr)
/**
 * @brief Print a notice message                                                                                        @hideinitializer
 */
#define NOTICE_S(expr)  _FPSDK_LOGGING_STR(NOTICE,  expr)
/**
 * @brief Print a info message                                                                                          @hideinitializer
 */
#define INFO_S(expr)    _FPSDK_LOGGING_STR(INFO,    expr)

#if !defined(NDEBUG) || defined(_DOXYGEN_) // Only for non-Release builds
/**
 * @brief Print a trace message (only debug builds, compiled out in release builds)                                     @hideinitializer
 */
#  define TRACE_S(expr) _FPSDK_LOGGING_STR(TRACE,   expr)
#else
#  define TRACE_S(...) /* nothing */
#endif
// clang-format on
///@}

#if !defined(NDEBUG) || defined(_DOXYGEN_)  // Only for non-Release builds
//! Conditional compilation for non-Release builds
#  define IF_TRACE(...) __VA_ARGS__
#else
#  define IF_TRACE(...) /* nothing */
#endif
// Note: for other levels use LoggingIsLevel()

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Logging verbosity levels, default is INFO
 *
 * The logging levels loosely follow syslog levels (indicated in [] below, see also
 * https://en.wikipedia.org/wiki/Syslog)
 *
 * Libraries (fpsdk_common, fpsdk_ros1, ...) code shall only use WARNING and DEBUG.
 */
enum class LoggingLevel : int
{  // clang-format off
    FATAL    = 2,  //!< [2/crit]    Hard errors, critical conditions (for apps). Cannot be silenced.
    ERROR    = 3,  //!< [3/err]     Errors (for apps)
    WARNING  = 4,  //!< [4/warning] Warnings (for libs and apps)
    NOTICE   = 5,  //!< [5/notice]  Significant stuff, for example headings (for apps)
    INFO     = 6,  //!< [6/info]    Interesting stuff, the default level (for apps)
    DEBUG    = 7,  //!< [7/debug]   Debugging (for libs and apps)
    TRACE    = 8,  //!< [7/debug]   Extra debugging, only compiled-in in non-Release builds
};  // clang-format on

LoggingLevel& operator++(LoggingLevel& level);      //!< Increase verbosity (pre-increment)
LoggingLevel& operator--(LoggingLevel& level);      //!< Decrease verbosity (pre-decrement)
LoggingLevel operator++(LoggingLevel& level, int);  //!< Increase verbosity (post-increment)
LoggingLevel operator--(LoggingLevel& level, int);  //!< Decrease verbosity (post-decrement)

/**
 * @brief Stringify log level
 *
 * @param[in]  level  The logging level
 *
 * @returns a unique string identifying the level
 */
const char* LoggingLevelStr(const LoggingLevel level);

/**
 * @brief Check if given level would print
 *
 * @param[in]  level  The logging level in question
 *
 * @returns true if the given level would print, false otherwise
 */
bool LoggingIsLevel(const LoggingLevel level);

/**
 * @brief Logging "colours"
 */
enum class LoggingColour
{
    AUTO = 0,  //!< Automatic (default), use colours if stderr is an interactive terminal
    YES,       //!< Use colours (terminal escape sequences)
    NO,        //!< Do not use colours
    JOURNAL,   //!< Use systemd journal level indicators (instead of terminal colours), useful for systemd services
};

/**
 * @brief Stringify log level
 *
 * @param[in]  colour  The logging colour
 *
 * @returns a unique string identifying the level
 */
const char* LoggingColourStr(const LoggingColour colour);

/**
 * @brief Logging timestamps
 */
enum class LoggingTimestamps
{
    NONE = 0,  //!< No timestamps
    RELATIVE,  //!< Relative timestamps (since first logging message, sssss.sss format)
    ABSOLUTE,  //!< Absolute timestamps (local time, yyyy-mm-dd hh::mm:ss.sss format)
};

/**
 * @brief Stringify log level
 *
 * @param[in]  timestamps  The logging timestamps
 *
 * @returns a unique string identifying the level
 */
const char* LoggingTimestampsStr(const LoggingTimestamps timestamps);

struct LoggingParams;  // forward declaration

/**
 * @brief Custom logging write (to screen, file, ...) function signature
 *
 * @param[in]  params  Logging parameters
 * @param[in]  level   Logging level for the message
 * @param[in]  str     The message
 */
using LoggingWriteFunc = void (*)(const LoggingParams& params, const LoggingLevel level, const char* str);

/**
 * @brief Default logging write function
 *
 * @param[in]  params  Logging parameters
 * @param[in]  level   Logging level for the message
 * @param[in]  str     The message
 */
void LoggingDefaultWriteFn(const LoggingParams& params, const LoggingLevel level, const char* str);

/**
 * @brief Logging parameters
 */
struct LoggingParams
{
    /**
     * @brief Constructor
     *
     * @param[in]  level       Logging level
     * @param[in]  colour      Logging colours
     * @param[in]  timestamps  Logging timestamps
     */
    LoggingParams(const LoggingLevel level = LoggingLevel::INFO, const LoggingColour colour = LoggingColour::AUTO,
        const LoggingTimestamps timestamps = LoggingTimestamps::NONE);
    LoggingLevel level_;            //!< Logging level
    LoggingColour colour_;          //!< Level colours
    LoggingTimestamps timestamps_;  //!< Timestamps
    LoggingWriteFunc fn_;           //!< Custom logging write function
    void* user_ = nullptr;          //!< User data, ignored by fpsdk::logging
};

/**
 * @brief Configure logging
 *
 * @param[in]  params  Logging parameters
 *
 * Examples:
 *
 * @code{.cpp}
 * LoggingSetParams({ LoggingLevel::DEBUG });
 * LoggingSetParams({ LoggingLevel::DEBUG, LoggingColour::YES });
 * LoggingSetParams({ LoggingLevel::DEBUG, LoggingColour::AUTO, LoggingTimestamps::RELATIVE });
 * @endcode
 *
 * @returns a copy of the applied logging parameters
 */
LoggingParams LoggingSetParams(const LoggingParams& params);

/**
 * @brief Get current logging params
 *
 * @returns a copy of the current logging params
 */
LoggingParams LoggingGetParams();

/**
 * @brief Print a log message
 *
 * @note Use INFO(), DEBUG(), WARNING() etc. instead of this function.
 *
 * @param[in]  level   Logging level
 * @param[in]  repeat  Number of times the message was repeated (for DEBUG_THR() etc.)
 * @param[in]  fmt     printf() style format string and optional arguments to the format string
 */
void LoggingPrint(const LoggingLevel level, const std::size_t repeat, const char* fmt, ...) PRINTF_ATTR(3);

/**
 * @brief Print a hexdump
 *
 * @note Typically, use DEBUG_HEXDUMP() or TRACE_HEXDUMP() instead of this function.
 *
 * @param[in]  level   Logging level
 * @param[in]  data    Pointer to start of data to dump
 * @param[in]  size    Size of data to dump
 * @param[in]  prefix  Prefix to add to each line, can be NULL to omit
 * @param[in]  fmt     printf() style format string, can be NULL to omit
 * @param[in]  ...     printf() style format arguments
 */
void LoggingHexdump(const LoggingLevel level, const uint8_t* data, const std::size_t size, const char* prefix,
    const char* fmt, ...) PRINTF_ATTR(5);

// Helper macros
#ifndef _DOXYGEN_
#  define _FPSDK_LOGGING_LOG(_level_, ...) \
      ::fpsdk::common::logging::LoggingPrint(::fpsdk::common::logging::LoggingLevel::_level_, 0, __VA_ARGS__)
#  define _FPSDK_LOGGING_HEX(_level_, _data_, _size_, _prefix_, ...) \
      ::fpsdk::common::logging::LoggingHexdump(                      \
          ::fpsdk::common::logging::LoggingLevel::_level_, _data_, _size_, _prefix_, __VA_ARGS__)
#  define _FPSDK_LOGGING_STR(_level_, _expr_)                                                              \
      do {                                                                                                 \
          if (::fpsdk::common::logging::LoggingIsLevel(::fpsdk::common::logging::LoggingLevel::_level_)) { \
              std::stringstream ss;                                                                        \
              ss << _expr_;                                                                                \
              ::fpsdk::common::logging::LoggingPrint(                                                      \
                  ::fpsdk::common::logging::LoggingLevel::_level_, 0, "%s", ss.str().c_str());             \
          }                                                                                                \
      } while (0)
#  define _FPSDK_LOGGING_THR(_level_, _millis_, ...)                                       \
      do {                                                                                 \
          static uint32_t __last = 0;                                                      \
          static uint32_t __repeat = 0;                                                    \
          const uint32_t __now = ::fpsdk::common::time::GetMillis();                       \
          __repeat++;                                                                      \
          if ((__last == 0) || ((__now - __last) >= (_millis_))) {                         \
              __last = __now;                                                              \
              ::fpsdk::common::logging::LoggingPrint(                                      \
                  ::fpsdk::common::logging::LoggingLevel::_level_, __repeat, __VA_ARGS__); \
              __repeat = 0;                                                                \
          }                                                                                \
      } while (0)
#endif  // _DOXYGEN_

/* ****************************************************************************************************************** */
}  // namespace logging
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_LOGGING_HPP__
