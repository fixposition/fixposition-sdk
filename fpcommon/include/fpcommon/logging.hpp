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
 * @page FPCOMMON_LOGGING Logging
 *
 * API: fp::common::logging
 *
 */
#ifndef __FPCOMMON_LOGGING_HPP__
#define __FPCOMMON_LOGGING_HPP__

/* LIBC/STL */
#include <cinttypes>  // PRI.. macros, often used with formats
#include <cstdarg>
#include <sstream>

/* EXTERNAL */

/* PACKAGE */
#include "string.hpp"

namespace fp {
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
 * @brief Print a fatal message   @hideinitializer
 */
#define FATAL(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::FATAL,   "Fatal: "   fmt, ## __VA_ARGS__)
/**
 * @brief Print a error message   @hideinitializer
 */
#define ERROR(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::ERROR,   "Error: "   fmt, ## __VA_ARGS__)
/**
 * @brief Print a warning message   @hideinitializer
 */
#define WARNING(fmt, ...) fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::WARNING, "Warning: " fmt, ## __VA_ARGS__)
/**
 * @brief Print a notice message   @hideinitializer
 */
#define NOTICE(fmt, ...)  fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::NOTICE,              fmt, ## __VA_ARGS__)
/**
 * @brief Print a info message   @hideinitializer
 */
#define INFO(fmt, ...)    fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::INFO,                fmt, ## __VA_ARGS__)
/**
 * @brief Print a debug message   @hideinitializer
 */
#define DEBUG(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::DEBUG,               fmt, ## __VA_ARGS__)
/**
 * @brief Print a debug hexdump   @hideinitializer
 */
#define DEBUG_HEXDUMP(data, size, prefix, fmt, ...) fp::common::logging::LoggingHexdump(fp::common::logging::LoggingLevel::DEBUG, data, size, prefix, fmt, ## __VA_ARGS__)
#if !defined(NDEBUG) || defined(_DOXYGEN_) // Only for non-Release builds
/**
 * @brief Print a trace message (only debug builds, compiled out in release builds)  @hideinitializer
 */
#  define TRACE(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::TRACE,               fmt, ## __VA_ARGS__)
/**
 * @brief Print a trace hexdump (only debug builds, compiled out in release builds)   @hideinitializer
 */
#  define TRACE_HEXDUMP(data, size, prefix, fmt, ...) fp::common::logging::LoggingHexdump(fp::common::logging::LoggingLevel::TRACE, data, size, prefix, fmt, ## __VA_ARGS__)
#else
#  define TRACE(...)         /* nothing */
#  define TRACE_HEXDUMP(...) /* nothing */
#endif
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
 * @brief Print a fatal message   @hideinitializer
 */
#define FATAL_S(expr)   if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::FATAL)) { \
    std::stringstream ss; ss << expr; FATAL("%s", ss.str().c_str()); }
/**
 * @brief Print a error message   @hideinitializer
 */
#define ERROR_S(expr)   if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::ERROR)) { \
    std::stringstream ss; ss << expr; ERROR("%s", ss.str().c_str()); }
/**
 * @brief Print a warning message   @hideinitializer
 */
#define WARNING_S(expr) if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::WARNING)) { \
    std::stringstream ss; ss << expr; WARNING("%s", ss.str().c_str()); }
/**
 * @brief Print a debug message   @hideinitializer
 */
#define DEBUG_S(expr)   if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::DEBUG)) { \
    std::stringstream ss; ss << expr; DEBUG("%s", ss.str().c_str()); }
/**
 * @brief Print a notice message   @hideinitializer
 */
#define NOTICE_S(expr)  if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::NOTICE)) { \
    std::stringstream ss; ss << expr; NOTICE("%s", ss.str().c_str()); }
/**
 * @brief Print a info message   @hideinitializer
 */
#define INFO_S(expr)    if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::INFO)) { \
    std::stringstream ss; ss << expr; INFO("%s", ss.str().c_str()); }
#if !defined(NDEBUG) || defined(_DOXYGEN_) // Only for non-Release builds
/**
 * @brief Print a trace message (only debug builds, compiled out in release builds)   @hideinitializer
 */
#  define TRACE_S(expr)   if (fp::common::logging::LoggingIsLevel(fp::common::logging::LoggingLevel::TRACE)) { \
    std::stringstream ss; ss << expr; TRACE("%s", ss.str().c_str()); }
#else
#  define TRACE_S(...) /* nothing */
#endif
// clang-format on
///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Logging verbosity levels, default is INFO
 *
 * The logging levels loosely follow syslog levels (indicated in [] below, see also
 * https://en.wikipedia.org/wiki/Syslog)
 *
 * Libraries (fpcommon, fpros1, ...) code shall only use WARNING and DEBUG.
 */
enum class LoggingLevel {
    FATAL,    //!< [2/crit]    Hard errors, critical conditions (for apps). Cannot be silenced.
    ERROR,    //!< [3/err]     Errors (for apps)
    WARNING,  //!< [4/warning] Warnings (for libs and apps)
    NOTICE,   //!< [5/notice]  Significant stuff, for example headings (for apps)
    INFO,     //!< [6/info]    Interesting stuff, the default level (for apps)
    DEBUG,    //!< [7/debug]   Debugging (for libs and apps)
    TRACE     //!< [7/debug]   Extra debugging, only compiled-in in non-Release builds
};

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
enum class LoggingColour {
    AUTO = 0,  //!< Automatic (default), use colours if stderr is an interactive terminal
    YES,       //!< Use colours (terminal escape sequences)
    NO,        //!< Do not use colours
    JOURNAL,   //!< Use systemd journal level indicators (instead of terminal colours), useful for systemd services
};

struct LoggingParams;  // forward declaration

/**
 * @brief Custom logging print function signature
 */
using LoggingPrintFunc = void (*)(const LoggingParams&, const LoggingLevel, const char*);

/**
 * @brief Logging parameters
 */
struct LoggingParams {
    /**
     * @brief Constructor
     *
     * @param[in]  level   Logging level
     * @param[in]  colour  Logging colours
     */
    LoggingParams(const LoggingLevel level = LoggingLevel::INFO, const LoggingColour colour = LoggingColour::AUTO);
    LoggingLevel level_;    //!< Logging level
    LoggingColour colour_;  //!< Level colours
    LoggingPrintFunc fn_;   //!< Custom logging print function
};

/**
 * @brief Configure logging
 *
 * @param[in]  params  Logging parameters
 *
 * Examples:
 *
 * @code{.cpp}
 * LoggingSetParams({LoggingLevel::DEBUG});
 * LoggingSetParams({LoggingLevel::DEBUG, LoggingColour::YES});
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
 * @param[in]  level  Logging level
 * @param[in]  fmt    printf() style format string
 * @param[in]  ...    arguments to the format string
 */
void LoggingPrint(const LoggingLevel level, const char* fmt, ...) PRINTF_ATTR(2);

/**
 * @brief Print a hexdump
 *
 * @note Typically, use DEBUG_HEXDUMP() or TRACE_HEXDUMP() instead of this function.
 *
 * @param[in]  level   Logging level
 * @param[in]  data    Pointer to start of data to dump
 * @param[in]  size    Size of data to dump
 * @param[in]  prefix  Prefix to add to each line, can be NULL to omit
 * @param[in]  fmt     printf() style format string (for a first line to print), can be NULL to omit
 * @param[in]  ...     Arguments to the format string
 */
void LoggingHexdump(const LoggingLevel level, const uint8_t* data, const uint64_t size, const char* prefix,
    const char* fmt, ...) PRINTF_ATTR(5);

/* ****************************************************************************************************************** */
}  // namespace logging
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_LOGGING_HPP__
