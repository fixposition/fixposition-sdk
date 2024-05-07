/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com)
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Minimal logging
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
namespace logging {
/* ****************************************************************************************************************** */

/**
 * @name Printf() style logging
 *
 * For example, `INFO("Hello world, the numner is %d", 42);`
 *
 * @{
 */
// clang-format off
#ifndef NDEBUG // Only for non-Release builds
#  define TRACE(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::TRACE,               fmt, ## __VA_ARGS__)
#define TRACE_HEXDUMP(data, size, prefix, fmt, ...) fp::common::logging::LoggingHexdump(fp::common::logging::LoggingLevel::TRACE, data, size, prefix, fmt, ## __VA_ARGS__)
#else
#  define TRACE(...) /* nothing */
#  define TRACE_HEXDUMP(...) /* nothing */
#endif
#define DEBUG(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::DEBUG,               fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)    fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::INFO,                fmt, ## __VA_ARGS__)
#define NOTICE(fmt, ...)  fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::NOTICE,              fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::WARNING, "Warning: " fmt, ## __VA_ARGS__)
#define ERROR(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::ERROR,   "Error: "   fmt, ## __VA_ARGS__)
#define FATAL(fmt, ...)   fp::common::logging::LoggingPrint(fp::common::logging::LoggingLevel::FATAL,   "Fatal: "   fmt, ## __VA_ARGS__)
#define DEBUG_HEXDUMP(data, size, prefix, fmt, ...) fp::common::logging::LoggingHexdump(fp::common::logging::LoggingLevel::DEBUG, data, size, prefix, fmt, ## __VA_ARGS__)
// clang-format on
///@}

/**
 * @name C++ style logging
 *
 * For example, `INFO_S("Hello world, the numner is " << 42);`
 *
 * @{
 */
// clang-format off
#ifndef NDEBUG // Only for non-Release builds
#  define TRACE_S(expr)   do { std::stringstream _ss; _ss << expr; TRACE(  "%s", _ss.str().c_str()); } while (false)
#else
#  define TRACE_S(...) /* nothing */
#endif
#define DEBUG_S(expr)   do { std::stringstream _ss; _ss << expr; DEBUG(  "%s", _ss.str().c_str()); } while (false)
#define INFO_S(expr)    do { std::stringstream _ss; _ss << expr; INFO(   "%s", _ss.str().c_str()); } while (false)
#define NOTICE_S(expr)  do { std::stringstream _ss; _ss << expr; NOTICE( "%s", _ss.str().c_str()); } while (false)
#define WARNING_S(expr) do { std::stringstream _ss; _ss << expr; WARNING("%s", _ss.str().c_str()); } while (false)
#define ERROR_S(expr)   do { std::stringstream _ss; _ss << expr; ERROR(  "%s", _ss.str().c_str()); } while (false)
#define FATAL_S(expr)   do { std::stringstream _ss; _ss << expr; FATAL(  "%s", _ss.str().c_str()); } while (false)
// clang-format on
///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Logging verbosity levels, default is INFO
 */
enum class LoggingLevel {
    FATAL,    //!< Panic! Cannot be silenced.
    ERROR,    //!< Errors, reserved for apps, should not be used by libraries
    WARNING,  //!< Warnings
    NOTICE,   //!< Noticable stuff
    INFO,     //!< Interesting stuff, the default level
    DEBUG,    //!< Debugging
    TRACE     //!< Extra debugging, on compiled-in in non-Release builds
};

LoggingLevel& operator++(LoggingLevel& level);      //!< pre-increment
LoggingLevel& operator--(LoggingLevel& level);      //!< pre-decrement
LoggingLevel operator++(LoggingLevel& level, int);  //!< post-increment
LoggingLevel operator--(LoggingLevel& level, int);  //!< post-decrement

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
    JOURNAL,   //!< Use systemd level indicators (instead of terminal colours)
};

/**
 * @brief Logging parameters
 */
struct LoggingParams {
    LoggingParams(const LoggingLevel level = LoggingLevel::INFO, const LoggingColour colour = LoggingColour::AUTO);
    LoggingLevel level_;                             //!< Logging level
    LoggingColour colour_;                           //!< Level colours
    void (*fn_)(const LoggingParams&, const char*);  //!< Custom logging print function
};

/**
 * @brief Configure logging
 *
 * @param[in]  params  Logging parameters
 *
 * Examples:
 *
 * @code{.cpp}
 * LoggingSetup({LoggingLevel::DEBUG});
 * LoggingSetup({LoggingLevel::DEBUG, LoggingColour::YES});
 * @endcode
 *
 * @returns a copy of the applied logging parameters
 */
LoggingParams LoggingSetup(const LoggingParams& params);

/**
 * @brief Print a log message
 *
 * @note Use INFO(), DEBUG(), WARNING() etc. instead of this function.
 *
 * @param[in] level  Logging level
 * @param[in] fmt    printf() style format string
 * @param[in] ...    arguments to the format string
 */
void LoggingPrint(const LoggingLevel level, const char* fmt, ...) PRINTF_ATTR(2);

/**
 * @brief Print a hexdump
 *
 * @note Typically, use TRACE_HEXDUMP() or DEBUG_HEXDUMP() instead of this function.
 *
 * @param[in] level  Logging level
 * @param[in] data   Pointer to start of data to dump
 * @param[in] size   Size of data to dump
 * @param[in] prefix prefix to add to each line, can be NULL to omit
 * @param[in] fmt    printf() style format string (for a first line to print), can be NULL to omit
 * @param[in] ...    arguments to the format string
 */
void LoggingHexdump(const LoggingLevel level, const uint8_t* data, const uint64_t size, const char* prefix,
    const char* fmt, ...) PRINTF_ATTR(5);

/* ****************************************************************************************************************** */
}  // namespace logging
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_LOGGING_HPP__
