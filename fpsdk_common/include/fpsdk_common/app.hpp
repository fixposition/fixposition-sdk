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
 * @brief Fixposition SDK: Utilities for apps
 *
 * @page FPSDK_COMMON_APPS Utilities for apps
 *
 * **API**: fpsdk_common/app.hpp and fpsdk::common::app
 *
 */
#ifndef __FPSDK_COMMON_APP_HPP__
#define __FPSDK_COMMON_APP_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Utilities for apps
 */
namespace app {
/* ****************************************************************************************************************** */

/**
 * @brief Helper to catch SIGINT (CTRL-c)
 *
 * On construction this installs a handler for SIGINT. On destruction it sets the handler back to its previous state.
 * Note that signal handlers are global and therefore you can only use one SigIntHelper in a app.
 *
 * Example:
 *
 * @code{.cpp}
 * SigIntHelper sigint;
 * while (!sigint.ShouldAbort()) {
 *    // do stuff..
 * }
 *
 * if (sigint.ShouldAbort()) {
 *    INFO("We've been asked to stop");
 * }
 * @endcode
 */
class SigIntHelper
{
   public:
    SigIntHelper();
    ~SigIntHelper();
    /**
     * @brief Check if signal was raised and we should abort
     *
     * @returns true if signal was raised and we should abort, false otherwise
     */
    bool ShouldAbort();

    /**
     * @brief Wait (block) until signal is raised and we should abort
     *
     * @param[in]  millis  Wait at most this long [ms], 0 = forever
     *
     * @returns true if the signal was raised, fals if timeout expired
     */
    bool WaitAbort(const uint32_t millis = 0);
};

/**
 * @brief Helper to print a strack trace on SIGSEGV and SIGABRT
 *
 * On construction this installs a handler for SIGSEGV and SIGABRT, which prints a stack trace.
 * Note that signal handlers are global and therefore you can only use one StacktraceHelper in a app.
 * It is probably a good idea to only include this in non-Release builds.
 *
 * Example:
 *
 * @code{.cpp}
 * int main(int, char**) {
 * #ifndef NDEBUG
 *     StacktraceHelper stacktrace;
 * #endif
 *
 *     // Do stuff...
 *
 *     return 0;
 * }
 * @endcode
 */
class StacktraceHelper
{
   public:
    StacktraceHelper();
    ~StacktraceHelper();
};

/**
 * @brief Prints a stacktrace to stderr
 */
void PrintStacktrace();

/* ****************************************************************************************************************** */
}  // namespace app
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_APP_HPP__
