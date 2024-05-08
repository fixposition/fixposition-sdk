/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: app utilities
 */
#ifndef __FPCOMMON_APP_HPP__
#define __FPCOMMON_APP_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */

namespace fp {
namespace common {
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
}  // namespace fp
#endif  // __FPCOMMON_APP_HPP__
