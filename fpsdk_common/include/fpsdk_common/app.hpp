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
#include "logging.hpp"

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

/**
 * @brief Program options
 */
class ProgramOptions
{
   public:
    /**
     * @brief A program option
     */
    struct Option
    {
        char flag;          //!< The flag (reserved: 'h', 'V', 'v', 'q', '?', '*', ':')
        bool has_argument;  //!< True if flag requires an an argument, false if not
    };

    /**
     * @brief Constructor
     */
    ProgramOptions(const std::string& app_name, const std::vector<Option>& options);

    /**
     * @brief Destructor
     */
    virtual ~ProgramOptions();

    /**
     * @brief Load arguments from argv[]
     *
     * @param[in,out]  argc  Number of arguments
     * @param[in,out]  argv  Command-line arguments
     * @return
     */
    bool LoadFromArgv(int argc, char** argv);

    /**
     * @brief Print the help screen and exit(0)
     */
    virtual void PrintHelp() = 0;

    /**
     * @brief Handle a command-line flag argument
     *
     * @param[in]  option    The option
     * @param[in]  argument  Optional argument
     *
     * @returns true if option was accepted, false otherwise
     */
    virtual bool HandleOption(const Option& option, const std::string& argument) = 0;

    /**
     * @brief Check options, and handle non-flag arguments
     *
     * @param[in]  args  The non-flag arguments
     *
     * @returns true if options are good, false otherwise
     */
    virtual bool CheckOptions(const std::vector<std::string>& args)
    {
        (void)args;
        return true;
    }

    //! Help screen for common options  @hideinitializer
    static constexpr const char* COMMON_FLAGS_HELP = /* clang-format off */
        "    -h       -- Print program help screen, and exit\n"
        "    -V       -- Print program, version and license information, and exit\n"
        "    -v / -q  -- Increase / decrease logging verbosity, multiple flags accumulate\n";  // clang-format on

    std::string app_name_;                                                              //!< App name
    logging::LoggingLevel logging_level_ = logging::LoggingLevel::INFO;                 //!< Logging verbosity level
    logging::LoggingTimestamps logging_timestamps_ = logging::LoggingTimestamps::NONE;  //!< Logging timestamps
    std::vector<std::string> argv_;                                                     //!< argv[] of program

   private:
    std::vector<Option> options_;  //!< Program options
    void PrintVersion();
};

/* ****************************************************************************************************************** */
}  // namespace app
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_APP_HPP__
