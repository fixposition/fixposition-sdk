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
#include "time.hpp"

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
 * When the signal is received it (optionally) prints a message and ShouldAbort() and WaitAbort() react accordingly.
 * Also, the signal handler is reset to the previous state (typically, but not necessarily, the default handler) and any
 * further SIGINT triggers the previously set handler. This allows for apps to handle the the first SIGINT
 * nicely and allows the user to "SIGINT again" and trigger the previous handler (typically the default one,
 * i.e. "hard abort").
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
    /**
     * @brief Constructor
     *
     * @param[in]  warn  Print a WARNING() (true, default) or a DEBUG() (false) on signal
     */
    SigIntHelper(const bool warn = true);

    /**
     * @brief Destructor
     */
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
     * @returns true if the signal was raised, false if timeout expired
     */
    bool WaitAbort(const uint32_t millis = 0);
};

/**
 * @brief Helper to catch SIGPIPE
 *
 * On construction this installs a handler for SIGPIE. On destruction it sets the handler back to its previous state.
 * Note that signal handlers are global and therefore you can only use one SigPipeHelper in a app.
 */
class SigPipeHelper
{
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  warn  Print a WARNING() (true) or a DEBUG() (false, default)
     */
    SigPipeHelper(const bool warn = false);

    /**
     * @brief Destructor
     */
    ~SigPipeHelper();

    /**
     * @brief Check if signal was raised
     *
     * @returns true if signal was raised, false otherwise
     */
    bool Raised();
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
     *
     * Reserved options are: 'h' / "help", 'V' / "version", 'v' / "verbose", 'q' / "quiet", 'J' / "journal",  '?', '*',
     * and ':'.
     */
    struct Option
    {
        char flag;                   //!< The flag (some are reserved, see above)
        bool has_argument;           //!< True if flag requires an an argument, false if not
        const char* name = nullptr;  //!< Long option name (or nullptr, some are reserved, see above)
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
     * @brief Print version information
     */
    virtual void PrintVersion();

    /**
     * @brief Handle a command-line flag argument
     *
     * @param[in]  option    The option
     * @param[in]  argument  Option argument (if the option requires one)
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
    virtual bool CheckOptions(const std::vector<std::string>& args);

   protected:
    //! Help screen for common options  @hideinitializer
    static constexpr const char* COMMON_FLAGS_HELP = /* clang-format off */
        "    -h, --help     -- Print program help screen, and exit\n"
        "    -V, --version  -- Print program, version and license information, and exit\n"
        "    -v, --verbose  -- Increase logging verbosity, multiple flags accumulate\n"
        "    -q, --quiet    -- Decrease logging verbosity, multiple flags accumulate\n"
        "    -J, --journal  -- Use systemd journal logging markers instead of colours\n";  // clang-format on

    std::string app_name_;                   //!< App name
    logging::LoggingParams logging_params_;  //!< Logging params
    std::vector<std::string> argv_;          //!< argv[] of program

   private:
    std::vector<Option> options_;  //!< Program options
};

/**
 * @brief App performance stats
 */
struct PerfStats
{
    PerfStats();    //!< Constructor
    void Reset();   //!< Reset stats
    void Update();  //!< Update stats

    // Data, becomes valid after first call to Update()
    double mem_curr_ = 0.0;  //!< Current memory usage [MiB]
    double mem_peak_ = 0.0;  //!< Peak memory usage [MiB]
    double cpu_curr_ = 0.0;  //!< Current (= average since last call to Update()) CPU usage [%]
    double cpu_avg_ = 0.0;   //!< Average (since start) CPU usage [%]
    double cpu_peak_ = 0.0;  //!< Peak CPU usage [%]
    time::Duration uptime_;  //!< Time since start
    uint64_t pid_ = 0;       //!< Process ID

   private:
    time::Time start_;      //!< Start time
    uint64_t start_m_ = 0;  //!< CPU usage
    uint64_t start_c_ = 0;  //!< CPU usage
    uint64_t last_m_ = 0;   //!< CPU usage
    uint64_t last_c_ = 0;   //!< CPU usage
};

/**
 * @brief Memory usage
 */
struct MemUsage
{
    double size_ = 0.0;      //!< Total size [MiB]
    double resident_ = 0.0;  //!< Resident set size [MiB]
    double shared_ = 0.0;    //!< Resident shared [MiB]
    double text_ = 0.0;      //!< Text (code) [MiB]
    double data_ = 0.0;      //!< Data + stack [MiB]
};

/**
 * @brief Get memory usage
 *
 * @returns the current memory usage
 */
MemUsage GetMemUsage();

/* ****************************************************************************************************************** */
}  // namespace app
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_APP_HPP__
