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
 */

/* LIBC/STL */
#include <csignal>
#include <iostream>
#include <unistd.h>

/* EXTERNAL */
#include <boost/stacktrace.hpp>
#include <getopt.h>

/* PACKAGE */
#include "fpsdk_common/app.hpp"
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/path.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/thread.hpp"
#include "fpsdk_common/utils.hpp"

namespace fpsdk {
namespace common {
namespace app {
/* ****************************************************************************************************************** */

static bool g_sigint_abort = false;
static bool g_sigint_warn = false;
static sighandler_t g_sigint_old_handler = SIG_IGN;
static fpsdk::common::thread::BinarySemaphore g_sigint_sem;

static void SigIntHandler(int signum)
{
    if ((signum == SIGINT) && !g_sigint_abort) {
        if (g_sigint_warn) {
            WARNING("Caught SIGINT, aborting...");
        } else {
            DEBUG("Caught SIGINT, aborting...");
        }
        g_sigint_abort = true;

        // Handle signal only once, next time let the original handler deal with it
        std::signal(SIGINT, g_sigint_old_handler == SIG_IGN ? SIG_DFL : g_sigint_old_handler);
        g_sigint_old_handler = SIG_IGN;

        g_sigint_sem.Notify();
    }
}

SigIntHelper::SigIntHelper(const bool warn)
{
    if (g_sigint_old_handler == SIG_IGN) {
        g_sigint_old_handler = std::signal(SIGINT, SigIntHandler);
        g_sigint_warn = warn;
    }
}

SigIntHelper::~SigIntHelper()
{
    std::signal(SIGINT, g_sigint_old_handler == SIG_IGN ? SIG_DFL : g_sigint_old_handler);
    g_sigint_sem.Notify();
}

bool SigIntHelper::ShouldAbort()
{
    return g_sigint_abort;
}

bool SigIntHelper::WaitAbort(const uint32_t millis)
{
    // Wait with timeout
    if (millis > 0) {
        return g_sigint_sem.WaitFor(millis) == thread::WaitRes::WOKEN;
    }
    // Wait forever
    else {
        while (true) {
            if (g_sigint_sem.WaitFor(1234) == thread::WaitRes::WOKEN) {
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

static bool g_sigpipe_raised = false;
static bool g_sigpipe_warn = false;
static sighandler_t g_sigpipe_old_handler = SIG_IGN;

static void SigPipeHandler(int signum)
{
    if ((signum == SIGPIPE) && !g_sigpipe_raised) {
        if (g_sigpipe_warn) {
            WARNING("Caught SIGPIPE");
        } else {
            DEBUG("Caught SIGPIPE");
        }
        g_sigpipe_raised = true;

        // Handle signal only once, next time let the original handler deal with it
        std::signal(SIGPIPE, g_sigpipe_old_handler == SIG_IGN ? SIG_DFL : g_sigpipe_old_handler);
        g_sigpipe_old_handler = SIG_IGN;
    }
}

SigPipeHelper::SigPipeHelper(const bool warn)
{
    if (g_sigpipe_old_handler == SIG_IGN) {
        g_sigpipe_old_handler = std::signal(SIGPIPE, SigPipeHandler);
        g_sigpipe_warn = warn;
    }
}

SigPipeHelper::~SigPipeHelper()
{
    std::signal(SIGPIPE, g_sigpipe_old_handler == SIG_IGN ? SIG_DFL : g_sigpipe_old_handler);
}

bool SigPipeHelper::Raised()
{
    return g_sigpipe_raised;
}

// ---------------------------------------------------------------------------------------------------------------------

static sighandler_t g_stacktrace_old_sigsegv = SIG_IGN;
static sighandler_t g_stacktrace_old_sigabrt = SIG_IGN;

static void StacktraceHandler(int /*signum*/)
{
    std::cerr << boost::stacktrace::stacktrace();

    std::signal(SIGSEGV, g_stacktrace_old_sigsegv == SIG_IGN ? SIG_DFL : g_stacktrace_old_sigsegv);
    g_stacktrace_old_sigsegv = SIG_IGN;
    std::signal(SIGABRT, g_stacktrace_old_sigabrt == SIG_IGN ? SIG_DFL : g_stacktrace_old_sigabrt);
    g_stacktrace_old_sigabrt = SIG_IGN;

    std::raise(SIGABRT);
}

StacktraceHelper::StacktraceHelper()
{
    if (g_stacktrace_old_sigsegv == SIG_IGN) {
        g_stacktrace_old_sigsegv = std::signal(SIGSEGV, StacktraceHandler);
    }
    if (g_stacktrace_old_sigabrt == SIG_IGN) {
        g_stacktrace_old_sigabrt = std::signal(SIGABRT, StacktraceHandler);
    }
}

StacktraceHelper::~StacktraceHelper()
{
    std::signal(SIGSEGV, g_stacktrace_old_sigsegv == SIG_IGN ? SIG_DFL : g_stacktrace_old_sigsegv);
    std::signal(SIGABRT, g_stacktrace_old_sigabrt == SIG_IGN ? SIG_DFL : g_stacktrace_old_sigabrt);
}

// ---------------------------------------------------------------------------------------------------------------------

void PrintStacktrace()
{
    std::cerr << boost::stacktrace::stacktrace();
}

/* ****************************************************************************************************************** */

ProgramOptions::ProgramOptions(const std::string& app_name, const std::vector<Option>& options) /* clang-format off */ :
    app_name_        { app_name },
    logging_params_  { logging::LoggingGetParams() },
    options_         { options }  // clang-format on
{
}

ProgramOptions::~ProgramOptions()
{
}

// ---------------------------------------------------------------------------------------------------------------------

bool ProgramOptions::LoadFromArgv(int argc, char** argv)
{
    using namespace fpsdk::common::logging;

    // Save original program options
    for (int ix = 0; ix < argc; ix++) {
        argv_.push_back(argv[ix]);
    }

    // Build arguments for getopt_long()
    std::vector<struct option> long_opts = {
        // clang-format off
        { "version",  no_argument, NULL, 'V' },
        { "help",     no_argument, NULL, 'h' },
        { "verbose",  no_argument, NULL, 'v' },
        { "quiet",    no_argument, NULL, 'q' },
        { "journal",  no_argument, NULL, 'J' },
    };  // clang-format on
    std::string short_opts = ":hVvqJ";
    for (const auto& option : options_) {
        long_opts.push_back({ option.name != nullptr ? option.name : "dummy",
            option.has_argument ? required_argument : no_argument, NULL, option.flag });
        short_opts += string::Sprintf("%c%s", option.flag, option.has_argument ? ":" : "");
    }
    long_opts.push_back({ NULL, 0, NULL, 0 });

    // Process command line arguments
    bool ok = true;
    while (true) {
        int opt_ix = 0;
        const int opt = getopt_long(argc, argv, short_opts.c_str(), long_opts.data(), &opt_ix);
        if (opt < 0) {
            break;
        }
        switch (opt) {
            // Common flags
            case 'V':
                PrintVersion();
                exit(EXIT_SUCCESS);
                break;
            case 'h':
                PrintVersion();
                PrintHelp();
                exit(EXIT_SUCCESS);
                break;
            case 'v':
                logging_params_.level_++;
                break;
            case 'q':
                logging_params_.level_--;
                break;
            case 'J':
                logging_params_.colour_ = logging::LoggingColour::JOURNAL;
                break;
            // Special getopt_long() cases
            case '?':
                WARNING("Invalid option '-%c'", optopt);
                ok = false;
                break;
            case ':':
                WARNING("Missing argument to option '-%c'", optopt);
                ok = false;
                break;
            // App options
            default:
                // Find option
                const auto entry =
                    std::find_if(options_.cbegin(), options_.cend(), [opt](auto& cand) { return cand.flag == opt; });
                const std::string arg = string::Sprintf("%s", entry->has_argument ? optarg : "");
                if ((entry == options_.cend()) || !HandleOption(*entry, arg)) {
                    WARNING("Bad option '-%c%s%s'", opt, arg.empty() ? "" : " ", arg.empty() ? "" : arg.c_str());
                    ok = false;
                }
                break;
        }
    }

    // Setup debugging
    if (logging_params_.level_ >= LoggingLevel::DEBUG) {
        logging_params_.timestamps_ = LoggingTimestamps::RELATIVE;
    }
    LoggingSetParams(logging_params_);
    DEBUG("logging = %s %s %s", LoggingLevelStr(logging_params_.level_), LoggingColourStr(logging_params_.colour_),
        LoggingTimestampsStr(logging_params_.timestamps_));

    // Non-flag arguments
    std::vector<std::string> args;
    while (optind < argc) {
        args.push_back(argv[optind]);
        optind++;
    }

    if (!CheckOptions(args)) {
        ok = false;
    }

    if (!ok) {
        WARNING("Bad options. Try '%s -h'...", app_name_.c_str());
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool ProgramOptions::CheckOptions(const std::vector<std::string>& /*args*/)
{
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void ProgramOptions::PrintVersion()
{
    std::fprintf(stdout, "%s (%s, %s, %s)\n%s\n%s\n", app_name_.c_str(),
#ifdef NDEBUG
        "release",
#else
        "debug",
#endif
#if defined(FPSDK_USE_ROS1)
        "ROS1",
#elif defined(FPSDK_USE_ROS2)
        "ROS2",
#else
        "no ROS",
#endif
        utils::GetVersionString(), utils::GetCopyrightString(), utils::GetLicenseString());
}

/* ****************************************************************************************************************** */

static inline uint64_t GetClockNs(const clockid_t id)
{
    struct timespec t;
    return (clock_gettime(id, &t) == 0) ? ((uint64_t)t.tv_sec * (uint64_t)1000000000) + (uint64_t)t.tv_nsec : 0;
}

// ---------------------------------------------------------------------------------------------------------------------

PerfStats::PerfStats()
{
    start_.SetClockRealtime();
    start_m_ = GetClockNs(CLOCK_MONOTONIC);
    start_c_ = GetClockNs(CLOCK_PROCESS_CPUTIME_ID);
}

// ---------------------------------------------------------------------------------------------------------------------

void PerfStats::Update()
{
    // Get current times
    const uint64_t time_m = GetClockNs(CLOCK_MONOTONIC);
    const uint64_t time_c = GetClockNs(CLOCK_PROCESS_CPUTIME_ID);

    // Update PID (it could have changed!)
    pid_ = getpid();

    // Memory
    std::vector<uint8_t> data;
    std::size_t resident = 0;
    if (path::FileSlurp("/proc/self/statm", data) && !data.empty() &&
        (std::sscanf((const char*)data.data(), "%*s %" SCNuMAX, &resident) == 1) && (resident > 0)) {
        const double page_size = (double)sysconf(_SC_PAGESIZE) * (1.0f / 1024.0f / 1024.0f);
        mem_curr_ = (double)resident * page_size;
        if (mem_curr_ > mem_peak_) {
            mem_peak_ = mem_curr_;
        }
    } else {
        mem_curr_ = 0.0;
    }

    // CPU
    if (time_m > start_m_) {
        cpu_avg_ = (double)(time_c - start_c_) / (double)(time_m - start_m_) * 1e2;
    }
    if (time_m > last_m_) {
        cpu_curr_ = (double)(time_c - last_c_) / (double)(time_m - last_m_) * 1e2;
        if (cpu_curr_ > cpu_peak_) {
            cpu_peak_ = cpu_curr_;
        }
    }
    last_c_ = time_c;
    last_m_ = time_m;

    // Uptime
    time::Time now;
    if (now.SetClockRealtime()) {
        now.Diff(start_, uptime_);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void PerfStats::Reset()
{
    *this = PerfStats();
}

/* ****************************************************************************************************************** */

MemUsage GetMemUsage()
{
    MemUsage mem_usage;
    std::vector<uint8_t> statm;
    std::size_t size = 0, resident = 0, shared = 0, text = 0, data = 0;
    if (path::FileSlurp("/proc/self/statm", statm) && !statm.empty() &&
        (std::sscanf((const char*)statm.data(), "%" SCNuMAX " %" SCNuMAX " %" SCNuMAX " %" SCNuMAX " %*s %" SCNuMAX,
             &size, &resident, &shared, &text, &data) == 5)) {
        const double page_size = (float)sysconf(_SC_PAGESIZE) * (1.0f / 1024.0f / 1024.0f);
        // clang-format off
            mem_usage.size_     = (double)size     * page_size;
            mem_usage.resident_ = (double)resident * page_size;
            mem_usage.shared_   = (double)shared   * page_size;
            mem_usage.text_     = (double)text     * page_size;
            mem_usage.data_     = (double)data     * page_size;
        // clang-format on
    }
    return mem_usage;
}

/* ****************************************************************************************************************** */
}  // namespace app
}  // namespace common
}  // namespace fpsdk
