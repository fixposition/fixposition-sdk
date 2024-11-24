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

/* EXTERNAL */
#include <boost/stacktrace.hpp>
#include <getopt.h>

/* PACKAGE */
#include "fpsdk_common/app.hpp"
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/thread.hpp"
#include "fpsdk_common/utils.hpp"

namespace fpsdk {
namespace common {
namespace app {
/* ****************************************************************************************************************** */

static bool g_siginit_abort = false;
static sighandler_t g_sigint_old_handler = SIG_IGN;
static fpsdk::common::thread::BinarySemaphore g_sigint_sem;

static void SigIntHandler(int signum)
{
    if ((signum == SIGINT) && !g_siginit_abort) {
        WARNING("Caught SIGINT, aborting...");
        g_siginit_abort = true;

        // Handle signal only once, next time let the original handler deal with it
        std::signal(SIGINT, g_sigint_old_handler == SIG_IGN ? SIG_DFL : g_sigint_old_handler);
        g_sigint_old_handler = SIG_IGN;

        g_sigint_sem.Notify();
    }
}

SigIntHelper::SigIntHelper()
{
    if (g_sigint_old_handler == SIG_IGN) {
        g_sigint_old_handler = std::signal(SIGINT, SigIntHandler);
    }
}

SigIntHelper::~SigIntHelper()
{
    std::signal(SIGINT, g_sigint_old_handler == SIG_IGN ? SIG_DFL : g_sigint_old_handler);
    g_sigint_sem.Notify();
}

bool SigIntHelper::ShouldAbort()
{
    return g_siginit_abort;
}

bool SigIntHelper::WaitAbort(const uint32_t millis)
{
    // Wait with timeout
    if (millis > 0) {
        return g_sigint_sem.WaitFor(millis);
    }
    // Wait forever
    else {
        while (true) {
            if (g_sigint_sem.WaitFor(1234)) {
                return true;
            }
        }
    }
    return false;
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
    app_name_   { app_name },
    options_    { options }  // clang-format on
{
}

ProgramOptions::~ProgramOptions()
{
}

// ---------------------------------------------------------------------------------------------------------------------

bool ProgramOptions::LoadFromArgv(int argc, char** argv)
{
    // Save original program options
    for (int ix = 0; ix < argc; ix++) {
        argv_.push_back(argv[ix]);
    }

    // Build arguments for getopt_long()
    std::vector<struct option> long_opts = {
        // clang-format off
        { "version", no_argument, NULL, 'V' },
        { "help",    no_argument, NULL, 'h' },
        { "verbose", no_argument, NULL, 'v' },
        { "quiet",   no_argument, NULL, 'q' },
    };  // clang-format on
    std::string short_opts = ":hVvq";
    for (const auto& option : options_) {
        long_opts.push_back({ "dummy", option.has_argument ? required_argument : no_argument, NULL, option.flag });
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
                logging_++;
                break;
            case 'q':
                logging_--;
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
    logging::LoggingSetParams({ logging_ });
    DEBUG("logging = %s", logging::LoggingLevelStr(logging_));

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
        WARNING("Try '%s -h'...", app_name_.c_str());
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

void ProgramOptions::PrintVersion()
{
    std::fprintf(stdout, "%s (%s, %s)\n%s\n%s\n", app_name_.c_str(),
#ifdef NDEBUG
        "release",
#else
        "debug",
#endif
        utils::GetVersionString(), utils::GetCopyrightString(), utils::GetLicenseString());
}

/* ****************************************************************************************************************** */
}  // namespace app
}  // namespace common
}  // namespace fpsdk
