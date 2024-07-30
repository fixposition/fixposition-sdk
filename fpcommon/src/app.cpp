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

/* PACKAGE */
#include "fpcommon/app.hpp"
#include "fpcommon/logging.hpp"
#include "fpcommon/thread.hpp"

namespace fp {
namespace common {
namespace app {
/* ****************************************************************************************************************** */

static bool g_siginit_abort = false;
static sighandler_t g_sigint_old_handler = SIG_IGN;
static fp::common::thread::BinarySemaphore g_sigint_sem;

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
}  // namespace app
}  // namespace common
}  // namespace fp
