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
 * @brief Fixposition SDK: timetool main
 */

/* LIBC/STL */
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <string>

/* EXTERNAL */
#include <sys/timex.h>
#include <unistd.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
namespace timetool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;

// ---------------------------------------------------------------------------------------------------------------------

// Program options
class TimeToolOptions : public ProgramOptions
{
   public:
    TimeToolOptions()  // clang-format off
        : ProgramOptions("timetool", { }) {};  // clang-format on

    std::string op_;
    std::vector<std::string> args_;

    void PrintHelp() override final
    {
        // clang-format off
        std::fputs(
            "\n"
            "Time related utilities\n"
            "\n"
            "Usage:\n"
            "\n"
            "    timetool [flags] <op> [<args...>]\n"
            "\n"
            "Where:\n"
            "\n", stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
            "\n"
            "Available <op>erations and their <args...> are:\n"
            "\n"
            "    adjtimex(2) related operations:\n"
            "\n"
            "        ADJ_TAI         -- Get (print) current value of ADJ_TAI\n"
            "        ADJ_TAI <val>   -- Set current value of ADJ_TAI to <val> seconds, and print it\n"
            "\n"
            "    fp::common::time::Time related functions:\n"
            "\n"
            "        GetLeapseconds  -- Get (print) current leapseconds (TAI - UTC) from built-in table\n"
            "\n"
            "\n", stdout);
        // clang-format on
    }

    bool HandleOption(const Option& option, const std::string& /*argument*/) final
    {
        bool ok = true;
        switch (option.flag) {
            default:
                ok = false;
                break;
        }
        return ok;
    }

    bool CheckOptions(const std::vector<std::string>& args) final
    {
        bool ok = true;
        if (args.empty()) {
            WARNING("Need an <op>");
            return false;
        }

        op_ = args[0];
        if (args.size() > 0) {
            std::copy(args.begin() + 1, args.end(), std::back_inserter(args_));
        }

        DEBUG("op    = %s", op_.c_str());
        DEBUG("args  = [%s]", StrJoin(args_, "] [").c_str());

        return ok;
    }
};

/* ****************************************************************************************************************** */

class TimeTool
{
   public:
    TimeTool(const TimeToolOptions& opts) : opts_{ opts }
    {
    }

    bool Run()
    {
        bool ok = false;

        // Set/get TAI offset
        if (opts_.op_ == "ADJ_TAI") {
            struct timex tx;
            std::memset(&tx, 0, sizeof(tx));
            int tai = 0;

            // Set
            if ((opts_.args_.size() == 1) && StrToValue(opts_.args_[0], tai)) {
                tx.constant = tai;
                tx.modes = ADJ_TAI;
                if (adjtimex(&tx) < 0) {
                    WARNING("adjtimex({ modes=ADJ_TAI, constant=%d }) fail: %s", tai, StrError(errno).c_str());
                } else {
                    std::printf("%d\n", tx.tai);
                    ok = true;
                }
            }
            // Get
            else if (opts_.args_.size() == 0) {
                if (adjtimex(&tx) < 0) {
                    WARNING("adjtimex() fail: %s", StrError(errno).c_str());
                } else {
                    std::printf("%d\n", tx.tai);
                    ok = true;
                }
            }
        }
        // Get current leapseconds a.k.a. TAI offset
        else if ((opts_.op_ == "GetLeapseconds") && (opts_.args_.size() == 0)) {
            std::printf("%d\n", Time::FromClockRealtime().GetLeapseconds());
            ok = true;
        }

        if (!ok) {
            WARNING("Bad <op> or <args...>");
        }

        return ok;
    }

   private:
    // -----------------------------------------------------------------------------------------------------------------

    TimeToolOptions opts_;  //!< Program options
};

/* ****************************************************************************************************************** */
}  // namespace timetool
}  // namespace apps
}  // namespace fpsdk

/* ****************************************************************************************************************** */

int main(int argc, char** argv)
{
    using namespace fpsdk::apps::timetool;
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    TimeToolOptions opts;
    if (!opts.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    if (ok) {
        TimeTool app(opts);
        ok = app.Run();
    }

    // Are we happy?
    if (ok) {
        return EXIT_SUCCESS;
    } else {
        ERROR("Failed");
        return EXIT_FAILURE;
    }
}

/* ****************************************************************************************************************** */
