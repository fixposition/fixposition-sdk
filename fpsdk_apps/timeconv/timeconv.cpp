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
 * @brief Fixposition SDK: timeconv main
 */

/* LIBC/STL */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>

/* EXTERNAL */
#include <unistd.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
namespace timeconv {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;

// ---------------------------------------------------------------------------------------------------------------------

// Program options
class TimeConvOptions : public ProgramOptions
{
   public:
    TimeConvOptions()  // clang-format off
        : ProgramOptions("timeconv", { { 'i', false, "ignore" }, { 'p', true, "prec" } }) {};  // clang-format on

    // clang-format off
    bool ignore_bad_ = false;
    int prec_ = 9;
    std::vector<std::string> input_;
    // clang-format on

    void PrintHelp() override final
    {
        // clang-format off
        std::fputs(
            "\n"
            "Convert time between different time systems\n"
            "\n"
            "Usage:\n"
            "\n"
            "    timeconv [flags] [<input...>]\n"
            "\n"
            "Where:\n"
            "\n", stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
            "\n"
            "    -i, --ignore        -- Ignore errors in input (only from stdin, not for input on command line)\n"
            "    -p <n>, --prec <n>  -- Precision (number of fractional digits) for float outputs (default: 3)\n"
            "    <input...>          -- Input data, see below\n"
            "\n"
            "A single <input> is either provided on the command line or on stdin, one <input> per line.\n"
            "The program converts the input to the desired output. One <input> has the following format:\n"
            "\n"
            "    <out> <in> <fields...>\n"
            "\n"
            "The <out> and <in> are the output resp. the input format. They can be specified using\n"
            "the numeric value or a mnemonic (in brackes) given below. Depending on the input format a\n"
            "a different number of <fields> must be provided. The <input> is then converted to a output\n"
            "line with <in> and <out> reversed and the <fields> replaced with the corresponding output\n"
            "format fields. The supported formats are (<...> are the required fields):\n"
            "\n"
            "    11 (clock_rt)  current system time from CLOCK_REALTIME (input only)\n"
            "    12 (clock_tai) current system time from CLOCK_TAI (input only)\n"
            "\n"
            "    21             integer <seconds> and <nanoseconds> (atomic)\n"
            "    22             integer <nanoseconds> (atomic)\n"
            "    23 (atomic)    float <seconds> (atomic)\n"
            "    24             integer (truncated) <seconds> (atomic)\n"
            "\n"
            "    31 (posix)     float <seconds> (POSIX)\n"
            "    32             integer (truncated) <seconds> (POSIX)\n"
            "    33             integer <seconds> and <nanoseconds> (POSIX)\n"
            "    34 (tai)       float <seconds> (TAI)\n"
            "    35             integer (truncated) <seconds> (TAI)\n"
            "\n"
            "    41 (gps)       GPS integer <week> and float <tow>\n"
            "    42 (galileo)   Galileo integer <week> and float <tow>\n"
            "    43 (beidou)    BeiDou integer <week> and float <tow>\n"
            "    44 (glonass)   GLONASS integer <N4>, integer <Nt> and float <TOD>\n"
            "\n"
            "    51 (utc)       integer <year>, <month>, <day>, <hour>, <minute> and float <second> (UTC)\n"
            "    52 (doy)       float <day_of_year> (output only)\n"
            "\n"
            "     0   Ignore, don't generate output\n"
            "\n"
            "The 'atomic' time is seconds since 1970-01-01 00:00:00 UTC, including leapseconds. POSIX is\n"
            "seconds since the same epoch, but not including leap seconds, i.e. with ambiguities at/during\n"
            "leapseconds events. Note that 'strict' POSIX rules are used, which may not be consistent with\n"
            "a Linux system's CLOCK_REALTIME. TAI is like 'atomic' but with an offset of 10 (leapsconds).\n"
            "Input format 12 may not work correctly if the system isn't configured for leapseconds.\n"
            "Conversion involving floats may be subject to loss in precision.\n"
            "See also the documentation in time.hpp.\n"
            "\n"
            "Examples:\n"
            "\n"
            "    Get current system time as GPS week number and time of week:\n"
            "\n"
            "        timeconv 41 11\n"
            "        echo 41 11 | timeconv\n"
            "\n"
            "    Convert GPS time to BeiDou time:\n"
            "\n"
            "        timeconv  43 41 2341 100000\n"
            "\n"
            "    Get current time in UTC, GPS, Galileo, BeiDou and GLONASS time:\n"
            "\n"
            "        echo -e \"51 11\\n41 11\\n42 11\\n 43 11\\n 44 11\" | timeconv -p 0\n"
            "\n"
            "\n", stdout);
        // clang-format on
    }

    bool HandleOption(const Option& option, const std::string& argument) final
    {
        bool ok = true;
        switch (option.flag) {
            case 'i':
                ignore_bad_ = true;
                break;
            case 'p':
                if (!StrToValue(argument, prec_) || (prec_ < 0) || (prec_ > 12)) {
                    ok = false;
                }
                break;
            default:
                ok = false;
                break;
        }
        return ok;
    }

    bool CheckOptions(const std::vector<std::string>& args) final
    {
        bool ok = true;
        input_ = args;
        DEBUG("ignore_bad = %s", ToStr(ignore_bad_));
        DEBUG("num_frac   = %d", prec_);
        DEBUG("input_     = [ %s ]",
            StrJoin(StrMap(input_, [](const std::string& s) { return "\"" + s + "\""; }), ", ").c_str());
        return ok;
    }
};

/* ****************************************************************************************************************** */

class TimeConv
{
   public:
    TimeConv(const TimeConvOptions& opts) : opts_{ opts }
    {
    }

    bool Run()
    {
        bool ok = true;

        // Input from command line
        std::string error;
        if (!opts_.input_.empty()) {
            ok = ProcessInput(opts_.input_, error);
            if (!error.empty()) {
                WARNING("%s", error.c_str());
            }
        }
        // Process input lines
        else {
            char line[10000];
            while (ok && (fgets(line, sizeof(line), stdin) != NULL)) {
                // @todo strip # style comments from input
                std::vector<std::string> input;
                const char* sep = " \t\n";
                for (char *save = NULL, *tok = strtok_r(line, sep, &save); tok != NULL;
                    tok = strtok_r(NULL, sep, &save)) {
                    input.push_back(tok);
                }
                // Ignore empty lines
                if (input.empty()) {
                    continue;
                }
                if (!ProcessInput(input, error) && !opts_.ignore_bad_) {
                    if (!error.empty()) {
                        WARNING("%s", error.c_str());
                    }
                    ok = false;
                }
            }
        }

        return ok;
    }

   private:
    // -----------------------------------------------------------------------------------------------------------------

    TimeConvOptions opts_;  //!< Program options

    bool ProcessInput(const std::vector<std::string>& input, std::string& error)
    {
        DEBUG("ProcessInput() %s",
            StrJoin(StrMap(input, [](const std::string& s) { return "\"" + s + "\""; }), ", ").c_str());

        if (input.size() < 2) {
            error = "too few fields";
            return false;
        }

        const std::unordered_map<std::string, int> FMTS = { { "clock_rt", 11 }, { "clock_tai", 12 }, { "atomic", 23 },
            { "posix", 31 }, { "tai", 34 }, { "gps", 41 }, { "galileo", 42 }, { "beidou", 43 }, { "glonass", 44 },
            { "utc", 51 }, { "doy", 52 } };
        const auto in_str = StrToLower(input[1]);
        const auto out_str = StrToLower(input[0]);
        int in = 0;
        int out = 0;
        auto in_fmt = FMTS.find(in_str);
        if (in_fmt != FMTS.end()) {
            in = in_fmt->second;
        } else {
            StrToValue(in_str, in);
        }
        auto out_fmt = FMTS.find(out_str);
        if (out_fmt != FMTS.end()) {
            out = out_fmt->second;
        } else {
            StrToValue(out_str, out);
        }

        bool ok = true;
        Time t;
        switch (in) {
            case 11:
                ok = t.SetClockRealtime();
                in = 0;
                break;
            case 12:
                ok = t.SetClockTai();
                in = 0;
                break;
            case 21: {
                uint32_t sec = 0;
                uint32_t nsec = 0;
                ok = ((input.size() == 4) && StrToValue(input[2], sec) && StrToValue(input[3], nsec) &&
                      t.SetSecNSec(sec, nsec));
                break;
            }
            case 22: {
                uint64_t nsec = 0;
                ok = ((input.size() == 3) && StrToValue(input[2], nsec) && t.SetNSec(nsec));
                break;
            }
            case 23: {
                double sec = 0.0;
                ok = ((input.size() == 3) && StrToValue(input[2], sec) && t.SetSec(sec));
                break;
            }
            case 24: {
                double sec = 0.0;
                ok = ((input.size() == 3) && StrToValue(input[2], sec) && t.SetSec(std::floor(sec)));
                break;
            }
            case 31: {
                double sec = 0.0;
                ok = ((input.size() == 3) && StrToValue(input[2], sec)) && t.SetPosixSec(sec);
                break;
            }
            case 32: {
                double sec = 0.0;
                ok = ((input.size() == 3) && StrToValue(input[2], sec) && t.SetPosix(std::floor(sec)));
                break;
            }
            case 33: {
                RosTime rostime;
                ok = ((input.size() == 4) && StrToValue(input[2], rostime.sec_) &&
                      StrToValue(input[3], rostime.nsec_) && t.SetRosTime(rostime));
                break;
            }
            case 34: {
                double sec = 0.0;
                ok = ((input.size() == 3) && StrToValue(input[2], sec)) && t.SetTaiSec(sec);
                break;
            }
            case 35: {
                double sec = 0.0;
                ok = ((input.size() == 3) && StrToValue(input[2], sec) && t.SetTai(std::floor(sec)));
                break;
            }
            case 41: {
                WnoTow wnotow(WnoTow::Sys::GPS);
                ok = ((input.size() == 4) && StrToValue(input[2], wnotow.wno_) && StrToValue(input[3], wnotow.tow_) &&
                      t.SetWnoTow(wnotow));
                break;
            }
            case 42: {
                WnoTow wnotow(WnoTow::Sys::GAL);
                ok = ((input.size() == 4) && StrToValue(input[2], wnotow.wno_) && StrToValue(input[3], wnotow.tow_) &&
                      t.SetWnoTow(wnotow));
                break;
            }
            case 43: {
                WnoTow wnotow(WnoTow::Sys::BDS);
                ok = ((input.size() == 4) && StrToValue(input[2], wnotow.wno_) && StrToValue(input[3], wnotow.tow_) &&
                      t.SetWnoTow(wnotow));
                break;
            }
            case 45: {
                GloTime glotime;
                ok = ((input.size() == 5) && StrToValue(input[2], glotime.N4_) && StrToValue(input[3], glotime.Nt_) &&
                      StrToValue(input[3], glotime.TOD_) && t.SetGloTime(glotime));
                break;
            }
            case 51: {
                UtcTime utctime;
                ok = ((input.size() == 8) && StrToValue(input[2], utctime.year_) &&
                      StrToValue(input[3], utctime.month_) && StrToValue(input[4], utctime.day_) &&
                      StrToValue(input[5], utctime.hour_) && StrToValue(input[6], utctime.min_) &&
                      StrToValue(input[7], utctime.sec_) && t.SetUtcTime(utctime));
                break;
            }
            // case 52: {
            //     double doy = 0.0;
            //     ok = ((input.size() == 3) && StrToValue(input[2], doy) && t.SetDayOfYear(doy));
            // }
            default:
                ok = false;
                break;
        }
        if (!ok) {
            error = Sprintf("bad <in> %s (%d)", in_str.c_str(), in);
            return false;
        }

        const int pp = (opts_.prec_ > 0 ? 1 : 0) + opts_.prec_;
        switch (out) {
            case 21:
                std::printf("%2d %2d %11" PRIu32 " %9" PRIu32, in, out, t.sec_, t.nsec_);
                break;
            case 22:
                std::printf("%2d %2d %20" PRIu64, in, out, t.GetNSec());
                break;
            case 23:
                std::printf("%2d %2d %*.*f", in, out, 11 + pp, opts_.prec_, t.GetSec(opts_.prec_));
                break;
            case 24:
                std::printf("%2d %2d %11" PRIu32, in, out, t.sec_);
                break;
            case 31: {
                std::printf("%2d %2d %*.*f", in, out, 11 + pp, opts_.prec_, t.GetPosixSec());
                break;
            }
            case 32:
                std::printf("%2d %2d %11" PRIi64, in, out, t.GetPosix());
                break;
            case 33: {
                const auto rostime = t.GetRosTime();
                std::printf("%2d %2d %11" PRIu32 " %9" PRIu32, in, out, rostime.sec_, rostime.nsec_);
                break;
            }
            case 34:
                std::printf("%2d %2d %*.*f", in, out, 11 + pp, opts_.prec_, t.GetTaiSec());
                break;
            case 35:
                std::printf("%2d %2d %11" PRIi64, in, out, t.GetTai());
                break;
            case 41: {
                const auto wnotow = t.GetWnoTow(WnoTow::Sys::GPS, opts_.prec_);
                std::printf("%2d %2d %4d %*.*f", in, out, wnotow.wno_, 6 + pp, opts_.prec_, wnotow.tow_);
                break;
            }
            case 42: {
                const auto wnotow = t.GetWnoTow(WnoTow::Sys::GAL, opts_.prec_);
                std::printf("%2d %2d %4d %*.*f", in, out, wnotow.wno_, 6 + pp, opts_.prec_, wnotow.tow_);
                break;
            }
            case 43: {
                const auto wnotow = t.GetWnoTow(WnoTow::Sys::BDS, opts_.prec_);
                std::printf("%2d %2d %4d %*.*f", in, out, wnotow.wno_, 6 + pp, opts_.prec_, wnotow.tow_);
                break;
            }
            case 44: {
                const auto glotime = t.GetGloTime(opts_.prec_);
                std::printf(
                    "%2d %2d %2d %4d %*.*f", in, out, glotime.N4_, glotime.Nt_, 5 + pp, opts_.prec_, glotime.TOD_);
                break;
            }
            case 51: {
                const auto utc = t.GetUtcTime(opts_.prec_);
                std::printf("%2d %2d %4d %2d %2d %2d %2d %*.*f", in, out, utc.year_, utc.month_, utc.day_, utc.hour_,
                    utc.min_, 2 + pp, opts_.prec_, utc.sec_);
                break;
            }
            case 52:
                std::printf("%2d %2d %*.*f", in, out, 3 + pp, opts_.prec_, t.GetDayOfYear(opts_.prec_));
                break;
            case 0:
                break;
            default:
                ok = false;
                break;
        }
        if (ok) {
            std::printf("  # input: %s\n", StrJoin(input, " ").c_str());
        } else {
            error = Sprintf("bad <out> %s (%d)", out_str.c_str(), out);
            return false;
        }

        return true;
    }
};

/* ****************************************************************************************************************** */
}  // namespace timeconv
}  // namespace apps
}  // namespace fpsdk

/* ****************************************************************************************************************** */

int main(int argc, char** argv)
{
    using namespace fpsdk::apps::timeconv;
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    TimeConvOptions opts;
    if (!opts.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    if (ok) {
        TimeConv app(opts);
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
