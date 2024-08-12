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
 * @brief Fixposition SDK: fpltool params (command line arguments)
 */

/* LIBC/STL */
#include <cinttypes>
#include <vector>

/* EXTERNAL */
#include <getopt.h>
#include <unistd.h>

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/utils.hpp>

/* PACKAGE */
#include "fpltool_args.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::logging;
using namespace fpsdk::common::string;
using namespace fpsdk::common::utils;

// ---------------------------------------------------------------------------------------------------------------------

static void PrintVersionInfo()
{
    std::fprintf(stdout, "fpltool (%s, %s)\n%s\n%s\n",
#ifdef NDEBUG
        "release",
#else
        "debug",
#endif
        GetVersionString(), GetCopyrightString(), GetLicenseString());
}

// ---------------------------------------------------------------------------------------------------------------------

static void PrintHelpScreen()
{
    std::fputs(FpltoolArgs::USAGE_HELP, stdout);
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpltoolArgs::LoadFromArgv(int argc, char** argv)
{
    bool ok = true;

    // Save original argv[]
    for (int ix = 0; ix < argc; ix++) {
        argv_.push_back(argv[ix]);
    }

    bool progress_set = false;

    while (true) {
        // Command line options
        static const struct option long_opts[] = {
            // clang-format off
            { "version",         no_argument,       NULL, 'V' },
            { "help",            no_argument,       NULL, 'h' },
            { "verbose",         no_argument,       NULL, 'v' },
            { "quiet",           no_argument,       NULL, 'q' },
            { "output",          required_argument, NULL, 'o' },
            { "force",           no_argument,       NULL, 'f' },
            { "extra",           no_argument,       NULL, 'x' },
            { "progress",        no_argument,       NULL, 'p' },
            { "no-progress",     no_argument,       NULL, 'P' },
            { "compress",        no_argument,       NULL, 'c' },
            { "skip",            required_argument, NULL, 'S' },
            { "proc",            required_argument, NULL, 'D' },
            { NULL, 0, NULL, 0 },
        };  // clang-format on
        const char* short_opts = ":hVvqo:fxpPcS:D:";

        // Process all command line options
        int opt_ix = 0;
        const int opt = getopt_long(argc, argv, short_opts, long_opts, &opt_ix);
        if (opt < 0) {
            break;
        }
        switch (opt) {
            case 'V':
                PrintVersionInfo();
                exit(EXIT_SUCCESS);
                break;
            case 'h':
                PrintVersionInfo();
                PrintHelpScreen();
                exit(EXIT_SUCCESS);
                break;
            case 'v':
                logging_++;
                break;
            case 'q':
                logging_--;
                break;
            case 'f':
                overwrite_ = true;
                break;
            case 'o':
                output_ = optarg;
                break;
            case 'x':
                extra_++;
                break;
            case 'p':
                progress_++;
                progress_set = true;
                break;
            case 'P':
                progress_ = 0;
                progress_set = true;
                break;
            case 'c':
                compress_++;
                break;
            case 'S':
                if (!StrToValue(optarg, skip_)) {
                    WARNING("Invalid argument to option '-S': %s", optarg);
                    ok = false;
                }
                break;
            case 'D':
                if (!StrToValue(optarg, duration_) || (duration_ < 1)) {
                    WARNING("Invalid argument to option '-D': %s", optarg);
                    ok = false;
                }
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
        }
    }

    // Setup debugging
    LoggingSetParams({logging_});

    // There must a remaining argument, which is the command
    if (optind < argc) {
        command_str_ = argv[optind];
        // clang-format off
        if      (command_str_ == "meta")    { command_ = Command::META; }
        else if (command_str_ == "dump")    { command_ = Command::DUMP; }
        else if (command_str_ == "rosbag")  { command_ = Command::ROSBAG; }
        else if (command_str_ == "trim")    { command_ = Command::TRIM; }
        else if (command_str_ == "record")  { command_ = Command::RECORD; }
        // clang-format on
        else {
            WARNING("Unknown command '%s'", command_str_.c_str());
            ok = false;
        }
        optind++;
    } else {
        WARNING("Missing command or wrong arguments. Try 'fpltool -h'...");
        ok = false;
    }

    // Any further positional arguments
    while (optind < argc) {
        inputs_.push_back(argv[optind]);
        optind++;
    }

    // Default enable progress output and colours if run interactively
    if (!progress_set && (isatty(fileno(stdin)) == 1)) {
        progress_ = 1;
    }

    // Debug
    DEBUG("command_      = '%s'", command_str_.c_str());
    DEBUG("logging_      = %s", fpsdk::common::logging::LoggingLevelStr(logging_));
    for (std::size_t ix = 0; ix < inputs_.size(); ix++) {
        DEBUG("inputs_[%" PRIuMAX "]    = '%s'", ix, inputs_[ix].c_str());
    }
    DEBUG("output        = '%s'", output_.c_str());
    DEBUG("overwrite     = %s", overwrite_ ? "true" : "false");
    DEBUG("extra         = %d", extra_);
    DEBUG("progress      = %d", progress_);
    DEBUG("compress      = %d", compress_);
    DEBUG("skip          = %d", skip_);
    DEBUG("duration      = %d", duration_);

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
