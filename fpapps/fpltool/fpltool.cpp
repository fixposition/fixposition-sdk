/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com)
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition fpl logfile tool
 */

/* LIBC/STL */
#include <cstdio>

/* EXTERNAL */
#include <unistd.h>

/* Fixposition SDK */
#include <fpcommon/app.hpp>
#include <fpcommon/logging.hpp>

/* PACKAGE */
#include "fpltool_args.hpp"
#include "fpltool_dump.hpp"
#include "fpltool_meta.hpp"
#include "fpltool_record.hpp"
#include "fpltool_rosbag.hpp"
#include "fpltool_trim.hpp"

/* ****************************************************************************************************************** */

using namespace fpltool;

int main(int argc, char** argv)
{
#ifndef NDEBUG
    fp::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    FpltoolArgs args;
    if (!args.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    // Run command
    if (ok) {
        switch (args.command_) { /* clang-format off */
            case FpltoolArgs::Command::DUMP:        ok = DoDump(args);   break;
            case FpltoolArgs::Command::META:        ok = DoMeta(args);   break;
            case FpltoolArgs::Command::ROSBAG:      ok = DoRosbag(args); break;
            case FpltoolArgs::Command::TRIM:        ok = DoTrim(args);   break;
            case FpltoolArgs::Command::RECORD:      ok = DoRecord(args); break;
            case FpltoolArgs::Command::UNSPECIFIED: ok = false;          break;
        }  // clang-format on
    }

    // Are we happy?
    if (ok) {
        INFO("Done: %s", args.command_str_.c_str());
        return EXIT_SUCCESS;
    } else {
        ERROR("Failed: %s", args.command_str_.c_str());
        return EXIT_FAILURE;
    }
}

/* ****************************************************************************************************************** */
