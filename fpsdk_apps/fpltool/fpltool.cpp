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
 * @brief Fixposition SDK: fpltool main
 */

/* LIBC/STL */
#include <cstdio>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>

/* PACKAGE */
#include "fpltool_dump.hpp"
#include "fpltool_extract.hpp"
#include "fpltool_meta.hpp"
#include "fpltool_opts.hpp"
#include "fpltool_record.hpp"
#include "fpltool_trim.hpp"

/* ****************************************************************************************************************** */

using namespace fpsdk::apps::fpltool;

int main(int argc, char** argv)
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    FplToolOptions opts;
    if (!opts.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    // Run command
    if (ok) {
        switch (opts.command_) { /* clang-format off */
            case FplToolOptions::Command::DUMP:        ok = DoDump(opts);    break;
            case FplToolOptions::Command::META:        ok = DoMeta(opts);    break;
            case FplToolOptions::Command::TRIM:        ok = DoTrim(opts);    break;
            case FplToolOptions::Command::EXTRACT:     ok = DoExtract(opts); break;
            // Aliases
            case FplToolOptions::Command::ROSBAG:
                opts.formats_.push_back(opts.FORMAT_ROS);
                ok = DoExtract(opts);
                break;
            // Not implemented, invalid
            case FplToolOptions::Command::RECORD:      ok = DoRecord(opts);  break;
            case FplToolOptions::Command::UNSPECIFIED: ok = false;           break;
        }  // clang-format on
    }

    // Are we happy?
    if (ok) {
        INFO("Done: %s", opts.command_str_.c_str());
        return EXIT_SUCCESS;
    } else {
        ERROR("Failed: %s", opts.command_str_.c_str());
        return EXIT_FAILURE;
    }
}

/* ****************************************************************************************************************** */
