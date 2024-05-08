/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief fpltool: params (command line arguments)
 */
#ifndef __FPLTOOL_FPLTOOL_PARAMS_HPP__
#define __FPLTOOL_FPLTOOL_PARAMS_HPP__

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpcommon/logging.hpp>

/* PACKAGE */

namespace fp {
namespace fpltool {
/* ****************************************************************************************************************** */

class FpltoolArgs
{
   public:
    enum class Command {
        UNSPECIFIED,  //!< Bad command
        DUMP,         //!< Dump a log (for debugging)
        META,         //!< Get logfile meta data
        ROSBAG,       //!< Create ROS bag from log
        TRIM,         //!< Trim fpl file
        RECORD,       //!< Record a log
    };

    using LoggingLevel = fp::common::logging::LoggingLevel;

    FpltoolArgs() /* clang-format off */ :
        command_       { Command::UNSPECIFIED },
        command_str_   { "unspecified" },

        logging_       { LoggingLevel::INFO },
        overwrite_     { false },
        extra_         { 0 },
        progress_      { 0 },
        compress_      { 0 },
        skip_          { 0 },
        duration_      { 0 }  // clang-format on
          {};

    bool LoadFromArgv(int argc, char** argv);
    // clang-format off
    Command                   command_;        //!< Command to execute
    std::string               command_str_;    //!< String of command_, for debugging
    LoggingLevel              logging_;        //!< Logging (verbosity) level
    std::vector<std::string>  inputs_;         //!< List of input (files), or other positional arguments
    std::string               output_;         //!< Output (file or directory, depending on command)
    bool                      overwrite_;      //!< Overwrite existing output files
    int                       extra_;          //!< Enable extra output, such as hexdumps
    int                       progress_;       //!< Do progress reports
    int                       compress_;       //!< Compress output
    uint32_t                  skip_;           //!< Skip start [sec]
    uint32_t                  duration_;       //!< Duration [sec]
    std::vector<std::string>  argv_;           //!< argv[] of program
    // clang-format on

    //! Help screen
    static constexpr const char* USAGE_HELP = /* clang-format off */
        "fpltool (" FP_VERSION_STRING ") -- Copyright (c) Fixposition AG\n"
        "\n"
        "Usage:\n"
        "\n"
        "    fpltool [<flags>] <command> [...]\n"
        "\n"
        "Where (availability of flags depends on <command>, see below):\n"
        "    -v / -q  -- Increase / decrese logging verbosity, multiple flags accumulate\n"
        "    -p / -P  -- Show / don't show progress (default: automatic)\n"
        "    -f       -- Force overwrite output (default: refuse to overwrite existing output files)\n"
        "    -c       -- Compress output (e.g. ROS bags), -c -c to compress more\n"
        "    -x       -- Add extra output, multiple -x can be given\n"
        "    -o <out> -- Output to <out>\n"
        "    -S <sec> -- Skip <sec> seconds from start of log (approximate!) (default: 0, i.e. no skip)\n"
        "    -D <sec> -- Process <sec> seconds of log (approximate!) (default: everything)\n"
        "    \n"
        "Print information about the data in a .fpl logfile, along with status and meta data\n"
        "\n"
        "    fpltool [-vqpPx] dump <fpl-file>\n"
        "\n"
        "Print the meta data from a .fpl logfile (as YAML)\n"
        "\n"
        "    fpltool [-vqpP] meta <fpl-file>\n"
#ifdef FP_USE_ROS1
        "\n"
        "Generate a ROS bag from a .fpl logfile\n"
        "\n"
        "    fpltool [-vqpPfoSD] <fpl-file>\n"
#endif
        "\n"
        "Trim start and/or end of .fpl logfile, i.e. extract a portion of the file. Note that this process is\n"
        "inaccurate and the effective start time and duration of the resulting file may be off by 30 to 60 seconds.\n"
        "Therefore, both the start time (-S) and the duration (-D) must be at least 60 seconds.\n"
        "\n"
        "    fpltool [-vqpPfo] -S <sec> -D <sec> <fpl-file>\n"
        "\n"
        "Examples:\n"
        "\n"
        "    Create a some.bag from a some.fpl logfile:\n"
        "\n"
        "        fpltool rosbag some.fpl\n"
        "\n"
        "    Create a compressed another.bag with 2 minutes of data starting 60 seconds into some.fpl logfile:\n"
        "\n"
        "        fpltool rosbag some.fpl -c -c -o another.bag -S 60 -D 120\n"
        "\n"
        "    Show meta data of some.fpl:\n"
        "\n"
        "        fpltool meta some.fpl\n"
        "\n"
        "    Trim a verylong.fpl into a shorter one:\n"
        "\n"
        "        fpltool trim some.fpl -S 3600 -D 1800\n"
        "\n";  // clang-format on
};

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace fp
#endif  // __FPLTOOL_FPLTOOL_PARAMS_HPP__
