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
 * @brief Fixposition SDK: fpltool options (command line arguments)
 */
#ifndef __FPSDK_APPS_FPLTOOL_FPLTOOL_ARGS_HPP__
#define __FPSDK_APPS_FPLTOOL_FPLTOOL_ARGS_HPP__

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */
#include <unistd.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

/**
 * @brief Program options
 */
class FplToolOptions : public fpsdk::common::app::ProgramOptions
{
   public:
    FplToolOptions()  // clang-format off
        : ProgramOptions("fpltool", {
            { 'f', false }, { 'o', true }, { 'x', false }, { 'p', false },
            { 'P', false }, { 'c', false }, { 'S', true }, { 'D', true } }) {};  // clang-format on

    /**
     * @brief Commands, modes of operation
     */
    enum class Command
    {
        UNSPECIFIED,  //!< Bad command
        DUMP,         //!< Dump a log (for debugging)
        META,         //!< Get logfile meta data
        ROSBAG,       //!< Create ROS bag from log
        TRIM,         //!< Trim .fpl file
        RECORD,       //!< Record a log
        EXTRACT,      //!< Extract .fpl file to various (non-ROS) files
    };

    // clang-format off
    Command                   command_   = Command::UNSPECIFIED;  //!< Command to execute
    std::string               command_str_;                       //!< String of command_, for debugging
    std::vector<std::string>  inputs_;                            //!< List of input (files), or other positional arguments
    std::string               output_;                            //!< Output (file or directory, depending on command)
    bool                      overwrite_ = false;                 //!< Overwrite existing output files
    int                       extra_     = 0;                     //!< Enable extra output, such as hexdumps
    int                       progress_  = 0;                     //!< Do progress reports
    int                       compress_  = 0;                     //!< Compress output
    uint32_t                  skip_      = 0;                     //!< Skip start [sec]
    uint32_t                  duration_  = 0;                     //!< Duration [sec]
    // clang-format on

    void PrintHelp() final
    {
        // clang-format off
        std::fputs(
            "\n"
            "Tool for .fpl logfiles\n"
            "\n"
            "Usage:\n"
            "\n"
            "    parsertool [flags] <input> [<input> ...]\n"
            "\n"
            "Usage:\n"
            "\n"
            "    fpltool [<flags>] <command> [...]\n"
            "\n"
            "Where (availability of flags depends on <command>, see below):\n"
            "\n" ,stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
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
            "    fpltool [-vqpPfoSD] robag <fpl-file>\n"
#endif
            "\n"
            "Trim start and/or end of .fpl logfile, i.e. extract a portion of the file. Note that this process is\n"
            "inaccurate and the effective start time and duration of the resulting file may be off by 30 to 60 seconds.\n"
            "Therefore, both the start time (-S) and the duration (-D) must be at least 60 seconds.\n"
            "\n"
            "    fpltool [-vqpPfo] -S <sec> -D <sec> <fpl-file>\n"
            "\n"
            "Extract (som) non-ROS data from the .fpl logfile\n"
            "\n"
            "    fpltool [-vqpP] extract <fpl-file>\n"
            "\n"
            "Examples:\n"
#ifdef FP_USE_ROS1
            "\n"
            "    Create a some.bag from a some.fpl logfile:\n"
            "\n"
            "        fpltool rosbag some.fpl\n"
            "\n"
            "    Create a compressed another.bag with 2 minutes of data starting 60 seconds into some.fpl logfile:\n"
            "\n"
            "        fpltool rosbag some.fpl -c -c -o another.bag -S 60 -D 120\n"
#endif
            "\n"
            "    Show meta data of some.fpl:\n"
            "\n"
            "        fpltool meta some.fpl\n"
            "\n"
            "    Trim a verylong.fpl into a shorter one:\n"
            "\n"
            "        fpltool trim some.fpl -S 3600 -D 1800\n"
            "\n"
            "    Extract data from some.fpl:\n"
            "\n"
            "        fpltool extract some.fpl\n"
            "\n"
            "        This results in various data files suitable for further processing. For example, the input/output\n"
            "        messages received/sent by the sensor can be processed by the parsertool:\n"
            "\n"
            "            parsertool some_userio.raw\n"
            "\n"
            "\n", stdout);
        // clang-format on
    }

    bool HandleOption(const Option& option, const std::string& argument) final
    {
        bool ok = true;
        switch (option.flag) {
            case 'f':
                overwrite_ = true;
                break;
            case 'o':
                output_ = argument;
                break;
            case 'x':
                extra_++;
                break;
            case 'p':
                progress_++;
                progress_set_ = true;
                break;
            case 'P':
                progress_ = 0;
                progress_set_ = true;
                break;
            case 'c':
                compress_++;
                break;
            case 'S':
                if (!fpsdk::common::string::StrToValue(argument, skip_)) {
                    ok = false;
                }
                break;
            case 'D':
                if (!fpsdk::common::string::StrToValue(argument, duration_) || (duration_ < 1)) {
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

        // There must a remaining argument, which is the command
        if (args.size() > 0) {
            command_str_ = args[0];
            // clang-format off
            if      (command_str_ == "meta")     { command_ = Command::META;     }
            else if (command_str_ == "dump")     { command_ = Command::DUMP;     }
            else if (command_str_ == "rosbag")   { command_ = Command::ROSBAG;   }
            else if (command_str_ == "trim")     { command_ = Command::TRIM;     }
            else if (command_str_ == "record")   { command_ = Command::RECORD;   }
            else if (command_str_ == "extract")  { command_ = Command::EXTRACT;  }
            // clang-format on
            else {
                WARNING("Unknown command '%s'", command_str_.c_str());
                ok = false;
            }
        } else {
            WARNING("Missing command or wrong arguments");
            ok = false;
        }

        // Any further positional arguments
        for (std::size_t ix = 1; ix < args.size(); ix++) {
            inputs_.push_back(args[ix]);
        }

        // Default enable progress output and colours if run interactively
        if (!progress_set_ && (isatty(fileno(stdin)) == 1)) {
            progress_ = 1;
        }

        // Debug
        DEBUG("command_      = '%s'", command_str_.c_str());
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

   private:
    bool progress_set_ = false;
};

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_ARGS_HPP__
