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
#include <fpsdk_common/string.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

/**
 * @brief Program options
 */
class FplToolOptions : public common::app::ProgramOptions
{
   public:
    FplToolOptions()  // clang-format off
        : ProgramOptions("fpltool", {
            { 'f', false, "force"       },
            { 'o', true,  "output"      },
            { 'x', false, "extra"       },
            { 'p', false, "progress"    },
            { 'P', false, "no-progress" },
            { 'c', false, "compress"    },
            { 'S', true,  "skip"        },
            { 'D', true,  "duration"    },
            { 'e', true,  "formats"     } }) {};  // clang-format on

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
    std::vector<std::string>  formats_;                           //!< List of output formats for extraction
    // clang-format on

    static constexpr const char* FORMAT_JSONL = "jsonl";
    static constexpr const char* FORMAT_RAW = "raw";
    static constexpr const char* FORMAT_FILE = "file";
    static constexpr const char* FORMAT_ROS = "ros";

    void PrintHelp() override final
    {
        // clang-format off
        std::fputs(
            "\n"
            "Tool for .fpl files (recordings)\n"
            "\n"
            "Usage:\n"
            "\n"
            "    fpltool [<flags>] <command> <fpl-file> [...]\n"
            "\n"
            "Where (availability of flags depends on <command>, see below):\n"
            "\n", stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
            "\n"
            "    -p, --progress        -- Show progress (default: automatic)\n"
            "    -P, --no-progress     -- Don't show progress (default: automatic)\n"
            "    -f, --force           -- Force overwrite output (default: refuse to overwrite existing output files)\n"
            "    -c, --compress        -- Compress output (e.g. ROS bags), -c -c to compress more\n"
            "    -x, --extra           -- Add extra output, multiple -x can be given\n"
            "    -o, --output <out>    -- Output file prefix <out> (default: derive from <fpl-file> name)\n"
            "    -S, --skip <sec>      -- Skip <sec> seconds from start of log (default: 0, i.e. no skip)\n"
            "    -D, --duration <sec>  -- Process <sec> seconds of log (default: everything)\n"
            "    -e, --formats <fmts>  -- Comma-separated list of output formats for the extract <command> (default: all)\n"
            "    <command>             -- The command, see below\n"
            "    <fpl-file>            -- The .fpl (or .fpl.gz) file to process\n"
            "    \n"
            "The available <command>s are:\n"
            "\n"
            "    meta -- Print the meta data\n"
            "\n"
            "        fpltool [-vqpP] meta <fpl-file>\n"
            "\n"
            "        The meta printed to stdout is suitable for further processing (yq, Python, ...).\n"
            "\n"
            "    dump -- Print information about the data in a .fpl file\n"
            "\n"
            "        fpltool [-vqpPx] dump <fpl-file>\n"
            "\n"
            "        By default it prints the statistics and meta data to stdout. With -x it prints a line for all data\n"
            "        in the <fpl-file> (sequence number, offset into <fpl-file>, size of chunk, type of data, debug info).\n"
            "        Adding another -x adds a hexdump.\n"
            "        Note that with -x this can print a lot of output to stdout!\n"
            "\n"
            "    trim -- Trim a .fpl file to a shorter .fpl file\n"
            "\n"
            "        fpltool [-vqpPfoc] -S <start-sec> -D <duration> <fpl-file>\n"
            "\n"
            "        This shortens the <fpl-file> by removing (skipping) <start-sec> seconds of data from the beginning\n"
            "        and using <duration> sections from that point in the <fpl-file>. Note that this process is inaccurate\n"
            "        and the effective start time and duration of the resulting file may be off by 30 to 60 seconds.\n"
            "        Therefore, both the <start-sec> and <duration> must be at least 60 seconds.\n"
            "\n"
            "    rosbag -- Extract (some of) the data to a ROS bag\n"
            "\n"
            "        This is an alias of 'extract -e ros'. See that for details.\n"
            "\n"
            "    extract -- Extract the data in a .fpl file\n"
            "\n"
            "        fpltool [-vqpPfocSD] [-e <fmts>] extract <fpl-file>\n"
            "\n"
            "        The data is extracted to different files in the current directory. The files are named like the\n"
            "        <fpl-file> with added suffixes and different file extension, depending on the kind of data that\n"
            "        is extracted. By default the data is extracted to all supported formats. Specify a comma-separated\n"
            "        list of formats in <fmts> to limit to some formats. The available output data formats are:\n"
            "\n"
            "        jsonl  -- All data in JSONL format (see below)\n"
            "        raw    -- Stream messages (I/O messages, raw messages from GNSS receiver, ...)\n"
            "        file   -- Recorded files (configuration, ...)\n"
            "        ros    -- ROS data extracted to a ROS bag. This option is only available when compiled with ROS\n"
            "                  (1 or 2) support (see output of 'fpltool -V' to check what your version is). For ROS1\n"
            "                  the standard .bag file format is used. For ROS2 the standard sqlite3 format is used,\n"
            "                  unless compression is used, in which case the mcap format is used.\n"
            "\n"
            "        The jsonl format consists of one JSON object per line. Depending on the data different fields are\n"
            "        available. The fields starting with a '_' (_type, ...) should always be present. The fields not\n"
            "        starting with a '_' are decoded values. What values can be decoded depends on the data in question,\n"
            "        the sensor configuration and software version, and the fpltool support for the data. Typically,\n"
            "        the following kind of JSON objects can be found in the exacted .jsonl file:\n"
            "        \n"
            "        Log meta data: { \"_type\": \"LOGMETA\", \"_yaml\": \"...\", ... }\n"
            "\n"
            "            Where _yaml contains the raw meta data in YAML format. The additional fields (...) are decoded\n"
            "            from that (hw_uid, product_model, sw_version, etc.)\n"
            "\n"
            "        Log status data: { \"_type\": \"LOGSTATUS\", \"_yaml\": \"...\",  ... }\n"
            "\n"
            "            Where _yaml contains the raw status data in YAML format. The additional fields (...) are decoded\n"
            "            from that (state, log_duration, log_size, etc.)\n"
            "\n"
            "        ROS data: { \"_type\": \"ROSMSGBIN\", \"_topic\": \"...\", \"_msg\": \"...\", \"_stamp\": ..., ... }\n"
            "            Where _topic is the ROS topic (e.g. /imu/data), _msg is the message type (e.g. sensor_msgs/Imu),\n"
            "            _stamp is the recording (!) timestamp, and ... are the decoded data (e.g. header with frame_id,\n"
            "            seq and stamp, linear_acceleration, etc.). See the ROS1 documentation for the definitions of\n"
            "            the decoded fields.\n"
            "\n"
            "        Stream messages: { \"_type\": \"STREAMMSG\", \"_stream\": \"...\", \"_stamp\": ..., \"_raw\": \"...\",\n"
            "                           \"_raw_b64\": \"...\", \"_proto\": \"...\", \"_name\": \"...\", \"_seq\": ...,\n"
            "                           \"_valid\": ..., ... } \n"
            "\n"
            "            Where _stream is the stream name (userio, gnss1, ...), _stamp is the recording (!) timestamp,\n"
            "            _raw or _raw_b64 is the raw message data (_raw if data is non-binary, _raw_b64 = base64 encoded\n"
            "            binary data), _proto is the protocol name (FP_A, NMEA, ...), _name is the message name\n"
            "            (FP_A-ODOMETRY, NMEA-GN-RMC, ...), _seq is the sequence counter, and _valid indicates if the\n"
            "            message could be decoded (true) or not (false). Note that decoded fields (...) may be null.\n"
            "            See the protocol (UBX, FP_A, NMEA) specification documentation for the definitions of the\n"
            "            decoded fields.\n"
            "\n"
            "        Files: { \"_type\": \"FILEDUMP\", \"_filename\": \"...\", \"_mtime\": \"...\", \"_stamp\": ...,\n"
            "                 \"_data\": \"...\", \"_data_b64\": \"...\" } \n"
            "\n"
            "            Where _filename is the filename of the file, _mtime its last modification time and _data or\n"
            "            or _data_b64 is the file contents (_data if it is non-binary, _data_b64 = base64 encoded data).\n"
            "\n"
            "Examples:\n"
            "\n"
            "    Print the sensor UID of the sensor that recorded the fpl:\n"
            "\n"
            "        fpltool -qq meta some.fpl | yq .hw_uid\n"
            "\n"
            "    Save detailed info about recorded data to a text file:\n"
            "\n"
            "        fpltool -x dump some.fpl > some.txt\n"
            "\n"
            "    Extract all data from a .fpl file:\n"
            "\n"
            "        fpltool extract some.fpl"
            "\n"
            "    Create a ROS some.bag file (ROS1) resp. some_bag directory (ROS2) from a .fpl file:\n"
            "\n"
            "        fpltool extract -e ros some.fpl\n"
            "        fpltool robag some.fpl                  # shortcut\n"
            "\n"
            "    Create a compressed another.bag (res. another_bag) with 2 minutes of data starting 60 seconds into some.fpl:\n"
            "\n"
            "        fpltool rosbag some.fpl -c -c -o another.bag -S 60 -D 120\n"
            "\n"
            "    Check what is in the extracted ROS bag:\n"
            "\n"
            "        rosbag info some.bag             # ROS 1\n"
            "        ros2 bag info some_bag           # ROS 2 (default, see above)\n"
            "        mcap info some_bag/some.mcap     # ROS 2 (with compression, see above)\n"
            "\n"
            "    Check what is in some_userio.raw obtained by 'extract some.fpl':\n"
            "\n"
            "        parsertool some_userio.raw > some_userio.txt\n"
            "\n"
            "    Pretty-print some_all.jsonl obtained by 'extract some.fpl':\n"
            "\n"
            "        jq < some_all.jsonl\n"
            "\n"
            "    Pretty-print all some_all.jsonl obtained by 'extract some.fpl':\n"
            "\n"
            "        jq < some_all.jsonl\n"
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
            "    Pretty-print all log status from some_all.jsonl obtained by 'extract some.fpl':\n"
            "\n"
            "        jq '.|select(._type==\"LOGSTATUS\")' < some_all.jsonl\n"
            "\n"
            "    Pretty-print all IMU samples from some_all.jsonl obtained by 'extract some.fpl':\n"
            "\n"
            "        jq '.|select(._type==\"ROSMSGBIN\" and ._topic==\"/imu/data\")' < some_all.jsonl\n"
            "\n"
            "    Plot IMU acceleration from some_all.jsonl obtained by 'extract some.fpl':\n"
            "\n"
            "        jq -r '.|select(._topic==\"/imu/data\") | .linear_acceleration | join(\" \")' \\\n"
            "            < some_all.jsonl > plot.dat\n"
            "        gnuplot -p -e 'plot \"plot.dat\" using 1 w l t \"x\", \"\" using 2 w l t \"y\", \"\" using 3 w l t \"z\"'\n"
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
                if (!common::string::StrToValue(argument, skip_)) {
                    ok = false;
                }
                break;
            case 'D':
                if (!common::string::StrToValue(argument, duration_) || (duration_ < 1)) {
                    ok = false;
                }
                break;
            case 'e': {
                formats_ = common::string::StrSplit(argument, ",");
                break;
            }
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
        DEBUG("overwrite     = %s", common::string::ToStr(overwrite_));
        DEBUG("extra         = %d", extra_);
        DEBUG("progress      = %d", progress_);
        DEBUG("compress      = %d", compress_);
        DEBUG("skip          = %d", skip_);
        DEBUG("duration      = %d", duration_);
        DEBUG("formats       = %s", common::string::StrJoin(formats_, " ").c_str());

        return ok;
    }

    std::string GetOutputPrefix(const std::string& input) const
    {
        if (!output_.empty()) {
            return output_;
        }

        std::string output = input;
        common::string::StrReplace(output, ".gz", "");
        common::string::StrReplace(output, ".fpl", "");
        if ((skip_ > 0) || (duration_ > 0)) {
            output += "_S" + std::to_string(skip_) + "_D" + std::to_string(duration_);
        }

        return output;
    }

   private:
    bool progress_set_ = false;
};

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_ARGS_HPP__
