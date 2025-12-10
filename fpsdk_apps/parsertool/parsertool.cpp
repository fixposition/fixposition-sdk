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
 * @brief Fixposition SDK: parsertool main
 */

/* LIBC/STL */
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>

/* EXTERNAL */
#include <unistd.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */
#include "../common/common.hpp"

namespace fpsdk {
namespace apps {
namespace parsertool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::parser;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;
using namespace fpsdk::apps::common;

// ---------------------------------------------------------------------------------------------------------------------

// Program options
class ParserToolOptions : public ProgramOptions
{
   public:
    ParserToolOptions()  // clang-format off
        : ProgramOptions("parsertool", {
            { 'x', false, "hexdump" }, { 's', false, "save" }, { 'f', true, "filter" }, { 'c', false, "stdout" }
        }) {};  // clang-format on

    // clang-format off
    bool                      hexdump_ = false;  //!< Do hexdump of messages
    bool                      split_   = false;  //!< Split individual messages and save to files
    std::vector<std::string>  inputs_;           //!< Input file(s)
    std::vector<std::string>  filters_;          //!< Filters
    bool                      stdout_  = false;  //!< Write message to stdout instead of info about it
    // clang-format on

    void PrintHelp() override final
    {
        // clang-format off
        std::fputs(
            "\n"
            "This parses input data from stdin or file(s) into individual messages (frames) and\n"
            "dumps some information about them to stdout.\n"
            "\n"
            "Usage:\n"
            "\n"
            "    parsertool [flags] [<input> ...]\n"
            "\n"
            "Where:\n"
            "\n", stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
            "\n"
            "    -x, --hexdump  -- Print hexdump of each message, not with -c\n"
            "    -s, --save     -- Save each (!) message into a separate (!) file in the current directory, not with -c\n"
            "    -f <filter>, --filter <filter>\n"
            "                   -- Filter input, where <filter> is a comma-separated list of full or partial message.\n"
            "                      names. Multiple -f can be given.\n"
            "    -c, --stdout   -- Write messages to stdout instead of printing info, useful with -f\n"
            "    <input>        -- File or device to read data from (instead of stdin)\n"
            "\n"
            "Examples:\n"
            "\n"
            "    Parse messages from a file:\n"
            "\n"
            "        parsertool fpsdk_common/test/data/mixed.bin\n"
            "\n"
            "    Read data from stdin:\n"
            "\n"
            "        cat fpsdk_common/test/data/mixed.bin | parsertool\n"
            "        parsertool < fpsdk_common/test/data/mixed.bin\n"
            "\n"
            "    Read from a TCP/IP socket server:\n"
            "\n"
            "        nc 10.0.1.1 21000 | parsertool\n"
            "\n"
            "    Read from a serial port (such as a USB to serial converter):\n"
            "\n"
            "        stty -F /dev/ttyUSB0 115200 raw -echo; parsertool /dev/ttyUSB0\n"
            "\n"
            "    Filter all FP_A-ODOMETRY:\n"
            "\n"
            "        parsertool -f FP_A-ODOMETRY fpsdk_common/test/data/mixed.bin\n"
            "\n"
            "    Filter all NMEA and FP_A messages. Either command does the same.\n"
            "\n"
            "        parsertool -f FP_A -f NMEA fpsdk_common/test/data/mixed.bin\n"
            "        parsertool -f FP_A,NMEA fpsdk_common/test/data/mixed.bin\n"
            "\n"
            "    Filter all NMEA and FP_A messages and store to output.txt file:\n"
            "\n"
            "        parsertool -f FP_A,NMEA -c fpsdk_common/test/data/mixed.bin > output.txt\n"
            "\n"
            "\n", stdout);
        // clang-format on
    }

    bool HandleOption(const Option& option, const std::string& argument) final
    {
        bool ok = true;
        switch (option.flag) {
            case 'x':
                hexdump_ = true;
                break;
            case 's':
                split_ = true;
                break;
            case 'f':
                for (auto& f : StrSplit(argument, ",")) {
                    filters_.push_back(f);
                }
                break;
            case 'c':
                stdout_ = true;
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

        // Any further positional arguments are inputs
        inputs_ = args;

        for (std::size_t ix = 0; ix < inputs_.size(); ix++) {
            DEBUG("inputs_[%" PRIuMAX "] = '%s'", ix, inputs_[ix].c_str());
        }
        DEBUG("hexdump    = %s", ToStr(hexdump_));
        DEBUG("split      = %s", ToStr(split_));
        for (std::size_t ix = 0; ix < filters_.size(); ix++) {
            DEBUG("filters[%" PRIuMAX "] = %s", ix, filters_[ix].c_str());
        }
        DEBUG("stdout     = %s", ToStr(stdout_));

        return ok;
    }
};

/* ****************************************************************************************************************** */

class ParserTool
{
   public:
    ParserTool(const ParserToolOptions& opts) : opts_{ opts }, offs_{ 0 }
    {
    }

    bool Run()
    {
        // Print header
        if (!opts_.stdout_) {
            PrintMessageHeader();
        }

        // Process data from stdin
        if (opts_.inputs_.empty()) {
            INFO("Reading from stdin");
            std::ios_base::sync_with_stdio(false);
            ProcessInput(std::cin);
        }

        // Process all input files
        bool ok = true;
        for (const auto& input_file : opts_.inputs_) {
            INFO("Reading from %s", input_file.c_str());
            std::ifstream input(input_file, std::ios::binary);
            if (input.good()) {
                ProcessInput(input);
            } else {
                WARNING("Failed reading from %s: %s", input_file.c_str(), StrError(errno).c_str());
                ok = false;
            }
            if (sigint_.ShouldAbort() || !ok) {
                break;
            }
        }

        //! Print statistics
        if (!opts_.stdout_) {
            PrintParserStats();
        }

        return ok;
    }

   private:
    // -----------------------------------------------------------------------------------------------------------------

    ParserToolOptions opts_;  //!< Program options
    SigIntHelper sigint_;     //!< Handle SIGINT (C-c) to abort nicely
    Parser parser_;           //!< Parser
    std::size_t offs_;        //!< Offset of message in input data
    ParserStats stats_;       //!< Parser statistics

    // -----------------------------------------------------------------------------------------------------------------

    void ProcessInput(std::istream& input)
    {
        // Run all data through the parser and print what it finds...

        ParserMsg msg;
        const auto tenms = Duration::FromNSec(10000000);

        while (!sigint_.ShouldAbort() && input.good() && !input.eof()) {
            // Read a chunk of data from the logfile
            uint8_t data[MAX_ADD_SIZE];
            const int size = input.readsome((char*)data, sizeof(data));

            // We may be at end of file
            if (size <= 0) {
                // End of file
                if (input.peek() == EOF) {
                    break;
                }
                // More data may be available later (e.g. when reading from stdin, pipe, device)
                // @todo Use select()
                else {
                    tenms.Sleep();  // @todo necessary?
                    continue;
                }
            }

            // Add the chunk of data to the parser
            if (!parser_.Add(data, size)) {
                WARNING("Parser overflow");
                parser_.Reset();
                continue;
            }

            // Run parser and print messages to the screen
            while (parser_.Process(msg)) {
                PrintMessageData(msg, offs_, opts_.hexdump_);
            }
        }

        // There may be some remaining data in the parser
        while (parser_.Flush(msg)) {
            PrintMessageData(msg, offs_, opts_.hexdump_);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------

    void SaveMessage(const ParserMsg& msg)
    {
        const auto output_file = Sprintf("%06" PRIuMAX "_%s.bin", msg.seq_, msg.name_.c_str());
        std::ofstream output(output_file, std::ios::binary);
        output.write((const char*)msg.Data(), msg.Size());
        output.close();
    }

    // -----------------------------------------------------------------------------------------------------------------

    void PrintMessageHeader()
    {
        // Keep in sync with PrintMessageData()
        std::printf("------- Seq#     Offset  Size Protocol Message                        Info\n");
    }

    // -----------------------------------------------------------------------------------------------------------------

    void PrintMessageData(const ParserMsg& msg, const std::size_t offs, const bool hexdump)
    {
        if (!CheckFilter(msg)) {
            return;
        }

        if (opts_.stdout_) {
            std::fwrite(msg.Data(), msg.Size(), 1, stdout);
        } else {
            msg.MakeInfo();
            std::printf("message %06" PRIuMAX " %8" PRIuMAX " %5" PRIuMAX " %-8s %-30s %s\n", msg.seq_, offs,
                msg.Size(), ProtocolStr(msg.proto_), msg.name_.c_str(), msg.info_.empty() ? "-" : msg.info_.c_str());
            if (hexdump) {
                for (auto& line : HexDump(msg.data_)) {
                    std::printf("%s\n", line.c_str());
                }
            }
            if (opts_.split_) {
                SaveMessage(msg);
            }
        }

        stats_.Update(msg);
        offs_ += msg.Size();
    }

    // -----------------------------------------------------------------------------------------------------------------

    void PrintParserStats()
    {
        const auto& s = stats_;
        const double p_n = (s.n_msgs_ > 0 ? 100.0 / (double)s.n_msgs_ : 0.0);
        const double p_s = (s.s_msgs_ > 0 ? 100.0 / (double)s.s_msgs_ : 0.0);
        std::printf("Stats:     Messages               Bytes\n");
        const char* fmt = "%-8s %10" PRIu64 " (%5.1f%%) %10" PRIu64 " (%5.1f%%)\n";
        // clang-format off
        std::printf(fmt, "Total",                              s.n_msgs_,   (double)s.n_msgs_   * p_n, s.s_msgs_,   (double)s.s_msgs_   * p_s);
        std::printf(fmt, ProtocolStr(Protocol::FP_A),   s.n_fpa_,    (double)s.n_fpa_    * p_n, s.s_fpa_,    (double)s.s_fpa_    * p_s);
        std::printf(fmt, ProtocolStr(Protocol::FP_B),   s.n_fpb_,    (double)s.n_fpb_    * p_n, s.s_fpb_,    (double)s.s_fpb_    * p_s);
        std::printf(fmt, ProtocolStr(Protocol::NMEA),   s.n_nmea_,   (double)s.n_nmea_   * p_n, s.s_nmea_,   (double)s.s_nmea_   * p_s);
        std::printf(fmt, ProtocolStr(Protocol::UBX),    s.n_ubx_,    (double)s.n_ubx_    * p_n, s.s_ubx_,    (double)s.s_ubx_    * p_s);
        std::printf(fmt, ProtocolStr(Protocol::RTCM3),  s.n_rtcm3_,  (double)s.n_rtcm3_  * p_n, s.s_rtcm3_,  (double)s.s_rtcm3_  * p_s);
        std::printf(fmt, ProtocolStr(Protocol::NOV_B),  s.n_novb_,   (double)s.n_novb_   * p_n, s.s_novb_,   (double)s.s_novb_   * p_s);
        std::printf(fmt, ProtocolStr(Protocol::UNI_B),  s.n_unib_,   (double)s.n_unib_   * p_n, s.s_unib_,   (double)s.s_unib_   * p_s);
        std::printf(fmt, ProtocolStr(Protocol::SBF),    s.n_sbf_,    (double)s.n_sbf_ *    p_n, s.s_sbf_,    (double)s.s_sbf_    * p_s);
        std::printf(fmt, ProtocolStr(Protocol::QGC),    s.n_qgc_,    (double)s.n_qgc_ *    p_n, s.s_qgc_,    (double)s.s_qgc_    * p_s);
        std::printf(fmt, ProtocolStr(Protocol::SPARTN), s.n_spartn_, (double)s.n_spartn_ * p_n, s.s_spartn_, (double)s.s_spartn_ * p_s);
        std::printf(fmt, ProtocolStr(Protocol::OTHER),  s.n_other_,  (double)s.n_other_  * p_n, s.s_other_,  (double)s.s_other_  * p_s);
        // clang-format on
    }

    // -----------------------------------------------------------------------------------------------------------------

    bool CheckFilter(const ParserMsg& msg)
    {
        if (opts_.filters_.empty()) {
            return true;
        }
        for (const auto& f : opts_.filters_) {
            if (StrStartsWith(msg.name_, f)) {
                return true;
            }
        }
        return false;
    }
};

/* ****************************************************************************************************************** */
}  // namespace parsertool
}  // namespace apps
}  // namespace fpsdk

/* ****************************************************************************************************************** */

int main(int argc, char** argv)
{
    using namespace fpsdk::apps::parsertool;
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    ParserToolOptions opts;
    if (!opts.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    if (ok) {
        ParserTool app(opts);
        ok = app.Run();
    }

    // Are we happy?
    if (ok) {
        INFO("Done");
        return EXIT_SUCCESS;
    } else {
        ERROR("Failed");
        return EXIT_FAILURE;
    }
}

/* ****************************************************************************************************************** */
