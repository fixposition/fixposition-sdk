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
 * @brief Fixposition SDK: yaml2shell main
 */

/* LIBC/STL */
#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <string>
#include <vector>

/* EXTERNAL */
#include <yaml-cpp/yaml.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/string.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
namespace yaml2shell {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::path;
using namespace fpsdk::common::string;

// ---------------------------------------------------------------------------------------------------------------------

// Program options
class YamlToShellOptions : public ProgramOptions
{
   public:
    YamlToShellOptions()  // clang-format off
        : ProgramOptions("yaml2shell",
            { { 'p', true }, { 'o', true }, { 'n', true }, { 's', true }, { 'u', false }, { 'f', true } })
        {}  // clang-format on

    std::string input_;
    std::string output_;
    std::string prefix_;
    bool upper_ = false;
    uint32_t max_num_ = 0;
    uint32_t max_size_ = 1024 * 1024;
    std::string filter_;

    void PrintHelp() override final
    {
        // clang-format off
        std::fputs(
            "\n"
            "Utility to dump a yaml to shell variables, much like 'yq --output-format shell some.yaml'.\n"
            "The output can be sourced in shell scripts or used as systemd EnvironmentFile."
            "\n"
            "Usage:\n"
            "\n"
            "    yaml2shell [flags] [<some.yaml>]\n"
            "\n"
            "Where:\n"
            "\n", stdout);
        std::fputs(COMMON_FLAGS_HELP, stdout);
        std::fputs(
            "\n"
            "    -p <str>  -- Prefix for variable names (default: none)\n"
            "    -o <file> -- Output to file instead of stdout\n"
            "    -u        -- Uppercase variable names\n"
            "    -n <num>  -- Maximum number of variables to output (default: 0, i.e. unlimited)\n"
            "    -s <size> -- Maximum size of output [bytes], 0 = unlimited (default: 1 MiB)\n"
            "    -f <re>   -- Filter output (on variable name) using a regex, case insensitive unless uppercase\n"
            "                 chars are used in the <re> string\n"
            "\n"
            "If <some.yaml> is given the program reads the YAML from that file. Otherwise it processes YAML\n"
            "from stdin. For example:\n"
            "\n"
            "    yaml2shell some.yaml\n"
            "    cat some.yaml | yaml2shell\n"
            "\n"
            "Examle YAML input:\n"
            "\n"
            "    foo: 123\n"
            "    bar:\n"
            "        a: foo\n"
            "        b: false\n"
            "    empty:\n"
            "    xyz:\n"
            "        - 2\n"
            "        - 3\n"
            "\n"
            "    Output (default):    (with '-p cfg_')     (with '-p cfg -u')    (with '-f ^(foo|empty) -u')\n"
            "    \n"
            "        foo='123'        cfg_foo='123'        CFG_FOO='123'         FOO='123'\n"
            "        bar_a='foo'      cfg_bar_a='foo'      CFG_BAR_A='foo'       EMPTY=''\n"
            "        bar_b='false'    cfg_bar_b='false'    CFG_BAR_B='false'\n"
            "        empty=''         cfg_empty=''         CFG_EMPTY=''\n"
            "        xyz_0='2'        cfg_xyz_0='2'        CFG_XYZ_0='2'\n"
            "        xyz_1='3'        cfg_xyz_1='3'        CFG_XYZ_1='3'\n"
            "\n", stdout);
        // clang-format on
    }

    bool HandleOption(const Option& option, const std::string& argument) final
    {
        bool ok = true;
        switch (option.flag) {
            case 'p':
                prefix_ = argument;
                break;
            case 'o':
                output_ = argument;
                break;
            case 'n':
                ok = StrToValue(argument, max_num_);
                break;
            case 's':
                ok = StrToValue(argument, max_size_);
                break;
            case 'u':
                upper_ = true;
                break;
            case 'f':
                filter_ = argument;
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
        // Exactly 0 or 1 argument required
        if (args.size() == 1) {
            input_ = args[0];
        } else if (!args.empty()) {
            ok = false;
        }

        DEBUG("input     = %s", input_.c_str());
        DEBUG("prefix    = %s", prefix_.c_str());
        DEBUG("output    = %s", output_.c_str());
        DEBUG("upper     = %s", ToStr(upper_));
        DEBUG("max_num_  = %" PRIu32, max_num_);
        DEBUG("max_size  = %" PRIu32, max_size_);
        DEBUG("filter    = %s", filter_.c_str());

        return ok;
    }
};

/* ****************************************************************************************************************** */

class YamlToShell
{
   public:
    YamlToShell(const YamlToShellOptions& opts) : opts_{ opts }
    {
    }

    bool Run()
    {
        // Have filter?
        if (!opts_.filter_.empty()) {
            try {
                filter_ = std::make_unique<std::regex>(opts_.filter_,
                    opts_.filter_ == StrToLower(opts_.filter_) ? std::regex_constants::icase
                                                               : (std::regex_constants::syntax_option_type)0);
            } catch (std::exception& ex) {
                WARNING("Bad filter %s: %s", opts_.filter_.c_str(), ex.what());
                return false;
            }
        }

        // std::regex_constants::icase

        // Read data, input cannot be larger than max allowed output
        std::vector<uint8_t> data;
        // - from file
        if (!opts_.input_.empty()) {
            if ((opts_.max_size_ == 0) || (FileSize(opts_.input_) < opts_.max_size_)) {
                if (!FileSlurp(opts_.input_, data)) {
                    return false;
                }
            } else {
                WARNING("Input too large");
                return false;
            }
        }
        // - from stdin
        else {
            while (!std::cin.eof() && !std::cin.fail()) {
                uint8_t buf[100 * 1024];
                std::cin.read((char*)buf, sizeof(buf));
                data.insert(data.end(), buf, buf + std::cin.gcount());
                if ((opts_.max_size_ == 0) || (data.size() > opts_.max_size_)) {
                    WARNING("Input too large");
                    return false;
                }
            }
            opts_.input_ = "stdin";
        }
        if (data.empty()) {
            WARNING("No data from %s", opts_.input_.c_str());
            return false;
        }

        DEBUG("Read %" PRIuMAX " bytes from %s", data.size(), opts_.input_.c_str());
        // TRACE_HEXDUMP(data.data(), data.size(), NULL, NULL);

        // Output
        if (!opts_.output_.empty()) {
            try {
                out_ = std::make_unique<std::ofstream>(opts_.output_, std::ios::binary);
                if (!out_ || out_->fail()) {
                    throw std::runtime_error(StrError(errno).c_str());
                }
            } catch (std::exception& ex) {
                WARNING("Open %s fail: %s", opts_.output_.c_str(), ex.what());
                return false;
            }
        }

        bool ok = true;

        // Parse and dump YAML
        try {
            YAML::Node yaml = YAML::Load((const char*)data.data());
            ok = Dump(yaml, opts_.prefix_);
        } catch (std::exception& ex) {
            WARNING("Bad YAML: %s", ex.what());
            ok = false;
        }

        return ok;
    }

   private:
    // -----------------------------------------------------------------------------------------------------------------

    YamlToShellOptions opts_;
    std::size_t num_ = 0;
    std::size_t size_ = 0;
    std::unique_ptr<std::ofstream> out_;
    static constexpr int MAX_DEPTH = 10000;
    std::unique_ptr<std::regex> filter_;

    bool Dump(const YAML::Node& node, const std::string& prefix, const int depth = 0)
    {
        TRACE("%*sDump(%s, %d) defined=%s null=%s scalar=%s map=%s sequence=%s tag=%s", 4 * depth, "", prefix.c_str(),
            depth, ToStr(node.IsDefined()), ToStr(node.IsNull()), ToStr(node.IsScalar()), ToStr(node.IsMap()),
            ToStr(node.IsSequence()), node.Tag().c_str());
        if (depth > MAX_DEPTH) {
            WARNING("max depth exceeded");
            return false;
        }

        bool ok = true;
        switch (node.Type()) {
            // Not expected
            case YAML::NodeType::Undefined:
                ok = false;
                break;
            case YAML::NodeType::Null:
                ok = PrintVar(prefix, "");
                break;
            case YAML::NodeType::Scalar:
                if (node.Tag().size() > 1) {
                    WARNING("Ignoring node '%s' with tag %s", prefix.c_str(), node.Tag().c_str());
                } else {
                    ok = PrintVar(prefix, node.as<std::string>());
                }
                // {
                //     // clang-format off
                //     bool b; int64_t i; double d;
                //     if      (node.Tag() == "!")                       { TRACE("--> string (1)"); }
                //     else if (YAML::convert<bool>::decode(node, b))    { TRACE("--> bool"); }
                //     else if (YAML::convert<int64_t>::decode(node, i)) { TRACE("--> int"); }
                //     else if (YAML::convert<double>::decode(node, d))  { TRACE("--> double"); }
                //     else                                              { TRACE("--> string (2)"); }
                //     // clang-format on
                // }
                break;
            case YAML::NodeType::Sequence:
                for (std::size_t ix = 0; ok && (ix < node.size()); ix++) {
                    ok = Dump(node[ix], (depth == 0 ? prefix : prefix + "_") + std::to_string(ix), depth + 1);
                }
                break;
            case YAML::NodeType::Map:
                for (auto it = node.begin(); ok && (it != node.end()); it++) {
                    ok =
                        Dump(it->second, (depth == 0 ? prefix : prefix + "_") + it->first.as<std::string>(), depth + 1);
                }
                break;
        }

        return ok;
    }

    bool PrintVar(std::string var, std::string val)
    {
        bool ok = true;

        // Sanitise variable name
        // - Replace anything not _a-zA-z0-9 with '_'
        // - Optionally uppercase
        // - Add '_' if it starts with a digit
        for (auto it = var.begin(); it != var.end(); it++) {
            if (!(((*it >= 'a') && (*it <= 'z')) || ((*it >= 'A') && (*it <= 'Z')) || ((*it >= '0') && (*it <= '9')) ||
                    (*it = '_'))) {
                *it = '_';
            }
            if (opts_.upper_) {
                *it = toupper(*it);
            }
        }
        if ((var[0] >= '0') && (var[0] <= '9')) {
            var.insert(var.begin(), '_');
        }

        // Filter?
        if (filter_ && !std::regex_match(var, *filter_)) {
            return ok;
        }

        // Sanitise variable value
        // - "Escape" single quotes
        StrReplace(val, "'", "'\"'\"'");

        // A YAML only consisting of a single scalar won't have a variable name
        if (var.empty()) {
            var = "value";  // like yq
        }

        std::string str = Sprintf("%s='%s'\n", var.c_str(), val.c_str());
        size_ += str.size();
        num_++;
        if (((opts_.max_size_ != 0) && (size_ > opts_.max_size_)) ||
            ((opts_.max_num_ != 0) && (num_ > opts_.max_num_))) {
            WARNING("Max num or size exceeded");
            ok = false;
        } else {
            if (out_) {
                *out_ << str;
                if (out_->fail()) {
                    ok = false;
                }
            } else {
                std::cout << str;
            }
        }
        return ok;
    }
};

/* ****************************************************************************************************************** */
}  // namespace yaml2shell
}  // namespace apps
}  // namespace fpsdk

/* ****************************************************************************************************************** */

int main(int argc, char** argv)
{
    using namespace fpsdk::apps::yaml2shell;
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    bool ok = true;

    // Parse command line arguments
    YamlToShellOptions opts;
    if (!opts.LoadFromArgv(argc, argv)) {
        ok = false;
    }

    if (ok) {
        YamlToShell app(opts);
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
