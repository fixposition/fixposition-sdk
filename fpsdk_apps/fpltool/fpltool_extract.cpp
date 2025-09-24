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
 * @brief Fixposition SDK: fpltool extract
 */

/* LIBC/STL */
#include <map>
#include <memory>
#include <string>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/string.hpp>

/* PACKAGE */
#include "fpltool_extract.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::string;
using namespace fpsdk::common::path;

bool DoExtract(const FplToolOptions& opts)
{
    if (opts.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }
    const std::string input_fpl = opts.inputs_[0];

    // Determine prefix for output file names
    std::string output_prefix;
    if (opts.output_.empty()) {
        output_prefix = input_fpl;
        StrReplace(output_prefix, ".fpl", "");
        if ((opts.skip_ > 0) || (opts.duration_ > 0)) {
            output_prefix += "_S" + std::to_string(opts.skip_) + "-D" + std::to_string(opts.duration_);
        }
    } else {
        output_prefix = opts.output_;
    }

    // Open input log
    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Output files, and a helper func to write to them
    std::map<std::string, std::unique_ptr<OutputFile>> output_files;
    auto WriteOutput = [&output_files, &input_fpl, &opts, &output_prefix](
                           const std::string& name, const std::vector<uint8_t>& data) -> bool {
        // Create and open new file if necessary
        auto entry = output_files.find(name);
        if (entry == output_files.end()) {
            entry = output_files.insert({ name, std::make_unique<OutputFile>() }).first;
            const std::string path = output_prefix + "_" + name + ".raw";
            if (!opts.overwrite_ && PathExists(path)) {
                WARNING("Output file %s already exists", path.c_str());
                return false;
            }
            NOTICE("Extracting %s to %s", input_fpl.c_str(), path.c_str());
            if (!entry->second->Open(path)) {
                return false;
            }
        }
        if (!entry->second->Write(data)) {
            return false;
        }
        return true;
    };

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process log
    FplMessage log_msg;
    double progress = 0.0;
    double rate = 0.0;
    bool ok = true;
    uint32_t time_into_log = 0;
    while (!sig_int.ShouldAbort() && reader.Next(log_msg) && ok) {
        // Report progress
        if (opts.progress_ > 0) {
            if (reader.GetProgress(progress, rate)) {
                INFO("Extracting... %.1f%% (%.0f MiB/s)\r", progress, rate);
            }
        }

        // Check if we want to skip this message
        const bool skip = ((opts.skip_ > 0) && (time_into_log < opts.skip_));

        // Maybe we can abort early
        if ((opts.duration_ > 0) && (time_into_log > (opts.skip_ + opts.duration_))) {
            DEBUG("abort early");
            break;
        }

        // Process message
        const auto log_type = log_msg.PayloadType();
        switch (log_type) {
            case FplType::STREAMMSG:
                if (!skip) {
                    const StreamMsg streammsg(log_msg);
                    if (streammsg.valid_) {
                        if (!WriteOutput(streammsg.stream_name_, streammsg.msg_data_)) {
                            ok = false;
                        }
                    } else {
                        WARNING("Invalid STREAMMSG");
                        ok = false;
                    }
                }
                break;
            case FplType::LOGSTATUS: {
                const LogStatus logstatus(log_msg);
                if (logstatus.valid_) {
                    time_into_log = logstatus.log_duration_;
                }
                break;
            }
            case FplType::LOGMETA:
            case FplType::ROSMSGDEF:
            case FplType::ROSMSGBIN:
            case FplType::BLOB:
            case FplType::UNSPECIFIED:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X:
                break;
        }
    }

    // We were interrupted
    if (sig_int.ShouldAbort()) {
        ok = false;
    }

    // Close output files
    for (auto& entry : output_files) {
        const std::string path = entry.second->Path();
        entry.second->Close();
        const double raw_size = FileSize(path) / 1024.0 / 1024.0;
        if (ok) {
            INFO("Wrote file %s (%.1f MiB)", path.c_str(), raw_size);
        } else {
            WARNING("Incomplete raw %s (%.1f MiB)", path.c_str(), raw_size);
        }
    }

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
