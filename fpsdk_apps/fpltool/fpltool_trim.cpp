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
 * @brief Fixposition SDK: fpltool trim
 */

/* LIBC/STL */
#include <unordered_map>
#include <unordered_set>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */
#include "fpltool_trim.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::path;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;

bool DoTrim(const FplToolOptions& opts)
{
    if (opts.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }
    const std::string input_fpl = opts.inputs_[0];

    if ((opts.skip_ < 60) || (opts.duration_ < 60)) {
        WARNING("The -S as well as the -D arguments must be >= 60");
        return false;
    }

    // Determine output file name
    std::string output_fpl = opts.GetOutputPrefix(input_fpl) + ".fpl" + (opts.compress_ ? ".gz" : "");

    // Open input log
    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Open output bag
    if (!opts.overwrite_ && PathExists(output_fpl)) {
        WARNING("Output file %s already exists", output_fpl.c_str());
        return false;
    }
    OutputFile fpl;
    if (!fpl.Open(output_fpl)) {
        return false;
    }

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process log
    NOTICE("Trimming %s (%" PRIu32 " + %" PRIu32 ") to %s", input_fpl.c_str(), opts.skip_, opts.duration_,
        output_fpl.c_str());
    FplMessage log_msg;
    double progress = 0.0;
    double rate = 0.0;
    bool ok = true;
    bool fpl_ok = true;
    bool wrote_meta = false;
    std::unordered_map<std::string, FplMessage> msgdefs_fpl;
    std::unordered_set<std::string> msgdefs_written;
    FplMessage last_status;
    uint32_t time_into_log = 0;
    uint64_t n_used = 0;
    while (fpl_ok && !sig_int.ShouldAbort() && reader.Next(log_msg)) {
        // Report progress
        if (opts.progress_ > 0) {
            if (reader.GetProgress(progress, rate)) {
                INFO("Processing... %.1f%% (%.0f MiB/s) -- time into log %" PRIu32 ", used %" PRIu64 " msgs\r",
                    progress, rate, time_into_log, n_used);
            }
        }

        // Decisions to use messages or not
        const bool use0 = (time_into_log >= (opts.skip_ - 60)) && (time_into_log < (opts.skip_ + opts.duration_));
        const bool use1 = (time_into_log >= opts.skip_) && (time_into_log < (opts.skip_ + opts.duration_));

        // Process message
        const auto log_type = log_msg.PayloadType();
        switch (log_type) {
            case FplType::LOGMETA:
                if (!wrote_meta) {
                    if (fpl.Write(log_msg.Raw())) {
                        wrote_meta = true;
                        n_used++;
                    } else {
                        fpl_ok = false;
                    }
                }
                break;
            case FplType::ROSMSGDEF: {
                const RosMsgDef rosmsgdef(log_msg);
                // Collect message definitions
                if (rosmsgdef.valid_) {
                    if (msgdefs_fpl.find(rosmsgdef.topic_name_) == msgdefs_fpl.end()) {
                        msgdefs_fpl.emplace(rosmsgdef.topic_name_, log_msg);
                    }
                } else {
                    WARNING("Invalid ROSMSGDEF");
                    ok = false;
                }
                break;
            }
            case FplType::ROSMSGBIN:
                if (use1) {
                    const RosMsgBin rosmsgbin(log_msg);
                    if (rosmsgbin.valid_) {
                        // Only if message definition known
                        const auto entry = msgdefs_fpl.find(rosmsgbin.topic_name_);
                        if (entry != msgdefs_fpl.end()) {
                            // Write message definition once, and first
                            if (msgdefs_written.find(rosmsgbin.topic_name_) == msgdefs_written.end()) {
                                if (fpl.Write(entry->second.Raw())) {
                                    n_used++;
                                } else {
                                    fpl_ok = false;
                                }
                                msgdefs_written.emplace(rosmsgbin.topic_name_);
                            }
                            // Write message
                            if (fpl.Write(log_msg.Raw())) {
                                n_used++;
                            } else {
                                fpl_ok = false;
                            }
                        }
                    } else {
                        WARNING("Invalid ROSMSGBIN");
                        ok = false;
                    }
                }
                break;
            case FplType::LOGSTATUS: {
                const LogStatus logstatus(log_msg);
                last_status = log_msg;
                if (logstatus.valid_) {
                    time_into_log = logstatus.log_duration_;
                }
                if (use0) {
                    if (fpl.Write(log_msg.Raw())) {
                        n_used++;
                    } else {
                        fpl_ok = false;
                    }
                }
                break;
            }
            case FplType::STREAMMSG:
            case FplType::INT_X:
                if (use1) {
                    if (fpl.Write(log_msg.Raw())) {
                        n_used++;
                    } else {
                        fpl_ok = false;
                    }
                }
                break;
            case FplType::FILEDUMP:
            case FplType::INT_F:
                if (fpl.Write(log_msg.Raw())) {
                    n_used++;
                } else {
                    fpl_ok = false;
                }
                break;
            case FplType::INT_D:
                if (use0) {
                    if (fpl.Write(log_msg.Raw())) {
                        n_used++;
                    } else {
                        fpl_ok = false;
                    }
                }
                break;
            case FplType::BLOB:
            case FplType::UNSPECIFIED:
                WARNING("Unexpected %s", FplTypeStr(log_type));
                ok = false;
                break;
        }
    }

    // We were interrupted or failed writing output
    if (sig_int.ShouldAbort() || !fpl_ok) {
        ok = false;
    }

    // Check
    LogStatus logstatus(last_status);
    if (!logstatus.valid_) {
        WARNING("Bad input log");
        ok = false;
    } else if (logstatus.log_duration_ < (opts.skip_ + opts.duration_)) {
        WARNING("Too short input log resp. too long skip + duration (%" PRIu32 " < %" PRIu32 " + %" PRIu32 ")",
            logstatus.log_duration_, opts.skip_, opts.duration_);
        ok = false;
    } else {
        fpl.Write(last_status.Raw());
        n_used++;
    }

    fpl.Close();

    const double fpl_size = FileSize(output_fpl) / 1024.0 / 1024.0;
    if (ok) {
        INFO("Wrote fpl %s (%.0f MiB)", output_fpl.c_str(), fpl_size);
    } else {
        WARNING("Incomplete fpl %s (%.0f MiB)", output_fpl.c_str(), fpl_size);
    }

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
