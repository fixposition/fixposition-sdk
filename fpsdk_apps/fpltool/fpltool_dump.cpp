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
 * @brief Fixposition SDK: fpltool dump
 */

/* LIBC/STL */
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>

/* PACKAGE */
#include "fpltool_dump.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::string;

static void PrintHexDump(const int level, const std::string& prefix, const uint8_t* data, const int size)
{
    if (level <= 0) {
        return;
    }
    const auto lines = HexDump(data, size);
    // Level 1: first and last 4 lines
    // Level 2: first and last 10 lines
    // Level 3+: all of it
    const std::size_t n = lines.size();
    const std::size_t thrs = (level > 1 ? 10 : 4);
    std::size_t ix0 = (n > (2 * thrs) ? thrs - 1 : n - 1);
    std::size_t ix1 = (n > (2 * thrs) ? n - thrs : n);
    for (std::size_t ix = 0; ix <= ix0; ix++) {
        std::printf("%s%s\n", prefix.c_str(), lines[ix].c_str());
    }
    if (ix1 < n) {
        std::printf("%s...\n", prefix.c_str());
    }
    for (std::size_t ix = ix1; ix < n; ix++) {
        std::printf("%s%s\n", prefix.c_str(), lines[ix].c_str());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool DoDump(const FplToolOptions& opts)
{
    if (opts.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }
    const std::string input_fpl = opts.inputs_[0];

    NOTICE("Dumping %s", input_fpl.c_str());

    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Message counts
    std::map<FplType, uint64_t> counts;

    // Process all messages
    FplMessage log_msg;
    double progress = 0.0;
    double rate = 0.0;
    std::unique_ptr<LogMeta> first_logmeta;
    std::unique_ptr<LogStatus> last_logstatus;

    while (!sig_int.ShouldAbort() && reader.Next(log_msg)) {
        // "extra" levels
        // - <= 0: report progress while scanning through the log
        // - == 1: also print message info
        // - >= 2: also add (partial) hexdump for each message

        // Report progress
        if (opts.progress_ > 0) {
            if (reader.GetProgress(progress, rate)) {
                INFO("Dumping... %.1f%% (%.0f MiB/s)\r", progress, rate);
            }
        }

        // Get some info
        const auto log_type = log_msg.PayloadType();
        std::string info = "-";
        switch (log_type) {
            case FplType::LOGMETA: {
                const LogMeta logmeta(log_msg);
                info = logmeta.info_;
                if (logmeta.valid_) {
                    first_logmeta = std::make_unique<LogMeta>(std::move(logmeta));
                }
                break;
            }
            case FplType::ROSMSGDEF: {
                const RosMsgDef rosmsgdef(log_msg);
                info = rosmsgdef.info_;
                break;
            }
            case FplType::ROSMSGBIN: {
                const RosMsgBin rosmsgbin(log_msg);
                info = rosmsgbin.info_;
                break;
            }
            case FplType::STREAMMSG: {
                const StreamMsg streammsg(log_msg);
                info = streammsg.info_;
                break;
            }
            case FplType::LOGSTATUS: {
                const LogStatus logstatus(log_msg);
                info = logstatus.info_;
                if (logstatus.valid_) {
                    last_logstatus = std::make_unique<LogStatus>(std::move(logstatus));
                }
                break;
            }
            case FplType::FILEDUMP: {
                const FileDump filedump(log_msg);
                info = filedump.info_;
                break;
            }
            case FplType::BLOB:
            case FplType::UNSPECIFIED:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X: {
                const int size = log_msg.RawSize();
                info = HexDump(log_msg.RawData(), std::min(size, 8))[0].substr(14, 23);
                break;
            }
        }

        // Print summary of the message
        if (opts.extra_ > 0) {
            std::printf("data %8" PRIu64 " 0x%08" PRIx64 " 0x%06" PRIx32 " %-15s %s\n", log_msg.file_seq_,
                log_msg.file_pos_, log_msg.RawSize(), FplTypeStr(log_type), info.c_str());
        }

        // Print hexdump
        PrintHexDump(opts.extra_ - 1, "    ", log_msg.RawData(), log_msg.RawSize());

        // Update counts
        auto entry = counts.find(log_type);
        if (entry == counts.end()) {
            entry = counts.emplace(log_type, 0).first;
        }
        entry->second++;
    }
    std::printf("\n");

    // Print counts
    uint64_t total_count = 0;
    for (const auto& entry : counts) {
        std::printf("stats   %-18s  %" PRIu64 "\n", FplTypeStr(entry.first), entry.second);
        total_count += entry.second;
    }
    std::printf("stats   total               %" PRIu64 "\n", total_count);
    std::printf("\n");

    // Print log meta data
    if (first_logmeta) {
        std::printf("meta    hw_uid              %s\n", first_logmeta->hw_uid_.c_str());
        std::printf("meta    product_model       %s\n", first_logmeta->product_model_.c_str());
        std::printf("meta    sw_version          %s\n", first_logmeta->sw_version_.c_str());
        std::printf("meta    log_start_time_iso  %s\n", first_logmeta->log_start_time_iso_.c_str());
        std::printf("meta    log_profile         %s\n", first_logmeta->log_profile_.c_str());
        std::printf("meta    log_target          %s\n", first_logmeta->log_target_.c_str());
        std::printf("meta    log_filename        %s\n", first_logmeta->log_filename_.c_str());
        std::printf("\n");
    } else {
        WARNING("No LOGMETA found");
    }

    // Print log status
    if (last_logstatus) {
        std::printf("status  state               %s\n", last_logstatus->state_.c_str());
        std::printf("status  queue_size          %" PRIu32 "\n", last_logstatus->queue_size_);
        std::printf("status  queue_peak          %" PRIu32 "\n", last_logstatus->queue_peak_);
        std::printf("status  queue_skip          %" PRIu32 "\n", last_logstatus->queue_skip_);
        std::printf("status  queue_bsize         %" PRIu32 "\n", last_logstatus->queue_bsize_);
        std::printf("status  queue_bpeak         %" PRIu32 "\n", last_logstatus->queue_bpeak_);
        std::printf("status  log_count           %" PRIu32 "\n", last_logstatus->log_count_);
        std::printf("status  log_errors          %" PRIu32 "\n", last_logstatus->log_errors_);
        std::printf("status  log_size            %" PRIu64 "\n", last_logstatus->log_size_);
        std::printf("status  log_duration        %" PRIu32 "\n", last_logstatus->log_duration_);
        std::printf("\n");
    } else {
        WARNING("No LOGSTATUS found");
    }

    return true;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
