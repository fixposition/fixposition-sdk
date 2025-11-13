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
#include <set>
#include <string>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/ros1.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/to_json/fpl.hpp>
#include <fpsdk_common/types.hpp>
#if defined(FPSDK_USE_ROS1)
#  include <fpsdk_ros1/bagwriter.hpp>
#elif defined(FPSDK_USE_ROS2)
#  include <fpsdk_ros2/bagwriter.hpp>
#endif

/* PACKAGE */
#include "fpltool_extract.hpp"
#include "fpltool_utils.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::path;
using namespace fpsdk::common::ros1;
using namespace fpsdk::common::string;
using namespace fpsdk::common::types;
#ifdef FPSDK_USE_ROS1
using namespace fpsdk::ros1::bagwriter;
#endif
#ifdef FPSDK_USE_ROS2
using namespace fpsdk::ros2::bagwriter;
#endif

// ---------------------------------------------------------------------------------------------------------------------

bool DoExtract(const FplToolOptions& opts)
{
    if (opts.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }

    const std::string input_fpl = opts.inputs_[0];
    const std::string output_prefix = opts.GetOutputPrefix(input_fpl);

    // Open input log
    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Output files
    OutputFileHelper output(output_prefix, opts);
    const char* JSONL_NAME = (opts.compress_ > 0 ? "all.jsonl.gz" : "all.jsonl");
    const char* RAW_EXT = (opts.compress_ > 0 ? ".raw.gz" : ".raw");

    // Check which output formats we want
    bool doJsonl = opts.formats_.empty();
    bool doRaw = opts.formats_.empty();
    bool doFile = opts.formats_.empty();
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    bool doRos = opts.formats_.empty();
#else
    bool doRos = false;
#endif

    for (auto& fmt : opts.formats_) {
        if (fmt == opts.FORMAT_JSONL) {
            doJsonl = true;
        } else if (fmt == opts.FORMAT_RAW) {
            doRaw = true;
        } else if (fmt == opts.FORMAT_FILE) {
            doFile = true;
        } else if (fmt == opts.FORMAT_ROS) {
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
            doRos = true;
#else
            WARNING("Cannot extract to ROS bag. This fpltool is not built with ROS support.");
            return false;
#endif
        } else {
            WARNING("Bad argument '%s' to option -e, --formats", fmt.c_str());
            return false;
        }
    }

    if (!doJsonl && !doRaw && !doFile && !doRos) {
        WARNING("No output formats selected");
        return false;
    }

    NOTICE("Extracting from %s to %s_...", input_fpl.c_str(), output_prefix.c_str());

#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    BagWriter bag;
#  ifdef FPSDK_USE_ROS1
    const auto output_bag = output_prefix + ".bag";  // File
#  else
    const auto output_bag = output_prefix + "_bag";  // Directory! (even for single-file .mcap)
#  endif
    if (PathExists(output_bag)) {
        if (!opts.overwrite_) {
            WARNING("Output bag %s already exists", output_bag.c_str());
            return false;
        } else {
            RemoveAll(output_bag);
        }
    }
    if (doRos && !bag.Open(output_bag, opts.compress_)) {
        return false;
    }

    NOTICE("Extracting to %s", output_bag.c_str());
#endif

    // Helper for converting ROS1 messages to JSON
    RosMsgHelper rosmsg_helper;
    // Helper for processing STREAMMSGs
    ParserMsgHelper parsermsg_helper;

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process log
    FplMessage log_msg;
    double progress = 0.0;
    double rate = 0.0;
    bool ok = true;
    bool have_meta = false;
    std::set<std::string> files_dumped;
    uint32_t time_into_log = 0;
    std::size_t errors = 0;
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
            case FplType::LOGSTATUS: {
                const LogStatus logstatus(log_msg);
                if (logstatus.valid_) {
                    time_into_log = logstatus.log_duration_;
                    if (!skip && doJsonl && !output.WriteJson(JSONL_NAME, logstatus)) {
                        ok = false;
                    }
                } else {
                    WARNING("Invalid LOGSTATUS");
                    errors++;
                }
                break;
            }

            case FplType::LOGMETA: {
                const LogMeta logmeta(log_msg);
                if (logmeta.valid_) {
                    if ((!have_meta || !skip) && doJsonl && !output.WriteJson(JSONL_NAME, logmeta)) {
                        ok = false;
                    }
                    have_meta = true;
                } else {
                    WARNING("Invalid LOGMETA");
                    errors++;
                }
                break;
            }

            case FplType::STREAMMSG:
                if (!skip) {
                    const StreamMsg streammsg(log_msg);
                    if (streammsg.valid_) {
                        if (doJsonl || doRos) {
                            parsermsg_helper.UpdateParserMsg(streammsg);
                        }
                        if (doRaw && !output.WriteData(streammsg.stream_name_ + RAW_EXT, streammsg.msg_data_)) {
                            ok = false;
                        }
                        if (doJsonl &&
                            !output.WriteStreamMsg(JSONL_NAME, streammsg, parsermsg_helper.GetParserMsg(true))) {
                            ok = false;
                        }
#if defined(FPSDK_USE_ROS1)  // || defined(FPSDK_USE_ROS2)  // @todo implement for ROS2, s.a. fpltools_utils.hpp
                        if (doRos) {
                            bag.WriteMessage(parsermsg_helper.GetRosMsg(), "/" + streammsg.stream_name_ + "/raw",
                                streammsg.rec_time_);
                        }
#endif
                    } else {
                        WARNING("Invalid STREAMMSG");
                        errors++;
                    }
                }
                break;

            case FplType::ROSMSGDEF: {
                RosMsgDef rosmsgdef(log_msg);
                if (rosmsgdef.valid_) {
                    rosmsgdef.topic_name_ = FixTopicName(rosmsgdef.topic_name_);

                    if (doJsonl || doRos) {
                        rosmsg_helper.AddDef(rosmsgdef);
                    }
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
                    if (doRos) {
                        bag.AddMsgDef(rosmsgdef);
                    }
#endif
                } else {
                    WARNING("Invalid ROSMSGDEF");
                    errors++;
                }
                break;
            }

            case FplType::ROSMSGBIN:
                if (!skip) {
                    RosMsgBin rosmsgbin(log_msg);
                    if (rosmsgbin.valid_) {
                        rosmsgbin.topic_name_ = FixTopicName(rosmsgbin.topic_name_);

                        nlohmann::json jdata;
                        if (doJsonl && rosmsg_helper.ToJson(rosmsgbin, jdata) && !output.WriteJson(JSONL_NAME, jdata)) {
                            ok = false;
                        }
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
                        if (doRos && !bag.WriteMessage(rosmsgbin)) {
                            ok = false;
                        }
#endif
                    } else {
                        WARNING("Invalid ROSMSGBIN");
                        errors++;
                    }
                }
                break;

            case FplType::FILEDUMP: {
                FileDump filedump(log_msg);
                if (filedump.valid_) {
                    const bool dumped_before = files_dumped.count(filedump.filename_) == 0;
                    if ((!dumped_before || !skip) && doFile &&
                        !output.WriteData(
                            FileDumpOutName(filedump) + (opts.compress_ > 0 ? ".gz" : ""), filedump.data_)) {
                        ok = false;
                    }
                    if ((!dumped_before || !skip) && doJsonl && !output.WriteJson(JSONL_NAME, filedump)) {
                        ok = false;
                    }
                    if (!dumped_before) {
                        files_dumped.emplace(filedump.filename_);
                    }
                } else {
                    WARNING("Invalid FILEDUMP");
                    errors++;
                }
            }

            case FplType::BLOB:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X:
            case FplType::UNSPECIFIED:
                break;
        }
        if (errors >= 100) {
            WARNING("Too many errors!");
            ok = false;
        }
    }

    // We were interrupted
    if (sig_int.ShouldAbort()) {
        ok = false;
    }

    // Close output files
    output.CloseAll(ok);
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    if (doRos) {
        bag.Close();
        if (ok) {
            INFO("Wrote bag %s (%s)", output_bag.c_str(), OutputSizeStr(output_bag).c_str());
        } else {
            WARNING("Incomplete bag %s (%s)", output_bag.c_str(), OutputSizeStr(output_bag).c_str());
        }
    }
#endif

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
