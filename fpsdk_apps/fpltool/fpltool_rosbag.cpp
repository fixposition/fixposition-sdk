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
 * @brief Fixposition SDK: fpltool rosbag
 */

/* LIBC/STL */
#include <string>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/string.hpp>
#ifdef FPSDK_USE_ROS1
#  include <fpsdk_ros1/bagwriter.hpp>
#  include <fpsdk_ros1/msgs.hpp>
#endif

/* PACKAGE */
#include "fpltool_rosbag.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */
#ifdef FPSDK_USE_ROS1

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::string;
using namespace fpsdk::common::path;
using namespace fpsdk::ros1::bagwriter;

static std::string TopicName(const std::string& in_topic)
{
    if (in_topic == "/fusion_optim/imu_biases") {
        return "/imu/biases";
    } else {
        return in_topic;
    }
}

bool DoRosbag(const FplToolOptions& opts)
{
    if (opts.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }
    const std::string input_fpl = opts.inputs_[0];

    // Determine output file name
    std::string output_bag;
    if (opts.output_.empty()) {
        output_bag = input_fpl;
        StrReplace(output_bag, ".fpl", "");
        if ((opts.skip_ > 0) || (opts.duration_ > 0)) {
            output_bag += "_S" + std::to_string(opts.skip_) + "-D" + std::to_string(opts.duration_);
        }
        output_bag += ".bag";
    } else {
        output_bag = opts.output_;
    }

    // Open input log
    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Open output bag
    if (!opts.overwrite_ && PathExists(output_bag)) {
        WARNING("Output file %s already exists", output_bag.c_str());
        return false;
    }
    BagWriter bag;
    if (!bag.Open(output_bag, opts.compress_)) {
        return false;
    }

    // Prepare message for stream messages
    fpsdk_ros1::ParserMsg stream_msg_ros;
    stream_msg_ros.protocol = fpsdk::ros1::msgs::ParserMsg::PROTOCOL_UNSPECIFIED;
    stream_msg_ros.seq = 0;

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process log
    NOTICE("Extracting %s to %s", input_fpl.c_str(), output_bag.c_str());
    FplMessage log_msg;
    double progress = 0.0;
    double rate = 0.0;
    bool ok = true;
    uint32_t time_into_log = 0;
    while (!sig_int.ShouldAbort() && reader.Next(log_msg)) {
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
            case FplType::LOGMETA: {
                break;
            }
            case FplType::ROSMSGDEF: {
                const RosMsgDef rosmsgdef(log_msg);
                if (rosmsgdef.valid_) {
                    bag.AddMsgDef(
                        TopicName(rosmsgdef.topic_name_), rosmsgdef.msg_name_, rosmsgdef.msg_md5_, rosmsgdef.msg_def_);
                } else {
                    WARNING("Invalid ROSMSGDEF");
                    ok = false;
                }
                break;
            }
            case FplType::ROSMSGBIN:
                if (!skip) {
                    const RosMsgBin rosmsgbin(log_msg);
                    if (!rosmsgbin.valid_ ||
                        !bag.WriteMessage(rosmsgbin.msg_data_, TopicName(rosmsgbin.topic_name_), rosmsgbin.rec_time_)) {
                        WARNING("Invalid ROSMSGBIN");
                        ok = false;
                    }
                }
                break;
            case FplType::STREAMMSG:
                if (!skip) {
                    const StreamMsg streammsg(log_msg);
                    if (streammsg.valid_) {
                        stream_msg_ros.stamp = { streammsg.rec_time_.sec_, streammsg.rec_time_.nsec_ };
                        stream_msg_ros.data = streammsg.msg_data_;
                        // stream_msg_ros.name = ...; // @todo run data through parser
                        // stream_msg_ros.protocol = ...; // @todo run data through parser
                        const std::string topic =
                            "/" + (streammsg.stream_name_ == "corr" ? "ntrip" : streammsg.stream_name_) + "/raw";
                        bag.WriteMessage(stream_msg_ros, topic, streammsg.rec_time_);
                        // stream_msg_ros.seq++; // @todo per topic
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

    bag.Close();

    const double bag_size = FileSize(output_bag) / 1024.0 / 1024.0;
    if (ok) {
        INFO("Wrote bag %s (%.0f MiB)", output_bag.c_str(), bag_size);
    } else {
        WARNING("Incomplete bag %s (%.0f MiB)", output_bag.c_str(), bag_size);
    }

    return ok;
}

/* ****************************************************************************************************************** */
#else   // FPSDK_USE_ROS1

bool DoRosbag(const FplToolOptions& opts)
{
    WARNING("Command %s unavailable, ROS functionality is not compiled in", opts.command_str_.c_str());
    return false;
}
#endif  // FPSDK_USE_ROS1

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
