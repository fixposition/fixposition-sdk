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
#ifdef FP_USE_ROS1
#  include <fpsdk_ros1/bagwriter.hpp>
#  include <fpsdk_ros1/ext/ros_msgs.hpp>
#endif

/* PACKAGE */
#include "fpltool_rosbag.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */
#ifdef FP_USE_ROS1

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::string;
using namespace fpsdk::common::path;
using namespace fpsdk::ros1::bagwriter;

bool DoRosbag(const FpltoolArgs& args)
{
    if (args.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }
    const std::string input_fpl = args.inputs_[0];

    // Determine output file name
    std::string output_bag;
    if (args.output_.empty()) {
        output_bag = input_fpl;
        StrReplace(output_bag, ".fpl", "");
        if ((args.skip_ > 0) || (args.duration_ > 0)) {
            output_bag += "_S" + std::to_string(args.skip_) + "-D" + std::to_string(args.duration_);
        }
        output_bag += ".bag";
    } else {
        output_bag = args.output_;
    }

    // Open input log
    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Open output bag
    if (!args.overwrite_ && PathExists(output_bag)) {
        WARNING("Output file %s already exists", output_bag.c_str());
        return false;
    }
    BagWriter bag;
    if (!bag.Open(output_bag, args.compress_)) {
        return false;
    }

    // Prepare message for stream messages
    std_msgs::UInt8MultiArray stream_msg_ros;
    stream_msg_ros.layout.dim.push_back(std_msgs::MultiArrayDimension());
    stream_msg_ros.layout.dim[0].stride = 1;
    stream_msg_ros.layout.dim[0].label = "bytes";

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
        if (args.progress_ > 0) {
            if (reader.GetProgress(progress, rate)) {
                INFO("Extracting... %.1f%% (%.0f MiB/s)\r", progress, rate);
            }
        }

        // Check if we want to skip this message
        const bool skip = ((args.skip_ > 0) && (time_into_log < args.skip_));

        // Maybe we can abort early
        if ((args.duration_ > 0) && (time_into_log > (args.skip_ + args.duration_))) {
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
                    bag.AddMsgDef(rosmsgdef.topic_name_, rosmsgdef.msg_name_, rosmsgdef.msg_md5_, rosmsgdef.msg_def_);
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
                        !bag.WriteMessage(rosmsgbin.msg_data_, rosmsgbin.topic_name_, rosmsgbin.rec_time_)) {
                        WARNING("Invalid ROSMSGBIN");
                        ok = false;
                    }
                }
                break;
            case FplType::STREAMMSG:
                if (!skip) {
                    const StreamMsg streammsg(log_msg);
                    if (streammsg.valid_) {
                        stream_msg_ros.layout.dim[0].size = streammsg.msg_data_.size();
                        stream_msg_ros.data = streammsg.msg_data_;
                        const std::string topic =
                            "/" + (streammsg.stream_name_ == "corr" ? "ntrip" : streammsg.stream_name_) + "/raw";
                        bag.WriteMessage(stream_msg_ros, topic, streammsg.rec_time_);
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
#else   // FP_USE_ROS1

bool DoRosbag(const FpltoolArgs& args)
{
    WARNING("Command %s unavailable, ROS functionality is not compiled in", args.command_str_.c_str());
    return false;
}
#endif  // FP_USE_ROS1

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
