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
 * @brief Fixposition SDK: ROS2 bag writer
 */

/* LIBC/STL */
#include <stdexcept>

/* EXTERNAL */
#include <std_msgs/msg/string.hpp>
#include "fpsdk_ros2/ext/msgs.hpp"

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/ros1.hpp>

/* PACKAGE */
#include "fpsdk_ros2/bagwriter.hpp"
#include "fpsdk_ros2/ros1.hpp"
#include "fpsdk_ros2/utils.hpp"

namespace fpsdk {
namespace ros2 {
namespace bagwriter {
/* ****************************************************************************************************************** */

BagWriter::BagWriter()
{
}

BagWriter::~BagWriter()
{
    Close();
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::Open(const std::string& path, const int compress)
{
    Close();
    bag_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions opts;
    opts.uri = path;
    if (compress > 0) {
        opts.storage_id = "mcap";  // apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap
        opts.storage_preset_profile = (compress > 1 ? "zstd_small" : "zstd_fast");
    } else {
        opts.storage_id = "sqlite3";
    }
    try {
        bag_->open(opts);
    } catch (const std::exception& ex) {
        WARNING("BagWriter: open fail %s: %s. Maybe %s storage plugin is not installed?", path.c_str(), ex.what(),
            opts.storage_id.c_str());
        bag_.reset();
        return false;
    }
    DEBUG("BagWriter: %s", path.c_str());

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::Close()
{
    if (bag_) {
        bag_->close();
        bag_.reset();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::AddMsgDef(const common::fpl::RosMsgDef& rosmsgdef)
{
    if (rosmsgdef.valid_ && (defs_.find(rosmsgdef.topic_name_) == defs_.end())) {
        DEBUG("BagWriter: %s", rosmsgdef.info_.c_str());
        defs_[rosmsgdef.topic_name_] = rosmsgdef;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

template <typename Ros1MsgT, typename Ros2MsgT>
inline bool WriteMessageEx(
    const common::fpl::RosMsgDef& rosmsgdef, const common::fpl::RosMsgBin& rosmsgbin, ros2::bagwriter::BagWriter& bag)
{
    if (rosmsgdef.msg_name_ == ros::message_traits::datatype<Ros1MsgT>()) {
        Ros1MsgT ros1;
        common::ros1::DeserializeMessage(rosmsgbin.msg_data_, ros1);
        Ros2MsgT ros2;
        ros2::ros1::Ros1ToRos2(ros1, ros2);
        bag.WriteMessage(ros2, rosmsgbin.topic_name_, rosmsgbin.rec_time_);
        return true;
    } else {
        return false;
    }
}

bool BagWriter::WriteMessage(const common::fpl::RosMsgBin& rosmsgbin)
{
    if (!rosmsgbin.valid_) {
        return false;
    }

    const auto& entry = defs_.find(rosmsgbin.topic_name_);
    if (entry == defs_.end()) {
        WARNING("BagWriter: missing message definition for %s", rosmsgbin.topic_name_.c_str());

        WARNING_THR(1000, "Missing ROSMSGDEF for ROSMSGBIN %s", rosmsgbin.info_.c_str());
        return false;
    }

    const auto& rosmsgdef = entry->second;

    // For ROS2 we have to instantiate the ROS1 message in order to be able to convert it to the corresponding ROS2
    // message type, which includes the message meta data and definition. This can then be written to the bag. Only some
    // conversions are implemented.
    try {
        if (WriteMessageEx<sensor_msgs::Imu, sensor_msgs::msg::Imu>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<sensor_msgs::Temperature, sensor_msgs::msg::Temperature>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<sensor_msgs::Image, sensor_msgs::msg::Image>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<nav_msgs::Odometry, nav_msgs::msg::Odometry>(rosmsgdef, rosmsgbin, *this) ||
            WriteMessageEx<tf2_msgs::TFMessage, tf2_msgs::msg::TFMessage>(rosmsgdef, rosmsgbin, *this)) {
        } else {
            throw std::runtime_error("conversion not implemented");
        }
    } catch (std::exception& ex) {
        WARNING("BagWriter: write fail: %s %s", rosmsgdef.msg_name_.c_str(), ex.what());
        return false;
    }

    return true;
}

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros2
}  // namespace fpsdk
