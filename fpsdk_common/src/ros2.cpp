/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS2 types and utils
 *
 * @note This is only available if compiled with ROS2, see @ref FPSDK_BUILD_DEPS.
 */
#if FPSDK_USE_ROS2

/* LIBC/STL */
#  include <stdexcept>

/* EXTERNAL */

/* PACKAGE */
#  include "fpsdk_common/logging.hpp"
#  include "fpsdk_common/ros2.hpp"

namespace fpsdk {
namespace common {
namespace ros2 {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::logging;

// ---------------------------------------------------------------------------------------------------------------------

static std::unique_ptr<rclcpp::Logger> g_logger;

static void sLoggingFn(const LoggingParams& /*params*/, const LoggingLevel level, const char* str)
{
    // By default these will appear under the "fpsdk_common" logger
    switch (level) {  // clang-format off
        case LoggingLevel::TRACE:   RCLCPP_DEBUG((*g_logger), "%s", str); break;
        case LoggingLevel::DEBUG:   RCLCPP_DEBUG((*g_logger), "%s", str); break;
        case LoggingLevel::INFO:    RCLCPP_INFO( (*g_logger), "%s", str); break;
        case LoggingLevel::NOTICE:  RCLCPP_INFO( (*g_logger), "%s", str); break;
        case LoggingLevel::WARNING: RCLCPP_WARN( (*g_logger), "%s", str); break;
        case LoggingLevel::ERROR:   RCLCPP_ERROR((*g_logger), "%s", str); break;
        case LoggingLevel::FATAL:   RCLCPP_FATAL((*g_logger), "%s", str); break;
    }  // clang-format on
}

void RedirectLoggingToRosConsole(const char* logger_name)
{
    g_logger = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(logger_name));
    LoggingParams params = LoggingGetParams();
    params.fn_ = sLoggingFn;
    params.level_ = LoggingLevel::TRACE;  // We leave it up to ROS to decide what to print
    LoggingSetParams(params);
}

// ---------------------------------------------------------------------------------------------------------------------

rclcpp::Time ConvTime(const fpsdk::common::time::Time& time, rcl_clock_type_t clock_type)
{
    const auto rt = time.GetRosTime();
    return rclcpp::Time(rt.sec_, rt.nsec_, clock_type);
}

fpsdk::common::time::Time ConvTime(const rclcpp::Time& time)
{
    const uint64_t nsec = time.nanoseconds();
    return fpsdk::common::time::Time::FromRosTime(
        { static_cast<uint32_t>(nsec / 1000000000), static_cast<uint32_t>(nsec % 1000000000) });
}
/* ****************************************************************************************************************** */

BagWriter::BagWriter()
{
}

BagWriter::~BagWriter()
{
    Close();
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::Open(const std::string& path, const bool mcap, const int compress)
{
    Close();
    bag_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions opts;
    opts.uri = path;
    if (mcap) {
        opts.storage_id = "mcap";  // apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap
        opts.storage_preset_profile = (compress > 0 ? "zstd_small" : "zstd_fast");
    } else {
        opts.storage_id = "sqlite3";
    }
    try {
        bag_->open(opts);
    } catch (const std::exception& ex) {
        WARNING("BagWriter: open fail %s: %s. Maybe the %s storage plugin is not installed?", path.c_str(), ex.what(),
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
    const common::fpl::RosMsgDef& rosmsgdef, const common::fpl::RosMsgBin& rosmsgbin, BagWriter& bag)
{
    if (rosmsgdef.msg_name_ == ros::message_traits::datatype<Ros1MsgT>()) {
        Ros1MsgT ros1;
        common::ros1::DeserializeMessage(rosmsgbin.msg_data_, ros1);
        Ros2MsgT ros2;
        Ros1ToRos2(ros1, ros2);
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
}  // namespace ros2
}  // namespace common
}  // namespace fpsdk
#endif  // FPSDK_USE_ROS2
