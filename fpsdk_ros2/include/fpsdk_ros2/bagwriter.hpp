/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS2 bag writer
 *
 * @page FPSDK_ROS2_BAGWRITER ROS2 bag writer
 *
 * **API**: fpsdk_ros1/bagwriter.hpp and fpsdk::ros1::bagwriter
 *
 */
#ifndef __FPSDK_ROS2_BAGWRITER_HPP__
#define __FPSDK_ROS2_BAGWRITER_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include "fpsdk_ros2/ext/rosbag2_cpp_writer.hpp"

/* Fixposition SDK */
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fpsdk {
namespace ros2 {
/**
 * @brief ROS2 bag writer
 */
namespace bagwriter {
/* ****************************************************************************************************************** */

/**
 * @brief ROS2 bag writer helper
 */
class BagWriter
{
   public:
    BagWriter();
    ~BagWriter();

    /**
     * @brief Open bag for writing
     *
     * @param[in]  path      Path of the bag directory
     * @param[in]  compress  Compress bag, 0 = no compression, 1 = ...
     *                       Note: not fully implemented
     *
     * @returns true if bag was sucessfully opened
     */
    bool Open(const std::string& path, const int compress = 0);

    /**
     * @brief Close bag
     */
    void Close();

    /**
     * @brief Write a message to the bag
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Bag record time
     *
     * @returns true if message was added, false otherwise (message definition missing)
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const rclcpp::Time& time)
    {
        bool ok = false;
        try {
            if (bag_) {
                bag_->write(msg, topic, time);
                ok = true;
            }
        } catch (const std::exception& ex) {
            WARNING("BagWriter: write fail: %s", ex.what());
        }
        return ok;
    }

    /**
     * @brief Write a message to the bag
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Bag record time
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const common::time::RosTime& time)
    {
        return WriteMessage(msg, topic, rclcpp::Time(time.sec_, time.nsec_, RCL_ROS_TIME));
    }

    /**
     * @brief Add ROS message definition from .fpl
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  rosmsgdef  The message definition
     */
    void AddMsgDef(const common::fpl::RosMsgDef& rosmsgdef);

    /**
     * @brief Write message from .fpl
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  rosmsgbin  The recorded message
     *
     * @returns true if message was added, false otherwise (e.g. ROS1->ROS2 conversion not implemented)
     */
    bool WriteMessage(const common::fpl::RosMsgBin& rosmsgbin);

   private:
    std::unique_ptr<rosbag2_cpp::Writer> bag_;            //!< Bag file handle
    std::map<std::string, common::fpl::RosMsgDef> defs_;  //!< Message definitions (connection headers)
};

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros2
}  // namespace fpsdk
#endif  // __FPSDK_ROS2_BAGWRITER_HPP__
