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
 * @brief Fixposition SDK: ROS1 bag writer
 *
 * @page FPSDK_ROS1_BAGWRITER ROS1 bag writer
 *
 * **API**: fpsdk_ros1/bagwriter.hpp and fpsdk::ros1::bagwriter
 *
 */
#ifndef __FPSDK_ROS1_BAGWRITER_HPP__
#define __FPSDK_ROS1_BAGWRITER_HPP__

/* LIBC/STL */
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

/* EXTERNAL */
#include "fpsdk_ros1/ext/ros_time.hpp"
#include "fpsdk_ros1/ext/rosbag_bag.hpp"

/* Fixposition SDK */
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/time.hpp>

/* PACKAGE */

namespace fpsdk {
namespace ros1 {
/**
 * @brief ROS1 bag writer
 */
namespace bagwriter {
/* ****************************************************************************************************************** */

/**
 * @brief ROS1 bag writer helper
 */
class BagWriter
{
   public:
    BagWriter();
    ~BagWriter();

    /**
     * @brief Open bag for writing
     *
     * @param[in]  path       Path/filename of the bag file
     * @param[in]  compress   Compress bag, 0 = no compression, 1 = LZ4, 2+ = BZ2
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
    bool WriteMessage(const T& msg, const std::string& topic, const ros::Time& time)
    {
        bool ok = false;
        if (bag_) {
            try {
                bag_->write(topic, time, msg);
                ok = true;
            } catch (const rosbag::BagException& ex) {
                WARNING("BagWriter: write fail: %s", ex.what());
            }
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
     *
     * @returns true if message was added, false otherwise (message definition missing)
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const common::time::RosTime& time = {})
    {
        return WriteMessage<T>(msg, topic, ros::Time(time.sec_, time.nsec_));
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
     * @returns true if message was added, false otherwise (message definition missing)
     */
    bool WriteMessage(const common::fpl::RosMsgBin& rosmsgbin);

   private:
    std::unique_ptr<rosbag::Bag> bag_;                                  //!< Bag file handle
    std::map<std::string, boost::shared_ptr<ros::M_string>> msg_defs_;  //!< Message definitions (connection headers)
};

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros1
}  // namespace fpsdk
#endif  // __FPSDK_ROS1_BAGWRITER_HPP__
