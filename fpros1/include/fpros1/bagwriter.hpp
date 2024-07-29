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
 * @page FPROS1_BAGWRITER ROS1 bag writer
 *
 * API: fp::ros1::bagwriter
 *
 */
#ifndef __FPROS1_BAGWRITER_HPP__
#define __FPROS1_BAGWRITER_HPP__

/* LIBC/STL */
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

/* EXTERNAL */
#include "fpros1/ext/ros_time.hpp"
#include "fpros1/ext/rosbag_bag.hpp"

/* Fixposition SDK */
#include <fpcommon/time.hpp>

/* PACKAGE */

namespace fp {
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
     * @param path       Path/filename of the bag file
     * @param compress   Compress bag, 0 = no compression, 1 = LZ4, 2+ = BZ2
     *
     * @returns true if bag was sucessfully opened
     */
    bool Open(const std::string& path, const int compress);

    /**
     * @brief Close bag
     */
    void Close();

    /**
     * @brief Write a message to the bag, do nothing if no bag is open
     *
     * @param[in]  msg    The message
     * @param[in]  topic  Optional topic name, if not given the original topic name is used
     *
     * The bag record time of the msg is used in the output bag. This time is stored and used for writing messages
     * that have no time (templated method below).
     */
    void WriteMessage(const rosbag::MessageInstance& msg, const std::string& topic = "");

    /**
     * @brief Write a message to the bag, do nothing if no bag is open
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Optional bag record time, if ros::Time() is given the time from the last written message
     *                    (see method above) is used
     */
    template <typename T>
    void WriteMessage(const T& msg, const std::string& topic, const ros::Time& time = {})
    {
        if (bag_) {
            if (!time.isZero()) {
                time_ = time;
            }
            bag_->write(topic, time_, msg);
        }
    }

    using RosTime = fp::common::time::RosTime;  //!< Shortcut

    /**
     * @brief Write a message to the bag, do nothing if no bag is open
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Optional bag record time, if ros::Time() is given the time from the last written message
     *                    (see method above) is used
     */
    template <typename T>
    void WriteMessage(const T& msg, const std::string& topic, const RosTime& time = {})
    {
        WriteMessage<T>(msg, topic, ros::Time(time.sec_, time.nsec_));
    }

    /**
     * @brief Add ROS message definition
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  topic     The topic name
     * @param[in]  msg_name  The message name (a.k.a. data type)
     * @param[in]  msg_md5   The message MD5 sum
     * @param[in]  msg_def   The message definition
     */
    void AddMsgDef(
        const std::string& topic, const std::string& msg_name, const std::string& msg_md5, const std::string& msg_def);

    /**
     * @brief Write raw binary (serialised) message
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  data   The binary message data
     * @param[in]  topic  The topic name
     * @param[in]  time   Optional bag record time, if not given the time from the last written message is used
     *
     * @returns true if message was added, false otherwise (message definition missing)
     */
    bool WriteMessage(const std::vector<uint8_t>& data, const std::string& topic, const ros::Time& time = {});

    /**
     * @brief Write raw binary (serialised) message
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  data   The binary message data
     * @param[in]  topic  The topic name
     * @param[in]  time   Optional bag record time, if not given the time from the last written message is used
     *
     * @returns true if message was added, false otherwise (message definition missing)
     */
    bool WriteMessage(const std::vector<uint8_t>& data, const std::string& topic, const RosTime& time = {});

   private:
    std::unique_ptr<rosbag::Bag> bag_;                                  //!< Bag file handle
    ros::Time time_;                                                    //!< Last known bag record time
    std::map<std::string, boost::shared_ptr<ros::M_string>> msg_defs_;  //!< Message definitions (connection headers)
};

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros1
}  // namespace fp
#endif  // __FPROS1_BAGWRITER_HPP__
