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
 * @brief Fixposition SDK: ROS1 types and utils
 *
 * @page FPSDK_COMMON_ROS1 ROS1 types and utils
 *
 * **API**: fpsdk_common/ros1.hpp and fpsdk::common::ros1
 *
 * @note Some of this always available, even when built in a non-ROS or ROS2 environment, some of it is only available
 *       when built in a ROS1 environment, see @ref FPSDK_BUILD_DEPS.
 */
#ifndef __FPSDK_COMMON_ROS1_HPP__
#define __FPSDK_COMMON_ROS1_HPP__

/* LIBC/STL */
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* ROS1 from fpsdk_common/rosnoros or real ROS1 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wunused-function"
//
#if FPSDK_USE_ROS1
#  include <ros/console.h>
#  include <ros/time.h>
#endif
//
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/ByteMultiArray.h>
#include <tf2_msgs/TFMessage.h>
//
#include <ros/serialization.h>
//
#pragma GCC diagnostic pop

/* PACKAGE */
#include "fpl.hpp"
#include "time.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief ROS1 types and utils
 */
namespace ros1 {
/* ****************************************************************************************************************** */

#ifndef _DOXYGEN_
// Helper for ros de-serialization. We could DeserializeMessage() using ros::serialization::deserialize(IStream(buf),
// msg). However, the IStream() wants a mutable buffer, which is not nice. ConstBuffer() provides the Stream() interface
// without needing the mutability of the buffer itself.
struct ConstBuffer
{
    explicit ConstBuffer(const std::vector<uint8_t>& buf)
        : data_{ buf.data() }, end_{ buf.data() + static_cast<uint32_t>(buf.size()) }
    {
    }
    ConstBuffer(const uint8_t* data, size_t size) : data_(data), end_(data + static_cast<uint32_t>(size))
    {
    }

    static const ros::serialization::StreamType stream_type = ros::serialization::stream_types::Input;
    inline const uint8_t* getData()
    {
        return data_;
    }
    inline const uint8_t* advance(uint32_t len)
    {
        const uint8_t* old_data = data_;
        data_ += len;
        if (data_ > end_) {
            ros::serialization::throwStreamOverrun();
        }
        return old_data;
    }
    inline uint32_t getLength()
    {
        return static_cast<uint32_t>(end_ - data_);
    }

    template <typename T>
    inline void next(T& t)
    {
        ros::serialization::deserialize(*this, t);
    }

    template <typename T>
    inline ConstBuffer& operator>>(T& t)
    {
        ros::serialization::deserialize(*this, t);
        return *this;
    }

   private:
    const uint8_t* data_;
    const uint8_t* end_;
};
#endif  // _DOXYGEN_

/**
 * @brief Deserialise ROS1 message
 *
 * @tparam RosMsgT  The ROS1 message type
 * @param[in]  buf  The serialised ROS message
 * @param[out] msg  The deserialised ROS message
 */
template <typename RosMsgT>
inline void DeserializeMessage(const std::vector<uint8_t>& buf, RosMsgT& msg)
{
#if FPSDK_USE_ROS1
    ros::serialization::IStream s((uint8_t*)buf.data(), static_cast<uint32_t>(buf.size()));  // :-(
    ros::serialization::deserialize(s, msg);
#else
    ConstBuffer m(buf);
    ros::serialization::Serializer<RosMsgT>::read(m, msg);
#endif
}

/**
 * @brief Convert to ROS time (atomic -> POSIX)
 *
 * @param[in]  time  The Time object (atomic)
 *
 * @returns the ROS time object (POSIX)
 */
ros::Time ConvTime(const time::Time& time);

/**
 * @brief Convert from ROS time (POSIX -> atomic)
 *
 * @param[in]  time  The ROS time object (POSIX)
 *
 * @returns the Time object (atomic)
 */
time::Time ConvTime(const ros::Time& time);

/**
 * @brief ROS1 bag (rosbag) writer
 *
 * This writes ROS1 bag files (rosbag format version 2.0). Unlike the rosbag library from ROS1 this does not need ROS
 * and therefore it works in all builds of the SDK (with ROS1, with ROS2, and without any ROS). Messages can either be
 * given as ROS1 message objects (see WriteMessage()) or as already serialised data from a .fpl logfile (see
 * AddMsgDef() and WriteMessage()).
 *
 * Compared to the original rosbag library this implementation has some limitations:
 *
 * - Only "bz2" chunk compression is available (and only if compiled with BZip2, see @ref FPSDK_BUILD_DEPS)
 * - No encryption
 * - Only one connection per topic
 * - Write only (no reading, no appending to existing bags)
 *
 * The resulting bags are valid rosbag version 2.0 files and can be read by any ROS1 tool (rosbag info, rosbag play,
 * rqt_bag, ...) as well as other tools that understand the format.
 */
class BagWriter
{
   public:
    BagWriter();
    ~BagWriter();

    BagWriter(const BagWriter&) = delete;             //!< No copy
    BagWriter& operator=(const BagWriter&) = delete;  //!< No copy

    /**
     * @brief Open bag for writing
     *
     * @param[in]  path       Path/filename of the bag file (an existing file is overwritten)
     * @param[in]  compress   Compress bag, 0 = no compression, 1+ = BZ2 (if compiled in)
     *
     * @returns true if bag was sucessfully opened, false otherwise (bad path, compression requested but bz2 support not
     *          compiled in, ...)
     */
    bool Open(const std::string& path, const int compress = 0);

    /**
     * @brief Close bag
     *
     * @note Failing to close the bag (for example, by not destroying the object) leaves an unusable (unindexed) file
     *       behind. The index is only written on close.
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
     * @returns true if message was added, false otherwise (bag not open, write error)
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const time::RosTime& time = {})
    {
        const uint32_t size = ros::serialization::serializationLength(msg);
        std::vector<uint8_t> data(size);
        if (size > 0) {
            ros::serialization::OStream stream(data.data(), size);
            ros::serialization::serialize(stream, msg);
        }
        return WriteSerialised(topic, time, ros::message_traits::datatype<T>(), ros::message_traits::md5sum<T>(),
            ros::message_traits::definition<T>(), data.data(), data.size());
    }

    /**
     * @brief Write a message to the bag
     *
     * @tparam     T      ROS message type
     * @param[in]  msg    The message
     * @param[in]  topic  Topic name
     * @param[in]  time   Bag record time
     *
     * @returns true if message was added, false otherwise (bag not open, write error)
     */
    template <typename T>
    bool WriteMessage(const T& msg, const std::string& topic, const ros::Time& time)
    {
        return WriteMessage<T>(msg, topic, time::RosTime(time.sec, time.nsec));
    }

    /**
     * @brief Add ROS message definition from .fpl
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  rosmsgdef  The message definition
     */
    void AddMsgDef(const fpl::RosMsgDef& rosmsgdef);

    /**
     * @brief Write message from .fpl
     *
     * @note No checks on the provided data are done!
     *
     * @param[in]  rosmsgbin  The recorded message
     *
     * @returns true if message was added, false otherwise (message definition missing, bag not open, write error)
     */
    bool WriteMessage(const fpl::RosMsgBin& rosmsgbin);

   private:
#ifndef _DOXYGEN_
    struct Impl;  // The actual bag file writer, see ros1.cpp
    std::unique_ptr<Impl> impl_;

    bool WriteSerialised(const std::string& topic, const time::RosTime& time, const std::string& msg_name,
        const std::string& msg_md5, const std::string& msg_def, const uint8_t* data, const std::size_t size);
#endif  // _DOXYGEN_
};

#if FPSDK_USE_ROS1 || defined(_DOXYGEN_)
/**
 * @brief Redirect fpsdk:common::logging to ROS console
 *
 * @note This is only available when built in a ROS1 environment.
 *
 * This configures the fpsdk::common::logging facility to output via the ROS console. This does *not* configure the ROS
 * console (logger level, logger name, etc.).
 *
 * The mapping of fpsdk::common::logging::LoggingLevel to ros::console::levels is as follows:
 *
 * - TRACE and DEBUG --> DEBUG
 * - INFO and NOTICE --> INFO
 * - WARNING         --> WARN
 * - ERROR           --> ERROR
 * - FATAL           --> FATAL
 *
 * @param[in]  logger_name  The name of the logger. The default value should give the caller package's
 *                          ROSCONSOLE_DEFAULT_NAME, for example, "ros1_fpsdk_demo". That is, typically this argument
 *                          should be left empty (the default value).
 */
void RedirectLoggingToRosConsole(const char* logger_name = ROSCONSOLE_DEFAULT_NAME /* = caller's package name */);
#endif

/* ****************************************************************************************************************** */
}  // namespace ros1
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_ROS1_HPP__
