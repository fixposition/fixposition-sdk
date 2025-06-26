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
 */
#ifndef __FPSDK_COMMON_ROS1_HPP__
#define __FPSDK_COMMON_ROS1_HPP__

/* LIBC/STL */
#include <cstdint>
#include <memory>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* ROS */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wunused-function"
//
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <tf2_msgs/TFMessage.h>
//
#include <ros/serialization.h>
//
#pragma GCC diagnostic pop

/* PACKAGE */

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
    ConstBuffer(const std::vector<uint8_t>& buf)
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

/* ****************************************************************************************************************** */
}  // namespace ros1
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_ROS1_HPP__
