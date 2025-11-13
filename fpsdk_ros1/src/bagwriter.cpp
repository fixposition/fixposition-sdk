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

/* EXTERNAL */
#include "fpsdk_ros1/ext/ros.hpp"
#include "fpsdk_ros1/ext/topic_tools.hpp"

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>

/* PACKAGE */
#include "fpsdk_ros1/bagwriter.hpp"

namespace fpsdk {
namespace ros1 {
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
    bag_ = std::make_unique<rosbag::Bag>();
    const char* compress_str = "none";
    if (compress > 1) {
        bag_->setCompression(rosbag::compression::CompressionType::BZ2);
        compress_str = "bz2";
    } else if (compress > 0) {
        bag_->setCompression(rosbag::compression::CompressionType::LZ4);
        compress_str = "lz4";
    }
    try {
        bag_->open(path, rosbag::bagmode::Write);
    } catch (const rosbag::BagException& ex) {
        WARNING("BagWriter: open fail %s: %s", path.c_str(), ex.what());
        bag_.reset();
        return false;
    }
    DEBUG("BagWriter: %s (%s)", path.c_str(), compress_str);
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
    if (rosmsgdef.valid_ && (msg_defs_.find(rosmsgdef.topic_name_) == msg_defs_.end())) {
        DEBUG("BagWriter: %s", rosmsgdef.info_.c_str());
        auto hdr = boost::make_shared<ros::M_string>();
        hdr->emplace(std::string("message_definition"), rosmsgdef.msg_def_);
        hdr->emplace(std::string("topic"), rosmsgdef.topic_name_);
        hdr->emplace(std::string("md5sum"), rosmsgdef.msg_md5_);
        hdr->emplace(std::string("type"), rosmsgdef.msg_name_);
        msg_defs_.emplace(rosmsgdef.topic_name_, hdr);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::WriteMessage(const common::fpl::RosMsgBin& rosmsgbin)
{
    if (!rosmsgbin.valid_) {
        return false;
    }
    // For ROS1 we can directly write the serialised data. We should already know the message meta data (see AddMsgDef()
    // above).

    auto msg_defs_entry = msg_defs_.find(rosmsgbin.topic_name_);
    if (msg_defs_entry == msg_defs_.end()) {
        WARNING("BagWriter: missing message definition for %s", rosmsgbin.topic_name_.c_str());
        return false;
    }

    struct ShapeShifterReadHelper
    { /* clang-format off */
        ShapeShifterReadHelper(const uint8_t* data, const uint32_t size) : data_{data}, size_{size} {}
        const uint8_t* data_;
        const uint32_t size_;
        std::size_t getLength() { return size_; }
        const uint8_t* getData() { return data_; }
    };  // clang-format on

    topic_tools::ShapeShifter ros_msg;
    ShapeShifterReadHelper stream(rosmsgbin.msg_data_.data(), rosmsgbin.msg_data_.size());
    ros_msg.read(stream);

    try {
        if (bag_) {
            bag_->write(rosmsgbin.topic_name_, ros::Time(rosmsgbin.rec_time_.sec_, rosmsgbin.rec_time_.nsec_), ros_msg,
                msg_defs_entry->second);
        }
    } catch (std::exception& e) {
        WARNING("BagWriter: write fail: %s", e.what());
        return false;
    }

    return true;
}

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros1
}  // namespace fpsdk
