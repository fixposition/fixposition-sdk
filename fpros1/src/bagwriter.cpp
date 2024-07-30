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
 * @brief Fixposition SDK: ROS1 bag writer
 */

/* LIBC/STL */

/* EXTERNAL */
#include "fpros1/ext/topic_tools.hpp"

/* Fixposition SDK */
#include <fpcommon/logging.hpp>

/* PACKAGE */
#include "fpros1/bagwriter.hpp"

namespace fp {
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
    } catch (const rosbag::BagException& e) {
        WARNING("BagWriter: fail open %s: %s", path.c_str(), e.what());
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

void BagWriter::WriteMessage(const rosbag::MessageInstance& msg, const std::string& topic)
{
    time_ = msg.getTime();
    if (bag_) {
        bag_->write(topic.empty() ? msg.getTopic() : topic, time_, msg);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void BagWriter::AddMsgDef(
    const std::string& topic, const std::string& msg_name, const std::string& msg_md5, const std::string& msg_def)
{
    if (msg_defs_.find(topic) == msg_defs_.end()) {
        DEBUG("BagWriter: %s, %s, %s, <%" PRIu64 ">", topic.c_str(), msg_name.c_str(), msg_md5.c_str(), msg_def.size());
        auto hdr = boost::make_shared<ros::M_string>();
        hdr->emplace(std::string("message_definition"), msg_def);
        hdr->emplace(std::string("topic"), topic);
        hdr->emplace(std::string("md5sum"), msg_md5);
        hdr->emplace(std::string("type"), msg_name);
        msg_defs_.emplace(topic, hdr);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool BagWriter::WriteMessage(const std::vector<uint8_t>& data, const std::string& topic, const RosTime& time)
{
    return WriteMessage(data, topic, ros::Time(time.sec_, time.nsec_));
}

bool BagWriter::WriteMessage(const std::vector<uint8_t>& data, const std::string& topic, const ros::Time& time)
{
    if (!time.isZero()) {
        time_ = time;
    }

    auto msg_defs_entry = msg_defs_.find(topic);
    if (msg_defs_entry == msg_defs_.end()) {
        ROS_WARN("BagWriter: missing message definition for %s", topic.c_str());
        return false;
    }

    struct ShapeShifterReadHelper { /* clang-format off */
        ShapeShifterReadHelper(const uint8_t* data, const uint32_t size) : data_{data}, size_{size} {}
        const uint8_t* data_;
        const uint32_t size_;
        std::size_t getLength() { return size_; }
        const uint8_t* getData() { return data_; }
    };  // clang-format on

    topic_tools::ShapeShifter ros_msg;
    ShapeShifterReadHelper stream(data.data(), data.size());
    ros_msg.read(stream);

    try {
        bag_->write(topic, time_, ros_msg, msg_defs_entry->second);
    } catch (std::exception& e) {
        ROS_WARN("BagWriter: write fail: %s", e.what());
        return false;
    }

    return true;
}

/* ****************************************************************************************************************** */
}  // namespace bagwriter
}  // namespace ros1
}  // namespace fp
