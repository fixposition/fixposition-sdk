/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: .fpl logfile helpers
 */

/* LIBC/STL */
#include <cmath>
#include <cstring>
#include <fstream>

/* EXTERNAL */
#include "zfstream.hpp"  // Shipped with package

/* PACKAGE */
#include "fpcommon/fpl.hpp"
#include "fpcommon/logging.hpp"
#include "fpcommon/path.hpp"
#include "fpcommon/string.hpp"
#include "fpcommon/time.hpp"
#include "fpcommon/yaml.hpp"

namespace fp {
namespace common {
namespace fpl {
/* ****************************************************************************************************************** */

const char* FplTypeStr(const FplType type)
{
    switch (type) {
            // clang-format off
        case FplType::UNSPECIFIED: return "UNSPECIFIED";
        case FplType::ROSMSGDEF:   return "ROSMSGDEF";
        case FplType::ROSMSGBIN:   return "ROSMSGBIN";
        case FplType::LOGMETA:     return "LOGMETA";
        case FplType::STREAMMSG:   return "STREAMMSG";
        case FplType::LOGSTATUS:   return "LOGSTATUS";
        case FplType::BLOB:        return "BLOB";
        case FplType::INT_D:       return "INT_D";
        case FplType::INT_F:       return "INT_F";
        case FplType::INT_X:       return "INT_X";
            // clang-format on
    }
    return "?";
}

/* ****************************************************************************************************************** */

FplMessage::FplMessage() /* clang-format off */ :
    file_pos_       { 0 },
    file_size_      { 0 },
    file_seq_       { 0 },
    payload_type_   { FplType::UNSPECIFIED },
    payload_size_   { 0 }  // clang-format on
{
    // Deliberately using heap
    message_.reserve(MAX_SIZE);
}

// ---------------------------------------------------------------------------------------------------------------------

int FplMessage::Parse(const uint8_t* data, const uint32_t size)
{
    payload_size_ = 0;
    payload_type_ = FplType::UNSPECIFIED;
    message_.clear();

    if (size == 0) {
        return 0;  // nada
    }
    if (data[0] != (uint8_t)(MAGIC & 0xff)) {
        return 0;  // nada
    }
    if (size < 2) {
        return -1;  // wait
    }
    if (data[1] != (uint8_t)((MAGIC >> 8) & 0xff)) {
        return 0;  // nada
    }
    if (size < HEADER_SIZE) {
        return -1;  // wait
    }
    const uint32_t payload_size =
        ((uint32_t)(data[7]) << 24) | ((uint32_t)(data[6]) << 16) | ((uint32_t)(data[5]) << 8) | (uint32_t)(data[4]);
    const uint32_t total_size = 8 + payload_size + (2 * sizeof(uint8_t));
    if (total_size > MAX_SIZE) {
        return 0;  // nada
    }
    if (size < total_size) {
        return -1;  // wait
    }
    uint8_t ck1 = 0;
    uint8_t ck2 = 0;
    for (uint32_t ix = 0; ix < (total_size - 2); ix++) {
        ck1 += data[ix];
        ck2 += ck1;
    }
    if ((ck1 != data[total_size - 2]) || (ck2 != data[total_size - 1])) {
        return 0;  // nada
    }

    message_ = {data, data + total_size};
    payload_size_ = payload_size;
    payload_type_ = FplType::UNSPECIFIED;
    const uint16_t payload_type = ((uint16_t)message_[3] << 8) | (uint16_t)message_[2];
    switch ((FplType)payload_type) {
        case FplType::UNSPECIFIED:
        case FplType::ROSMSGDEF:
        case FplType::ROSMSGBIN:
        case FplType::LOGMETA:
        case FplType::STREAMMSG:
        case FplType::LOGSTATUS:
            payload_type_ = (FplType)payload_type;
            break;
        case FplType::BLOB:
        case FplType::INT_D:
        case FplType::INT_F:
        case FplType::INT_X:
            break;
    }
    if (payload_type_ == FplType::UNSPECIFIED) {
        switch (payload_type & 0x00ff) { /* clang-format off*/
            case 0x00001:
                payload_type_ = FplType::INT_D;
                break;
            case 0x00003:
                payload_type_ = FplType::INT_F;
                break;
            default:
                payload_type_ = FplType::INT_X;
                break;
        }  // clang-format on
    }

    return total_size;
}

// ---------------------------------------------------------------------------------------------------------------------

const std::vector<uint8_t>& FplMessage::Raw() const
{
    return message_;
}

const uint8_t* FplMessage::RawData() const
{
    return message_.data();
}

// ---------------------------------------------------------------------------------------------------------------------

uint32_t FplMessage::RawSize() const
{
    return message_.size();
}

// ---------------------------------------------------------------------------------------------------------------------

FplType FplMessage::PayloadType() const
{
    return payload_type_;
}

// ---------------------------------------------------------------------------------------------------------------------

const uint8_t* FplMessage::PayloadData() const
{
    return message_.size() > HEADER_SIZE ? &message_[HEADER_SIZE] : NULL;
}

// ---------------------------------------------------------------------------------------------------------------------

uint32_t FplMessage::PayloadSize() const
{
    return payload_size_;
}

// ---------------------------------------------------------------------------------------------------------------------

void FplMessage::SetBlob(const uint8_t* data, const uint32_t size)
{
    message_.resize(size, 0x00);
    std::memcpy(&message_[0], data, size);
    payload_size_ = 0;
    payload_type_ = FplType::BLOB;
}

/* ****************************************************************************************************************** */

FplParser::FplParser() /* clang-format off */ :
    offs_       { 0 },
    size_       { 0 },
    file_pos_   { 0 },
    file_seq_   { 0 }  // clang-format-on
{
    // Deliberately using heap
    buf_.resize(5 * FplMessage::MAX_SIZE, 0x00);
    Reset();
}

// ---------------------------------------------------------------------------------------------------------------------

void FplParser::Reset() {
    file_pos_ += offs_ + size_;
    size_ = 0;
    offs_ = 0;
}
// ---------------------------------------------------------------------------------------------------------------------

bool FplParser::Add(const uint8_t* data, const std::size_t size) {
    if ((data == NULL) || (size == 0)) {
        return false;
    }
    if ((offs_ + size_ + size) > buf_.size()) {
        WARNING("FplParser: overflow");
        return false;
    }

    std::memcpy(&buf_[offs_ + size_], data, size);
    size_ += size;
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplParser::Process(FplMessage& log_msg) {
    while (size_ > 0) {
        const int msg_size = log_msg.Parse(&buf_[offs_], size_);
        if (msg_size < 0) {
            return false;
        } else if (msg_size == 0) {
            offs_++;
            size_--;
            if (offs_ >= FplMessage::MAX_SIZE) {
                EmitBlob(log_msg);
                return true;
            }
        } else {
            if (offs_ > 0) {
                EmitBlob(log_msg);
                return true;
            } else {
                size_ -= msg_size;
                if (size_ > 0) {
                    std::memmove(&buf_[0], &buf_[msg_size], size_);
                }
                log_msg.file_pos_ = file_pos_;
                log_msg.file_size_ = msg_size;
                file_pos_ += msg_size;
                file_seq_++;
                log_msg.file_seq_ = file_seq_;
                return true;
            }

        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

void FplParser::EmitBlob(FplMessage& log_msg)
{
    const uint32_t blob_size = offs_;
    log_msg.SetBlob(&buf_[0], blob_size);
    std::memmove(&buf_[0], &buf_[blob_size], size_);
    offs_ = 0;
    log_msg.file_pos_ = file_pos_;
    log_msg.file_size_ = blob_size;
    file_pos_ += blob_size;
    file_seq_++;
    log_msg.file_seq_ = file_seq_;
}

/* ****************************************************************************************************************** */

FplFileReader::FplFileReader() /* clang-format off */ :
    pos_             { 0 },
    count_           { 0 },
    size_            { 0 },
    is_gz_           { false },
    last_progress_   { -1.0 },
    last_pos_        { 0 },
    last_secs_       { 0.0 }  // clang-format on
{
    // Deliberately using heap
    buf_.resize(FplMessage::MAX_SIZE, 0x00);
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplFileReader::Open(const std::string& path)
{
    path_.clear();
    fh_.reset();
    is_gz_ = fp::common::string::StrEndsWith(path, ".gz");
    if (is_gz_) {
        fh_ = std::make_unique<gzifstream>(path.c_str());
    } else {
        fh_ = std::make_unique<std::ifstream>(path, std::ios::binary);
    }
    if (fh_->fail()) {
        WARNING("FplFileReader: fail open %s: %s", path.c_str(), std::strerror(errno));
        fh_.reset();
        return false;
    }
    path_ = path;
    size_ = fp::common::path::FileSize(path);
    last_progress_ = -1.0;
    last_pos_ = 0;
    last_secs_ = fp::common::time::GetSecs();
    DEBUG("FplFileReader: %s (%.1f MiB)", path.c_str(), (double)size_ / 1024.0 / 1024.0);
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplFileReader::Next(FplMessage& log_msg)
{
    while (true) {
        if (parser_.Process(log_msg)) {
            pos_ = log_msg.file_pos_ + log_msg.RawSize();
            count_++;
            return true;
        }
        if (fh_) {
            fh_->read((char*)buf_.data(), buf_.size());
            const auto size = fh_->gcount();
            if (size > 0) {
                if (!parser_.Add(buf_.data(), size)) {
                    fh_.reset();
                    path_.clear();
                    return false;
                }
            } else {
                fh_.reset();
                path_.clear();
                return false;
            }
        } else {
            return false;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplFileReader::GetProgress(double& progress, double& rate)
{
    progress = (size_ > 0 ? (float)((double)pos_ / (double)size_ * 1e2) : 0.0f);
    // Print at least every 5% or 50k messages
    if (((progress - last_progress_) >= 5.0f) || ((count_ % 50000) == 0)) {
        // Throughput
        const double secs = fp::common::time::GetSecs();
        const double dt = secs - last_secs_;
        const uint64_t dp = pos_ - last_pos_;
        rate = ((dt > 0.0) && (dp > 0.0) ? (double)dp / dt / 1024.0 / 1024.0 : 0.0);
        last_pos_ = pos_;
        last_secs_ = secs;
        last_progress_ = progress;
        return true;
    } else {
        return false;
    }
}

/* ****************************************************************************************************************** */

LogMeta::LogMeta(const FplMessage& log_msg)
{
    bool ok = false;
    const uint32_t payload_size = log_msg.PayloadSize();
    if ((log_msg.PayloadType() == FplType::LOGMETA) && (payload_size > 1)) {
        const uint8_t* payload = log_msg.PayloadData();
        const uint32_t yaml_len = strlen((const char*)payload);
        yaml_ = {(const char*)payload, std::min(payload_size, yaml_len)};
        YAML::Node meta;
        if (fp::common::yaml::StringToYaml(yaml_, meta)) {
            try {
                const int meta_ver = meta["meta_ver"].as<int>();
                if (meta_ver >= 1) {
                    // clang-format off
                    hw_uid_               = meta["hw_uid"].as<std::string>();
                    hw_product_           = meta["hw_product"].as<std::string>();
                    sw_version_           = meta["sw_version"].as<std::string>();
                    log_start_time_posix_ = meta["log_start_time_posix"].as<uint32_t>();
                    log_start_time_iso_   = meta["log_start_time_iso"].as<std::string>();
                    log_profile_          = meta["log_profile"].as<std::string>();
                    log_target_           = meta["log_target"].as<std::string>();
                    log_filename_         = meta["log_filename"].as<std::string>();
                    yaml_ += "\n";
                    info_ =
                         "hw_uid="         + hw_uid_ +
                        " hw_product="     + hw_product_ +
                        " sw_version="     + sw_version_ +
                        " log_profile="    + log_profile_ +
                        " log_target="     + log_target_ +
                        " log_filename="   + log_filename_ +
                        " log_start_time=" + log_start_time_iso_;
                    // clang-format on
                    ok = true;
                } else {
                    throw std::runtime_error("bad/unknown version");
                }
            } catch (std::exception& ex) {
                WARNING("LogMeta: bad meta data: %s", ex.what());
            }
        }
    }
    if (!ok) {
        WARNING("LogMeta: invalid message");
        info_ = "<invalid>";
    }
    valid_ = ok;
}

// ---------------------------------------------------------------------------------------------------------------------

LogStatus::LogStatus(const FplMessage& log_msg)
{
    bool ok = false;
    const uint32_t payload_size = log_msg.PayloadSize();
    if ((log_msg.PayloadType() == FplType::LOGSTATUS) && (payload_size > 1)) {
        const uint8_t* payload = log_msg.PayloadData();
        // strlen() because there may be multiple trailing \0 (due to padding)
        const uint32_t yaml_len = strlen((const char*)payload);
        yaml_ = {(const char*)payload, std::min(payload_size, yaml_len)};
        YAML::Node status;
        if (fp::common::yaml::StringToYaml(yaml_, status)) {
            try {
                const int status_ver = status["status_ver"].as<int>();
                if (status_ver >= 1) {  // clang-format off
                    state_        = status["state"].as<std::string>();
                    queue_size_   = status["queue_size"].as<uint32_t>();
                    queue_peak_   = status["queue_peak"].as<uint32_t>();
                    queue_skip_   = status["queue_skip"].as<uint32_t>();
                    log_count_    = status["log_count"].as<uint32_t>();
                    log_errors_   = status["log_errors"].as<uint32_t>();
                    log_size_     = status["log_size"].as<uint64_t>();
                    log_duration_ = status["log_duration"].as<uint32_t>();
                    yaml_ += "\n";
                    info_ =
                         "state="        + state_ +
                        " queue_size="   + std::to_string(queue_size_) +
                        " queue_peak="   + std::to_string(queue_peak_) +
                        " queue_skip="   + std::to_string(queue_skip_) +
                        " log_count="    + std::to_string(log_count_) +
                        " log_errors="   + std::to_string(log_errors_) +
                        " log_size="     + std::to_string(log_size_) +
                        " log_duration=" + std::to_string(log_duration_);  // clang-format on
                    ok = true;
                } else {
                    throw std::runtime_error("bad/unknown version");
                }
                if (status_ver >= 2) {
                    log_time_posix_ = status["log_time_posix"].as<uint32_t>();
                    log_time_iso_ = status["log_time_iso"].as<std::string>();
                    pos_avail_ = status["pos_avail"].as<bool>();
                    pos_lat_ = status["pos_lat"].as<double>();
                    pos_lon_ = status["pos_lon"].as<double>();
                    pos_height_ = status["pos_height"].as<double>();
                    info_ += " log_time=" + log_time_iso_ +
                             fp::common::string::Sprintf(" pos=%.6f/%.6f/%.0f", pos_avail_ ? pos_lat_ : NAN,
                                 pos_avail_ ? pos_lon_ : NAN, pos_avail_ ? pos_height_ : NAN);
                }

            } catch (std::exception& ex) {
                WARNING("LogStatus: bad status data: %s", ex.what());
            }
        }
    }
    if (!ok) {
        WARNING("LogStatus: invalid message");
        info_ = "<invalid>";
    }
    valid_ = ok;
}

// ---------------------------------------------------------------------------------------------------------------------

RosMsgDef::RosMsgDef(const FplMessage& log_msg)
{
    bool ok = true;
    if (log_msg.PayloadType() == FplType::ROSMSGDEF) {
        const uint8_t* payload = log_msg.PayloadData();
        const uint32_t payload_size = log_msg.PayloadSize();

        // clang-format off
        const char*       topic_name     = (const char*)&payload[0];
        const std::size_t topic_name_len = std::strlen(topic_name);
        const char*       msg_name       = (const char*)&payload[topic_name_len + 1];
        const std::size_t msg_name_len   = std::strlen(msg_name);
        const char*       msg_md5        = (const char*)&payload[topic_name_len + 1 + msg_name_len + 1];
        const std::size_t msg_md5_len    = std::strlen(msg_md5);
        const char*       msg_def        = (const char*)&payload[topic_name_len + 1 + msg_name_len + 1 + msg_md5_len + 1];
        const std::size_t msg_def_len    = std::strlen(msg_def);
        // clang-format on

        if ((topic_name_len > FplMessage::MAX_NAME_LEN) || (msg_name_len > FplMessage::MAX_NAME_LEN) ||
            (msg_md5_len > FplMessage::MAX_NAME_LEN) ||
            ((topic_name_len + 1 + msg_name_len + 1 + msg_md5_len + 1 + msg_def_len + 1) > payload_size)) {
            ok = false;
        } else {
            topic_name_ = std::string(topic_name);
            msg_name_ = std::string(msg_name);
            msg_md5_ = std::string(msg_md5);
            msg_def_ = std::string(msg_def);
        }
    } else {
        ok = false;
    }
    if (ok) {
        info_ = "topic_name=" + topic_name_ + " msg_name=" + msg_name_ + " msg_md5=" + msg_md5_ + " msg_def=<" +
                std::to_string(msg_def_.size()) + ">";
    } else {
        WARNING("RosMsgDef: invalid message");
        info_ = "<invalid>";
    }
    valid_ = ok;
}

// ---------------------------------------------------------------------------------------------------------------------

RosMsgBin::RosMsgBin(const FplMessage& log_msg)
{
    bool ok = true;
    if (log_msg.PayloadType() == FplType::ROSMSGBIN) {
        const uint8_t* payload = log_msg.PayloadData();
        const uint32_t payload_size = log_msg.PayloadSize();

        // Time
        std::memcpy(&rec_time_.sec_, &payload[0], sizeof(rec_time_.sec_));
        std::memcpy(&rec_time_.nsec_, &payload[sizeof(rec_time_.sec_)], sizeof(rec_time_.nsec_));

        // Topic name
        const uint32_t name_offs = sizeof(rec_time_);
        const char* topic_name = (const char*)&payload[name_offs];
        topic_name_ = std::string(topic_name);
        if (topic_name_.size() > FplMessage::MAX_NAME_LEN) {
            topic_name_.clear();
            ok = false;
        }

        // The serialised ROS message wrapped into the helper so that we can read() it into the ShapeShifter message
        if (ok) {
            const uint32_t msg_offs = name_offs + topic_name_.size() + 1;
            const uint8_t* msg_bin = &payload[msg_offs];
            const uint32_t msg_size = payload_size - msg_offs;
            msg_data_ = {msg_bin, msg_bin + msg_size};
        }
    } else {
        ok = false;
    }
    if (ok) {
        info_ = fp::common::string::Sprintf("rec_time=%.3f topic_name=%s msg_data=<%" PRIu64 ">", rec_time_.ToSec(),
            topic_name_.c_str(), msg_data_.size());
    } else {
        WARNING("RosMsgBin: invalid message");
        info_ = "<invalid>";
    }
    valid_ = ok;
}

// ---------------------------------------------------------------------------------------------------------------------

StreamMsg::StreamMsg(const FplMessage& log_msg)
{
    bool ok = true;
    if (log_msg.PayloadType() == FplType::STREAMMSG) {
        const uint8_t* payload = log_msg.PayloadData();
        const uint32_t payload_size = log_msg.PayloadSize();

        // Time
        std::memcpy(&rec_time_.sec_, &payload[0], sizeof(rec_time_.sec_));
        std::memcpy(&rec_time_.nsec_, &payload[sizeof(rec_time_.sec_)], sizeof(rec_time_.nsec_));

        // Stream name
        const uint32_t name_offs = sizeof(rec_time_);
        const char* stream_name = (const char*)&payload[name_offs];
        stream_name_ = std::string(stream_name);
        if (stream_name_.size() > FplMessage::MAX_NAME_LEN) {
            stream_name_.clear();
            ok = false;
        }

        // Frame size
        const uint32_t size_offs = name_offs + stream_name_.size() + 1;
        uint32_t frame_size = 0;
        if (ok && (payload_size > (size_offs + sizeof(frame_size)))) {
            std::memcpy(&frame_size, &payload[size_offs], sizeof(frame_size));
        } else {
            ok = false;
        }

        // Frame data
        const uint32_t frame_offs = size_offs + sizeof(frame_size);
        if (ok && (payload_size >= (frame_offs + frame_size))) {
            const uint8_t* frame_bin = &payload[frame_offs];
            msg_data_ = {frame_bin, frame_bin + frame_size};
        } else {
            ok = false;
        }
    } else {
        ok = false;
    }
    if (ok) {
        info_ = fp::common::string::Sprintf("rec_time=%.3f stream_name=%s msg_data=<%" PRIu64 ">", rec_time_.ToSec(),
            stream_name_.c_str(), msg_data_.size());
    } else {
        WARNING("StreamMsg: invalid message");
        info_ = "<invalid>";
    }
    valid_ = ok;
}

/* ****************************************************************************************************************** */
}  // namespace fpl
}  // namespace common
}  // namespace fp
