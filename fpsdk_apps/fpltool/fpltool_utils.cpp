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
 * @brief Fixposition SDK: fpltool extract
 */

/* LIBC/STL */
#include <exception>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/ros1.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/to_json/fpl.hpp>
#include <fpsdk_common/to_json/fpl_ros1.hpp>
#include <fpsdk_common/to_json/nmea.hpp>
#include <fpsdk_common/to_json/parser.hpp>
#include <fpsdk_common/to_json/parser_fpa.hpp>
#include <fpsdk_common/to_json/parser_fpb.hpp>
#include <fpsdk_common/to_json/ros1.hpp>
#include <fpsdk_common/to_json/time.hpp>
#include <fpsdk_common/types.hpp>
#if defined(FPSDK_USE_ROS1)
#  include <fpsdk_ros1/msgs.hpp>
#endif

/* PACKAGE */
#include "fpltool_utils.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::parser;
using namespace fpsdk::common::parser::fpa;
using namespace fpsdk::common::parser::fpb;
using namespace fpsdk::common::parser::nmea;
using namespace fpsdk::common::path;
using namespace fpsdk::common::ros1;
using namespace fpsdk::common::string;
using namespace fpsdk::common::types;

// ---------------------------------------------------------------------------------------------------------------------

OutputFileHelper::OutputFileHelper(const std::string& prefix, const FplToolOptions& opts) /* clang-format off */ :
        prefix_   { prefix },
        opts_     { opts }  // clang-format on
{
}
OutputFileHelper::~OutputFileHelper()
{
    CloseAll();
}

bool OutputFileHelper::WriteData(const std::string& name, const std::vector<uint8_t>& data)
{
    auto f = GetOutputFile(name);
    return f ? f->Write(data) : false;
}

bool OutputFileHelper::WriteJson(const std::string& name, const nlohmann::json& json)
{
    auto f = GetOutputFile(name);
    if (!f) {
        return false;
    }
    return f->Write(json.dump()) && f->Write("\n");
}

bool OutputFileHelper::WriteStreamMsg(const std::string& name, const StreamMsg& streammsg, const ParserMsg& parsermsg)
{
    nlohmann::json jdata = streammsg;  // magic to_json() (_type, _stamp, _stream, _raw/_raw_b64)
    jdata.update(parsermsg);           // magic to_json() (_proto, _name, _seq, _info, _raw/_raw_b64)

    // We can decode some messages of some protocols
    if (msg_.proto_ == Protocol::FP_A) {
        jdata.update(FpaDecodeMessage(msg_.data_));  // magic to_json()
    } else if (msg_.proto_ == Protocol::NMEA) {
        jdata.update(NmeaDecodeMessage(msg_.data_));  // magic to_json()
    } else if (msg_.proto_ == Protocol::FP_B) {
        jdata.update(to_json_FP_B(msg_));  // magic to_json()
    }
    return WriteJson(name, jdata);
}

void OutputFileHelper::CloseAll(const bool ok)
{
    for (auto& file : files_) {
        const std::string path = file.second->Path();
        file.second->Close();
        const auto size_str = OutputSizeStr(path);
        if (ok) {
            INFO("Wrote file %s (%s)", path.c_str(), size_str.c_str());
        } else {
            WARNING("Incomplete file %s (%s)", path.c_str(), size_str.c_str());
        }
    }
    files_.clear();
}

OutputFile* OutputFileHelper::GetOutputFile(const std::string& name)
{
    auto file = files_.find(name);
    if (file == files_.end()) {
        const std::string path = prefix_ + "_" + name;
        file = files_.emplace(name, std::make_unique<OutputFile>()).first;
        if (!opts_.overwrite_ && PathExists(path)) {
            WARNING("Output file %s already exists", path.c_str());
            return nullptr;
        }
        NOTICE("Extracting to %s", path.c_str());
        if (!file->second->Open(path)) {
            return nullptr;
        }
    }
    return file->second.get();
}

// ---------------------------------------------------------------------------------------------------------------------

ParserMsgHelper::ParserMsgHelper()
{
}
ParserMsgHelper::~ParserMsgHelper()
{
}

void ParserMsgHelper::UpdateParserMsg(const common::fpl::StreamMsg& streammsg)
{
    parser_.Reset();
    if (!parser_.Add(streammsg.msg_data_) || !parser_.Process(msg_) ||
        (msg_.data_.size() != streammsg.msg_data_.size())) {
        msg_.proto_ = Protocol::OTHER;
        msg_.name_ = ProtocolStr(Protocol::OTHER);
        msg_.data_ = streammsg.msg_data_;
    }
    msg_.info_.clear();
    auto seq = seq_.find(streammsg.stream_name_);
    if (seq == seq_.end()) {
        seq = seq_.emplace(streammsg.stream_name_, 1).first;
    }
    msg_.seq_ = seq->second++;

#if defined(FPSDK_USE_ROS1)
    rosmsg_.stamp = { streammsg.rec_time_.sec_, streammsg.rec_time_.nsec_ };
    rosmsg_.name = msg_.name_;
    rosmsg_.seq++;
    rosmsg_.data = msg_.data_;
    switch (msg_.proto_) {  // clang-format off
        case Protocol::FP_A:   rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_FP_A;   break;
        case Protocol::FP_B:   rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_FP_B;   break;
        case Protocol::NMEA:   rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_NMEA;   break;
        case Protocol::UBX:    rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_UBX;    break;
        case Protocol::RTCM3:  rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_RTCM3;  break;
        case Protocol::UNI_B:  rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_UNI_B;  break;
        case Protocol::NOV_B:  rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_NOV_B;  break;
        case Protocol::SPARTN: rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_SPARTN; break;
        case Protocol::SBF:    rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_SBF;    break;
        case Protocol::QGC:    rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_QGC;    break;
        case Protocol::OTHER:
        default:               rosmsg_.protocol = fpsdk_ros1::ParserMsg::PROTOCOL_OTHER;  break;
    }  // clang-format on
#elif defined(FPSDK_USE_ROS2)
    // @todo implement for ROS2
#endif
}

const ParserMsg& ParserMsgHelper::GetParserMsg(const bool make_info) const
{
    if (make_info) {
        msg_.MakeInfo();
    }
    return msg_;
}

#if defined(FPSDK_USE_ROS1)
const fpsdk_ros1::ParserMsg& ParserMsgHelper::GetRosMsg()
{
    msg_.MakeInfo();
    rosmsg_.info = msg_.info_;
    return rosmsg_;
}
#elif defined(FPSDK_USE_ROS2)
// @todo implement for ROS2
#endif

// ---------------------------------------------------------------------------------------------------------------------

RosMsgHelper::RosMsgHelper()
{
}

RosMsgHelper::~RosMsgHelper()
{
}

void RosMsgHelper::AddDef(const RosMsgDef& rosmsgdef)
{
    if (rosmsgdef.valid_ && (defs_.find(rosmsgdef.topic_name_) == defs_.end())) {
        DEBUG("RosMsgHelper: %s", rosmsgdef.info_.c_str());
        defs_.emplace(rosmsgdef.topic_name_, rosmsgdef);
    }
}

bool RosMsgHelper::ToJson(const RosMsgBin& rosmsgbin, nlohmann::json& jdata)
{
    if (!rosmsgbin.valid_) {
        return false;
    }

    const auto& entry = defs_.find(rosmsgbin.topic_name_);
    if (entry == defs_.end()) {
        WARNING_THR(1000, "Missing ROSMSGDEF for ROSMSGBIN %s", rosmsgbin.info_.c_str());
        return false;
    }

    const auto& rosmsgdef = entry->second;
    TRACE("ROSMSGBIN %s using ROSMSGDEF %s", rosmsgbin.info_.c_str(), rosmsgdef.info_.c_str());

    // Try the implemented conversions
    bool ok = false;
    try {
        if (RosMsgToJson<sensor_msgs::Imu>(rosmsgdef, rosmsgbin, jdata) ||
            RosMsgToJson<sensor_msgs::Temperature>(rosmsgdef, rosmsgbin, jdata) ||
            RosMsgToJson<sensor_msgs::Image>(rosmsgdef, rosmsgbin, jdata) ||
            RosMsgToJson<nav_msgs::Odometry>(rosmsgdef, rosmsgbin, jdata) ||
            RosMsgToJson<tf2_msgs::TFMessage>(rosmsgdef, rosmsgbin, jdata)) {
            jdata["_type"] = FplTypeStr(FplType::ROSMSGBIN);
            jdata["_msg"] = rosmsgdef.msg_name_;
            jdata["_topic"] = rosmsgbin.topic_name_;
            jdata["_stamp"] = rosmsgbin.rec_time_;
            ok = true;
        } else {
            throw std::runtime_error("conversion not implemented");
        }
    } catch (std::exception& ex) {
        WARNING(
            "ROSMSGBIN %s ROSMSGDEF %s ToJson fail: %s", rosmsgbin.info_.c_str(), rosmsgdef.info_.c_str(), ex.what());
    }
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

std::string FileDumpOutName(const common::fpl::FileDump& filedump)
{
    const auto parts = StrSplit(filedump.filename_, ".", 2);
    std::string outname = parts[0];
    const auto m = filedump.mtime_.GetUtcTime(0);
    outname += Sprintf("_%04d%02d%02d-%02d%02d%02.0f", m.year_, m.month_, m.day_, m.hour_, m.min_, m.sec_);
    if (parts.size() > 1) {
        outname += "." + parts[1];
    }
    return outname;
}

// ---------------------------------------------------------------------------------------------------------------------

const std::string& FixTopicName(const std::string& in_topic)
{
    if (in_topic == "/fusion_optim/imu_biases") {
        static std::string str = "/imu/biases";
        return str;
    } else {
        return in_topic;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::string OutputSizeStr(const std::string& path)
{
    const double size = (double)(PathIsDirectory(path) ? DirSize(path) : FileSize(path));
    if (size < 1024.0) {
        return Sprintf("%.0f B", size);
    } else if (size < (1024.0 * 1024.0)) {
        return Sprintf("%.1f KiB", size / 1024.0);
    } else {
        return Sprintf("%.1f MiB", size / 1024.0 / 1024.0);
    }
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
