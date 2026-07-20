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
#include <map>
#include <memory>
#include <set>
#include <string>

/* EXTERNAL */
#include <nlohmann/json.hpp>
#if defined(FPSDK_USE_ROS1)
#  include <fpsdk_ros1/ext/ros_msgs.hpp>
#elif defined(FPSDK_USE_ROS2)
#  include <fpsdk_ros2/ext/msgs.hpp>
#  include <fpsdk_ros2/ext/rclcpp.hpp>
#endif

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/cam.hpp>
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/parser/fpb.hpp>
#include <fpsdk_common/parser/nmea.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/ros1.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>
#include <fpsdk_common/to_json/fpl.hpp>
#include <fpsdk_common/to_json/fpl_ros1.hpp>
#include <fpsdk_common/to_json/nmea.hpp>
#include <fpsdk_common/to_json/parser.hpp>
#include <fpsdk_common/to_json/parser_fpa.hpp>
#include <fpsdk_common/to_json/parser_fpb.hpp>
#include <fpsdk_common/to_json/ros1.hpp>
#include <fpsdk_common/to_json/time.hpp>
#include <fpsdk_common/types.hpp>

/* PACKAGE */
#include "fpltool_extract.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::cam;
using namespace fpsdk::common::fpl;
using namespace fpsdk::common::parser;
using namespace fpsdk::common::parser::fpa;
using namespace fpsdk::common::parser::fpb;
using namespace fpsdk::common::parser::nmea;
using namespace fpsdk::common::path;
using namespace fpsdk::common::ros1;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;
using namespace fpsdk::common::types;
using namespace fpsdk::common::video;
#ifdef FPSDK_USE_ROS1
using namespace fpsdk::ros1::bagwriter;
#endif
#ifdef FPSDK_USE_ROS2
using namespace fpsdk::ros2::bagwriter;
#endif

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::FplToolExtract(FplToolOptions& opts) /* clang-format off */ :
    opts_   { opts }  // clang-format on
{
}

bool FplToolExtract::Run()
{
    // Check options, determine output file names
    if (opts_.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }

    const std::string input_fpl = opts_.inputs_[0];
    output_prefix_ = opts_.GetOutputPrefix(input_fpl);
    jsonl_name_ = (opts_.compress_ > 0 ? "all.jsonl.gz" : "all.jsonl");
    raw_ext_ = (opts_.compress_ > 0 ? ".raw.gz" : ".raw");

    // Open input log
    FplFileReader fpl_reader;
    if (!fpl_reader.Open(input_fpl)) {
        return false;
    }

    // Check which output formats we want
    do_jsonl_ = opts_.formats_.empty();
    do_raw_ = opts_.formats_.empty();
    do_file_ = opts_.formats_.empty();
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    do_ros_ = opts_.formats_.empty();
#else
    do_ros_ = false;
#endif
    do_cam_ = opts_.formats_.empty();
    for (auto& fmt : opts_.formats_) {  // clang-format off
        if      (fmt == opts_.FORMAT_JSONL) { do_jsonl_ = true; }
        else if (fmt == opts_.FORMAT_RAW)   { do_raw_ = true; }
        else if (fmt == opts_.FORMAT_FILE)  { do_file_ = true; }
        else if (fmt == opts_.FORMAT_CAM)   { do_cam_ = true; }
        else if (fmt == opts_.FORMAT_ROS) {  // clang-format on
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
            do_ros_ = true;
#else
            WARNING("Cannot extract to ROS bag. This fpltool is not built with ROS support.");
            return false;
#endif
        } else {
            WARNING("Bad argument '%s' to option -e, --formats", fmt.c_str());
            return false;
        }
    }
    if (!do_jsonl_ && !do_raw_ && !do_file_ && !do_ros_ && !do_cam_) {
        WARNING("No output formats selected");
        return false;
    }

    NOTICE("Extracting from %s to %s_...", input_fpl.c_str(), output_prefix_.c_str());
    TicToc tt;

#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
#  ifdef FPSDK_USE_ROS1
    const auto output_bag = output_prefix_ + ".bag";  // File
#  else
    const auto output_bag = output_prefix_ + "_bag";  // Directory! (even for single-file .mcap)
#  endif
    if (PathExists(output_bag)) {
        if (!opts_.overwrite_) {
            WARNING("Output bag %s already exists", output_bag.c_str());
            return false;
        } else {
            RemoveAll(output_bag);
        }
    }
    if (do_ros_ && !bag_.Open(output_bag, opts_.compress_)) {
        return false;
    }

    NOTICE("Extracting to %s", output_bag.c_str());
#endif

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process log
    double progress = 0.0;
    double rate = 0.0;
    bool ok = true;
    FplMessage fpl_msg;
    bool do_extract = (opts_.skip_ == 0);
    uint32_t time_into_log = 0;
    std::size_t errors = 0;
    while (!sig_int.ShouldAbort() && fpl_reader.Next(fpl_msg) && ok) {
        // Report progress
        if (opts_.progress_ > 0) {
            if (fpl_reader.GetProgress(progress, rate)) {
                INFO("Extracting... %.1f%% (%.0f MiB/s)\r", progress, rate);
            }
        }

        do_extract = (time_into_log >= opts_.skip_);
        if ((opts_.duration_ > 0) && (time_into_log > (opts_.skip_ + opts_.duration_))) {
            DEBUG("abort early");
            break;
        }

        // Process message
        const auto log_type = fpl_msg.PayloadType();
        ProcRes res = ProcRes::OK;
        switch (log_type) {  // clang-format off
            case FplType::LOGSTATUS: res = ProcessLogStatus(fpl_msg, do_extract, time_into_log); break;
            case FplType::LOGMETA:   res = ProcessLogMeta(fpl_msg, do_extract);                  break;
            case FplType::STREAMMSG: res = ProcessStreamMsg(fpl_msg, do_extract);                break;
            case FplType::ROSMSGDEF: res = ProcessRosMsgDef(fpl_msg, do_extract);                break;
            case FplType::ROSMSGBIN: res = ProcessRosMsgBin(fpl_msg, do_extract);                break;
            case FplType::FILEDUMP:  res = ProcessFileDump(fpl_msg, do_extract);                 break;
            case FplType::CAMDATA:   res = ProcessCamData(fpl_msg, do_extract);                  break;
            case FplType::BLOB:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X:
            case FplType::UNSPECIFIED: break;
        }  // clang-format on

        switch (res) {
            case ProcRes::OK:
                break;
            case ProcRes::ERROR:
                if (errors >= 100) {
                    WARNING("Too many errors, giving up");
                    ok = false;
                }
                break;
            case ProcRes::FATAL:
                WARNING("Giving up");
                ok = false;
                break;
        }
    }

    // We were interrupted
    if (sig_int.ShouldAbort()) {
        ok = false;
    }

    // Close output files
    CloseAll(ok);
#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    if (do_ros_) {
        bag_.Close();
        if (ok) {
            INFO("Wrote bag %s (%s)", output_bag.c_str(), OutputSizeStr(output_bag).c_str());
        } else {
            WARNING("Incomplete bag %s (%s)", output_bag.c_str(), OutputSizeStr(output_bag).c_str());
        }
    }
#endif

    const auto dur_wall = tt.Toc().GetSec();
    const double dur_log = (double)time_into_log - (double)opts_.skip_;
    if ((dur_log > 0.0) && (dur_wall > 0.0)) {
        NOTICE("Processed %.0fs of data in %.0fs (%.1fx)",  // clang-format off
            dur_log, dur_wall, dur_log / dur_wall);  // clang-format on
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessLogStatus(
    const common::fpl::FplMessage& fpl_msg, const bool do_extract, uint32_t& time_into_log)
{
    const LogStatus logstatus(fpl_msg);
    if (!logstatus.valid_) {
        WARNING("Invalid LOGSTATUS");
        return ProcRes::ERROR;
    }
    TRACE("LOGSTATUS %s", logstatus.info_.c_str());

    time_into_log = logstatus.log_duration_;

    if (do_extract && do_jsonl_ && !WriteJson(jsonl_name_, logstatus)) {
        return ProcRes::FATAL;
    }

    return ProcRes::OK;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessLogMeta(const common::fpl::FplMessage& fpl_msg, const bool do_extract)
{
    const LogMeta logmeta(fpl_msg);
    if (!logmeta.valid_) {
        WARNING("Invalid LOGMETA");
        return ProcRes::ERROR;
    }
    TRACE("LOGMETA %s", logmeta.info_.c_str());

    ProcRes res = ProcRes::OK;

    // Always save first LOGMETA, even with skip
    if ((!have_logmeta_ || do_extract) && do_jsonl_ && !WriteJson(jsonl_name_, logmeta)) {
        res = ProcRes::FATAL;
    }
    have_logmeta_ = true;

    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessRosMsgDef(const FplMessage& fpl_msg, const bool do_extract)
{
    UNUSED(do_extract);

    RosMsgDef rosmsgdef(fpl_msg);
    if (!rosmsgdef.valid_) {
        WARNING("Invalid ROSMSGDEF");
        return ProcRes::ERROR;
    }
    TRACE("ROSMSGDEF %s", rosmsgdef.info_.c_str());

    rosmsgdef.topic_name_ = FixTopicName(rosmsgdef.topic_name_);

    if (do_jsonl_ || do_ros_) {
        if ((rosmsgdefs_.find(rosmsgdef.topic_name_) == rosmsgdefs_.end())) {
            rosmsgdefs_.emplace(rosmsgdef.topic_name_, rosmsgdef);
        }
    }

#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    if (do_ros_) {
        bag_.AddMsgDef(rosmsgdef);
    }
#endif

    return ProcRes::OK;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessRosMsgBin(const FplMessage& fpl_msg, const bool do_extract)
{
    if (!do_extract) {
        return ProcRes::OK;
    }

    RosMsgBin rosmsgbin(fpl_msg);
    if (!rosmsgbin.valid_) {
        WARNING("Invalid ROSMSGBIN");
        return ProcRes::ERROR;
    }
    TRACE("ROSMSGBIN %s", rosmsgbin.info_.c_str());

    rosmsgbin.topic_name_ = FixTopicName(rosmsgbin.topic_name_);

    if (do_jsonl_) {
        const auto& entry = rosmsgdefs_.find(rosmsgbin.topic_name_);
        if (entry == rosmsgdefs_.end()) {
            WARNING("Missing ROSMSGDEF for ROSMSGBIN %s", rosmsgbin.info_.c_str());
            return ProcRes::ERROR;
        }

        const auto& rosmsgdef = entry->second;
        TRACE("ROSMSGBIN %s using ROSMSGDEF %s", rosmsgbin.info_.c_str(), rosmsgdef.info_.c_str());

        // Try the implemented conversions
        nlohmann::json jdata;
        bool jok = false;
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
                jok = true;
            } else {
                throw std::runtime_error("conversion not implemented");
            }
        } catch (std::exception& ex) {
            WARNING("ROSMSGBIN %s ROSMSGDEF %s ToJson fail: %s", rosmsgbin.info_.c_str(), rosmsgdef.info_.c_str(),
                ex.what());
        }

        if (jok && !WriteJson(jsonl_name_, jdata)) {
            return ProcRes::FATAL;
        }
    }

#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
    if (do_ros_ && !bag_.WriteMessage(rosmsgbin)) {
        return ProcRes::FATAL;
    }
#endif

    return ProcRes::OK;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessStreamMsg(const FplMessage& fpl_msg, const bool do_extract)
{
    if (!do_extract) {
        return ProcRes::OK;
    }

    const StreamMsg streammsg(fpl_msg);
    if (!streammsg.valid_) {
        WARNING("Invalid STREAMMSG");
        return ProcRes::ERROR;
    }
    TRACE("STREAMMSG %s", streammsg.info_.c_str());

    if (do_raw_ && !WriteData(streammsg.stream_name_ + raw_ext_, streammsg.msg_data_)) {
        return ProcRes::FATAL;
    }

    if (do_jsonl_ || do_ros_) {
        parser_.Reset();
        ParserMsg msg;
        if (!parser_.Add(streammsg.msg_data_) || !parser_.Process(msg) ||
            (msg.data_.size() != streammsg.msg_data_.size())) {
            msg.proto_ = Protocol::OTHER;
            msg.name_ = ProtocolStr(Protocol::OTHER);
            msg.data_ = streammsg.msg_data_;
        }
        msg.info_.clear();
        auto seq = stream_seq_.find(streammsg.stream_name_);
        if (seq == stream_seq_.end()) {
            seq = stream_seq_.emplace(streammsg.stream_name_, 1).first;
        }
        msg.seq_ = seq->second++;

        if (do_jsonl_ && !WriteStreamMsg(jsonl_name_, streammsg, msg)) {
            return ProcRes::FATAL;
        }

#if defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2)
        if (do_ros_) {
#  if defined(FPSDK_USE_ROS1)
            std_msgs::ByteMultiArray rosmsg;
            ros::Time stamp;
#  else
            std_msgs::msg::ByteMultiArray rosmsg;
            rclcpp::Time stamp;
#  endif
            rosmsg.layout.dim.resize(1);
            rosmsg.layout.dim[0].label = msg.name_;
            rosmsg.layout.dim[0].size = msg.data_.size();
            rosmsg.layout.dim[0].stride = msg.data_.size();
            rosmsg.data = { msg.data_.data(), msg.data_.data() + msg.data_.size() };
#  if defined(FPSDK_USE_ROS1)
            stamp = { streammsg.rec_time_.sec_, streammsg.rec_time_.nsec_ };
#  else
            stamp = { (int)streammsg.rec_time_.sec_, streammsg.rec_time_.nsec_, RCL_ROS_TIME };
#  endif
            if (!bag_.WriteMessage(rosmsg, "/" + streammsg.stream_name_ + "/raw", stamp)) {
                return ProcRes::FATAL;
            }
        }
#endif
    }

    return ProcRes::OK;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessFileDump(const FplMessage& fpl_msg, const bool do_extract)
{
    FileDump filedump(fpl_msg);
    if (!filedump.valid_) {
        WARNING("Invalid FILEDUMP");
        return ProcRes::ERROR;
    }
    TRACE("FILEDUMP %s", filedump.info_.c_str());

    const bool dumped_before = files_dumped_.count(filedump.filename_) == 0;

    if ((!dumped_before || do_extract) && do_file_ &&
        !WriteData(FileDumpOutName(filedump) + (opts_.compress_ > 0 ? ".gz" : ""), filedump.data_)) {
        return ProcRes::FATAL;
    }
    if ((!dumped_before || do_extract) && do_jsonl_ && !WriteJson(jsonl_name_, filedump)) {
        return ProcRes::FATAL;
    }

    if (!dumped_before) {
        files_dumped_.emplace(filedump.filename_);
    }

    return ProcRes::OK;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessCamData(const FplMessage& fpl_msg, const bool do_extract)
{
    if (!do_extract) {
        return ProcRes::OK;
    }

    CamData camdata(fpl_msg);
    if (!camdata.valid_) {
        WARNING("Invalid CAMDATA");
        return ProcRes::ERROR;
    }
    TRACE("CAMDATA %s", camdata.info_.c_str());

    const std::string name = Sprintf(
        "%s-%s-%s", CamIdToStr(camdata.cam_id_), CamDataTypeToStr(camdata.type_), CamDataFmtToStr(camdata.fmt_));

    if (do_raw_ && do_cam_ && !WriteData(name + raw_ext_, camdata.data_)) {
        return ProcRes::FATAL;
    }

    if (do_jsonl_ && do_cam_ && !WriteJson(jsonl_name_, camdata)) {
        return ProcRes::FATAL;
    }

    ProcRes res = ProcRes::OK;

#if FPSDK_USE_FFMPEG && (defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2))
    VideoCodec codec = VideoCodec::UNSPECIFIED;
    switch (camdata.fmt_) {  // clang-format off
        case CamDataFmt::H264_NAL: codec = VideoCodec::H264; break;
        case CamDataFmt::H265_NAL: codec = VideoCodec::H265; break;
        case CamDataFmt::UNSPECIFIED:
        case CamDataFmt::MJPEG:
        case CamDataFmt::JPEG:
        case CamDataFmt::Y8:
        case CamDataFmt::NV12:
        case CamDataFmt::RGB24: break;
    }  // clang-format on
    while (do_cam_ && do_ros_ && (codec != VideoCodec::UNSPECIFIED)) {
        auto entry = decoder_states_.find(name);
        if (entry == decoder_states_.end()) {
            // We can only start decoding from the first I-frame onwards
            if (camdata.frm_ != CamDataFrm::I_FRAME) {
                break;
            }
            AsyncDecState state;
            state.dec_ = CreateVideoFrameDecoder({ name, codec, opts_.pixelfmt_, opts_.scale_ });
            entry = decoder_states_.emplace(name, std::move(state)).first;
        }

        if (!entry->second.dec_) {
            WARNING("No decoder for %s", name.c_str());
            return ProcRes::FATAL;
        }
        auto& dec = entry->second.dec_;
        auto& fut = entry->second.fut_;

        // Wait for decoding results from previous frame
        if (fut.valid()) {
            res = ProcessAsyncDecData(fut.get());
        }

        // Decode this frame
        fut = std::async(std::launch::async,
            [&dec, /*copy!*/ camdata]() -> AsyncDecData { return { camdata, dec->DecodeFrame(camdata.data_) }; });

        break;
    }
#endif

    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

#if FPSDK_USE_FFMPEG && (defined(FPSDK_USE_ROS1) || defined(FPSDK_USE_ROS2))
FplToolExtract::ProcRes FplToolExtract::ProcessAsyncDecData(const FplToolExtract::AsyncDecData& decdata)
{
    auto& camdata = decdata.camdata_;
    if (!decdata.img_) {
        WARNING("No image from CAMDATA %s", camdata.info_.c_str());
        return ProcRes::ERROR;
    }
    auto& img = *decdata.img_;
    TRACE("CAMDATA decoded %s -> %dx%d %s", camdata.info_.c_str(), img.width_, img.height_, PixelFmtToStr(img.fmt_));

#  if defined(FPSDK_USE_ROS1)
    sensor_msgs::Image rosmsg;
    rosmsg.header.stamp.fromNSec(camdata.meta_.ts_);
    rosmsg.header.seq = camdata.meta_.seq_;
#  else
    sensor_msgs::msg::Image rosmsg;
    rosmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(camdata.meta_.ts_), RCL_ROS_TIME);
#  endif
    rosmsg.header.frame_id = CamIdToStr(camdata.cam_id_);
    rosmsg.width = img.width_;
    rosmsg.height = img.height_;

    switch (img.fmt_) {  // clang-format off
        case PixelFmt::Y8:      rosmsg.encoding = "mono8"; rosmsg.step = img.width_;     break;
        case PixelFmt::RGB24:   rosmsg.encoding = "rgb8";  rosmsg.step = img.width_ * 3; break;
        case PixelFmt::GBRP:    rosmsg.encoding = "8UC3";  rosmsg.step = img.width_;     break;
        case PixelFmt::UNSPECIFIED: break;
    }  // clang-format on
    rosmsg.data = { img.data_.data(), img.data_.data() + img.data_.size() };
    // Abuse pixel (0, 0) to store the exposure duration in [0.1ms]
    if (!rosmsg.data.empty()) {
        const uint32_t dt01ms = camdata.meta_.dt_ / 100000;  // [ns] -> [0.1ms]
        rosmsg.data[0] = std::clamp<uint32_t>(dt01ms, 0, 255);
    }

#  if defined(FPSDK_USE_ROS1)
    ros::Time stamp;
    stamp.fromNSec(camdata.meta_.rec_time_);
#  else
    rclcpp::Time stamp(static_cast<int64_t>(camdata.meta_.rec_time_), RCL_ROS_TIME);
#  endif
    if (!bag_.WriteMessage(rosmsg, Sprintf("/%s/image", CamIdToStr(camdata.cam_id_)), stamp)) {
        return ProcRes::FATAL;
    }

    return ProcRes::OK;
}
#endif

// ---------------------------------------------------------------------------------------------------------------------

bool FplToolExtract::WriteData(const std::string& name, const std::vector<uint8_t>& data)
{
    auto f = GetOutputFile(name);
    return f ? f->Write(data) : false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplToolExtract::WriteJson(const std::string& name, const nlohmann::json& json)
{
    auto f = GetOutputFile(name);
    if (!f) {
        return false;
    }
    return f->Write(json.dump()) && f->Write("\n");
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplToolExtract::WriteStreamMsg(const std::string& name, const StreamMsg& streammsg, const ParserMsg& parsermsg)
{
    parsermsg.MakeInfo();
    nlohmann::json jdata = streammsg;  // magic to_json() (_type, _stamp, _stream, _data/_data_b64)
    jdata.update(parsermsg);           // magic to_json() (_proto, _name, _seq, _info, _data/_data_b64)

    // We can decode some messages of some protocols
    if (parsermsg.proto_ == Protocol::FP_A) {
        jdata.update(FpaDecodeMessage(parsermsg.data_));  // magic to_json()
    } else if (parsermsg.proto_ == Protocol::NMEA) {
        jdata.update(NmeaDecodeMessage(parsermsg.data_));  // magic to_json()
    } else if (parsermsg.proto_ == Protocol::FP_B) {
        jdata.update(to_json_FP_B(parsermsg));  // magic to_json()
    }
    return WriteJson(name, jdata);
}

// ---------------------------------------------------------------------------------------------------------------------

void FplToolExtract::CloseAll(const bool ok)
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

// ---------------------------------------------------------------------------------------------------------------------

OutputFile* FplToolExtract::GetOutputFile(const std::string& name)
{
    auto file = files_.find(name);
    if (file == files_.end()) {
        const std::string path = output_prefix_ + "_" + name;
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

const std::string& FplToolExtract::FixTopicName(const std::string& in_topic) const
{
    if (in_topic == "/fusion_optim/imu_biases") {
        static std::string str = "/imu/biases";
        return str;
    } else {
        return in_topic;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::string FplToolExtract::FileDumpOutName(const FileDump& filedump) const
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

std::string FplToolExtract::OutputSizeStr(const std::string& path) const
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
