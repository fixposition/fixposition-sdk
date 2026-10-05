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
#include <fpsdk_common/ros2.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>
#include <fpsdk_common/to_json/fpl.hpp>
#include <fpsdk_common/to_json/fpl_ros1.hpp>
#include <fpsdk_common/to_json/parser.hpp>
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
using namespace fpsdk::common::path;
using namespace fpsdk::common::ros1;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;
using namespace fpsdk::common::types;
using namespace fpsdk::common::video;
#if FPSDK_USE_ROS2
using namespace fpsdk::common::ros2;
#endif

// Maximum number of frames in a group of pictures (GOP), abort extraction if larger
static constexpr std::size_t MAX_GOP_SIZE = 100;

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

    // Check which output formats we want. Keep defaults in sync with help screen!
    do_jsonl_ = opts_.formats_.empty();
    do_raw_ = opts_.formats_.empty();
    do_file_ = opts_.formats_.empty();
    do_ros_ = false;
    do_cam_ = false;
    for (auto& fmt : opts_.formats_) {  // clang-format off
        if      (fmt == opts_.FORMAT_JSONL) { do_jsonl_ = true; }
        else if (fmt == opts_.FORMAT_RAW)   { do_raw_ = true; }
        else if (fmt == opts_.FORMAT_FILE)  { do_file_ = true; }
        else if (fmt == opts_.FORMAT_CAM)   { do_cam_ = true; }
        else if (fmt == opts_.FORMAT_ROS)   { do_ros_ = true; }  // clang-format on
        else {
            WARNING("Bad argument '%s' to option -e, --formats", fmt.c_str());
            return false;
        }
    }
    if (!do_jsonl_ && !do_raw_ && !do_file_ && !do_ros_ && !do_cam_) {
        WARNING("No output formats selected");
        return false;
    }

    DEBUG("do_jsonl=%s do_raw=%s do_file=%s do_ros=%s do_cam=%s", ToStr(do_jsonl_), ToStr(do_raw_), ToStr(do_file_),
        ToStr(do_ros_), ToStr(do_cam_));

    NOTICE("Extracting from %s to %s_...", input_fpl.c_str(), output_prefix_.c_str());
    TicToc tt;

    std::string output_bag;
    if (do_ros_) {
#if FPSDK_USE_ROS2
        output_bag = output_prefix_ + "_bag";  // Directory! (even for single-file .mcap)
#else
        output_bag = output_prefix_ + ".bag";  // File
#endif
        if (PathExists(output_bag)) {
            if (!opts_.overwrite_) {
                WARNING("Output bag %s already exists", output_bag.c_str());
                return false;
            } else {
                RemoveAll(output_bag);
            }
        }
#if FPSDK_USE_ROS2
        if (do_ros_ && !bag_.Open(output_bag, opts_.mcap_, opts_.compress_)) {
#else
        if (do_ros_ && !bag_.Open(output_bag, opts_.compress_)) {
#endif
            return false;
        }

        NOTICE("Extracting to %s", output_bag.c_str());
    }

#if FPSDK_USE_FFMPEG
    max_gops_pending_ = opts_.jobs_;
    DEBUG("max_gops_pending=%" PRIuMAX, max_gops_pending_);
#endif

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process log
    double progress = 0.0;
    double rate = 0.0;
    bool ok = true;
    FplMessage fpl_msg;
    bool do_extract = true;
    uint32_t time_into_log = 0;
    while (!sig_int.ShouldAbort() && fpl_reader.Next(fpl_msg) && ok) {
        // Report progress
        if (opts_.progress_ > 0) {
            if (fpl_reader.GetProgress(progress, rate)) {
                INFO("Extracting... %.1f%% (%.1f MiB/s)\r", progress, rate);
            }
        }

        do_extract = (time_into_log >= opts_.skip_);
        if ((opts_.duration_ > 0) && (time_into_log > (opts_.skip_ + opts_.duration_))) {
            DEBUG("abort early");
            break;
        }

        // Queue message for processing
        QueueItem item;
        item.do_extract_ = do_extract;
        switch (fpl_msg.PayloadType()) {
            case FplType::LOGSTATUS: {
                const LogStatus logstatus(fpl_msg);
                if (logstatus.valid_) {
                    time_into_log = logstatus.log_duration_;
                }
                break;
            }
            case FplType::CAMDATA:
#if FPSDK_USE_FFMPEG
                if (do_extract && do_cam_ && do_ros_) {
                    ok = QueueCamData(fpl_msg, item.job_, item.job_ix_);
                }
#endif
                break;
            case FplType::LOGMETA:
            case FplType::STREAMMSG:
            case FplType::ROSMSGDEF:
            case FplType::ROSMSGBIN:
            case FplType::FILEDUMP:
                break;
            case FplType::BLOB:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X:
            case FplType::UNSPECIFIED:
                continue;
        }
        item.fpl_msg_ = std::move(fpl_msg);
        queue_.push_back(std::move(item));

        // Process what we can
        if (ok) {
            ok = ProcessQueue(QueueWait::NONE);
        }
    }

    // We were interrupted
    if (sig_int.ShouldAbort()) {
        ok = false;
    }

    // Process what is left
#if FPSDK_USE_FFMPEG
    for (auto& entry : gop_jobs_) {
        entry.second->Close();
    }
    gop_jobs_.clear();
#endif
    if (ok) {
        ok = ProcessQueue(QueueWait::ALL);
    }
    if (!ok) {
        AbortQueue();
    }

    // Close output files
    CloseAll(ok);
    if (do_ros_) {
        bag_.Close();
        if (ok) {
            INFO("Wrote bag %s (%s)", output_bag.c_str(), OutputSizeStr(output_bag).c_str());
        } else {
            WARNING("Incomplete bag %s (%s)", output_bag.c_str(), OutputSizeStr(output_bag).c_str());
        }
    }

    const auto dur_wall = tt.Toc().GetSec();
    const double dur_log = (double)time_into_log - (double)opts_.skip_;
    if ((dur_log > 0.0) && (dur_wall > 0.0)) {
        NOTICE("Processed %.0fs of data in %.0fs (%.1fx)",  // clang-format off
            dur_log, dur_wall, dur_log / dur_wall);  // clang-format on
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplToolExtract::ProcessQueue(const QueueWait wait)
{
    bool may_wait = (wait != QueueWait::NONE);
    while (!queue_.empty()) {
        auto& item = queue_.front();

#if FPSDK_USE_FFMPEG
        // Camera frames that are not decoded yet hold back everything
        if (item.job_ && !item.job_->IsDecoded(item.job_ix_)) {
            if (!may_wait) {
                break;
            }
            may_wait = (wait == QueueWait::ALL);
        }
#endif

        const auto& fpl_msg = item.fpl_msg_;
        const bool do_extract = item.do_extract_;
        ProcRes res = ProcRes::OK;
        switch (fpl_msg.PayloadType()) {  // clang-format off
            case FplType::LOGSTATUS: res = ProcessLogStatus(fpl_msg, do_extract); break;
            case FplType::LOGMETA:   res = ProcessLogMeta(fpl_msg, do_extract);   break;
            case FplType::STREAMMSG: res = ProcessStreamMsg(fpl_msg, do_extract); break;
            case FplType::ROSMSGDEF: res = ProcessRosMsgDef(fpl_msg, do_extract); break;
            case FplType::ROSMSGBIN: res = ProcessRosMsgBin(fpl_msg, do_extract); break;
            case FplType::FILEDUMP:  res = ProcessFileDump(fpl_msg, do_extract);  break;
            case FplType::CAMDATA:   res = ProcessCamData(item);                  break;
            case FplType::BLOB:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X:
            case FplType::UNSPECIFIED: break;
        }  // clang-format on

        queue_.pop_front();

        switch (res) {
            case ProcRes::OK:
                break;
            case ProcRes::ERROR:
                errors_++;
                if (errors_ >= 100) {
                    WARNING("Too many errors, giving up");
                    return false;
                }
                break;
            case ProcRes::FATAL:
                WARNING("Giving up");
                return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void FplToolExtract::AbortQueue()
{
#if FPSDK_USE_FFMPEG
    for (auto& item : queue_) {
        if (item.job_) {
            item.job_->Abort();
        }
    }
    gops_pending_ = 0;
#endif
    queue_.clear();
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessLogStatus(const common::fpl::FplMessage& fpl_msg, const bool do_extract)
{
    const LogStatus logstatus(fpl_msg);
    if (!logstatus.valid_) {
        WARNING("Invalid LOGSTATUS");
        return ProcRes::ERROR;
    }
    TRACE("LOGSTATUS %s", logstatus.info_.c_str());

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

    if (do_ros_) {
        bag_.AddMsgDef(rosmsgdef);
    }

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

    if (do_ros_ && !bag_.WriteMessage(rosmsgbin)) {
        return ProcRes::FATAL;
    }

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

        if (do_ros_) {
#if FPSDK_USE_ROS2
            std_msgs::msg::ByteMultiArray rosmsg;
#else
            std_msgs::ByteMultiArray rosmsg;
#endif
            rosmsg.layout.dim.resize(1);
            rosmsg.layout.dim[0].label = msg.name_;
            rosmsg.layout.dim[0].size = msg.data_.size();
            rosmsg.layout.dim[0].stride = msg.data_.size();
            rosmsg.data = { msg.data_.data(), msg.data_.data() + msg.data_.size() };

            const RosTime stamp(streammsg.rec_time_.sec_, streammsg.rec_time_.nsec_);
            if (!bag_.WriteMessage(rosmsg, "/" + streammsg.stream_name_ + "/raw", stamp)) {
                return ProcRes::FATAL;
            }
        }
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

FplToolExtract::ProcRes FplToolExtract::ProcessCamData(const QueueItem& item)
{
    if (!item.do_extract_) {
        return ProcRes::OK;
    }

    fpsdk::common::fpl::CamData camdata(item.fpl_msg_);
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

#if FPSDK_USE_FFMPEG
    // Frames that were decoded (and stringified to JSON) in a GopJob
    if (item.job_) {
        auto& job = *item.job_;
        auto decdata = job.GetDecoded(item.job_ix_);
        job.num_done_++;
        if (job.closed_ && (job.num_done_ == job.num_frames_)) {
            gops_pending_--;
        }
        if (do_jsonl_ && do_cam_ && !WriteJson(jsonl_name_, decdata.json_)) {
            return ProcRes::FATAL;
        }
        return ProcessAsyncDecData(std::move(decdata));
    }
#endif

    if (do_jsonl_ && do_cam_ && !WriteJson(jsonl_name_, camdata)) {
        return ProcRes::FATAL;
    }

    return ProcRes::OK;
}

// ---------------------------------------------------------------------------------------------------------------------

#if FPSDK_USE_FFMPEG
bool FplToolExtract::QueueCamData(const FplMessage& fpl_msg, std::shared_ptr<GopJob>& job, std::size_t& job_ix)
{
    fpsdk::common::fpl::CamData camdata(fpl_msg);
    if (!camdata.valid_) {
        return true;  // ProcessCamData() will complain
    }

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
    if (codec == VideoCodec::UNSPECIFIED) {
        return true;
    }

    const std::string name = Sprintf(
        "%s-%s-%s", CamIdToStr(camdata.cam_id_), CamDataTypeToStr(camdata.type_), CamDataFmtToStr(camdata.fmt_));
    auto entry = gop_jobs_.find(name);

    // An I-frame starts a new GOP, which is decoded independently of the previous one
    if (camdata.frm_ == CamDataFrm::I_FRAME) {
        if (entry != gop_jobs_.end()) {
            auto& prev = *entry->second;
            prev.Close();
            if (prev.num_done_ < prev.num_frames_) {
                gops_pending_++;
            }
            gop_jobs_.erase(entry);
        }

        // Don't run too many decoders at the same time, and don't queue too much decoded data
        while (gops_pending_ >= max_gops_pending_) {
            if (!ProcessQueue(QueueWait::HEAD)) {
                return false;
            }
        }

        // Each decoder makes its own hw device, if hw decoding is used
        const VideoDecoderParams params = { name, codec, opts_.pixelfmt_, opts_.scale_, opts_.accel_ };
        entry = gop_jobs_.emplace(name, std::make_shared<GopJob>(params, do_jsonl_)).first;
    }
    // We can only start decoding from the first I-frame onwards
    else if (entry == gop_jobs_.end()) {
        return true;
    }

    if (entry->second->num_frames_ >= MAX_GOP_SIZE) {
        WARNING("Too many frames (> %" PRIuMAX ") without an I-frame in %s", MAX_GOP_SIZE, name.c_str());
        return false;
    }
    job = entry->second;
    job_ix = job->AddFrame(std::move(camdata));

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::GopJob::GopJob(const VideoDecoderParams& params, const bool do_json) /* clang-format off */ :
    do_json_   { do_json }  // clang-format on
{
    fut_ = std::async(std::launch::async, [this, params]() { Run(params); });
}

FplToolExtract::GopJob::~GopJob()
{
    Abort();
}

std::size_t FplToolExtract::GopJob::AddFrame(fpsdk::common::fpl::CamData&& camdata)
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        todo_.push_back(std::move(camdata));
    }
    cond_.notify_all();
    return num_frames_++;
}

void FplToolExtract::GopJob::Close()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        finish_ = true;
    }
    cond_.notify_all();
    closed_ = true;
}

void FplToolExtract::GopJob::Abort()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        finish_ = true;
        abort_ = true;
    }
    cond_.notify_all();
}

bool FplToolExtract::GopJob::IsDecoded(const std::size_t ix)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return ix < done_.size();
}

FplToolExtract::AsyncDecData FplToolExtract::GopJob::GetDecoded(const std::size_t ix)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this, ix]() { return ix < done_.size(); });
    return std::move(done_[ix]);
}

void FplToolExtract::GopJob::Run(const VideoDecoderParams& params)
{
    auto dec = CreateVideoFrameDecoder(params);
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]() { return !todo_.empty() || finish_; });
        if (todo_.empty() || abort_) {
            break;
        }
        AsyncDecData decdata = { std::move(todo_.front()), std::nullopt, false, "" };
        todo_.pop_front();
        lock.unlock();

        if (dec) {
            decdata.img_ = dec->DecodeFrame(decdata.camdata_.data_);
            decdata.dec_is_okay_ = dec->IsOkay();
        }
        if (do_json_) {
            decdata.json_ = nlohmann::json(decdata.camdata_).dump();
        }

        lock.lock();
        done_.push_back(std::move(decdata));
        lock.unlock();
        cond_.notify_all();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

FplToolExtract::ProcRes FplToolExtract::ProcessAsyncDecData(FplToolExtract::AsyncDecData&& decdata)
{
    auto& camdata = decdata.camdata_;
    if (!decdata.img_) {
        WARNING("No image from CAMDATA %s", camdata.info_.c_str());
        if (!decdata.dec_is_okay_) {
            return ProcRes::FATAL;
        } else {
            return ProcRes::ERROR;
        }
    }
    auto& img = *decdata.img_;
    TRACE("CAMDATA decoded %s -> %dx%d %s", camdata.info_.c_str(), img.width_, img.height_, PixelFmtToStr(img.fmt_));

#  if FPSDK_USE_ROS2
    sensor_msgs::msg::Image rosmsg;
    rosmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(camdata.ts_), RCL_ROS_TIME);
#  else
    sensor_msgs::Image rosmsg;
    rosmsg.header.stamp.fromNSec(camdata.ts_);
    rosmsg.header.seq = camdata.seq_;
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
    rosmsg.data = std::move(img.data_);
    // Abuse pixel (0, 0) to store the exposure duration in [0.1ms]
    if (!rosmsg.data.empty()) {
        const uint32_t dt01ms = camdata.dt_ / 100000;  // [ns] -> [0.1ms]
        rosmsg.data[0] = std::clamp<uint32_t>(dt01ms, 0, 255);
    }
    const RosTime stamp(camdata.rec_time_.sec_, camdata.rec_time_.nsec_);
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

bool FplToolExtract::WriteJson(const std::string& name, const std::string& json)
{
    auto f = GetOutputFile(name);
    if (!f) {
        return false;
    }
    return f->Write(json) && f->Write("\n");
}

// ---------------------------------------------------------------------------------------------------------------------

bool FplToolExtract::WriteStreamMsg(const std::string& name, const StreamMsg& streammsg, const ParserMsg& parsermsg)
{
    parsermsg.MakeInfo();
    auto jdata = ParserMsgToJson(parsermsg);  // magic to_json() (_proto, _name, _seq, _info, _data/_data_b64)
    jdata.update(streammsg);                  // magic to_json() (_type, _stamp, _stream, _data/_data_b64)
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
