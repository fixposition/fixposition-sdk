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
#ifndef __FPSDK_APPS_FPLTOOL_FPLTOOL_EXTRACT_HPP__
#define __FPSDK_APPS_FPLTOOL_FPLTOOL_EXTRACT_HPP__

/* LIBC/STL */
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* Fixposition SDK */
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/ros1.hpp>
#include <fpsdk_common/ros2.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_common/video.hpp>

/* PACKAGE */
#include "fpltool_opts.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

class FplToolExtract
{
   public:
    FplToolExtract(FplToolOptions& opts);
    bool Run();

   private:
    // Params
    FplToolOptions opts_;
    std::string output_prefix_;
    const char* jsonl_name_ = nullptr;
    const char* raw_ext_ = nullptr;
    bool do_jsonl_ = false;
    bool do_raw_ = false;
    bool do_file_ = false;
    bool do_ros_ = false;
    bool do_cam_ = false;

    // FPL
    // clang-format off
    enum class ProcRes { OK, ERROR, FATAL }; // clang-format off
    ProcRes ProcessLogStatus(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessLogMeta(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessRosMsgDef(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessRosMsgBin(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessStreamMsg(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessFileDump(const common::fpl::FplMessage& fpl_msg, const bool do_extract);

    // LOGMETA
    bool have_logmeta_ = false;

    // STREAMMSG
    common::parser::Parser parser_;
    std::map<std::string, uint64_t> stream_seq_;

    // ROSMSGDEF, ROSMSGBIN
    std::map<std::string, common::fpl::RosMsgDef> rosmsgdefs_;
    const std::string& FixTopicName(const std::string& in_topic) const;

    // FILEDUMP
    std::set<std::string> files_dumped_;
    std::string FileDumpOutName(const common::fpl::FileDump& filedump) const;

    // CAMDATA
#if FPSDK_USE_FFMPEG
    struct AsyncDecData
    {
        common::fpl::CamData camdata_;
        std::optional<common::video::ImageData> img_;
        bool dec_is_okay_ = true;
        std::string json_;
    };

    // Decoding of one group of pictures (GOP, one I-frame and the P-frames that follow it), using its own decoder
    // in its own thread. The main thread adds the frames as it reads them from the log and collects the decoded
    // frames in the same order.
    class GopJob : private common::types::NoCopyNoMove
    {
       public:
        GopJob(const common::video::VideoDecoderParams& params, const bool do_json);
        ~GopJob();
        std::size_t AddFrame(common::fpl::CamData&& camdata);  // Add frame to decode, returns the frame index
        void Close();                                          // No more frames will be added (end of GOP)
        void Abort();                                          // Stop decoding as soon as possible
        bool IsDecoded(const std::size_t ix);                  // Check if frame is decoded
        AsyncDecData GetDecoded(const std::size_t ix);         // Get decoded frame, waits for it if necessary
        std::size_t num_frames_ = 0;                           // Number of frames added (main thread only)
        std::size_t num_done_ = 0;                             // Number of frames collected (main thread only)
        bool closed_ = false;                                  // No more frames will be added (main thread only)

       private:
        void Run(const common::video::VideoDecoderParams& params);
        std::mutex mutex_;
        std::condition_variable cond_;
        std::deque<common::fpl::CamData> todo_;
        std::vector<AsyncDecData> done_;
        bool finish_ = false;
        bool abort_ = false;
        const bool do_json_;
        std::future<void> fut_;  // Must be last, so that the thread is gone before the other members are
    };
    std::map<std::string, std::shared_ptr<GopJob>> gop_jobs_;  // The GOP we're currently reading, per stream
    std::size_t gops_pending_ = 0;                             // Number of complete GOPs not fully written yet
    std::size_t max_gops_pending_ = 1;

    bool QueueCamData(const common::fpl::FplMessage& fpl_msg, std::shared_ptr<GopJob>& job, std::size_t& job_ix);
    ProcRes ProcessAsyncDecData(AsyncDecData&& decdata);
#endif

    // Messages are queued and processed in the order they are in the log. Camera frames that need decoding are held
    // back, along with everything that follows them, until they are decoded.
    struct QueueItem
    {
        common::fpl::FplMessage fpl_msg_;
        bool do_extract_ = true;
#if FPSDK_USE_FFMPEG
        std::shared_ptr<GopJob> job_;
        std::size_t job_ix_ = 0;
#endif
    };
    std::deque<QueueItem> queue_;
    enum class QueueWait { NONE, HEAD, ALL };
    bool ProcessQueue(const QueueWait wait);
    ProcRes ProcessCamData(const QueueItem& item);
    void AbortQueue();
    std::size_t errors_ = 0;

    // Output files
#if FPSDK_USE_ROS2
    common::ros2::BagWriter bag_;
#else
    common::ros1::BagWriter bag_;
#endif
    std::map<std::string, std::unique_ptr<common::path::OutputFile>> files_;
    common::path::OutputFile* GetOutputFile(const std::string& name);
    bool WriteData(const std::string& name, const std::vector<uint8_t>& data);
    bool WriteJson(const std::string& name, const nlohmann::json& json);
    bool WriteJson(const std::string& name, const std::string& json);
    bool WriteStreamMsg(
        const std::string& name, const common::fpl::StreamMsg& streammsg, const common::parser::ParserMsg& parsermsg);
    void CloseAll(const bool ok = false);

    std::string OutputSizeStr(const std::string& path) const;
};

/**
 * @brief Run FpltoolArgs::Command::EXTRACT
 *
 * @param[in]  opts  Options
 */
bool DoExtract(const FplToolOptions& opts);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_EXTRACT_HPP__
