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
#include <future>
#include <map>
#include <set>
#include <string>
#include <vector>

/* EXTERNAL */
#include <nlohmann/json.hpp>
#if defined(FPSDK_USE_ROS1)
#  include <fpsdk_ros1/bagwriter.hpp>
#elif defined(FPSDK_USE_ROS2)
#  include <fpsdk_ros2/bagwriter.hpp>
#endif

/* Fixposition SDK */
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/path.hpp>
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
    ProcRes ProcessLogStatus(const common::fpl::FplMessage& fpl_msg, const bool do_extract, uint32_t& time_into_log);
    ProcRes ProcessLogMeta(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessRosMsgDef(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessRosMsgBin(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessStreamMsg(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessFileDump(const common::fpl::FplMessage& fpl_msg, const bool do_extract);
    ProcRes ProcessCamData(const common::fpl::FplMessage& fpl_msg, const bool do_extract);

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
    };
    struct AsyncDecState
    {
        common::video::VideoFrameDecoderPtr dec_;
        std::future<AsyncDecData> fut_;
    };
    std::map<std::string, AsyncDecState> decoder_states_;

    ProcRes ProcessAsyncDecData(const AsyncDecData& decdata);
#endif

    // Output files
#if defined(FPSDK_USE_ROS1)
    ros1::bagwriter::BagWriter bag_;
#elif defined(FPSDK_USE_ROS2)
    ros2::bagwriter::BagWriter bag_;
#endif
    std::map<std::string, std::unique_ptr<common::path::OutputFile>> files_;
    common::path::OutputFile* GetOutputFile(const std::string& name);
    bool WriteData(const std::string& name, const std::vector<uint8_t>& data);
    bool WriteJson(const std::string& name, const nlohmann::json& json);
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
