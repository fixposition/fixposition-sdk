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
 * @brief Fixposition SDK: fpltool utils
 */
#ifndef __FPSDK_APPS_FPLTOOL_FPLTOOL_UTILS_HPP__
#define __FPSDK_APPS_FPLTOOL_FPLTOOL_UTILS_HPP__

/* LIBC/STL */
#include <map>
#include <memory>
#include <string>

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* Fixposition SDK */
#include <fpsdk_common/fpl.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/time.hpp>
#if defined(FPSDK_USE_ROS1)
#  include <fpsdk_ros1/msgs.hpp>
#endif

/* PACKAGE */
#include "fpltool_opts.hpp"

namespace fpsdk {
namespace apps {
namespace fpltool {
/* ****************************************************************************************************************** */

class OutputFileHelper
{
   public:
    OutputFileHelper(const std::string& prefix, const FplToolOptions& opts);
    ~OutputFileHelper();

    bool WriteData(const std::string& name, const std::vector<uint8_t>& data);
    bool WriteJson(const std::string& name, const nlohmann::json& json);
    bool WriteStreamMsg(
        const std::string& name, const common::fpl::StreamMsg& streammsg, const common::parser::ParserMsg& parsermsg);
    void CloseAll(const bool ok = false);

   private:
    std::string prefix_;
    FplToolOptions opts_;
    std::map<std::string, std::unique_ptr<common::path::OutputFile>> files_;
    common::parser::Parser parser_;
    common::parser::ParserMsg msg_;
    common::path::OutputFile* GetOutputFile(const std::string& name);
};

class ParserMsgHelper
{
   public:
    ParserMsgHelper();
    ~ParserMsgHelper();
    void UpdateParserMsg(const common::fpl::StreamMsg& streammsg);
    const common::parser::ParserMsg& GetParserMsg(const bool make_info = false) const;
#if defined(FPSDK_USE_ROS1)  // || defined(FPSDK_USE_ROS2)  // @todo implement for ROS2
    const fpsdk_ros1::ParserMsg& GetRosMsg();
#endif

   private:
    common::parser::Parser parser_;
    common::parser::ParserMsg msg_;
    std::map<std::string, uint64_t> seq_;
#if defined(FPSDK_USE_ROS1)  // || defined(FPSDK_USE_ROS2)  // @todo implement for ROS2
    fpsdk_ros1::ParserMsg rosmsg_;
#endif
};

class RosMsgHelper
{
   public:
    RosMsgHelper();
    ~RosMsgHelper();
    void AddDef(const common::fpl::RosMsgDef& rosmsgdef);
    bool ToJson(const common::fpl::RosMsgBin& rosmsgbin, nlohmann::json& jdata);

   private:
    std::map<std::string, common::fpl::RosMsgDef> defs_;
};

std::string FileDumpOutName(const common::fpl::FileDump& filedump);

const std::string& FixTopicName(const std::string& in_topic);

std::string OutputSizeStr(const std::string& path);

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_FPLTOOL_FPLTOOL_UTILS_HPP__
