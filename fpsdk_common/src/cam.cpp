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
 * @brief Fixposition SDK: Camera types and utilities
 */

/* LIBC/STL */
#include <cstring>

/* EXTERNAL */
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/tcp_stream.hpp>
#include <boost/beast/http.hpp>

/* PACKAGE */
#include "fpsdk_common/cam.hpp"
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/types.hpp"
#include "fpsdk_common/utils.hpp"

namespace fpsdk {
namespace common {
namespace cam {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::string;
using namespace fpsdk::common::types;
using namespace fpsdk::common::time;
using namespace fpsdk::common::utils;

const char* CamIdToStr(const CamId camid)
{
    switch (camid) {  // clang-format off
        case CamId::UNSPECIFIED: return "UNSPECIFIED";
        case CamId::CAM1:        return "CAM1";
        case CamId::CAM2:        return "CAM2";
        case CamId::CAM3:        return "CAM3";
        case CamId::CAM4:        return "CAM4";
    }  // clang-format on
    return "?";
}

CamId CamIdFromStrOr(const char* str, const CamId def)
{  // clang-format off
    if (std::strcmp(str, "UNSPECIFIED") == 0) { return CamId::UNSPECIFIED; }
    if (std::strcmp(str, "CAM1")        == 0) { return CamId::CAM1; }
    if (std::strcmp(str, "CAM2")        == 0) { return CamId::CAM2; }
    if (std::strcmp(str, "CAM3")        == 0) { return CamId::CAM3; }
    if (std::strcmp(str, "CAM4")        == 0) { return CamId::CAM4; }
    return def;  // clang-format on
}

CamId CamIdFromValOr(const uint8_t val, const CamId def)
{  // clang-format off
    switch (val) {
        case EnumToVal(CamId::UNSPECIFIED): return CamId::UNSPECIFIED;
        case EnumToVal(CamId::CAM1):        return CamId::CAM1;
        case EnumToVal(CamId::CAM2):        return CamId::CAM2;
        case EnumToVal(CamId::CAM3):        return CamId::CAM3;
        case EnumToVal(CamId::CAM4):        return CamId::CAM4;
    }
    return def;  // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

const char* CamDataTypeToStr(const CamDataType type)
{
    switch (type) {  // clang-format off
        case CamDataType::UNSPECIFIED: return "UNSPECIFIED";
        case CamDataType::HIRES_IMG:   return "HIRES_IMG";
        case CamDataType::LORES_IMG:   return "LORES_IMG";
        case CamDataType::HIRES_VID:   return "HIRES_VID";
        case CamDataType::LORES_VID:   return "LORES_VID";
    }  // clang-format on
    return "?";
}

CamDataType CamDataTypeFromStrOr(const char* str, const CamDataType def)
{  // clang-format off
    if (std::strcmp(str, "UNSPECIFIED") == 0) { return CamDataType::UNSPECIFIED; }
    if (std::strcmp(str, "HIRES_IMG")   == 0) { return CamDataType::HIRES_IMG; }
    if (std::strcmp(str, "LORES_IMG")   == 0) { return CamDataType::LORES_IMG; }
    if (std::strcmp(str, "HIRES_VID")   == 0) { return CamDataType::HIRES_VID; }
    if (std::strcmp(str, "LORES_VID")   == 0) { return CamDataType::LORES_VID; }
    return def;  // clang-format on
}

CamDataType CamDataTypeFromValOr(const uint8_t val, const CamDataType def)
{  // clang-format off
    switch (val) {
        case EnumToVal(CamDataType::UNSPECIFIED): return CamDataType::UNSPECIFIED;
        case EnumToVal(CamDataType::HIRES_IMG):   return CamDataType::HIRES_IMG;
        case EnumToVal(CamDataType::LORES_IMG):   return CamDataType::LORES_IMG;
        case EnumToVal(CamDataType::HIRES_VID):   return CamDataType::HIRES_VID;
        case EnumToVal(CamDataType::LORES_VID):   return CamDataType::LORES_VID;
    }
    return def;  // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

const char* CamDataFmtToStr(const CamDataFmt fmt)
{
    switch (fmt) {  // clang-format off
        case CamDataFmt::UNSPECIFIED: return "UNSPECIFIED";
        case CamDataFmt::H264_NAL:    return "H264_NAL";
        case CamDataFmt::H265_NAL:    return "H265_NAL";
        case CamDataFmt::MJPEG:       return "MJPEG";
        case CamDataFmt::JPEG:        return "JPEG";
        case CamDataFmt::Y8:          return "Y8";
        case CamDataFmt::NV12:        return "NV12";
        case CamDataFmt::RGB24:       return "RGB24";
    }  // clang-format on
    return "?";
}

CamDataFmt CamDataFmtFromStrOr(const char* str, const CamDataFmt def)
{  // clang-format off
    if (std::strcmp(str, "UNSPECIFIED") == 0) { return CamDataFmt::UNSPECIFIED; }
    if (std::strcmp(str, "H264_NAL")    == 0) { return CamDataFmt::H264_NAL; }
    if (std::strcmp(str, "H265_NAL")    == 0) { return CamDataFmt::H265_NAL; }
    if (std::strcmp(str, "MJPEG")       == 0) { return CamDataFmt::MJPEG; }
    if (std::strcmp(str, "JPEG")        == 0) { return CamDataFmt::JPEG; }
    if (std::strcmp(str, "Y8")          == 0) { return CamDataFmt::Y8; }
    if (std::strcmp(str, "NV12")        == 0) { return CamDataFmt::NV12; }
    if (std::strcmp(str, "RGB24")       == 0) { return CamDataFmt::RGB24; }
    return def;  // clang-format on
}

CamDataFmt CamDataFmtFromValOr(const uint8_t val, const CamDataFmt def)
{  // clang-format off
    switch (val) {
        case EnumToVal(CamDataFmt::UNSPECIFIED): return CamDataFmt::UNSPECIFIED;
        case EnumToVal(CamDataFmt::H264_NAL):    return CamDataFmt::H264_NAL;
        case EnumToVal(CamDataFmt::H265_NAL):    return CamDataFmt::H265_NAL;
        case EnumToVal(CamDataFmt::MJPEG):       return CamDataFmt::MJPEG;
        case EnumToVal(CamDataFmt::JPEG):        return CamDataFmt::JPEG;
        case EnumToVal(CamDataFmt::Y8):          return CamDataFmt::Y8;
        case EnumToVal(CamDataFmt::NV12):        return CamDataFmt::NV12;
        case EnumToVal(CamDataFmt::RGB24):       return CamDataFmt::RGB24;
    }
    return def;  // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

const char* CamDataFrmToStr(const CamDataFrm frm)
{
    switch (frm) {  // clang-format off
        case CamDataFrm::UNSPECIFIED: return "UNSPECIFIED";
        case CamDataFrm::I_FRAME:     return "I_FRAME";
        case CamDataFrm::P_FRAME:     return "P_FRAME";
        case CamDataFrm::FULL:        return "FULL";
        case CamDataFrm::OTHER:       return "OTHER";
    }  // clang-format on
    return "?";
}

CamDataFrm CamDataFrmFromStrOr(const char* str, const CamDataFrm def)
{  // clang-format off
    if (std::strcmp(str, "UNSPECIFIED") == 0) { return CamDataFrm::UNSPECIFIED; }
    if (std::strcmp(str, "I_FRAME")     == 0) { return CamDataFrm::I_FRAME; }
    if (std::strcmp(str, "P_FRAME")     == 0) { return CamDataFrm::P_FRAME; }
    if (std::strcmp(str, "FULL")        == 0) { return CamDataFrm::FULL; }
    if (std::strcmp(str, "OTHER")       == 0) { return CamDataFrm::OTHER; }
    return def;  // clang-format on
}

CamDataFrm CamDataFrmFromValOr(const uint8_t val, const CamDataFrm def)
{  // clang-format off
    switch (val) {
        case EnumToVal(CamDataFrm::UNSPECIFIED): return CamDataFrm::UNSPECIFIED;
        case EnumToVal(CamDataFrm::I_FRAME):     return CamDataFrm::I_FRAME;
        case EnumToVal(CamDataFrm::P_FRAME):     return CamDataFrm::P_FRAME;
        case EnumToVal(CamDataFrm::FULL):        return CamDataFrm::FULL;
        case EnumToVal(CamDataFrm::OTHER):       return CamDataFrm::OTHER;
    }
    return def;  // clang-format on
}

/* ****************************************************************************************************************** */

static std::string CamStreamParamsToStr(const CamStreamParams& params)
{
    return Sprintf("%s %s %s (rate %d, timeout %.1fs)", params.host_.c_str(), CamIdToStr(params.cam_id_),
        CamDataTypeToStr(params.type_), params.rate_, params.timeout_.GetSec());
}

// ---------------------------------------------------------------------------------------------------------------------

CamStream::CamStream(const CamStreamParams& params) /* clang-format off */ :
    params_   { params }  // clang-format on
{
    DEBUG("CamStream(%s) %s", params_.name_.c_str(), CamStreamParamsToStr(params_).c_str());
}

CamStream::~CamStream()
{
}

// ---------------------------------------------------------------------------------------------------------------------

class CamStreamImpl : public CamStream, private NoCopyNoMove
{
   public:
    CamStreamImpl(const CamStreamParams& params);
    ~CamStreamImpl();
    bool Connect() override final;
    void Disconnect() override final;
    bool NextFrame(CamData& data) override final;

   private:
    boost::asio::io_context ctx_;
    boost::beast::tcp_stream stream_;
    bool connected_ = false;
    boost::beast::flat_buffer read_buf_;
    boost::beast::http::response_parser<boost::beast::http::buffer_body> parser_;
    std::string boundary_;
    std::string buf_;
    static constexpr std::size_t MAX_BODY = 10 * 1024 * 1024;  // 10 MiB

    // std::string delim_;          // actual delimiter found in the body
    // bool delim_detected_ = false;
    // bool seen_iframe_ = false;   // start emitting only at the first I-frame

    void ResetTimeout();
    bool ReadMore();
};

// ---------------------------------------------------------------------------------------------------------------------

CamStreamImpl::CamStreamImpl(const CamStreamParams& params) /* clang-format off */ :
    CamStream(params),
    stream_   { ctx_ }  // clang-format on
{
}

CamStreamImpl::~CamStreamImpl()
{
    if (connected_) {
        Disconnect();
    }
}

// The response stream is something like this:
//
//     HTTP/1.1 200 OK\r\n
//     Server: fixposition-httpd/1.0.0\r\n
//     Cache-Control: no-store\r\n
//     Pragma: no-cache\r\n
//     Content-type: multipart/x-mixed-replace;boundary=randomword\r\n
//     Access-Control-Allow-Origin: *\r\n
//     Transfer-Encoding: chunked\r\n
//     Date: Wed, 17 Jun 2026 11:37:46 GMT\r\n
//     \r\n
//     --randomword\r\n
//     Content-type: video/hevc\r\n
//     X-Fp-Meta: ts=1783589420.070595; dt=0.005174; seq=236159; fmt=H265_NAL; frm=P_FRAME; width=1024; height=768;
//     hstride=0; vstride=0; planes=1\r\n Content-length: 23120\r\n
//     \r\n
//     <23120 bytes of raw H265 data>\r\n
//     \r\n
//     --randomword\r\n
//     Content-type: video/hevc\r\n
//     X-Fp-Meta: ts=1783589420.110603; dt=0.005174; seq=236160; fmt=H265_NAL; frm=I_FRAME; width=1024; height=768;
//     hstride=0; vstride=0; planes=1\r\n Content-length: 107751\r\n
//     \r\n
//     <107751 bytes of raw H265 data>\r\n
//     \r\n
//     .
//     .
//     .

// ---------------------------------------------------------------------------------------------------------------------

bool CamStreamImpl::Connect()
{
    // Resolve
    TRACE("CamStream(%s) resolve %s:80", params_.name_.c_str(), params_.host_.c_str());
    boost::beast::error_code ec;
    boost::asio::ip::tcp::resolver resolver(ctx_);
    boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(params_.host_, "80", ec);
    if (ec || endpoints.empty()) {
        WARNING("CamStream(%s) resolve %s:80 fail: %s", params_.name_.c_str(), params_.host_.c_str(),
            endpoints.empty() ? "no endpoints" : ec.message().c_str());
        return false;
    }

    // Connect
    TRACE("CamStream(%s) connect %s:%" PRIu16, params_.name_.c_str(),
        endpoints->endpoint().address().to_string().c_str(), endpoints->endpoint().port());
    ResetTimeout();
    stream_.connect(endpoints, ec);
    if (ec) {
        WARNING("CamStream(%s) connect fail: %s", params_.name_.c_str(), ec.message().c_str());
        return false;
    }
    connected_ = true;

    bool ok = true;
    while (ok) {
        const std::string api = Sprintf("/api/v2/camera/data?cam=%s&data=%s&rate=%d", CamIdToStr(params_.cam_id_),
            CamDataTypeToStr(params_.type_), params_.rate_);

        // Setup request
        boost::beast::http::request<boost::beast::http::empty_body> req(boost::beast::http::verb::get, api, 11);
        req.set(boost::beast::http::field::host, params_.host_);
        req.set(boost::beast::http::field::user_agent, GetUserAgentStr());
        req.set(boost::beast::http::field::accept, "*/*");

        // Send request
        TRACE("CamStream(%s) request", params_.name_.c_str());
        ResetTimeout();
        boost::beast::http::write(stream_, req, ec);
        if (ec) {
            WARNING("CamStream(%s) request fail: %s", params_.name_.c_str(), ec.message().c_str());
            ok = false;
            break;
        }

        // Wait for response header
        TRACE("CamStream(%s) response", params_.name_.c_str());
        parser_.body_limit(  // Endless HTTP response body has no limit
#if BOOST_VERSION < 107400
            std::numeric_limits<uint64_t>::max()
#else
            boost::none
#endif
        );
        ResetTimeout();
        boost::beast::http::read_header(stream_, read_buf_, parser_, ec);
        if (ec) {
            WARNING("CamStream(%s) response fail: %s", params_.name_.c_str(), ec.message().c_str());
            ok = false;
            break;
        }

        // Check response header
        const auto& res = parser_.get();
        if (res.result() != boost::beast::http::status::ok) {
            WARNING("CamStream(%s) response fail: %d %s", params_.name_.c_str(), res.result_int(),
                std::string(res.reason()).c_str());
            ok = false;
            break;
        }
        // for (const auto& header : res) {
        //     TRACE("CamStream(%s) header: %s = %s", params_.name_.c_str(), std::string(header.name_string()).c_str(),
        //         std::string(header.value()).c_str());
        // }

        // Get multipart/mixed-replace boundary value
        boundary_.clear();
        const auto header = res.find(boost::beast::http::field::content_type);
        if (header != res.end()) {
            // Probably a bad API request (sensor doesn't have this API, bad params, ...)
            if (boost::algorithm::istarts_with(header->value(), "application/json")) {
                ReadMore();  // Should give us e.g. {"_ok":false,"_message":"request fail"}
                WARNING("CamStream(%s) unexpected response: %s", params_.name_.c_str(), buf_.c_str());
                ok = false;
                break;
            }
            // Otherwise unexpected response
            else if (!boost::algorithm::istarts_with(header->value(), "multipart/x-mixed-replace")) {
                WARNING("CamStream(%s) response unexpected content type: %s", params_.name_.c_str(),
                    std::string(header->value()).c_str());
                ok = false;
                break;
            }
            // Must have a boundary
            const auto parts = StrSplit(std::string(header->value()), "boundary=");
            if ((parts.size() == 2) && (parts[1].size() > 1)) {
                boundary_ = "--" + parts[1] + "\r\n";
            }
        }
        if (boundary_.empty()) {
            WARNING("CamStream(%s) response fail: missing boundary in (%s: %s)", params_.name_.c_str(),
                std::string(header->name_string()).c_str(), std::string(header->value()).c_str());
            ok = false;
            break;
        }

        break;
    }

    if (ok) {
        DEBUG("CamStream(%s) connected", params_.name_.c_str());
        ResetTimeout();
    } else {
        Disconnect();
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

void CamStreamImpl::Disconnect()
{
    TRACE("CamStream(%s) shutdown", params_.name_.c_str());

    boost::beast::error_code ec;
    stream_.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    if (ec) {
        WARNING("CamStream(%s) shutdown fail: %s", params_.name_.c_str(), ec.message().c_str());
    }
    stream_.socket().close(ec);
    if (ec) {
        WARNING("CamStream(%s) close fail: %s", params_.name_.c_str(), ec.message().c_str());
    }
    connected_ = false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool CamStreamImpl::NextFrame(CamData& data)
{
    // --randomword\r\n
    // Content-type: video/hevc\r\n
    // X-Fp-Meta: ts=1783589420.070595; dt=0.005174; seq=236159; fmt=H265_NAL; frm=P_FRAME; width=1024; height=768;
    // hstride=0; vstride=0; planes=1\r\n Content-length: 23120\r\n
    // \r\n
    // <23120 bytes of raw H265 data>\r\n
    // \r\n

    // Read until start of next part (or read fails)
    std::size_t ix = std::string::npos;
    while (ix == std::string::npos) {
        if (!ReadMore()) {
            return false;
        }
        ix = buf_.find(boundary_);
    }

    buf_.erase(0, ix + boundary_.size());  // nope!

    // Read until end of headers
    ix = std::string::npos;
    while (ix == std::string::npos) {
        if (!ReadMore()) {
            return false;
        }
        ix = buf_.find("\r\n\r\n");
    }

    auto headers = buf_.substr(0, ix + 2);  // + "\r\n"
    buf_.erase(0, ix + 4);

    // Parse headers
    // @todo could probably use boost::beast::something...
    std::string content_type;
    std::size_t content_length = 0;
    std::string x_fp_meta;
    const auto parts = StrSplit(headers, "\r\n");
    for (auto& part : parts) {
        const auto kv = StrSplit(part, ":", 2);  // "Some-header: some-value" -> [ "Some-header", " some-value" ]
        if ((kv.size() == 2) && (kv[1].size() > 1)) {
            if (StrToLower(kv[0]) == "content-type") {
                content_type = kv[1].substr(1);
            } else if (StrToLower(kv[0]) == "x-fp-meta") {
                x_fp_meta = kv[1].substr(1);
            } else if (StrToLower(kv[0]) == "content-length") {
                StrToValue(kv[1].substr(1), content_length);
            }
        }
    }
    // TRACE("CamStream(%s) content_type=%s content_length=%" PRIuMAX " x_fp_meta=%s", params_.name_.c_str(),
    //     content_type.c_str(), content_length, x_fp_meta.c_str());
    if (content_type.empty() || (content_length == 0) || (content_length > MAX_BODY) || x_fp_meta.empty()) {
        WARNING("CamStream(%s) unexpected headers", params_.name_.c_str());
        return false;
    }

    // Wait for payload (multipart body)
    while (buf_.size() < (content_length + 2)) {  // body + \r\n
        if (!ReadMore()) {
            return false;
        }
    }

    // Get payload
    data.data_ = { (const uint8_t*)buf_.data(), (const uint8_t*)buf_.data() + content_length };
    buf_.erase(0, content_length + 2);

    // Parse meta data (be generous: get what we can, ignore what fails or is missing)
    data.cam_id_ = params_.cam_id_;
    data.type_ = params_.type_;
    for (auto& entry : StrSplit(x_fp_meta, "; ")) {
        const auto kv = StrSplit(entry, "=", 2);
        if ((kv.size() == 2) && !kv[0].empty() && !kv[1].empty()) {
            // clang-format off
            if      (kv[0] == "fmt")     { data.fmt_ = CamDataFmtFromStrOr(kv[1].c_str(), CamDataFmt::UNSPECIFIED); }
            else if (kv[0] == "frm")     { data.frm_ = CamDataFrmFromStrOr(kv[1].c_str(), CamDataFrm::UNSPECIFIED); }
            else if (kv[0] == "seq")     { StrToValue(kv[1], data.seq_);    }
            else if (kv[0] == "ts")      { StrToValue(kv[1], data.ts_);     }
            else if (kv[0] == "dt")      { StrToValue(kv[1], data.dt_);     }
            else if (kv[0] == "width")   { StrToValue(kv[1], data.width_);  }
            else if (kv[0] == "height")  { StrToValue(kv[1], data.height_); }
            // clang-format on
        }
    }
    data.valid_ = ((data.fmt_ != CamDataFmt::UNSPECIFIED) && (data.frm_ != CamDataFrm::UNSPECIFIED));

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void CamStreamImpl::ResetTimeout()
{
    stream_.expires_after(params_.timeout_.GetChronoMilli());
}

// ---------------------------------------------------------------------------------------------------------------------

bool CamStreamImpl::ReadMore()
{
    bool ok = true;
    while (ok) {
        char buf[65536];
        auto& body = parser_.get().body();
        body.data = buf;
        body.size = sizeof(buf);

        ResetTimeout();
        boost::beast::error_code ec;
        boost::beast::http::read_some(stream_, read_buf_, parser_, ec);
        const std::size_t n = sizeof(buf) - body.size;

        // Collect usable data
        if (n > 0) {
            buf_.append(buf, n);

            if (buf_.size() > MAX_BODY) {
                WARNING("CamStream(%s) read overflow", params_.name_.c_str());
                ok = false;
                break;
            }

            break;
        }
        // Should read again
        else if (ec == boost::beast::http::error::need_buffer) {
            continue;
        }
        // Message (body) complete, not expected
        else if (parser_.is_done()) {
            WARNING("CamStream(%s) message copmplete", params_.name_.c_str());
            ok = false;
            break;
        }
        // Some problem, e.g. end_of_stream
        else if (ec) {
            WARNING("CamStream(%s) read fail: %s", params_.name_.c_str(), ec.message().c_str());
            ok = false;
            break;
        }
        // else: try again
    }

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

std::unique_ptr<CamStream> CreateCamStream(const CamStreamParams& params)
{
    // Some sanity checks..
    bool ok = true;

    switch (params.cam_id_) {
        case CamId::CAM1:
        case CamId::CAM2:
        case CamId::CAM3:
        case CamId::CAM4:
            break;
        case CamId::UNSPECIFIED:
            ok = false;
            break;
    }

    switch (params.type_) {
        case CamDataType::HIRES_IMG:
        case CamDataType::HIRES_VID:
            if (params.rate_ != 1) {
                ok = false;
            }
            break;
        case CamDataType::LORES_IMG:
        case CamDataType::LORES_VID:
            if ((params.rate_ < params.RATE_MIN) || (params.rate_ > params.RATE_MAX)) {
                ok = false;
            }
            break;
        case CamDataType::UNSPECIFIED:
            ok = false;
            break;
    }

    if (params.timeout_ < Duration::FromSec(0.1)) {
        ok = false;
    }

    if (!ok) {
        WARNING("Bad params for CamStream(%s) %s", params.name_.c_str(), CamStreamParamsToStr(params).c_str());
        return nullptr;
    }

    return std::make_unique<CamStreamImpl>(params);
}

/* ****************************************************************************************************************** */
}  // namespace cam
}  // namespace common
}  // namespace fpsdk
