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
 * @brief Fixposition SDK: Video frame decoding
 */

/* LIBC/STL */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

/* EXTERNAL */
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/rational.h>
#include <libswscale/swscale.h>
}

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/time.hpp"
#include "fpsdk_common/types.hpp"
#include "fpsdk_common/video.hpp"

namespace fpsdk {
namespace common {
namespace video {
/* ****************************************************************************************************************** */
#if FPSDK_USE_FFMPEG

using namespace fpsdk::common::logging;
using namespace fpsdk::common::types;
using namespace fpsdk::common::time;
using namespace fpsdk::common::string;

const char* VideoCodecToStr(const VideoCodec codec_id)
{
    switch (codec_id) {  // clang-format off
        case VideoCodec::UNSPECIFIED:   return "UNSPECIFIED";
        case VideoCodec::H264:          return "H264";
        case VideoCodec::H265:          return "H265";
    }  // clang-format on
    return "?";
}

VideoCodec VideoCodecFromStrOr(const char* str, const VideoCodec def)
{  // clang-format off
    if (strcmp(str, "UNSPECIFIED") == 0) { return VideoCodec::UNSPECIFIED; }
    if (strcmp(str, "H264")        == 0) { return VideoCodec::H264; }
    if (strcmp(str, "H265")        == 0) { return VideoCodec::H265; }
    return def;  // clang-format on
}

const char* PixelFmtToStr(const PixelFmt fmt)
{
    switch (fmt) {  // clang-format off
        case PixelFmt::UNSPECIFIED:  return "UNSPECIFIED";
        case PixelFmt::Y8:           return "Y8";
        case PixelFmt::RGB24:        return "RGB24";
        case PixelFmt::GBRP:         return "GBRP";
    }  // clang-format on
    return "?";
}

static const char* PixelFmtToFlag(const PixelFmt fmt)
{
    switch (fmt) {  // clang-format off
        case PixelFmt::UNSPECIFIED:  break;
        case PixelFmt::Y8:           return "gray8";
        case PixelFmt::RGB24:        return "rgb24";
        case PixelFmt::GBRP:         return "gbrp";
    }  // clang-format on
    return "?";
}

PixelFmt PixelFmtFromStrOr(const char* str, const PixelFmt def)
{  // clang-format off
    if (strcmp(str, "UNSPECIFIED") == 0) { return PixelFmt::UNSPECIFIED; }
    if (strcmp(str, "Y8")          == 0) { return PixelFmt::Y8; }
    if (strcmp(str, "RGB24")       == 0) { return PixelFmt::RGB24; }
    if (strcmp(str, "GBRP")        == 0) { return PixelFmt::GBRP; }
    return def;  // clang-format on
}

const char* ScalingQualToStr(const ScalingQual qual)
{
    switch (qual) {  // clang-format off
        case ScalingQual::UNSPECIFIED:  return "UNSPECIFIED";
        case ScalingQual::BILINEAR:     return "BILINEAR";
        case ScalingQual::BICUBIC:      return "BICUBIC";
        case ScalingQual::LANCZOS:      return "LANCZOS";
        case ScalingQual::SINC:         return "SINC";
    }  // clang-format on
    return "?";
}

static const char* ScalingQualToFlag(const ScalingQual qual)
{
    switch (qual) {  // clang-format off
        case ScalingQual::UNSPECIFIED:  break;
        case ScalingQual::BILINEAR:     return "bilinear";
        case ScalingQual::BICUBIC:      return "bicubic";
        case ScalingQual::LANCZOS:      return "lanczos";
        case ScalingQual::SINC:         return "sinc";
    }  // clang-format on
    return "?";
}

ScalingQual ScalingQualFromStrOr(const char* str, const ScalingQual def)
{  // clang-format off
  if (strcmp(str, "UNSPECIFIED") == 0) { return ScalingQual::UNSPECIFIED; }
  if (strcmp(str, "BILINEAR")    == 0) { return ScalingQual::BILINEAR; }
  if (strcmp(str, "BICUBIC")     == 0) { return ScalingQual::BICUBIC; }
  if (strcmp(str, "LANCZOS")     == 0) { return ScalingQual::LANCZOS; }
  if (strcmp(str, "SINC")        == 0) { return ScalingQual::SINC; }
  return def;  // clang-format on
}

const char* HwAccelToStr(const HwAccel accel)
{
    switch (accel) {  // clang-format off
        case HwAccel::UNSPECIFIED:  return "UNSPECIFIED";
        case HwAccel::AUTO:         return "AUTO";
        case HwAccel::SW:           return "SW";
        case HwAccel::HW:           return "HW";
    }  // clang-format on
    return "?";
}

HwAccel HwAccelFromStrOr(const char* str, const HwAccel def)
{  // clang-format off
  if (strcmp(str, "UNSPECIFIED") == 0) { return HwAccel::UNSPECIFIED; }
  if (strcmp(str, "AUTO")        == 0) { return HwAccel::AUTO; }
  if (strcmp(str, "SW")          == 0) { return HwAccel::SW; }
  if (strcmp(str, "HW")          == 0) { return HwAccel::HW; }
  return def;  // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

static std::string VideoDecoderParamsToStr(const VideoDecoderParams& params)
{
    return Sprintf("%s %s %.1f %s %s", VideoCodecToStr(params.codec_), PixelFmtToStr(params.fmt_), params.scale_,
        ScalingQualToStr(params.qual_), HwAccelToStr(params.accel_));
}

// *********************************************************************************************************************

VideoFrameDecoder::VideoFrameDecoder(const VideoDecoderParams& params) /* clang-format off */ :
    params_   { params }  // clang-format on

{
    DEBUG("VideoFrameDecoder(%s) %s", params_.name_.c_str(), VideoDecoderParamsToStr(params_).c_str());

    // Setup FFmpeg logging. Note that this is global... :-/. We could set a custom handler (av_log_set_callback()) and
    // in there check which context it has been called from and then selectively output messages. However, that would be
    // quite involved to make it work across all instances of VideoFrameDecoder (and possibly other things that use
    // FFmpeg stuff in the same app). So we globally silence it (it's quite chatty otherwise, also at warning and error
    // levels!).
#  ifdef NDEBUG
    av_log_set_level(AV_LOG_FATAL);
#  else
    if (LoggingIsLevel(LoggingLevel::TRACE)) {
        av_log_set_level(AV_LOG_DEBUG);
    } else if (LoggingIsLevel(LoggingLevel::DEBUG)) {
        av_log_set_level(AV_LOG_WARNING);
    } else {
        av_log_set_level(AV_LOG_ERROR);
    }
#  endif
}

VideoFrameDecoder::~VideoFrameDecoder()
{
}

// ---------------------------------------------------------------------------------------------------------------------

std::optional<ImageData> VideoFrameDecoder::DecodeFrame(const std::vector<uint8_t>& data)
{
    return DecodeFrame(data.data(), data.size());
}

// *********************************************************************************************************************

// Software decoder
class VideoFrameDecoderImpl : public VideoFrameDecoder, private NoCopyNoMove
{
   public:
    VideoFrameDecoderImpl(const VideoDecoderParams& params);
    ~VideoFrameDecoderImpl();
    std::optional<ImageData> DecodeFrame(const uint8_t* data, const std::size_t size) override final;
    bool IsOkay() const override final;

    enum AVPixelFormat GetHwFormat(const enum AVPixelFormat* fmts) const;

   private:
    // clang-format off
    enum class State { DEINIT, INIT_AV, INIT_GRAPH, ERROR };
    State                 state_           = State::DEINIT;
    AVHWDeviceType        hw_type_         = AV_HWDEVICE_TYPE_NONE;  // AV_HWDEVICE_TYPE_VAAPI
    enum AVPixelFormat    src_fmt_         = AV_PIX_FMT_YUVJ420P;    // AV_PIX_FMT_VAAPI
    const AVCodec*        codec_           = nullptr;
    AVCodecContext*       ctx_             = nullptr;
    AVBufferRef*          hw_ctx_          = nullptr;  // hwdevice_ctx
    AVPacket*             pkt_             = nullptr;
    AVFrame*              frame_           = nullptr;
    AVFrame*              filt_frame_      = nullptr;
    AVFilterGraph*        graph_           = nullptr;
    AVFilterContext*      src_ctx_         = nullptr;
    AVFilterContext*      dst_ctx_         = nullptr;
    const AVFilter*       src_filt_        = nullptr;
    const AVFilter*       dst_filt_        = nullptr;
    // clang-format off

    bool InitAv();
    bool InitGraph();
    void Cleanup();

    // Buffer and helper for stringifying FFmpeg errors
    char av_errbuf_[1000] = {0};
    const char* AvErrStr(const int errnum);
};

VideoFrameDecoderImpl::VideoFrameDecoderImpl(const VideoDecoderParams& params) /* clang-format off */ :
    VideoFrameDecoder(params)  // clang-format on
{
}

VideoFrameDecoderImpl::~VideoFrameDecoderImpl()
{
    Cleanup();
}

// ---------------------------------------------------------------------------------------------------------------------

const char* VideoFrameDecoderImpl::AvErrStr(const int errnum)
{
    av_strerror(errnum, av_errbuf_, sizeof(av_errbuf_));
    return av_errbuf_;
}

bool VideoFrameDecoderImpl::IsOkay() const
{
    return state_ != State::ERROR;
}

// ---------------------------------------------------------------------------------------------------------------------

static enum AVPixelFormat GetHwFormatCb(AVCodecContext* ctx, const enum AVPixelFormat* fmts)
{
    return static_cast<VideoFrameDecoderImpl*>(ctx->opaque)->GetHwFormat(fmts);
}

enum AVPixelFormat VideoFrameDecoderImpl::GetHwFormat(const enum AVPixelFormat* fmts) const
{
    for (const enum AVPixelFormat* p = fmts; *p != AV_PIX_FMT_NONE; ++p) {
        if (*p == src_fmt_) {
            return *p;
        }
    }
    return AV_PIX_FMT_NONE;
}

bool VideoFrameDecoderImpl::InitAv()
{
    DEBUG("VideoFrameDecoderImpl(%s) init", params_.name_.c_str());

    bool ok = (state_ == State::DEINIT);
    while (ok) {
        // ----- Get/check codec -----

        switch (params_.codec_) {  // clang-format off
            case VideoCodec::H264: codec_ = avcodec_find_decoder(AV_CODEC_ID_H264); break;
            case VideoCodec::H265: codec_ = avcodec_find_decoder(AV_CODEC_ID_H265); break;
            case VideoCodec::UNSPECIFIED:
                break;
        }  // clang-format on
        if (!codec_) {
            WARNING("VideoFrameDecoder(%s) avcodec_find_decoder fail", params_.name_.c_str());
            ok = false;
            break;
        }

        // ----- Try to use hw acceleration -----

#  if 0  // Debugging, dev help
         // Check supported hw devices

        // AVHWDeviceType device_type = AV_HWDEVICE_TYPE_NONE;
        // while (true) {
        //     device_type = av_hwdevice_iterate_types(device_type);
        //     if (device_type == AV_HWDEVICE_TYPE_NONE) {
        //         break;
        //     }
        //     DEBUG("VideoDecoder(%s) hw device: %s", name_.c_str(), av_hwdevice_get_type_name(device_type));
        // }
        // hw device: vdpau
        // hw device: cuda
        // hw device: vaapi
        // hw device: qsv
        // hw device: drm
        // hw device: opencl
        // hw device: vulkan

        for (int ix = 0; ix < 100; ix++) {
            const AVCodecHWConfig* config = avcodec_get_hw_config(codec_, ix);
            if (!config) {
                DEBUG("nope %d", ix);
                break;
            }
            DEBUG("VideoFrameDecoder(%s) hw config %d: type %s, pixfmt %s, methods 0x%02x", params_.name_.c_str(), ix,
                av_hwdevice_get_type_name(config->device_type), av_get_pix_fmt_name(config->pix_fmt), config->methods);
        }
        // hw config 0: type cuda, pixfmt cuda, methods 0x03
        // hw config 1: type vaapi, pixfmt vaapi, methods 0x0b
        // hw config 2: type vdpau, pixfmt vdpau, methods 0x0b
        // hw config 3: type vulkan, pixfmt vulkan, methods 0x0b
#  endif
        const bool use_hw_accel = ((params_.accel_ == HwAccel::AUTO) || (params_.accel_ == HwAccel::HW));
        const bool req_hw_accel = (params_.accel_ == HwAccel::HW);

        if (use_hw_accel) {
            // We currently only support VA-API. We could probe for other kinds of hw accel here.
            AVBufferRef* hw = nullptr;
            const int hw_res = av_hwdevice_ctx_create(&hw, AV_HWDEVICE_TYPE_VAAPI, nullptr, nullptr, 0);
            if (hw_res < 0) {
                if (req_hw_accel) {
                    WARNING("VideoFrameDecoder(%s) Could not create hw device (%s)", params_.name_.c_str(),
                        AvErrStr(hw_res));
                    ok = false;
                    break;
                } else {
                    DEBUG("VideoFrameDecoder(%s) Could not create hw device (%s), using sw decoding",
                        params_.name_.c_str(), AvErrStr(hw_res));
                }
                hw = nullptr;
            } else {
                // hw_ctx_ = reinterpret_cast<void*>(hw);
                hw_ctx_ = av_buffer_ref(hw);
                hw_type_ = AV_HWDEVICE_TYPE_VAAPI;
                src_fmt_ = AV_PIX_FMT_VAAPI;
                // @todo this crashes..
                // ctx_->opaque = this;
                // ctx_->get_format = GetHwFormatCb;
                UNUSED(GetHwFormatCb);
            }
        }

        // ----- Setup codec context -----

        ctx_ = avcodec_alloc_context3(codec_);
        if (!ctx_) {
            WARNING("VideoFrameDecoder(%s) avcodec_alloc_context3 fail", params_.name_.c_str());
            ok = false;
            break;
        }
        if (hw_ctx_) {
            ctx_->hw_device_ctx = av_buffer_ref(hw_ctx_);
        }

        if (avcodec_open2(ctx_, codec_, nullptr) < 0) {
            WARNING("VideoFrameDecoder(%s) avcodec_open2 fail", params_.name_.c_str());
            ok = false;
            break;
        }

        // ----- Packet and frames for later use -----

        pkt_ = av_packet_alloc();
        if (!pkt_) {
            WARNING("VideoFrameDecoder(%s) av_packet_alloc fail", params_.name_.c_str());
            ok = false;
            break;
        }

        frame_ = av_frame_alloc();
        if (!frame_) {
            WARNING("VideoFrameDecoder(%s) av_frame_alloc fail", params_.name_.c_str());
            ok = false;
            break;
        }

        filt_frame_ = av_frame_alloc();
        if (!filt_frame_) {
            WARNING("VideoFrameDecoder(%s) av_frame_alloc fail", params_.name_.c_str());
            ok = false;
            break;
        }

        // ----- Filter graph handles -----

        graph_ = avfilter_graph_alloc();
        if (!graph_) {
            WARNING("VideoFrameDecoder(%s) avfilter_graph_alloc fail", params_.name_.c_str());
            ok = false;
            break;
        }

        src_filt_ = avfilter_get_by_name("buffer");
        if (!src_filt_) {
            WARNING("VideoFrameDecoder(%s) avfilter_get_by_name(buffer) fail", params_.name_.c_str());
            ok = false;
            break;
        }

        dst_filt_ = avfilter_get_by_name("buffersink");
        if (!dst_filt_) {
            WARNING("VideoFrameDecoder(%s) avfilter_get_by_name(buffersink) fail", params_.name_.c_str());
            ok = false;
            break;
        }

        break;
    }

    if (!ok) {
        WARNING("VideoFrameDecoder(%s) init fail", params_.name_.c_str());
        return false;
    }

    state_ = State::INIT_AV;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool VideoFrameDecoderImpl::InitGraph()
{
    bool ok = true;

    int src_width = 0;
    int src_height = 0;
    AVPixelFormat src_format = AV_PIX_FMT_NONE;  // TODO check <-> src_fmt_
    AVRational src_time_base = { 1, 25 };
    AVRational src_sar = { 1, 1 };

    src_width = frame_->width;
    src_height = frame_->height;
    src_format = static_cast<AVPixelFormat>(frame_->format);  // yuvj420p, check that
    if ((ctx_->time_base.num > 0) && (ctx_->time_base.den > 0)) {
        src_time_base = ctx_->time_base;
    } else {
        src_time_base.num = 1;
        src_time_base.den = 25;
    }
    if ((frame_->sample_aspect_ratio.num > 0) && (frame_->sample_aspect_ratio.den > 0)) {
        src_sar = frame_->sample_aspect_ratio;
    } else {
        src_sar.num = 1;
        src_sar.den = 1;
    }

    const double scale = std::clamp(params_.scale_, 0.1, 1.0);
    const int dst_width = std::round(src_width * scale);
    const int dst_height = std::round(src_height * scale);

    DEBUG("VideoFrameDecoderImpl(%s) graph (frame %dx%d %s, sar %d/%d, tb %d/%d) -> (%dx%d %s)",
        params_.name_.c_str(),                                           //
        src_width, src_height, av_get_pix_fmt_name(src_format),          //
        src_sar.num, src_sar.den, src_time_base.num, src_time_base.den,  //
        dst_width, dst_height, PixelFmtToStr(params_.fmt_));

    while (ok) {
        // Source buffer
#  if 0
        char src_args[256];
        std::snprintf(src_args, sizeof(src_args), "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
            src_width, src_height, src_format, src_time_base.num, src_time_base.den, src_sar.num, src_sar.den);
        TRACE("VideoFrameDecoderImpl(%s) avfilter_graph_create_filter(src) %s", params_.name_.c_str(), src_args);
        const int res_src = avfilter_graph_create_filter(&src_ctx_, src_filt_, "src", src_args, nullptr, graph_);
        if (res_src < 0) {
            WARNING("VideoFrameDecoder(%s) avfilter_graph_create_filter(src) fail: %s", params_.name_.c_str(), AvErrStr(res_src));
            ok = false;
            break;
        }
#  else
        src_ctx_ = avfilter_graph_alloc_filter(graph_, src_filt_, "src");
        if (!src_ctx_) {
            WARNING("VideoFrameDecoder(%s) avfilter_graph_alloc_filter(src) fail", params_.name_.c_str());
            ok = false;
            break;
        }

        AVBufferSrcParameters* par = av_buffersrc_parameters_alloc();
        if (!par) {
            WARNING("VideoFrameDecoder(%s) av_buffersrc_parameters_alloc(src) fail", params_.name_.c_str());
            ok = false;
            break;
        }
        par->format = src_fmt_;
        par->width = src_width;
        par->height = src_height;
        par->time_base = src_time_base;
        par->sample_aspect_ratio = src_sar;
        if (hw_ctx_) {
            if (frame_->hw_frames_ctx) {  // Should be present now that we have decoded the first frame
                par->hw_frames_ctx = av_buffer_ref(frame_->hw_frames_ctx);
            } else {
                WARNING("VideoFrameDecoder(%s) hw_frames_ctx missing", params_.name_.c_str());
                ok = false;
                break;
            }
        }
        const int res_par = av_buffersrc_parameters_set(src_ctx_, par);
        av_free(par);
        if (res_par < 0) {
            WARNING("VideoFrameDecoder(%s) av_buffersrc_parameters_set(src) fail: %s", params_.name_.c_str(),
                AvErrStr(res_par));
            ok = false;
            break;
        }
        const int res_str = avfilter_init_str(src_ctx_, nullptr);
        if (res_str < 0) {
            WARNING(
                "VideoFrameDecoderImpl(%s) avfilter_init_str(src) fail: %s", params_.name_.c_str(), AvErrStr(res_str));
            return false;
        }
#  endif
        // Dest buffer
#  if LIBAVFILTER_VERSION_INT < AV_VERSION_INT(10, 6, 100)
        TRACE("VideoFrameDecoderImpl(%s) avfilter_graph_create_filter(sink)", params_.name_.c_str());
        const int res_sink = avfilter_graph_create_filter(&dst_ctx_, dst_filt_, "dst", nullptr, nullptr, graph_);
        if (res_sink < 0) {
            WARNING("VideoFrameDecoder(%s) avfilter_graph_create_filter(sink) fail: %s", params_.name_.c_str(),
                AvErrStr(res_sink));
            return false;
        }
        // Constrain a buffersink to a single output pixel format
        enum AVPixelFormat out_pix_fmts[2] = { AV_PIX_FMT_NONE, AV_PIX_FMT_NONE };
        switch (params_.fmt_) {
            case PixelFmt::Y8:
                out_pix_fmts[0] = AV_PIX_FMT_GRAY8;
                break;
            case PixelFmt::RGB24:
                out_pix_fmts[0] = AV_PIX_FMT_RGB24;
                break;
            case PixelFmt::GBRP:
                out_pix_fmts[0] = AV_PIX_FMT_GBRP;
                break;
            case PixelFmt::UNSPECIFIED:
                break;
        }
        const int res_pix_fmts =
            av_opt_set_int_list(dst_ctx_, "pix_fmts", out_pix_fmts, AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
        if (res_pix_fmts < 0) {
            WARNING("VideoFrameDecoder(%s) av_opt_set_int_list(sink, pix_fmts) fail: %s", params_.name_.c_str(),
                AvErrStr(res_pix_fmts));
            ok = false;
            break;
        }
#  else  // >= 10.6.100
        char sink_args[256];
        std::snprintf(sink_args, sizeof(sink_args), "pixel_formats=%s", PixelFmtToFlag(params_.fmt_));
        TRACE("VideoFrameDecoderImpl(%s) avfilter_graph_create_filter(sink) %s", params_.name_.c_str(), sink_args);
        const int res_sink = avfilter_graph_create_filter(&dst_ctx_, dst_filt_, "dst", sink_args, nullptr, graph_);
        if (res_sink < 0) {
            WARNING("VideoFrameDecoder(%s) avfilter_graph_create_filter(sink) fail: %s", params_.name_.c_str(),
                AvErrStr(res_sink));
            ok = false;
            break;
        }
#  endif

        char filter_desc[256];
        if (hw_ctx_) {
            std::snprintf(filter_desc, sizeof(filter_desc), "scale_vaapi=w=%d:h=%d:format=nv12,hwdownload,format=nv12",
                dst_width, dst_height);
        } else {
            std::snprintf(filter_desc, sizeof(filter_desc), "scale=w=%d:h=%d:flags=%s,format=pix_fmts=%s", dst_width,
                dst_height, ScalingQualToFlag(params_.qual_), PixelFmtToFlag(params_.fmt_));
        }

        // Parse filter_desc connecting src_ctx -> sink_ctx into graph, then configure the graph
        AVFilterInOut* outputs = avfilter_inout_alloc();
        AVFilterInOut* inputs = avfilter_inout_alloc();
        if (!outputs || !inputs) {
            if (outputs) {
                avfilter_inout_free(&outputs);
            }
            if (inputs) {
                avfilter_inout_free(&inputs);
            }
            WARNING("VideoFrameDecoder(%s) avfilter_inout_alloc() fail", params_.name_.c_str());
            ok = false;
            break;
        }

        outputs->name = av_strdup("in");
        outputs->filter_ctx = src_ctx_;
        outputs->pad_idx = 0;
        outputs->next = nullptr;

        inputs->name = av_strdup("out");
        inputs->filter_ctx = dst_ctx_;
        inputs->pad_idx = 0;
        inputs->next = nullptr;

        TRACE("VideoFrameDecoderImpl(%s) avfilter_graph_parse_ptr(sink) %s", params_.name_.c_str(), filter_desc);

        const int res_graph = avfilter_graph_parse_ptr(graph_, filter_desc, &inputs, &outputs, nullptr);
        avfilter_inout_free(&inputs);
        avfilter_inout_free(&outputs);
        if (res_graph < 0) {
            WARNING(
                "VideoFrameDecoder(%s): avfilter_graph_parse_ptr fail: %s", params_.name_.c_str(), AvErrStr(res_graph));
            ok = false;
            break;
        }

        const int res_config = avfilter_graph_config(graph_, nullptr);
        if (res_config < 0) {
            WARNING("%s: avfilter_graph_config failed: %s", params_.name_.c_str(), AvErrStr(res_config));
            ok = false;
            break;
        }

        break;
    }

    if (!ok) {
        WARNING("VideoFrameDecoder(%s) filter fail", params_.name_.c_str());
        return false;
    }

    state_ = State::INIT_GRAPH;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

void VideoFrameDecoderImpl::Cleanup()
{
    if (graph_) {
        avfilter_graph_free(&graph_);
        graph_ = nullptr;
    }
    if (filt_frame_) {
        av_frame_free(&filt_frame_);
        filt_frame_ = nullptr;
    }
    if (frame_) {
        av_frame_free(&frame_);
        frame_ = nullptr;
    }
    if (pkt_) {
        av_packet_free(&pkt_);
        pkt_ = nullptr;
    }
    if (ctx_) {
        avcodec_free_context(&ctx_);
        ctx_ = nullptr;
    }
    if (hw_ctx_) {
        av_buffer_unref(&hw_ctx_);
        hw_ctx_ = nullptr;
    }

    codec_ = nullptr;
    src_filt_ = nullptr;
    dst_filt_ = nullptr;
    src_ctx_ = nullptr;  // @todo av_free(src_ctx_) ?
    dst_ctx_ = nullptr;  // @todo av_free(dst_ctx_) ?

    state_ = State::DEINIT;
}

// ---------------------------------------------------------------------------------------------------------------------

std::optional<ImageData> VideoFrameDecoderImpl::DecodeFrame(const uint8_t* data, const std::size_t size)
{
    TRACE("VideoFrameDecoderImpl(%s) decode %" PRIuMAX, params_.name_.c_str(), size);

    if ((data == nullptr) || (size == 0) || (size > 10000000) || (state_ == State::ERROR)) {
        return std::nullopt;
    }

    // Initialise on first call
    if ((state_ == State::DEINIT) && !InitAv()) {
        state_ = State::ERROR;
        return std::nullopt;
    }

#  ifndef NDEBUG
    TicToc tt;
#  endif

    // One chunk of data is one frame. Copy the data, as FFmpeg wants to own (and modify?) it (pkt->data is
    // non-const...).
    av_packet_unref(pkt_);  // cleanup from previous use
    pkt_->data = reinterpret_cast<uint8_t*>(av_memdup(data, size));
    if (!pkt_->data) {
        TRACE("VideoFrameDecoderImpl(%s) av_memdup fail", params_.name_.c_str());
        return std::nullopt;
    }
    pkt_->size = size;
    pkt_->flags = 0;  // Let FFmpeg work out what's inside. Could be AV_PKT_FLAG_KEY, AV_PKT_FLAG_TRUSTED, ...

    // Tell FFmpeg it owns this buffer and should av_free() it when done
    pkt_->buf = av_buffer_create(pkt_->data, pkt_->size, av_buffer_default_free, nullptr, 0);
    if (!pkt_->buf) {
        av_freep(&pkt_->data);
        TRACE("VideoFrameDecoderImpl(%s) av_buffer_create fail", params_.name_.c_str());
        return std::nullopt;
    }

    // Send packet to decoder algorithm
    const int res_send = avcodec_send_packet(ctx_, pkt_);
    // This should give FFmpeg debug output (when enabled) something like this:
    //   [hevc @ 0x56050a13e2c0] nal_unit_type: 32(VPS), nuh_layer_id: 0, temporal_id: 0
    //   [hevc @ 0x56050a13e2c0] nal_unit_type: 33(SPS), nuh_layer_id: 0, temporal_id: 0
    //   [hevc @ 0x56050a13e2c0] nal_unit_type: 34(PPS), nuh_layer_id: 0, temporal_id: 0
    //   [hevc @ 0x56050a13e2c0] nal_unit_type: 19(IDR_W_RADL), nuh_layer_id: 0, temporal_id: 0
    //   [hevc @ 0x56050a13e2c0] Decoding SPS
    //   [hevc @ 0x56050a13e2c0] Main profile bitstream
    //   [hevc @ 0x56050a13e2c0] Decoding VUI
    //   [hevc @ 0x56050a13e2c0] Decoding PPS
    //   [hevc @ 0x56050a13e2c0] Output frame with POC 0/0.
    //   [hevc @ 0x56050a13e2c0] Decoded frame with POC 0/0.
    TRACE("VideoFrameDecoderImpl(%s) avcodec_send_packet -> res_send=%d (%s)", params_.name_.c_str(), res_send,
        AvErrStr(res_send));
    // - 0                Success
    // - AVERROR(EAGAIN)  Input not accepted in current state, must do avcodec_receive_frame() first
    // - AVERROR_EOF      Decoder has been flushed, no new packets can be sent to it
    // - AVERROR(EINVAL)  Codec not opened, not a decoder, or requires flush
    // - AVERROR(ENOMEM)  Failed to add packet to internal queue, or similar
    if (res_send < 0) {  // Or other negative error codes...
        WARNING("VideoFrameDecoder(%s) avcodec_send_packet fail: %s", params_.name_.c_str(), AvErrStr(res_send));
        return std::nullopt;
    }

#  ifndef NDEBUG
    const double t_send = tt.TocMs(true);
#  endif

    // Receive decoded frame
    const int res_recv = avcodec_receive_frame(ctx_, reinterpret_cast<AVFrame*>(frame_));
    TRACE("VideoFrameDecoderImpl(%s) avcodec_receive_frame -> res_recv=%d (%s)", params_.name_.c_str(), res_recv,
        AvErrStr(res_recv));
    // AVERROR(EAGAIN)  Output is not available in this state - user must try to send new/more input
    // AVERROR_EOF      The codec has been fully flushed, and there will be no more output frames
    if ((res_recv == AVERROR(EAGAIN)) || (res_recv == AVERROR_EOF)) {
        return std::nullopt;
    }
    // AVERROR(EINVAL)  Codec not opened
    // other < 0 value  Other error
    else if (res_recv < 0) {
        WARNING("VideoFrameDecoder(%s) avcodec_receive_frame fail: %s", params_.name_.c_str(), AvErrStr(res_recv));
        return std::nullopt;
    }
    // else: 0 = Success, a frame was returned

#  ifndef NDEBUG
    const double t_recv = tt.TocMs(true);
#  endif

    // Initialise filter graph now that we know the frame size and pixel format
    if ((state_ == State::INIT_AV) && !InitGraph()) {
        state_ = State::ERROR;
        return std::nullopt;
    }

#  ifndef NDEBUG
    tt.Tic();
#  endif

    // Convert (pixel format, scale)

    // Run frame through filter
    const int res_add = av_buffersrc_add_frame_flags(src_ctx_, frame_, AV_BUFFERSRC_FLAG_KEEP_REF);
    av_frame_unref(frame_);
    if (res_add < 0) {
        WARNING("VideoFrameDecoder(%s) av_buffersrc_add_frame fail: %s", params_.name_.c_str(), AvErrStr(res_add));
        return std::nullopt;
    }
    const int res_sink = av_buffersink_get_frame(dst_ctx_, filt_frame_);
    if ((res_sink == AVERROR(EAGAIN)) || (res_sink == AVERROR_EOF)) {
        return std::nullopt;
    } else if (res_sink < 0) {
        WARNING(
            "VideoFrameDecoderImpl(%s) av_buffersink_get_frame fail: %s", params_.name_.c_str(), AvErrStr(res_sink));
        // state_ = State::ERROR; // @todo ???
        return std::nullopt;
    }

    // Copy out image data
    ImageData img;
    img.fmt_ = params_.fmt_;
    img.width_ = filt_frame_->width;
    img.height_ = filt_frame_->height;
    int img_size = 0;
    switch (params_.fmt_) {  // clang-format off
        case PixelFmt::Y8:           img_size = img.width_ * img.height_;     break;
        case PixelFmt::RGB24:
        case PixelFmt::GBRP:         img_size = img.width_ * img.height_ * 3; break;
        case PixelFmt::UNSPECIFIED:  break;
    }  // clang-format on
    img.data_.resize(img_size);
    const enum AVPixelFormat frame_fmt = static_cast<enum AVPixelFormat>(filt_frame_->format);
    TRACE("VideoFrameDecoderImpl(%s) av_image_copy_to_buffer %dx%d %s %d", params_.name_.c_str(), filt_frame_->width,
        filt_frame_->height, av_get_pix_fmt_name(frame_fmt), img_size);

    const int res_copy = av_image_copy_to_buffer(img.data_.data(), img_size, filt_frame_->data, filt_frame_->linesize,
        frame_fmt, filt_frame_->width, filt_frame_->height, /*align=*/1);
    av_frame_unref(filt_frame_);
    if (res_copy < 0) {
        WARNING(
            "VideoFrameDecoderImpl(%s) av_image_copy_to_buffer fail: %s", params_.name_.c_str(), AvErrStr(res_copy));
        // state_ = State::ERROR; // @todo ???
        return std::nullopt;
    } else if (res_copy != img_size) {
        WARNING(
            "VideoFrameDecoder(%s) av_image_copy_to_buffer size %d != %d", params_.name_.c_str(), res_copy, img_size);
        // state_ = State::ERROR; // @todo ???
        return std::nullopt;
    }

#  ifndef NDEBUG
    const double t_conv = tt.TocMs();
    TRACE("VideoFrameDecoderImpl(%s) t_send=%.1f t_recv=%.1f t_conv=%.1f total=%.1f", params_.name_.c_str(), t_send,
        t_recv, t_conv, t_send + t_recv + t_conv);
    img.t_dec_ = t_send + t_recv;
    img.t_conv_ = t_conv;
#  endif

    return img;
}

/* ****************************************************************************************************************** */

std::unique_ptr<VideoFrameDecoder> CreateVideoFrameDecoder(const VideoDecoderParams& params)
{
    // Some sanity checks..
    bool ok = true;

    switch (params.fmt_) {
        case PixelFmt::RGB24:
        case PixelFmt::Y8:
        case PixelFmt::GBRP:
            break;
        case PixelFmt::UNSPECIFIED:
            ok = false;
            break;
    }

    const AVCodec* codec = nullptr;
    switch (params.codec_) {  // clang-format off
        case VideoCodec::H264: codec = avcodec_find_decoder(AV_CODEC_ID_H264); break;
        case VideoCodec::H265: codec = avcodec_find_decoder(AV_CODEC_ID_H265); break;
        case VideoCodec::UNSPECIFIED: break;
    }  // clang-format on
    if (!codec) {
        ok = false;
    }

    switch (params.accel_) {
        case HwAccel::AUTO:
        case HwAccel::SW:
        case HwAccel::HW:
            break;
        case HwAccel::UNSPECIFIED:
            ok = false;
            break;
    }

    switch (params.qual_) {
        case ScalingQual::BILINEAR:
        case ScalingQual::BICUBIC:
        case ScalingQual::LANCZOS:
        case ScalingQual::SINC:
            break;
        case ScalingQual::UNSPECIFIED:
            ok = false;
            break;
    }

    if (!ok) {
        WARNING(
            "Bad params for VideoFrameDecoder(%s) %s", params.name_.c_str(), VideoDecoderParamsToStr(params).c_str());
        return nullptr;
    }

    return std::make_unique<VideoFrameDecoderImpl>(params);
}

#endif  // FPSDK_USE_FFMPEG
/* ****************************************************************************************************************** */
}  // namespace video
}  // namespace common
}  // namespace fpsdk
