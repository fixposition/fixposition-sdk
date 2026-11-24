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
// clang-format off
/**
 * @page FPSDK_COMMON_VIDEO Video frame decoding
 *
 * **API**: fpsdk_common/video.hpp and fpsdk::common::video
 *
 * @note This is only available if compiled with FFmpeg, see @ref FPSDK_BUILD_DEPS.
 *
 * @section FPSDK_COMMON_VIDEO_HWACCEL Hardware vs. software decoding
 *
 * Hardware decoding (fpsdk::common::video::HwAccel::HW, currently VA-API) is not necessarily faster than software
 * decoding (fpsdk::common::video::HwAccel::SW). The following are wall-clock times for "fpltool extract" of a 32 s
 * recording with two cameras (H.265, 1024x768, 25 fps, 10 frames per GOP, 1547 frames in total) decoded to RGB24
 * (a 3.5 GB ROS1 bag), using different numbers of parallel decoders (-j). Measured on an AMD Ryzen 7 PRO 8840HS
 * (8 cores, 16 threads) with Radeon 780M graphics, FFmpeg 7.1, Release build, median of three runs:
 *
 * | Decoders | -e jsonl,raw,file,ros,cam -a SW | -e jsonl,raw,file,ros,cam -a HW | -e ros,cam -a SW | -e ros,cam -a HW |
 * |----------|---------------------------------|---------------------------------|------------------|------------------|
 * | -j 1     | 5.3 s                           | 5.9 s                           | 4.4 s            | 5.7 s            |
 * | -j 2     | 4.7 s                           | 5.8 s                           | 4.7 s            | 6.2 s            |
 * | -j 4     | 4.2 s                           | 6.1 s                           | 4.2 s            | 5.6 s            |
 * | -j 8     | 4.2 s                           | 6.5 s                           | 4.0 s            | 5.5 s            |
 *
 * The following are decoding rates (average frames per second, fps) measured with a test program that only decodes
 * (VideoFrameDecoder::DecodeFrame()) on the same machine, for the first 100 frames (10 I frames, 90 P frames) of a
 * camera recording (H.265, 1024x768) decoded to RGB24, 10 repetitions. In "indep" mode each of the -j decoders decodes all
 * frames (that is, -j times the work). In "gop" mode each GOP is decoded by a new decoder and the GOPs are distributed
 * to the -j decoder threads, which is what "fpltool extract" does (with 11 GOPs this mode stops scaling at about -j 5):
 *
 * | Decoders | indep -a SW | indep -a HW | gop -a SW | gop -a HW |
 * |----------|-------------|-------------|-----------|-----------|
 * | -j 1     | 324 fps     | 304 fps     | 286 fps   | 225 fps   |
 * | -j 2     | 521 fps     | 643 fps     | 476 fps   | 372 fps   |
 * | -j 4     | 789 fps     | 751 fps     | 679 fps   | 480 fps   |
 * | -j 8     | 1030 fps    | 820 fps     | 800 fps   | 534 fps   |
 *
 * Findings:
 *
 * - Software decoding is faster than hardware decoding in almost all cases. Only with 2 or 3 decoders running at the
 *   same time, each with its own continuous stream of frames, hardware decoding is slightly faster.
 * - Decoding this kind of video is cheap on the CPU (about 1.2-1.6 ms per P frame and 8-15 ms per I frame on one core,
 *   depending on the recording). It scales with the number of decoders, but not linearly: with 8 decoders the
 *   throughput is about 3 times that of one decoder, as each decoder gets slower (for example, 1.6 ms per P frame with
 *   one decoder, 4.8 ms with 8 decoders).
 * - The hardware decoder decodes I frames faster than software (about 3.6 ms), but P frames slower (about 3.0 ms). It
 *   is limited by getting the decoded frames back from the GPU, and the GPU does not scale with the number of decoders
 *   (the throughput stops increasing at about 800 fps with 3 decoders). The conversion to the output pixel format is
 *   done in software in both cases.
 * - The hardware device is set up when the first frame is decoded, which costs about 10 ms (up to about 40 ms when
 *   many decoders do this at the same time). This makes hardware decoding even slower when a new decoder is used for
 *   each GOP.
 * - The results may be different for other video (larger frames, H.264, longer GOPs), GPUs and drivers.
 */
// clang-format on
#ifndef __FPSDK_COMMON_VIDEO_HPP__
#define __FPSDK_COMMON_VIDEO_HPP__

/* LIBC/STL */
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Video frame decoding
 */
namespace video {
/* ****************************************************************************************************************** */
#if FPSDK_USE_FFMPEG || defined(_DOXYGEN_)

/**
 * @brief Video codec
 */
enum class VideoCodec : int
{
    UNSPECIFIED = 0,  //!< Unspecified
    H264,             //!< H.264 Advanced Video Coding (AVC)
    H265,             //!< H.265 High Efficiency Video Coding (HEVC)
};

/**
 * @brief Stringify video codec enum
 *
 * @param[in]  codec  Codec ID
 *
 * @returns the stringification of the codec, or "?" for bad enum values
 */
const char* VideoCodecToStr(const VideoCodec codec);

/**
 * @brief Convert codec ID string to enum
 *
 * @param[in]  str  Codec ID string
 * @param[in]  def  Default codec ID, for bad str
 *
 * @returns the codec ID, or def if str was invalid
 */
VideoCodec VideoCodecFromStrOr(const char* str, const VideoCodec def);

/**
 * @brief Pixel format
 */
enum class PixelFmt : int
{
    UNSPECIFIED = 0,  //!< Invalid
    Y8,               //!< Planar Y (greyscale), 8bpp         (= FFmpeg AV_PIX_FMT_GRAY8)
    RGB24,            //!< Packed RGB 8:8:8, 24bpp, RGBRGB... (= FFmpeg AV_PIX_FMT_RGB24)
    GBRP,  //!< Planar GBR, 24bpp  [G][G][G]...[B][B][B]...[R][R][R]... (~ FFmpeg AV_PIX_FMT_GBRP/AV_PIX_FMT_GBR24P)
};

/**
 * @brief Stringify pixel format enum
 *
 * @param[in]  fmt  Pixel format
 *
 * @returns the stringification of the fmt, or "?" for bad enum values
 */
const char* PixelFmtToStr(const PixelFmt fmt);

/**
 * @brief Convert pixel format string to enum
 *
 * @param[in]  str  Pixel format string
 * @param[in]  def  Default pixel format, for bad str
 *
 * @returns the pixel format, or def if str was invalid
 */
PixelFmt PixelFmtFromStrOr(const char* str, const PixelFmt def);

/**
 * @brief Scaling method (only for sw decoding)
 */
enum class ScalingQual : int
{
    UNSPECIFIED = 0,  //!< Unspecified
    BILINEAR,         //!< Low quality, very fast
    BICUBIC,          //!< Medium/high quality, fast (B=0.0, C=0.5, "Catmull-Rom")
    LANCZOS,          //!< High quality, slower (radius=3.0)
    SINC,             //!< Very high quality, very slow (radius=3.0)
};

/**
 * @brief Stringify scaling method enum
 *
 * @param[in]  qual  Scaling method
 *
 * @returns the stringification of the qual, or "?" for bad enum values
 */
const char* ScalingQualToStr(const ScalingQual qual);

/**
 * @brief Convert scaling method string to enum
 *
 * @param[in]  str  Scaling method string
 * @param[in]  def  Default pixel format, for bad str
 *
 * @returns the scaling method, or def if str was invalid
 */
ScalingQual ScalingQualFromStrOr(const char* str, const ScalingQual def);

/**
 * @brief Hw acceleration
 */
enum class HwAccel : uint8_t
{
    UNSPECIFIED = 0,  //!< Unspecified
    AUTO = 1,         //!< Use hw accel if possible, fallback to SW
    SW = 2,           //!< Do not use any hw acceleration, use pure software implementation
    HW = 3,           //!< Use hw acceleration (currently, Linux Video Acceleration API, VA-API)
};

/**
 * @brief Stringify hw accel method enum
 *
 * @param[in]  accel  HW acceleration
 *
 * @returns the stringification of the accel, or "?" for bad enum values
 */
const char* HwAccelToStr(const HwAccel accel);

/**
 * @brief Convert scaling method string to enum
 *
 * @param[in]  str  HW acceleration string
 * @param[in]  def  Default HW acceleration, for bad str
 *
 * @returns the HW acceleration, or def if str was invalid
 */
HwAccel HwAccelFromStrOr(const char* str, const HwAccel def);

/**
 * @brief VideoFrameDecoder params and their defaults
 */
struct VideoDecoderParams
{  // clang-format off
    std::string  name_;                                //!< Name (for debug logging)
    VideoCodec   codec_  = VideoCodec::UNSPECIFIED;    //!< Codec
    PixelFmt     fmt_    = PixelFmt::UNSPECIFIED;      //!< Pixel format (of decoded image)
    double       scale_  = 1.0;                        //!< Scale factor (of decoded image), range 0.1-1.0
    HwAccel      accel_  = HwAccel::AUTO;              //!< Hw acceleration
    ScalingQual  qual_   = ScalingQual::BICUBIC;       //!< Scaling method
};  // clang-format on

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Decoded, converted and scaled image
 */
struct ImageData
{
    PixelFmt fmt_ = PixelFmt::UNSPECIFIED;  //!< Pixel format (of data_)
    int width_ = 0;                         //!< Width [px]
    int height_ = 0;                        //!< Height [px]
    std::vector<uint8_t> data_;             //!< Image data, memory layout depends on fmt_

#  ifndef NDEBUG
    double t_dec_ = 0.0;
    double t_conv_ = 0.0;
#  endif
};

/**
 * @brief Helper for decoding video frames
 *
 * @note This is only available if compiled with FFmpeg, see @ref FPSDK_BUILD_DEPS.
 */
class VideoFrameDecoder
{
   public:
    /**
     * @brief Constructor
     *
     * Don't use, use the CreateVideoFrameDecoder() factory function.
     *
     * @param[in]  params  The parameters
     */
    explicit VideoFrameDecoder(const VideoDecoderParams& params);

    /**
     * @brief Destructor
     */
    virtual ~VideoFrameDecoder();

    /**
     * @brief Decode one frame
     *
     * - This works for data that is available frame by frame. That is, it does not work for regular video streams where
     *   the data is chunked arbitrarily. Ideally, start by supplying an I frame incl. all the necessary NAL units
     *   (NALUs) to decode a first frame. After that, the following P frames should decode fine.
     * - For example, for HEVC (H.265) a full set of data for a I frame should contain:
     *   - NALU type 32 (VPS, Video Parameter Set)
     *   - NALU type 33 (SPS, Sequence Parameter Set)
     *   - NALU type 34 (PPS, Picture Parameter Set)
     *   - NALU type 19 (IDR, Instantaneous Decoding Refresh, and RADL, Random Access Decodable Leading) (possibly other
     *     types 16-23 (IRAP, Intra Random Access Point) NALUs might work)
     * - The video encoder should be configured accordingly (to repeat the VPS/SPS/PPS for each I-frame, and possibly to
     *   produce all/only I frames.
     * - Separate instances of this should be used to process different video streams
     *
     * @param[in]  data  Data for one (not more, not less) video frame
     * @param[in]  size  Size of data
     *
     * @returns the decoded, converted and scaled image, nullptr otherwise (not enough data, bad data, ...)
     */
    virtual std::optional<ImageData> DecodeFrame(const uint8_t* data, const std::size_t size) = 0;

    /**
     * @brief Decode video data
     *
     * @param[in]  data   Data for one (recommended) or more video frame(s)
     *
     * @returns the decoded, converted and scaled image, nullptr otherwise (not enough data, bad data, ...)
     */
    std::optional<ImageData> DecodeFrame(const std::vector<uint8_t>& data);

    /**
     * @brief Check if decoder is in error state
     *
     * This is useful to check when DecodeFrame() didn't return a frame. If the decoder is in error state, it cannot
     * be used anymore and must be discarded.
     *
     * @returns true if the decoder is okay, false if it is in error state
     */
    virtual bool IsOkay() const = 0;

   protected:
    VideoDecoderParams params_;  //!< Params
};

/**
 * @brief Pointer to a VideoFrameDecoder instance, see CreateVideoFrameDecoder()
 */
using VideoFrameDecoderPtr = std::unique_ptr<VideoFrameDecoder>;

/**
 * @brief Create a video frame decoder
 *
 * @param[in]  params  The parameters
 *
 * @returns the video frame decoder instances, or nullptr if it failed (bad params)
 */
VideoFrameDecoderPtr CreateVideoFrameDecoder(const VideoDecoderParams& params);

#endif  // FPSDK_USE_FFMPEG
/* ****************************************************************************************************************** */
}  // namespace video
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_VIDEO_HPP__
