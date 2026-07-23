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
 *
 * @page FPSDK_COMMON_VIDEO Video frame decoding
 *
 * **API**: fpsdk_common/video.hpp and fpsdk::common::video
 *
 */
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
#if FPSDK_USE_FFMPEG

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
    GBRP,  //!< Planar GBR, 24bpp  [R][R][R]...[G][G][G]...[B][B][B]... (~ FFmpeg AV_PIX_FMT_GBRP/AV_PIX_FMT_GBR24P)
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
 * Note that separate instances of this should be used to process different video streams.
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
    VideoFrameDecoder(const VideoDecoderParams& params);

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
