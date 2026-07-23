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
 *
 * @page FPSDK_COMMON_CAM Camera types and utilities
 *
 * **API**: fpsdk_common/cam.hpp and fpsdk::common::cam
 *
 */
#ifndef __FPSDK_COMMON_CAM_HPP__
#define __FPSDK_COMMON_CAM_HPP__

/* LIBC/STL */
#include <cstdint>
#include <memory>
#include <vector>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/time.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief Camera types and utilities
 */
namespace cam {
/* ****************************************************************************************************************** */

/**
 * @brief Camera ID
 */
enum class CamId : uint8_t  // clang-format off
{
    UNSPECIFIED = 0,    //!< Unspecified
    CAM1        = 1,    //!< Camera 1
    CAM2        = 2,    //!< Camera 2
    CAM3        = 3,    //!< Camera 3
    CAM4        = 4,    //!< Camera 4
};  // clang-format on

/**
 * @brief Stringify camera ID enum
 *
 * @param[in]  camid  Camera ID
 *
 * @returns the stringification of the CameraId, or "?" for bad enum values
 */
const char* CamIdToStr(const CamId camid);

/**
 * @brief Convert camera ID string to enum
 *
 * @param[in]  str  Camera ID string
 * @param[in]  def  Default camera ID, for bad str
 *
 * @returns the camera ID, or def if str was invalid
 */
CamId CamIdFromStrOr(const char* str, const CamId def);

/**
 * @brief Convert camera ID value to enum
 *
 * @param[in]  val  Camera ID value
 * @param[in]  def  Default camera data type, for bad str
 *
 * @returns the camera data type, or def if str was invalid
 */
CamId CamIdFromValOr(const uint8_t val, const CamId def);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Camera data type (type of data)
 */
enum class CamDataType : uint8_t  // clang-format off
{
    UNSPECIFIED = 0,    //!< Unspecified
    HIRES_IMG   = 1,    //!< High-res image
    LORES_IMG   = 2,    //!< Low-res image
    HIRES_VID   = 3,    //!< High-res video
    LORES_VID   = 4,    //!< Low-res video
};  // clang-format on

/**
 * @brief Stringify camera data type enum
 *
 * @param[in]  type  Camera data type
 *
 * @returns the stringification of the camera data type, or "?" for bad enum values
 */
const char* CamDataTypeToStr(const CamDataType type);

/**
 * @brief Convert camera data type string to enum
 *
 * @param[in]  str  Camera data type string
 * @param[in]  def  Default camera data type, for bad str
 *
 * @returns the camera data type, or def if str was invalid
 */
CamDataType CamDataTypeFromStrOr(const char* str, const CamDataType def);

/**
 * @brief Convert camera data type value to enum
 *
 * @param[in]  val  Camera data type value
 * @param[in]  def  Default camera data type, for bad str
 *
 * @returns the camera data type, or def if str was invalid
 */
CamDataType CamDataTypeFromValOr(const uint8_t val, const CamDataType def);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Camera data format
 */
enum class CamDataFmt : uint8_t  // clang-format off
{
    UNSPECIFIED   = 0,    //!< Invalid
    // Video
    H264_NAL      = 1,    //!< H.264 NAL (AVC)    (video/avc)
    H265_NAL      = 2,    //!< H.265 NAL (HEVC)   (video/hevc)
    MJPEG         = 3,    //!< Motion JPEG        (image/jpeg)
    // Image
    JPEG          = 4,    //!< JPEG               (image/jpeg)
    Y8            = 5,    //!< Planar Y (greyscale), 8bpp
    NV12          = 6,    //!< Planar YUV 4:2:0, 12bpp
    RGB24         = 7,    //!< Packed RGB 8:8:8, 24bpp
};  // clang-format on

/**
 * @brief Stringify camera data format enum
 *
 * @param[in]  fmt  Camera data format
 *
 * @returns the stringification of the camera data format, or "?" for bad enum values
 */
const char* CamDataFmtToStr(const CamDataFmt fmt);

/**
 * @brief Convert camera data format string to enum
 *
 * @param[in]  str  Camera data format string
 * @param[in]  def  Default camera data format, for bad str
 *
 * @returns the camera data format, or def if str was invalid
 */
CamDataFmt CamDataFmtFromStrOr(const char* str, const CamDataFmt def);

/**
 * @brief Convert camera data format value to enum
 *
 * @param[in]  val  Camera data format value
 * @param[in]  def  Default camera data format, for bad str
 *
 * @returns the camera data format, or def if str was invalid
 */
CamDataFmt CamDataFmtFromValOr(const uint8_t val, const CamDataFmt def);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Camera frame type
 */
enum class CamDataFrm : uint8_t  // clang-format off
{
    UNSPECIFIED = 0,    //!< Unspecified, invalid data
    I_FRAME     = 1,    //!< I-frame (= Horizon camsys MC_H26[45]_NALU_TYPE_I)
    P_FRAME     = 2,    //!< P-frame (= Horizon camsys MC_H26[45]_NALU_TYPE_P)
    FULL        = 3,    //!< Full frame (non-encoded, e.g. NV12 or JPEG)
    OTHER       = 4,    //!< Other frame type (unexpected)
};  // clang-format on

/**
 * @brief Stringify camera data frame type enum
 *
 * @param[in]  frm  Camera data frame type
 *
 * @returns the stringification of the camera data frame type, or "?" for bad enum values
 */
const char* CamDataFrmToStr(const CamDataFrm frm);

/**
 * @brief Convert camera data frame type string to enum
 *
 * @param[in]  str  Camera data frame type string
 * @param[in]  def  Default camera data frame type, for bad str
 *
 * @returns the camera data frame type, or def if str was invalid
 */
CamDataFrm CamDataFrmFromStrOr(const char* str, const CamDataFrm def);

/**
 * @brief Convert camera data frame type value to enum
 *
 * @param[in]  val  Camera data frame type value
 * @param[in]  def  Default camera data frame type, for bad str
 *
 * @returns the camera data frame type, or def if str was invalid
 */
CamDataFrm CamDataFrmFromValOr(const uint8_t val, const CamDataFrm def);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Camera data from CamStream (or from FPL, see fpl::CamData)
 */
struct CamData
{
    // clang-format off
    bool         valid_   = false;                     //!< Data valid, successfully extracted from message
    CamId        cam_id_  = CamId::UNSPECIFIED;        //!< = meta_.cam_id_ as enum (if value in range)
    CamDataType  type_    = CamDataType::UNSPECIFIED;  //!< = meta_.type_ as enum (if value in range)
    CamDataFmt   fmt_     = CamDataFmt::UNSPECIFIED;   //!< = meta_.fmt_ as enum (if value in range)
    CamDataFrm   frm_     = CamDataFrm::UNSPECIFIED;   //!< = meta_.frm_ as enum (if value in range)
    uint64_t     seq_     = 0;                         //!< Sequence number
    uint64_t     ts_      = 0;                         //!< Camera image time (middle of exposure) [CLOCK_REALTIME ns] (0 = invalid)
    uint32_t     dt_      = 0;                         //!< Exposure time (duration) [ns] (0 = not available)
    uint32_t     width_   = 0;                         //!< Width of the frame [px]
    uint32_t     height_  = 0;                         //!< Height of the fram [px]
    std::vector<uint8_t> data_;                        //!< Image/frame data, contents depends on fmt_ etc.
    // clang-format on
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Camera stream parameters
 */
struct CamStreamParams
{
    std::string name_;                             //!< Name (for debug logging)
    std::string host_;                             //!< Host name or IP address
    CamId cam_id_ = CamId::UNSPECIFIED;            //!< Camera ID
    CamDataType type_ = CamDataType::UNSPECIFIED;  //!< Camera data type
    int rate_ = 1;                       //!< Rate (1 = every frame, 2 = every 2nd frame, ...), not for video streams
    static constexpr int RATE_MIN = 1;   //!< Min rate_
    static constexpr int RATE_MAX = 25;  //!< Max rate_
    time::Duration timeout_ = time::Duration::FromSec(1.0);  //!< Timeout receiving data (> 0.1s)
};

/**
 * @brief Camera streaming from from PBx-A1 sensor
 */
class CamStream
{
   public:
    /**
     * @brief Constructor
     *
     * Do not use. Use CreateCamStream() factory function.
     *
     * @param[in]  params  Parameters
     */
    CamStream(const CamStreamParams& params);

    /**
     * @brief Destructor
     */
    virtual ~CamStream();

    /**
     * @brief Connect stream
     *
     * @returns true if the stream was connected successfully, false otherwise (bad params, network problem, ...)
     */
    virtual bool Connect() = 0;

    /**
     * @brief Disconnect stream
     *
     * Call to explicitly disconnect. The destructor calls this automatically.
     */
    virtual void Disconnect() = 0;

    /**
     * @brief Get next frame
     *
     * @param[out]  data  Camera image/frame data
     *
     * @returns true if a next frame was received and data is filled (it may still be not valid_!),
     *          false on error (lost connection, bad data, ...)
     */
    virtual bool NextFrame(CamData& data) = 0;

   protected:
    CamStreamParams params_;  //!< Parameters
};

/**
 * @brief Pointer to a CamStream instance, see CreateCamStream()
 */
using CamStreamPtr = std::unique_ptr<CamStream>;

/**
 * @brief Create a camera stream instance
 *
 * @param[in]  params  The parameters
 *
 * @returns the video frame decoder instances, or nullptr if it failed (bad params)
 */
CamStreamPtr CreateCamStream(const CamStreamParams& params);

/* ****************************************************************************************************************** */
}  // namespace cam
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_CAM_HPP__
