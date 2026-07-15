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

/* EXTERNAL */

/* PACKAGE */

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
 * @param[in]  def  Default Camer ID, for bad str
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
 * @brief Convert camera data frame type value to enum
 *
 * @param[in]  val  Camera data frame type value
 * @param[in]  def  Default camera data frame type, for bad str
 *
 * @returns the camera data frame type, or def if str was invalid
 */
CamDataFrm CamDataFrmFromValOr(const uint8_t val, const CamDataFrm def);

/* ****************************************************************************************************************** */
}  // namespace cam
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_CAM_HPP__
