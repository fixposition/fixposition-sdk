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

/* PACKAGE */
#include "fpsdk_common/cam.hpp"
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/types.hpp"

namespace fpsdk {
namespace common {
namespace cam {
/* ****************************************************************************************************************** */

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
    if (strcmp(str, "UNSPECIFIED") == 0) { return CamId::UNSPECIFIED; }
    if (strcmp(str, "CAM1")        == 0) { return CamId::CAM1; }
    if (strcmp(str, "CAM2")        == 0) { return CamId::CAM2; }
    if (strcmp(str, "CAM3")        == 0) { return CamId::CAM3; }
    if (strcmp(str, "CAM4")        == 0) { return CamId::CAM4; }
    return def;  // clang-format on
}

CamId CamIdFromValOr(const uint8_t val, const CamId def)
{  // clang-format off
    switch (val) {
        case types::EnumToVal(CamId::UNSPECIFIED): return CamId::UNSPECIFIED;
        case types::EnumToVal(CamId::CAM1):        return CamId::CAM1;
        case types::EnumToVal(CamId::CAM2):        return CamId::CAM2;
        case types::EnumToVal(CamId::CAM3):        return CamId::CAM3;
        case types::EnumToVal(CamId::CAM4):        return CamId::CAM4;
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
    if (strcmp(str, "UNSPECIFIED") == 0) { return CamDataType::UNSPECIFIED; }
    if (strcmp(str, "HIRES_IMG")   == 0) { return CamDataType::HIRES_IMG; }
    if (strcmp(str, "LORES_IMG")   == 0) { return CamDataType::LORES_IMG; }
    if (strcmp(str, "HIRES_VID")   == 0) { return CamDataType::HIRES_VID; }
    if (strcmp(str, "LORES_VID")   == 0) { return CamDataType::LORES_VID; }
    return def;  // clang-format on
}

CamDataType CamDataTypeFromValOr(const uint8_t val, const CamDataType def)
{  // clang-format off
    switch (val) {
        case types::EnumToVal(CamDataType::UNSPECIFIED): return CamDataType::UNSPECIFIED;
        case types::EnumToVal(CamDataType::HIRES_IMG):   return CamDataType::HIRES_IMG;
        case types::EnumToVal(CamDataType::LORES_IMG):   return CamDataType::LORES_IMG;
        case types::EnumToVal(CamDataType::HIRES_VID):   return CamDataType::HIRES_VID;
        case types::EnumToVal(CamDataType::LORES_VID):   return CamDataType::LORES_VID;
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

CamDataFmt CamDataFmtFromValOr(const uint8_t val, const CamDataFmt def)
{  // clang-format off
    switch (val) {
        case types::EnumToVal(CamDataFmt::UNSPECIFIED): return CamDataFmt::UNSPECIFIED;
        case types::EnumToVal(CamDataFmt::H264_NAL):    return CamDataFmt::H264_NAL;
        case types::EnumToVal(CamDataFmt::H265_NAL):    return CamDataFmt::H265_NAL;
        case types::EnumToVal(CamDataFmt::MJPEG):       return CamDataFmt::MJPEG;
        case types::EnumToVal(CamDataFmt::JPEG):        return CamDataFmt::JPEG;
        case types::EnumToVal(CamDataFmt::Y8):          return CamDataFmt::Y8;
        case types::EnumToVal(CamDataFmt::NV12):        return CamDataFmt::NV12;
        case types::EnumToVal(CamDataFmt::RGB24):       return CamDataFmt::RGB24;
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

CamDataFrm CamDataFrmFromValOr(const uint8_t val, const CamDataFrm def)
{  // clang-format off
    switch (val) {
        case types::EnumToVal(CamDataFrm::UNSPECIFIED): return CamDataFrm::UNSPECIFIED;
        case types::EnumToVal(CamDataFrm::I_FRAME):     return CamDataFrm::I_FRAME;
        case types::EnumToVal(CamDataFrm::P_FRAME):     return CamDataFrm::P_FRAME;
        case types::EnumToVal(CamDataFrm::FULL):        return CamDataFrm::FULL;
        case types::EnumToVal(CamDataFrm::OTHER):       return CamDataFrm::OTHER;
    }
    return def;  // clang-format on
}

// ---------------------------------------------------------------------------------------------------------------------

/* ****************************************************************************************************************** */
}  // namespace cam
}  // namespace common
}  // namespace fpsdk
