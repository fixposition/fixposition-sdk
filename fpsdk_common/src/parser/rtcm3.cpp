/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Based on work by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser RTCM3 routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstring>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/math.hpp"
#include "fpsdk_common/parser/rtcm3.hpp"
#include "fpsdk_common/trafo.hpp"
#include "fpsdk_common/types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace rtcm3 {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint16_t type_;
    uint16_t subtype_;
    const char* name_;
    const char* desc_;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_RTCM3_MSGINFO}
static constexpr std::array<MsgInfo, 119> MSG_INFO =
{{
    { RTCM3_TYPE1001_MSGID, 0, RTCM3_TYPE1001_STRID, "L1-only GPS RTK observables" },
    { RTCM3_TYPE1002_MSGID, 0, RTCM3_TYPE1002_STRID, "Extended L1-only GPS RTK observables" },
    { RTCM3_TYPE1003_MSGID, 0, RTCM3_TYPE1003_STRID, "L1/L2 GPS RTK observables" },
    { RTCM3_TYPE1004_MSGID, 0, RTCM3_TYPE1004_STRID, "Extended L1/L2 GPS RTK observables" },
    { RTCM3_TYPE1005_MSGID, 0, RTCM3_TYPE1005_STRID, "Stationary RTK reference station ARP" },
    { RTCM3_TYPE1006_MSGID, 0, RTCM3_TYPE1006_STRID, "Stationary RTK reference station ARP with antenna height" },
    { RTCM3_TYPE1007_MSGID, 0, RTCM3_TYPE1007_STRID, "Antenna descriptor" },
    { RTCM3_TYPE1008_MSGID, 0, RTCM3_TYPE1008_STRID, "Antenna descriptor and serial number" },
    { RTCM3_TYPE1009_MSGID, 0, RTCM3_TYPE1009_STRID, "L1-only GLONASS RTK observables" },
    { RTCM3_TYPE1010_MSGID, 0, RTCM3_TYPE1010_STRID, "Extended L1-only GLONASS RTK observables" },
    { RTCM3_TYPE1011_MSGID, 0, RTCM3_TYPE1011_STRID, "L1/L2 GLONASS RTK observables" },
    { RTCM3_TYPE1012_MSGID, 0, RTCM3_TYPE1012_STRID, "Extended L1/L2 GLONASS RTK observables" },
    { RTCM3_TYPE1030_MSGID, 0, RTCM3_TYPE1030_STRID, "GPS network RTK residual message" },
    { RTCM3_TYPE1031_MSGID, 0, RTCM3_TYPE1031_STRID, "GLONASS network RTK residual message" },
    { RTCM3_TYPE1032_MSGID, 0, RTCM3_TYPE1032_STRID, "Physical reference station position message" },
    { RTCM3_TYPE1033_MSGID, 0, RTCM3_TYPE1033_STRID, "Receiver and antenna descriptors" },
    { RTCM3_TYPE1230_MSGID, 0, RTCM3_TYPE1230_STRID, "GLONASS code-phase biases" },
    { RTCM3_TYPE1071_MSGID, 0, RTCM3_TYPE1071_STRID, "GPS MSM1 (C)" },
    { RTCM3_TYPE1072_MSGID, 0, RTCM3_TYPE1072_STRID, "GPS MSM2 (L)" },
    { RTCM3_TYPE1073_MSGID, 0, RTCM3_TYPE1073_STRID, "GPS MSM3 (C, L)" },
    { RTCM3_TYPE1074_MSGID, 0, RTCM3_TYPE1074_STRID, "GPS MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1075_MSGID, 0, RTCM3_TYPE1075_STRID, "GPS MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1076_MSGID, 0, RTCM3_TYPE1076_STRID, "GPS MSM6 (ext full C, ext full L, S)" },
    { RTCM3_TYPE1077_MSGID, 0, RTCM3_TYPE1077_STRID, "GPS MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1081_MSGID, 0, RTCM3_TYPE1081_STRID, "GLONASS MSM1 (C)" },
    { RTCM3_TYPE1082_MSGID, 0, RTCM3_TYPE1082_STRID, "GLONASS MSM2 (L)" },
    { RTCM3_TYPE1083_MSGID, 0, RTCM3_TYPE1083_STRID, "GLONASS MSM3 (C, L)" },
    { RTCM3_TYPE1084_MSGID, 0, RTCM3_TYPE1084_STRID, "GLONASS MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1085_MSGID, 0, RTCM3_TYPE1085_STRID, "GLONASS MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1086_MSGID, 0, RTCM3_TYPE1086_STRID, "GLONASS MSM6 (ext full C, ext full L, S)" },
    { RTCM3_TYPE1087_MSGID, 0, RTCM3_TYPE1087_STRID, "GLONASS MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1091_MSGID, 0, RTCM3_TYPE1091_STRID, "Galileo MSM1 (C)" },
    { RTCM3_TYPE1092_MSGID, 0, RTCM3_TYPE1092_STRID, "Galileo MSM2 (L)" },
    { RTCM3_TYPE1093_MSGID, 0, RTCM3_TYPE1093_STRID, "Galileo MSM3 (C, L)" },
    { RTCM3_TYPE1094_MSGID, 0, RTCM3_TYPE1094_STRID, "Galileo MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1095_MSGID, 0, RTCM3_TYPE1095_STRID, "Galileo MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1096_MSGID, 0, RTCM3_TYPE1096_STRID, "Galileo MSM6 (full C, full L, S)" },
    { RTCM3_TYPE1097_MSGID, 0, RTCM3_TYPE1097_STRID, "Galileo MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1101_MSGID, 0, RTCM3_TYPE1101_STRID, "SBAS MSM1 (C)" },
    { RTCM3_TYPE1102_MSGID, 0, RTCM3_TYPE1102_STRID, "SBAS MSM2 (L)" },
    { RTCM3_TYPE1103_MSGID, 0, RTCM3_TYPE1103_STRID, "SBAS MSM3 (C, L)" },
    { RTCM3_TYPE1104_MSGID, 0, RTCM3_TYPE1104_STRID, "SBAS MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1105_MSGID, 0, RTCM3_TYPE1105_STRID, "SBAS MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1106_MSGID, 0, RTCM3_TYPE1106_STRID, "SBAS MSM6 (full C, full L, S)" },
    { RTCM3_TYPE1107_MSGID, 0, RTCM3_TYPE1107_STRID, "SBAS MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1111_MSGID, 0, RTCM3_TYPE1111_STRID, "QZSS MSM1 (C)" },
    { RTCM3_TYPE1112_MSGID, 0, RTCM3_TYPE1112_STRID, "QZSS MSM2 (L)" },
    { RTCM3_TYPE1113_MSGID, 0, RTCM3_TYPE1113_STRID, "QZSS MSM3 (C, L)" },
    { RTCM3_TYPE1114_MSGID, 0, RTCM3_TYPE1114_STRID, "QZSS MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1115_MSGID, 0, RTCM3_TYPE1115_STRID, "QZSS MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1116_MSGID, 0, RTCM3_TYPE1116_STRID, "QZSS MSM6 (full C, full L, S)" },
    { RTCM3_TYPE1117_MSGID, 0, RTCM3_TYPE1117_STRID, "QZSS MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1121_MSGID, 0, RTCM3_TYPE1121_STRID, "BeiDou MSM1 (C)" },
    { RTCM3_TYPE1122_MSGID, 0, RTCM3_TYPE1122_STRID, "BeiDou MSM2 (L)" },
    { RTCM3_TYPE1123_MSGID, 0, RTCM3_TYPE1123_STRID, "BeiDou MSM3 (C, L)" },
    { RTCM3_TYPE1124_MSGID, 0, RTCM3_TYPE1124_STRID, "BeiDou MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1125_MSGID, 0, RTCM3_TYPE1125_STRID, "BeiDou MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1126_MSGID, 0, RTCM3_TYPE1126_STRID, "BeiDou MSM6 (full C, full L, S)" },
    { RTCM3_TYPE1127_MSGID, 0, RTCM3_TYPE1127_STRID, "BeiDou MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1131_MSGID, 0, RTCM3_TYPE1131_STRID, "NavIC MSM1 (C)" },
    { RTCM3_TYPE1132_MSGID, 0, RTCM3_TYPE1132_STRID, "NavIC MSM2 (L)" },
    { RTCM3_TYPE1133_MSGID, 0, RTCM3_TYPE1133_STRID, "NavIC MSM3 (C, L)" },
    { RTCM3_TYPE1134_MSGID, 0, RTCM3_TYPE1134_STRID, "NavIC MSM4 (full C, full L, S)" },
    { RTCM3_TYPE1135_MSGID, 0, RTCM3_TYPE1135_STRID, "NavIC MSM5 (full C, full L, S, D)" },
    { RTCM3_TYPE1136_MSGID, 0, RTCM3_TYPE1136_STRID, "NavIC MSM6 (full C, full L, S)" },
    { RTCM3_TYPE1137_MSGID, 0, RTCM3_TYPE1137_STRID, "NavIC MSM7 (ext full C, ext full L, S, D)" },
    { RTCM3_TYPE1019_MSGID, 0, RTCM3_TYPE1019_STRID, "GPS ephemeris data" },
    { RTCM3_TYPE1020_MSGID, 0, RTCM3_TYPE1020_STRID, "GLONASS ephemeris data" },
    { RTCM3_TYPE1041_MSGID, 0, RTCM3_TYPE1041_STRID, "NavIC satellite ephemeris data" },
    { RTCM3_TYPE1042_MSGID, 0, RTCM3_TYPE1042_STRID, "BeiDou satellite ephemeris data" },
    { RTCM3_TYPE1044_MSGID, 0, RTCM3_TYPE1044_STRID, "QZSS ephemeris data" },
    { RTCM3_TYPE1045_MSGID, 0, RTCM3_TYPE1045_STRID, "Galileo F/NAV satellite ephemeris data" },
    { RTCM3_TYPE1046_MSGID, 0, RTCM3_TYPE1046_STRID, "Galileo I/NAV satellite ephemeris data" },
    { RTCM3_TYPE4050_MSGID, 0, RTCM3_TYPE4050_STRID, "STMicroelectronics proprietary" },
    { RTCM3_TYPE4051_MSGID, 0, RTCM3_TYPE4051_STRID, "Hi-Target proprietary" },
    { RTCM3_TYPE4052_MSGID, 0, RTCM3_TYPE4052_STRID, "Furuno proprietary" },
    { RTCM3_TYPE4053_MSGID, 0, RTCM3_TYPE4053_STRID, "Qualcomm proprietary" },
    { RTCM3_TYPE4054_MSGID, 0, RTCM3_TYPE4054_STRID, "Geodnet proprietary" },
    { RTCM3_TYPE4055_MSGID, 0, RTCM3_TYPE4055_STRID, "KRISO proprietary" },
    { RTCM3_TYPE4056_MSGID, 0, RTCM3_TYPE4056_STRID, "Dayou proprietary" },
    { RTCM3_TYPE4057_MSGID, 0, RTCM3_TYPE4057_STRID, "Sixents proprietary" },
    { RTCM3_TYPE4058_MSGID, 0, RTCM3_TYPE4058_STRID, "Anello proprietary" },
    { RTCM3_TYPE4059_MSGID, 0, RTCM3_TYPE4059_STRID, "NRCan proprietary" },
    { RTCM3_TYPE4060_MSGID, 0, RTCM3_TYPE4060_STRID, "ALES proprietary" },
    { RTCM3_TYPE4061_MSGID, 0, RTCM3_TYPE4061_STRID, "Geely proprietary" },
    { RTCM3_TYPE4062_MSGID, 0, RTCM3_TYPE4062_STRID, "SwiftNav proprietary" },
    { RTCM3_TYPE4063_MSGID, 0, RTCM3_TYPE4063_STRID, "CHCNAV proprietary" },
    { RTCM3_TYPE4064_MSGID, 0, RTCM3_TYPE4064_STRID, "NTLab proprietary" },
    { RTCM3_TYPE4065_MSGID, 0, RTCM3_TYPE4065_STRID, "Allystar proprietary" },
    { RTCM3_TYPE4066_MSGID, 0, RTCM3_TYPE4066_STRID, "Lantmateriet proprietary" },
    { RTCM3_TYPE4067_MSGID, 0, RTCM3_TYPE4067_STRID, "CIPPE proprietary" },
    { RTCM3_TYPE4068_MSGID, 0, RTCM3_TYPE4068_STRID, "Qianxun proprietary" },
    { RTCM3_TYPE4069_MSGID, 0, RTCM3_TYPE4069_STRID, "Veripos proprietary" },
    { RTCM3_TYPE4070_MSGID, 0, RTCM3_TYPE4070_STRID, "Wuhan MengXin proprietary" },
    { RTCM3_TYPE4071_MSGID, 0, RTCM3_TYPE4071_STRID, "Wuhan Navigation proprietary" },
    { RTCM3_TYPE4072_MSGID, 0, RTCM3_TYPE4072_STRID, "u-blox proprietary" },
    { RTCM3_TYPE4073_MSGID, 0, RTCM3_TYPE4073_STRID, "Mitsubishi proprietary" },
    { RTCM3_TYPE4074_MSGID, 0, RTCM3_TYPE4074_STRID, "Unicore proprietary" },
    { RTCM3_TYPE4075_MSGID, 0, RTCM3_TYPE4075_STRID, "Alberding proprietary" },
    { RTCM3_TYPE4076_MSGID, 0, RTCM3_TYPE4076_STRID, "IGS proprietary" },
    { RTCM3_TYPE4077_MSGID, 0, RTCM3_TYPE4077_STRID, "Hemisphere proprietary" },
    { RTCM3_TYPE4078_MSGID, 0, RTCM3_TYPE4078_STRID, "ComNav proprietary" },
    { RTCM3_TYPE4079_MSGID, 0, RTCM3_TYPE4079_STRID, "SubCarrier proprietary" },
    { RTCM3_TYPE4080_MSGID, 0, RTCM3_TYPE4080_STRID, "NavCom proprietary" },
    { RTCM3_TYPE4081_MSGID, 0, RTCM3_TYPE4081_STRID, "SNU GNSS proprietary" },
    { RTCM3_TYPE4082_MSGID, 0, RTCM3_TYPE4082_STRID, "CRCSI proprietary" },
    { RTCM3_TYPE4083_MSGID, 0, RTCM3_TYPE4083_STRID, "DLR proprietary" },
    { RTCM3_TYPE4084_MSGID, 0, RTCM3_TYPE4084_STRID, "Geodetics, Inc proprietary" },
    { RTCM3_TYPE4085_MSGID, 0, RTCM3_TYPE4085_STRID, "EUSPA proprietary" },
    { RTCM3_TYPE4086_MSGID, 0, RTCM3_TYPE4086_STRID, "inPosition proprietary" },
    { RTCM3_TYPE4087_MSGID, 0, RTCM3_TYPE4087_STRID, "Fugro proprietary" },
    { RTCM3_TYPE4088_MSGID, 0, RTCM3_TYPE4088_STRID, "IfEN proprietary" },
    { RTCM3_TYPE4089_MSGID, 0, RTCM3_TYPE4089_STRID, "Septentrio proprietary" },
    { RTCM3_TYPE4090_MSGID, 0, RTCM3_TYPE4090_STRID, "Geo++ proprietary" },
    { RTCM3_TYPE4091_MSGID, 0, RTCM3_TYPE4091_STRID, "Topcon proprietary" },
    { RTCM3_TYPE4092_MSGID, 0, RTCM3_TYPE4092_STRID, "Leica proprietary" },
    { RTCM3_TYPE4093_MSGID, 0, RTCM3_TYPE4093_STRID, "NovAtel proprietary" },
    { RTCM3_TYPE4094_MSGID, 0, RTCM3_TYPE4094_STRID, "Trimble proprietary" },
    { RTCM3_TYPE4095_MSGID, 0, RTCM3_TYPE4095_STRID, "Ashtech proprietary" },
}};
static constexpr std::array<MsgInfo, 2> SUB_INFO =
{{
    { RTCM3_TYPE4072_MSGID, RTCM3_TYPE4072_0_SUBID, RTCM3_TYPE4072_0_STRID, "u-blox proprietary: Reference station PVT" },
    { RTCM3_TYPE4072_MSGID, RTCM3_TYPE4072_1_SUBID, RTCM3_TYPE4072_1_STRID, "u-blox proprietary: Additional reference station information" },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_RTCM3_MSGINFO}
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

bool Rtcm3GetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    if ((msg == NULL) || (msg_size < (RTCM3_FRAME_SIZE + 3))) {
        return false;
    }

    const uint16_t type = Rtcm3Type(msg);

    // Try sub-type lookup table for proprietary messages
    if ((type >= 4001) && (type <= 4095)) {
        const uint16_t subtype = Rtcm3SubType(msg);  // maybe..
        for (auto& info : SUB_INFO) {
            if ((info.type_ == type) && (info.subtype_ == subtype)) {
                return std::snprintf(name, size, "%s", info.name_) < (int)size;
            }
        }
    }

    // Try the message name lookup table
    for (auto& info : MSG_INFO) {
        if (info.type_ == type) {
            return std::snprintf(name, size, "%s", info.name_) < (int)size;
        }
    }

    return std::snprintf(name, size, "RTCM3-TYPE%" PRIu16, type) < (int)size;
}

// ---------------------------------------------------------------------------------------------------------------------

const char* Rtcm3GetTypeDesc(const uint16_t type, const uint16_t subtype)
{
    if ((type >= 4001) && (type <= 4095)) {
        for (auto& info : SUB_INFO) {
            if ((info.type_ == type) && (info.subtype_ == subtype)) {
                return info.desc_;
            }
        }
    }
    for (auto& info : MSG_INFO) {
        if (info.type_ == type) {
            return info.desc_;
        }
    }
    return NULL;
}

// ---------------------------------------------------------------------------------------------------------------------

uint64_t Rtcm3GetUnsigned(const uint8_t* data, const std::size_t offs, const std::size_t size)
{
    uint64_t val = 0;
    if ((data != NULL) && (size > 0)) {
        for (std::size_t bo = 0; bo < size; bo++) {
            val <<= 1;
            const std::size_t bit = offs + bo;
            val |= (data[bit / 8] >> (7 - (bit % 8))) & 0x01;
        }
    }
    return val;
}

// ---------------------------------------------------------------------------------------------------------------------

int64_t Rtcm3GetSigned(const uint8_t* data, const std::size_t offs, const std::size_t size)
{
    uint64_t val = 0;
    if ((data != NULL) && (size > 0)) {
        for (std::size_t bo = 0; bo < size; bo++) {
            val <<= 1;
            const std::size_t bit = offs + bo;
            val |= (data[bit / 8] >> (7 - (bit % 8))) & 0x01;
        }
        // Sign-extend to full 64 bits
        if ((val & ((uint64_t)1 << (size - 1))) != 0) {
            for (std::size_t bo = size; bo < 64; bo++) {
                val |= (uint64_t)1 << bo;
            }
        }
    }
    return (int64_t)val;
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t Rtcm3CountBits(const uint64_t mask)
{
    std::size_t cnt = 0;
    for (uint64_t m = mask; m != 0; m >>= 1) {
        if ((m & 0x1) != 0) {
            cnt++;
        }
    }
    return cnt;
}

// ---------------------------------------------------------------------------------------------------------------------

void Rtcm3SetUnsigned(uint8_t* data, const std::size_t offs, const std::size_t size, const uint64_t value)
{
    if ((data != NULL) && (size > 0) && (size <= 64)) {
        uint64_t mask = math::Bit<uint64_t>(size - 1);
        for (std::size_t ix = offs; ix < (offs + size); ix++) {
            const uint8_t bit = math::Bit<uint8_t>(7 - (ix % 8));
            if (math::CheckBitsAll(value, mask)) {
                math::SetBits(data[ix / 8], bit);
            } else {
                math::ClearBits(data[ix / 8], bit);
            }
            mask >>= 1;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void Rtcm3SetSigned(uint8_t* data, const std::size_t offs, const std::size_t size, const int64_t value)
{
    Rtcm3SetUnsigned(data, offs, size, static_cast<uint64_t>(value));
}

// ---------------------------------------------------------------------------------------------------------------------

bool Rtcm3GetArp(const uint8_t* msg, Rtcm3Arp& arp)
{
    bool ok = false;
    if (msg != NULL) {
        const uint8_t* data = &msg[RTCM3_HEAD_SIZE];
        const uint16_t msg_type = Rtcm3Type(msg);
        switch (msg_type) {  // clang-format off
            case RTCM3_TYPE1006_MSGID:
                arp.ant_height_ = (double)Rtcm3GetUnsigned(data, 34, 38) * 0.0001; // DF028
                /* FALLTHROUGH */
            case RTCM3_TYPE1005_MSGID:
                arp.ref_sta_id_ =         Rtcm3GetUnsigned(data,  12, 12);          // DF003
                arp.gps_ind_    =         Rtcm3GetUnsigned(data,  30,  1);          // DF022
                arp.glo_ind_    =         Rtcm3GetUnsigned(data,  31,  1);          // DF023
                arp.gal_ind_    =         Rtcm3GetUnsigned(data,  32,  1);          // DF024
                arp.ref_ind_    =         Rtcm3GetUnsigned(data,  33,  1);          // DF141
                arp.ecef_x_     = (double)Rtcm3GetSigned(  data,  34, 38) * 0.0001; // DF025
                arp.osc_ind_    =         Rtcm3GetUnsigned(data,  72,  1);          // DF142
                arp.ecef_y_     = (double)Rtcm3GetSigned(  data,  74, 38) * 0.0001; // DF026
                arp.cyc_ind_    =         Rtcm3GetUnsigned(data, 112,  2);          // DF142
                arp.ecef_z_     = (double)Rtcm3GetSigned(  data, 114, 38) * 0.0001; // DF027
                ok = true;
                break;
            case RTCM3_TYPE1032_MSGID:
                arp.ref_sta_id_ =         Rtcm3GetUnsigned(data,  12, 12);          // DF003
                arp.ecef_x_     = (double)Rtcm3GetSigned(  data,  42, 38) * 0.0001; // DF025
                arp.ecef_y_     = (double)Rtcm3GetSigned(  data,  80, 38) * 0.0001; // DF026
                arp.ecef_z_     = (double)Rtcm3GetSigned(  data, 118, 38) * 0.0001; // DF027
                ok = true;
                break;
        }  // clang-format on
    }
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Rtcm3GetAnt(const uint8_t* msg, Rtcm3Ant& ant)
{
    bool ok = false;
    if (msg != NULL) {
        const uint8_t* data = &msg[RTCM3_HEAD_SIZE];
        const uint16_t msg_type = Rtcm3Type(msg);
        if ((msg_type == RTCM3_TYPE1007_MSGID) || (msg_type == RTCM3_TYPE1008_MSGID) ||
            (msg_type == RTCM3_TYPE1033_MSGID)) {
            std::memset(&ant, 0, sizeof(ant));

            int offs = 12;
            ant.ref_sta_id_ = Rtcm3GetUnsigned(data, offs, 12);  // DF003
            offs += 12;
            const int n = Rtcm3GetUnsigned(data, offs, 8);  // DF029
            offs += 8;
            for (int ix = 0; (ix < n) && (ix < ((int)sizeof(ant.ant_desc_) - 1)); ix++) {
                ant.ant_desc_[ix] = Rtcm3GetUnsigned(data, offs, 8);  // DF030
                offs += 8;
            }
            ant.ant_setup_id_ = Rtcm3GetUnsigned(data, offs, 8);  // DF031
            offs += 8;
            if ((msg_type == RTCM3_TYPE1008_MSGID) || (msg_type == RTCM3_TYPE1033_MSGID)) {
                const int m = Rtcm3GetUnsigned(data, offs, 8);  // DF032
                offs += 8;
                for (int ix = 0; (ix < m) && (ix < ((int)sizeof(ant.ant_serial_) - 1)); ix++) {
                    ant.ant_serial_[ix] = Rtcm3GetUnsigned(data, offs, 8);  // DF033
                    offs += 8;
                }
            }

            if (msg_type == RTCM3_TYPE1033_MSGID) {
                const int i = Rtcm3GetUnsigned(data, offs, 8);  // DF227
                offs += 8;
                for (int ix = 0; (ix < i) && (ix < ((int)sizeof(ant.rx_type_) - 1)); ix++) {
                    ant.rx_type_[ix] = Rtcm3GetUnsigned(data, offs, 8);  // DF228
                    offs += 8;
                }
                const int j = Rtcm3GetUnsigned(data, offs, 8);  // DF229
                offs += 8;
                for (int ix = 0; (ix < j) && (ix < ((int)sizeof(ant.rx_fw_) - 1)); ix++) {
                    ant.rx_fw_[ix] = Rtcm3GetUnsigned(data, offs, 8);  // DF230
                    offs += 8;
                }
                const int k = Rtcm3GetUnsigned(data, offs, 8);  // DF231
                offs += 8;
                for (int ix = 0; (ix < k) && (ix < ((int)sizeof(ant.rx_serial_) - 1)); ix++) {
                    ant.rx_serial_[ix] = Rtcm3GetUnsigned(data, offs, 8);  // DF232
                    offs += 8;
                }
            }

            ok = true;
        }
    }
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Rtcm3TypeToMsm(uint16_t msg_type, Rtcm3MsmGnss& gnss, Rtcm3MsmType& msm)
{
    const uint16_t gnss_val = ((msg_type - 1000) / 10) * 10;
    const uint16_t msm_val = msg_type % 10;
    if (
        // MSM1-7
        (msm_val >= types::EnumToVal(Rtcm3MsmType::MSM1)) && (msm_val <= types::EnumToVal(Rtcm3MsmType::MSM7)) &&
        // One of the known GNSS
        ((gnss_val == types::EnumToVal(Rtcm3MsmGnss::GPS)) || (gnss_val == types::EnumToVal(Rtcm3MsmGnss::GLO)) ||
            (gnss_val == types::EnumToVal(Rtcm3MsmGnss::GAL)) || (gnss_val == types::EnumToVal(Rtcm3MsmGnss::SBAS)) ||
            (gnss_val == types::EnumToVal(Rtcm3MsmGnss::QZSS)) || (gnss_val == types::EnumToVal(Rtcm3MsmGnss::BDS)) ||
            (gnss_val == types::EnumToVal(Rtcm3MsmGnss::NAVIC)))) {
        msm = (Rtcm3MsmType)msm_val;
        gnss = (Rtcm3MsmGnss)gnss_val;
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool Rtcm3GetMsmHeader(const uint8_t* msg, Rtcm3MsmHeader& header)
{
    if (msg == NULL) {
        return false;
    }
    const uint16_t msg_type = Rtcm3Type(msg);
    if (!Rtcm3TypeToMsm(msg_type, header.gnss_, header.msm_)) {
        return false;  // Not an MSM message
    }
    header.msg_type_ = msg_type;

    const uint8_t* data = &msg[RTCM3_HEAD_SIZE];
    header.ref_sta_id_ = Rtcm3GetUnsigned(data, 12, 12);  // DF003
    if (header.gnss_ == Rtcm3MsmGnss::GLO) {
        const int dow = Rtcm3GetUnsigned(data, 24, 3);   // DF416
        const int tod = Rtcm3GetUnsigned(data, 27, 27);  // DF034
        header.glo_tow_ = ((double)dow * 86400.0) + ((double)tod * 1e-3);
    } else {
        header.any_tow_ = (double)Rtcm3GetUnsigned(data, 24, 30) * 1e-3;  // DF004, DF416, DF248, DF427
    }
    // clang-format off
    header.multi_msg_bit_ = Rtcm3GetUnsigned(data,  54,  1);    // DF393
    header.iods_          = Rtcm3GetUnsigned(data,  55,  3);    // DF409
    // bit(7) reserved // DF001
    header.clk_steering_  = Rtcm3GetUnsigned(data,  65,  2);    // DF411
    header.ext_clock_     = Rtcm3GetUnsigned(data,  67,  2);    // DF412
    header.smooth_        = Rtcm3GetUnsigned(data,  69,  1);    // DF417
    header.smooth_int_    = Rtcm3GetUnsigned(data,  70,  3);    // DF418
    header.sat_mask_      = Rtcm3GetUnsigned(data,  73,  64);   // DF394
    header.sig_mask_      = Rtcm3GetUnsigned(data, 137, 32);    // DF395
    header.num_sat_       = Rtcm3CountBits(header.sat_mask_);
    header.num_sig_       = Rtcm3CountBits(header.sig_mask_);
    header.num_cell_      = header.num_sat_ * header.num_sig_;
    header.cell_mask_     = Rtcm3GetUnsigned(data, 169, header.num_cell_);  // DF396
    // clang-format on

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Rtcm3GetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }

    if ((msg == NULL) || (msg_size < (RTCM3_HEAD_SIZE + 2))) {
        info[0] = '\0';
        return false;
    }
    std::size_t len = 0;

    Rtcm3Arp arp;
    if ((len < size) && Rtcm3GetArp(msg, arp)) {
        const auto llh = trafo::TfWgs84LlhEcef({ arp.ecef_x_, arp.ecef_y_, arp.ecef_z_ });
        len =
            std::snprintf(info, size, "(#%d) %.2f %.2f %.2f (gps %d, glo %d, gal %d, ref %d, osc %d, cyc %d) %.6f/%.6f",
                arp.ref_sta_id_, arp.ecef_x_, arp.ecef_y_, arp.ecef_z_, arp.gps_ind_, arp.glo_ind_, arp.gal_ind_,
                arp.ref_ind_, arp.osc_ind_, arp.cyc_ind_, math::RadToDeg(llh.x()), math::RadToDeg(llh.y()));
    }

    Rtcm3Ant ant;
    if ((len < size) && Rtcm3GetAnt(msg, ant)) {
        len = std::snprintf(info, size, "(#%d) [%s] [%s] %" PRIu8 " [%s] [%s] [%s]", ant.ref_sta_id_, ant.ant_desc_,
            ant.ant_serial_, ant.ant_setup_id_, ant.rx_type_, ant.rx_fw_, ant.rx_serial_);
    }

    Rtcm3MsmHeader msm;
    if ((len < size) && Rtcm3GetMsmHeader(msg, msm)) {
        len = snprintf(info, size, "(#%d) %010.3f (%d * %d, %s)", msm.ref_sta_id_, msm.any_tow_, msm.num_sat_,
            msm.num_sig_, msm.multi_msg_bit_ ? "more" : "last");
    }

    const char* type_desc = Rtcm3GetTypeDesc(Rtcm3Type(msg));
    if ((len < size) && (type_desc != NULL)) {
        len += snprintf(&info[len], size - len, "%s%s", len > 0 ? " - " : "", type_desc);
    }

    return (len > 0) && (len < size);
}

/* ****************************************************************************************************************** */
}  // namespace rtcm3
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
