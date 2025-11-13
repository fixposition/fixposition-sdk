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
 * @brief Fixposition SDK: Parser NMEA routines and types
 */

/* LIBC/STL */
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <limits>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/math.hpp"
#include "fpsdk_common/parser/nmea.hpp"
#include "fpsdk_common/parser/types.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace nmea {
/* ****************************************************************************************************************** */

bool NmeaGetMessageMeta(NmeaMessageMeta& meta, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg == NULL) || (msg_size < 10)) {
        return false;
    }
    meta.payload_ix0_ = 0;
    meta.payload_ix1_ = 0;

    // 012345678901234567890123456789
    // $TTF*xx\r\n"
    // $GNGGA,...,...,...*xx\r\n
    // $PUBX,nn,...,...,...*xx\r\n
    // $FP,ODOMETRY,n,...,...,...*xx\r\n

    // Talker ID
    std::size_t offs = 0;
    if (msg[1] == 'P')  // Proprietary
    {
        meta.talker_[0] = 'P';
        meta.talker_[1] = '\0';
        offs = 2;
    } else {
        meta.talker_[0] = msg[1];
        meta.talker_[1] = msg[2];
        meta.talker_[2] = '\0';
        offs = 3;
    }

    // Sentence formatter
    std::size_t comma_ix = 0;
    for (std::size_t ix = offs; (ix < (msg_size - 4)) && (ix < (sizeof(meta.formatter_) - 1 + offs)); ix++) {
        if ((msg[ix] == ',') || (msg[ix] == '*')) {
            comma_ix = ix;
            break;
        }
    }
    if (comma_ix <= 0) {
        meta.formatter_[0] = '\0';
        return false;
    }

    std::size_t i_ix;
    std::size_t o_ix;
    const std::size_t max_o = sizeof(meta.formatter_) - 1;
    for (i_ix = offs, o_ix = 0; (i_ix < comma_ix) && (o_ix < max_o); i_ix++, o_ix++) {
        meta.formatter_[o_ix] = msg[i_ix];
    }
    meta.formatter_[o_ix] = '\0';

    // No payload
    if (msg[comma_ix] == '*') {
        meta.payload_ix0_ = 0;
        meta.payload_ix1_ = 0;
        return true;
    }

    meta.payload_ix0_ = comma_ix + 1;
    meta.payload_ix1_ = msg_size - 5 - 1;  // "*XX\r\n"

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    NmeaMessageMeta meta;
    if (!NmeaGetMessageMeta(meta, msg, msg_size)) {
        return false;
    }

    // u-blox proprietary
    if ((meta.talker_[0] == 'P') && (meta.formatter_[0] == 'U') && (meta.formatter_[1] == 'B') &&
        (meta.formatter_[2] == 'X') && (meta.formatter_[3] == '\0')) {
        const char* pubx = NULL;
        const char c1 = msg[meta.payload_ix0_];
        const char c2 = msg[meta.payload_ix0_ + 1];
        // clang-format off
        if      ( (c1 == '4') && (c2 == '1') ) { pubx = "CONFIG"; }
        else if ( (c1 == '0') && (c2 == '0') ) { pubx = "POSITION"; }
        else if ( (c1 == '4') && (c2 == '0') ) { pubx = "RATE"; }
        else if ( (c1 == '0') && (c2 == '3') ) { pubx = "SVSTATUS"; }
        else if ( (c1 == '0') && (c2 == '4') ) { pubx = "TIME"; }  // clang-format on
        else {
            return std::snprintf(name, size, "NMEA-PUBX-%c%c", c1, c2) < (int)size;
        }
        return std::snprintf(name, size, "NMEA-PUBX-%s", pubx) < (int)size;
    }
    // Standard NMEA, incl. other proprietary ("P" talker ID)
    else {
        return std::snprintf(name, size, "NMEA-%s-%s", meta.talker_[0] != '\0' ? meta.talker_ : "?",
                   meta.formatter_[0] != '\0' ? meta.formatter_ : "?") < (int)size;
    }

    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }
    std::size_t len = 0;
    NmeaMessageMeta meta;
    if (NmeaGetMessageMeta(meta, msg, msg_size) && (meta.payload_ix1_ > meta.payload_ix0_)) {
        char fmt[20];
        snprintf(fmt, sizeof(fmt), "%%.%ds", meta.payload_ix1_ - meta.payload_ix0_ + 1);
        len += snprintf(info, size, fmt, (const char*)&msg[meta.payload_ix0_]);
    } else {
        info[0] = '\0';
    }
    return (len > 0) && (len < size);
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaMakeMessage(std::vector<uint8_t>& msg, const std::string& payload)
{
    const std::size_t msg_size = NMEA_FRAME_SIZE + payload.size();
    if (msg_size > MAX_NMEA_SIZE) {
        return false;
    }
    msg.resize(msg_size);
    std::size_t ix = 0;
    msg[ix++] = '$';
    uint8_t ck = 0;
    for (const char c : payload) {  // clang-format off
        if ((c < 0x20) || (c > 0x7e) ||                               // invalid range,
            (c == '$') || (c == '\\') || (c == '!') || (c == '~') ||  // reserved...
            (c == '^') || (c == '*'))                                 // ...or sepecial (but do allow ',')
        {
            msg[ix] = '_';
        } else {
            msg[ix] = c;
        }
        ck ^= msg[ix];
        ix++;
    }  // clang-format on
    const int cka = ((ck >> 4) & 0x0f);
    const int ckb = (ck & 0x0f);
    msg[ix++] = '*';
    msg[ix++] = (cka > 9 ? 'A' - 10 + cka : '0' + cka);
    msg[ix++] = (ckb > 9 ? 'A' - 10 + ckb : '0' + ckb);
    msg[ix++] = '\r';
    msg[ix++] = '\n';
    return true;
}

bool NmeaMakeMessage(std::string& msg, const std::string& payload)
{
    std::vector<uint8_t> buf;
    if (NmeaMakeMessage(buf, payload)) {
        msg = string::BufToStr(buf);
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

NmeaCoordinates::NmeaCoordinates(const double degs, const int digits)
{
    sign_ = (degs >= 0.0);
    const double degs_abs = std::abs(degs);
    deg_ = (int)degs_abs;
    const double frac_deg = degs_abs - (double)deg_;
    min_ = math::RoundToFracDigits(frac_deg * 60.0, digits);
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaTalkerIdStr(const NmeaTalkerId talker)
{
    switch (talker) {  // clang-format off
        case NmeaTalkerId::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaTalkerId::PROPRIETARY: return "PROPRIETARY";
        case NmeaTalkerId::GPS_SBAS:    return "GPS_SBAS";
        case NmeaTalkerId::GLO:         return "GLO";
        case NmeaTalkerId::GAL:         return "GAL";
        case NmeaTalkerId::BDS:         return "BDS";
        case NmeaTalkerId::NAVIC:       return "NAVIC";
        case NmeaTalkerId::QZSS:        return "QZSS";
        case NmeaTalkerId::GNSS:        return "GNSS";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaFormatterStr(const NmeaFormatter formatter)
{
    switch (formatter) {  // clang-format off
        case NmeaFormatter::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaFormatter::GGA:         return "GGA";
        case NmeaFormatter::GLL:         return "GLL";
        case NmeaFormatter::RMC:         return "RMC";
        case NmeaFormatter::VTG:         return "VTG";
        case NmeaFormatter::GST:         return "GST";
        case NmeaFormatter::HDT:         return "HDT";
        case NmeaFormatter::ZDA:         return "ZDA";
        case NmeaFormatter::GSA:         return "GSA";
        case NmeaFormatter::GSV:         return "GSV";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaQualityGgaStr(const NmeaQualityGga qual)
{
    switch (qual) {  // clang-format off
        case NmeaQualityGga::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaQualityGga::NOFIX:       return "NOFIX";
        case NmeaQualityGga::SPP:         return "SPP";
        case NmeaQualityGga::DGNSS:       return "DGNSS";
        case NmeaQualityGga::PPS:         return "PPS";
        case NmeaQualityGga::RTK_FIXED:   return "RTK_FIXED";
        case NmeaQualityGga::RTK_FLOAT:   return "RTK_FLOAT";
        case NmeaQualityGga::ESTIMATED:   return "ESTIMATED";
        case NmeaQualityGga::MANUAL:      return "MANUAL";
        case NmeaQualityGga::SIM:         return "SIM";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaStatusGllRmcStr(const NmeaStatusGllRmc status)
{
    switch (status) {  // clang-format off
        case NmeaStatusGllRmc::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaStatusGllRmc::INVALID:     return "INVALID";
        case NmeaStatusGllRmc::VALID:       return "VALID";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaModeGllVtgStr(const NmeaModeGllVtg mode)
{
    switch (mode) {  // clang-format off
        case NmeaModeGllVtg::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaModeGllVtg::INVALID:     return "INVALID";
        case NmeaModeGllVtg::AUTONOMOUS:  return "AUTONOMOUS";
        case NmeaModeGllVtg::DGNSS:       return "DGNSS";
        case NmeaModeGllVtg::ESTIMATED:   return "ESTIMATED";
        case NmeaModeGllVtg::MANUAL:      return "MANUAL";
        case NmeaModeGllVtg::SIM:         return "SIM";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaModeRmcGnsStr(const NmeaModeRmcGns mode)
{
    switch (mode) {  // clang-format off
        case NmeaModeRmcGns::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaModeRmcGns::INVALID:     return "INVALID";
        case NmeaModeRmcGns::AUTONOMOUS:  return "AUTONOMOUS";
        case NmeaModeRmcGns::DGNSS:       return "DGNSS";
        case NmeaModeRmcGns::ESTIMATED:   return "ESTIMATED";
        case NmeaModeRmcGns::RTK_FIXED:   return "RTK_FIXED";
        case NmeaModeRmcGns::RTK_FLOAT:   return "RTK_FLOAT";
        case NmeaModeRmcGns::PRECISE:     return "PRECISE";
        case NmeaModeRmcGns::MANUAL:      return "MANUAL";
        case NmeaModeRmcGns::SIM:         return "SIM";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaNavStatusRmcStr(const NmeaNavStatusRmc navstatus)
{
    switch (navstatus) {  // clang-format off
        case NmeaNavStatusRmc::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaNavStatusRmc::SAFE:        return "SAFE";
        case NmeaNavStatusRmc::CAUTION:     return "CAUTION";
        case NmeaNavStatusRmc::UNSAFE:      return "UNSAFE";
        case NmeaNavStatusRmc::NA:          return "NA";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaOpModeGsaStr(const NmeaOpModeGsa opmode)
{
    switch (opmode) {  // clang-format off
        case NmeaOpModeGsa::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaOpModeGsa::MANUAL:      return "MANUAL";
        case NmeaOpModeGsa::AUTO:        return "AUTO";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaNavModeGsaStr(const NmeaNavModeGsa navmode)
{
    switch (navmode) {  // clang-format off
        case NmeaNavModeGsa::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaNavModeGsa::NOFIX:       return "NOFIX";
        case NmeaNavModeGsa::FIX2D:       return "FIX2D";
        case NmeaNavModeGsa::FIX3D:       return "FIX3D";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaSystemIdStr(const NmeaSystemId system)
{
    switch (system) {  // clang-format off
        case NmeaSystemId::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaSystemId::GPS_SBAS:    return "GPS_SBAS";
        case NmeaSystemId::GLO:         return "GLO";
        case NmeaSystemId::GAL:         return "GAL";
        case NmeaSystemId::BDS:         return "BDS";
        case NmeaSystemId::QZSS:        return "QZSS";
        case NmeaSystemId::NAVIC:       return "NAVIC";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* NmeaSignalIdStr(const NmeaSignalId signal)
{
    switch (signal) {  // clang-format off
        case NmeaSignalId::UNSPECIFIED: return "UNSPECIFIED";
        case NmeaSignalId::NONE:        return "NONE";
        case NmeaSignalId::GPS_L1CA:    return "GPS_L1CA";
        case NmeaSignalId::GPS_L2CL:    return "GPS_L2CL";
        case NmeaSignalId::GPS_L2CM:    return "GPS_L2CM";
        case NmeaSignalId::GPS_L5I:     return "GPS_L5I";
        case NmeaSignalId::GPS_L5Q:     return "GPS_L5Q";
        case NmeaSignalId::GAL_E1:      return "GAL_E1";
        case NmeaSignalId::GAL_E5A:     return "GAL_E5A";
        case NmeaSignalId::GAL_E5B:     return "GAL_E5B";
        case NmeaSignalId::GAL_E6BC:    return "GAL_E6BC";
        case NmeaSignalId::GAL_E6A:     return "GAL_E6A";
        case NmeaSignalId::BDS_B1ID:    return "BDS_B1ID";
        case NmeaSignalId::BDS_B2ID:    return "BDS_B2ID";
        case NmeaSignalId::BDS_B1C:     return "BDS_B1C";
        case NmeaSignalId::BDS_B2A:     return "BDS_B2A";
        case NmeaSignalId::BDS_B2B:     return "BDS_B2B";
        case NmeaSignalId::QZSS_L1CA:   return "QZSS_L1CA";
        case NmeaSignalId::QZSS_L1S:    return "QZSS_L1S";
        case NmeaSignalId::QZSS_L2CM:   return "QZSS_L2CM";
        case NmeaSignalId::QZSS_L2CL:   return "QZSS_L2CL";
        case NmeaSignalId::QZSS_L5I:    return "QZSS_L5I";
        case NmeaSignalId::QZSS_L5Q:    return "QZSS_L5Q";
        case NmeaSignalId::GLO_L1OF:    return "GLO_L1OF";
        case NmeaSignalId::GLO_L2OF:    return "GLO_L2OF";
        case NmeaSignalId::NAVIC_L5A:   return "NAVIC_L5A";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

// clang-format off
//                                                         GPS      SBAS      GAL      BDS      GLO       QZSS       NavIC
/*static*/ const NmeaVersion NmeaVersion::V410         = { 1, 32,   33, 64,   1, 36,   1, 63,   65, 96,    -1,  -1,   -1, -1 };
/*static*/ const NmeaVersion NmeaVersion::V410_UBX_EXT = { 1, 32,   33, 64,   1, 36,   1, 63,   65, 96,   193, 202,   -1, -1 };
/*static*/ const NmeaVersion NmeaVersion::V411         = { 1, 32,   33, 64,   1, 36,   1, 63,   65, 96,     1,  10,    1, 14 };
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaTime::operator==(const NmeaTime& rhs) const
{
    return (valid == rhs.valid) && (hours == rhs.hours) && (mins == rhs.mins) && (std::abs(secs - rhs.secs) < 1e-9);
}

bool NmeaTime::operator!=(const NmeaTime& rhs) const
{
    return !(*this == rhs);
}

bool NmeaDate::operator==(const NmeaDate& rhs) const
{
    return (valid == rhs.valid) && (years == rhs.years) && (months == rhs.months) && (days == rhs.days);
}

bool NmeaDate::operator!=(const NmeaDate& rhs) const
{
    return !(*this == rhs);
}

// Parse float
static bool StrToFloat(const std::string& str, double& value)
{
    if (str.empty() || (std::isspace(str[0]) != 0)) {
        return false;
    }

    double value_tmp = 0.0f;
    int num = 0;
    int count = std::sscanf(str.data(), "%lf%n", &value_tmp, &num);

    if ((count == 1) && ((std::size_t)num == str.size()) && std::isfinite(value_tmp)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

// Parse decimal (!) integer
static bool StrToInt(const std::string& str, int32_t& value)
{
    // No empty string, no leading whitespace
    if (str.empty() || (std::isspace(str[0]) != 0)) {
        return false;
    }

    // Parse
    int64_t value_tmp = 0;
    int num = 0;
    int count = std::sscanf(str.data(), "%" SCNd64 "%n", &value_tmp, &num);  // *decimal* integer, so SCNd, not SCNi
    // Number of values found must be 1 and the entire string must have been used (no trailing stuff)
    if ((count == 1) && ((std::size_t)num == str.size()) && (value_tmp >= INT32_MIN) && (value_tmp <= INT32_MAX)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Various helpers for the SetFromMsg() NMEA decoding functions

// Debug prints, for development
#if 0
#  define NMEA_TRACE(fmt, ...) fprintf(stderr, fmt "\n", ##__VA_ARGS__)
#else
#  define NMEA_TRACE(fmt, ...) /* nothing */
#endif

// Split NMEA sentence (message) into its meta data and the payload fields
struct NmeaParts
{
    NmeaMessageMeta meta_;
    std::vector<std::string> fields_;
};

static bool GetParts(NmeaParts& parts, const char* formatter, const uint8_t* msg, const std::size_t msg_size)
{
    bool ok = true;

    if (!NmeaGetMessageMeta(parts.meta_, msg, msg_size) || (parts.meta_.payload_ix0_ <= 0) ||
        (parts.meta_.payload_ix1_ <= 0)) {
        ok = false;
    } else {
        if (std::strcmp(parts.meta_.formatter_, formatter) != 0) {
            ok = false;
        }

        std::string payload((const char*)&msg[parts.meta_.payload_ix0_],
            (const char*)&msg[parts.meta_.payload_ix0_] + (parts.meta_.payload_ix1_ - parts.meta_.payload_ix0_ + 1));

        NMEA_TRACE("GetParts(...) talker=%s formatter=%s payload=%s ix0=%d ix1=%d size=%d", parts.meta_.talker_,
            parts.meta_.formatter_, payload.c_str(), parts.meta_.payload_ix0_, parts.meta_.payload_ix1_, (int)msg_size);

        std::size_t pos = 0;
        while ((pos = payload.find(",")) != std::string::npos) {
            std::string part = payload.substr(0, pos);
            payload.erase(0, pos + 1);
            parts.fields_.push_back(part);
        }
        parts.fields_.push_back(payload);
    }

    NMEA_TRACE(
        "GetParts(..., \"%s\", ..., ...)=%s #fields=%d", formatter, string::ToStr(ok), (int)parts.fields_.size());
    return ok;
}

// Get talker ID
bool GetTalker(NmeaTalkerId& talker, const char* talkerstr)
{
    bool ok = true;
    if (talkerstr[0] == 'P') {
        talker = NmeaTalkerId::PROPRIETARY;
    } else if (talkerstr[0] == 'G') {
        switch ((NmeaTalkerId)talkerstr[1]) {  // clang-format off
            case NmeaTalkerId::GPS_SBAS:     talker = NmeaTalkerId::GPS_SBAS; break;
            case NmeaTalkerId::GLO:          talker = NmeaTalkerId::GLO;      break;
            case NmeaTalkerId::GAL:          talker = NmeaTalkerId::GAL;      break;
            case NmeaTalkerId::BDS:          talker = NmeaTalkerId::BDS;      break;
            case NmeaTalkerId::NAVIC:        talker = NmeaTalkerId::NAVIC;    break;
            case NmeaTalkerId::QZSS:         talker = NmeaTalkerId::QZSS;     break;
            case NmeaTalkerId::GNSS:         talker = NmeaTalkerId::GNSS;     break;
            case NmeaTalkerId::UNSPECIFIED:
            case NmeaTalkerId::PROPRIETARY:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetTalker(\"%s\")=%s talker=%c", talkerstr, string::ToStr(ok), types::EnumToVal(talker));
    return ok;
}

// Get position, and optionally height
static bool GetLlh(NmeaLlh& llh, const std::vector<std::string>& fields, const int lat_ix, const int lon_ix,
    const int alt_ix, const int sep_ix, const bool required)
{
    bool ok = true;

    // Lat/lon
    auto& slat = fields[lat_ix];
    auto& slon = fields[lon_ix];
    if (slat.empty() || slon.empty() || (slat.size() < 7) || (slon.size() < 8) || (slat[4] != '.') ||
        (slon[5] != '.')) {
        if (required) {
            ok = false;
        }
    } else {
        double ilat = 0.0;
        double flat = 0.0;
        double ilon = 0.0;
        double flon = 0.0;
        if (StrToFloat(slat.substr(0, 2), ilat) && StrToFloat(slat.substr(2), flat) &&
            StrToFloat(slon.substr(0, 3), ilon) && StrToFloat(slon.substr(3), flon) && (ilat <= 90.0) &&
            (ilat >= -90.0) && (ilon <= 180.0) && (ilon >= -180.0) && (flat < 60.0) && (flon < 60.0)) {
            llh.latlon_valid = true;
            llh.lat = ilat + (flat / 60.0);
            if (fields[lat_ix + 1] == "S") {
                llh.lat = -llh.lat;
            }
            llh.lon = ilon + (flon / 60.0);
            if (fields[lon_ix + 1] == "W") {
                llh.lon = -llh.lon;
            }
        } else {
            ok = false;
        }
    }

    // Optional height
    if ((alt_ix >= 0) && (sep_ix >= 0)) {
        if (fields[alt_ix].empty()) {
            if (required) {
                ok = false;
            }
        } else {
            double heightval = 0.0;
            double sepval = 0.0;
            if (StrToFloat(fields[alt_ix], heightval) && (fields[alt_ix + 1] == "M") &&
                (fields[sep_ix].empty() || (StrToFloat(fields[sep_ix], sepval) && (fields[sep_ix + 1] == "M")))) {
                // Contrary to the NMEA standard we'll allow sep (fields[10]) to be empty and assume that height
                // (fields[9]) is ellipsoidal instead of mean sea level
                llh.height = heightval + sepval;
                llh.height_valid = true;
            } else {
                ok = false;
            }
        }
    }

    NMEA_TRACE("GetLlh(%d=\"%s\", %d=\"%s\", %d=\"%s\", %d=\"%s\", %s)=%s ll=%.12g/%.12g/%s h=%g/%s", lat_ix,
        lat_ix >= 0 ? fields[lat_ix].c_str() : "", lon_ix, lon_ix >= 0 ? fields[lon_ix].c_str() : "", alt_ix,
        alt_ix >= 0 ? fields[alt_ix].c_str() : "", sep_ix, sep_ix >= 0 ? fields[sep_ix].c_str() : "",
        string::ToStr(required), string::ToStr(ok), llh.lat, llh.lon, string::ToStr(llh.latlon_valid), llh.height,
        string::ToStr(llh.height_valid));
    return ok;
}

// Get time, with basic range checks, may be null
static bool GetTime(NmeaTime& time, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        double timeval = 0.0;
        if (StrToFloat(field, timeval)) {
            time.valid = !((timeval < 0.0) || (timeval > 235962.0));  // 00:00:00 .. 23:59:61.9999
            time.hours = std::floor(timeval / 10000.0);
            timeval -= time.hours * 10000;
            time.mins = std::floor(timeval / 100.0);
            timeval -= time.mins * 100.0;
            time.secs = timeval;
            return true;
        } else {
            ok = false;
        }
    }
    NMEA_TRACE("GetTime(\"%s\")=%s time=%d/%d/%g/%s", field.c_str(), string::ToStr(ok), time.hours, time.mins,
        time.secs, string::ToStr(time.valid));
    return ok;
}

// Get date (DDMMYY format), with basic range checks, may be null
static bool GetDateDdMmYy(NmeaDate& date, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        int dateval = 0;
        if (StrToInt(field, dateval)) {
            date.years = dateval % 100;
            date.years += (date.years >= 80 ? 1900 : 2000);
            date.months = (dateval / 100) % 100;
            date.days = (dateval / 10000) % 100;
            date.valid = ((date.years >= 2001) && (date.years <= 2099));  // 2001-01-01 ... 2099-12-31
        } else {
            ok = false;
        }
    }
    NMEA_TRACE("GetDateDdMmYy(\"%s\")=%s date=%d/%d/%d/%s", field.c_str(), string::ToStr(ok), date.years, date.months,
        date.days, string::ToStr(date.valid));
    return ok;
}

#if 0
// Get date (YYYYMMDD format), with basic range checks, may be null
static bool GetDateYyyyMmDd(NmeaDate& date, const std::string& field) {
    bool ok = true;
    if (!field.empty()) {
        int dateval = 0;
        if (StrToInt(field, dateval)) {
            // During coldstart the receiver reports 19800105...
            date.days = dateval % 100;
            date.months = (dateval / 100) % 100;
            date.years = (dateval / 10000) % 10000;
            date.valid = ((date.years >= 2001) && (date.years <= 2099));  // 2001-01-01 ... 2099-12-31
        } else {
            ok = false;
        }
    }
    NMEA_TRACE("GetDateYyyyMmDd(\"%s\")=%s date=%d/%d/%d/%s", field.c_str(), string::ToStr(ok), date.years,
               date.months, date.days, string::ToStr(date.valid));
    return ok;
}
#endif

// Get satellite
static bool GetSat(NmeaSat& sat, const std::string& field, const NmeaSystemId system, const bool required)
{
    bool ok = true;

    if (field.empty()) {
        if (required) {
            ok = false;
        }
    } else {
        sat.system = system;
        if (StrToInt(field, sat.svid) && sat.svid >= 0) {  // @todo more validity checks per system?
            sat.valid = true;
        } else {
            ok = false;
        }
    }
    NMEA_TRACE("GetSat(\"%s\")=%s svid=%d/%c/%s", field.c_str(), string::ToStr(ok), sat.svid,
        types::EnumToVal(sat.system), string::ToStr(sat.valid));
    return ok;
}

// Get GGA quality flag
static bool GetQualityGga(NmeaQualityGga& quality, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaQualityGga)field[0]) {  // clang-format off
            case NmeaQualityGga::NOFIX:        quality = NmeaQualityGga::NOFIX;      break;
            case NmeaQualityGga::SPP:          quality = NmeaQualityGga::SPP;        break;
            case NmeaQualityGga::DGNSS:        quality = NmeaQualityGga::DGNSS;      break;
            case NmeaQualityGga::PPS:          quality = NmeaQualityGga::PPS;        break;
            case NmeaQualityGga::RTK_FIXED:    quality = NmeaQualityGga::RTK_FIXED;  break;
            case NmeaQualityGga::RTK_FLOAT:    quality = NmeaQualityGga::RTK_FLOAT;  break;
            case NmeaQualityGga::ESTIMATED:    quality = NmeaQualityGga::ESTIMATED;  break;
            case NmeaQualityGga::MANUAL:       quality = NmeaQualityGga::MANUAL;     break;
            case NmeaQualityGga::SIM:          quality = NmeaQualityGga::SIM;        break;
            case NmeaQualityGga::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetQualityGga(\"%s\")=%s quality=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(quality));
    return ok;
}

// Get GLL/RMC status flag
static bool GetStatusGllRmc(NmeaStatusGllRmc& status, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaStatusGllRmc)field[0]) {  // clang-format off
            case NmeaStatusGllRmc::INVALID:      status = NmeaStatusGllRmc::INVALID;  break;
            case NmeaStatusGllRmc::VALID:        status = NmeaStatusGllRmc::VALID;    break;
            case NmeaStatusGllRmc::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetStatusGllRmc(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

// Get GLL/VTG mode flag
static bool GetModeGllVtg(NmeaModeGllVtg& mode, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaModeGllVtg)field[0]) {  // clang-format off
            case NmeaModeGllVtg::INVALID:      mode = NmeaModeGllVtg::INVALID;     break;
            case NmeaModeGllVtg::AUTONOMOUS:   mode = NmeaModeGllVtg::AUTONOMOUS;  break;
            case NmeaModeGllVtg::DGNSS:        mode = NmeaModeGllVtg::DGNSS;       break;
            case NmeaModeGllVtg::ESTIMATED:    mode = NmeaModeGllVtg::ESTIMATED;   break;
            case NmeaModeGllVtg::MANUAL:       mode = NmeaModeGllVtg::MANUAL;      break;
            case NmeaModeGllVtg::SIM:          mode = NmeaModeGllVtg::SIM;         break;
            case NmeaModeGllVtg::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetModeGllVtg(\"%s\")=%s mode=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(mode));
    return ok;
}

// Get RMC/GNS mode flag
static bool GetModeRmcGns(NmeaModeRmcGns& mode, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaModeRmcGns)field[0]) {  // clang-format off
            case NmeaModeRmcGns::INVALID:      mode = NmeaModeRmcGns::INVALID;     break;
            case NmeaModeRmcGns::AUTONOMOUS:   mode = NmeaModeRmcGns::AUTONOMOUS;  break;
            case NmeaModeRmcGns::DGNSS:        mode = NmeaModeRmcGns::DGNSS;       break;
            case NmeaModeRmcGns::ESTIMATED:    mode = NmeaModeRmcGns::ESTIMATED;   break;
            case NmeaModeRmcGns::RTK_FIXED:    mode = NmeaModeRmcGns::RTK_FIXED;   break;
            case NmeaModeRmcGns::RTK_FLOAT:    mode = NmeaModeRmcGns::RTK_FLOAT;   break;
            case NmeaModeRmcGns::PRECISE:      mode = NmeaModeRmcGns::PRECISE;     break;
            case NmeaModeRmcGns::MANUAL:       mode = NmeaModeRmcGns::MANUAL;      break;
            case NmeaModeRmcGns::SIM:          mode = NmeaModeRmcGns::SIM;         break;
            case NmeaModeRmcGns::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetModeRmcGns(\"%s\")=%s mode=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(mode));
    return ok;
}

// Get RMC status flag
static bool GetNavStatusRmc(NmeaNavStatusRmc& navstatus, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaNavStatusRmc)field[0]) {  // clang-format off
            case NmeaNavStatusRmc::SAFE:         navstatus = NmeaNavStatusRmc::SAFE;     break;
            case NmeaNavStatusRmc::CAUTION:      navstatus = NmeaNavStatusRmc::CAUTION;  break;
            case NmeaNavStatusRmc::UNSAFE:       navstatus = NmeaNavStatusRmc::UNSAFE;   break;
            case NmeaNavStatusRmc::NA:           navstatus = NmeaNavStatusRmc::NA;       break;
            case NmeaNavStatusRmc::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE(
        "GetNavStatusRmc(\"%s\")=%s navstatus=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(navstatus));
    return ok;
}

// Get GNS operation mode
static bool GetOpModeGsa(NmeaOpModeGsa& opmode, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaOpModeGsa)field[0]) {  // clang-format off
            case NmeaOpModeGsa::MANUAL:       opmode = NmeaOpModeGsa::MANUAL;  break;
            case NmeaOpModeGsa::AUTO:         opmode = NmeaOpModeGsa::AUTO;    break;
            case NmeaOpModeGsa::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetOpModeGsa(\"%s\")=%s opmode=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(opmode));
    return ok;
}

// Get GNS operation mode
static bool GetNavModeGsa(NmeaNavModeGsa& navmode, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaNavModeGsa)field[0]) {  // clang-format off
            case NmeaNavModeGsa::NOFIX:        navmode = NmeaNavModeGsa::NOFIX;  break;
            case NmeaNavModeGsa::FIX2D:        navmode = NmeaNavModeGsa::FIX2D;  break;
            case NmeaNavModeGsa::FIX3D:        navmode = NmeaNavModeGsa::FIX3D;  break;
            case NmeaNavModeGsa::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetNavModeGsa(\"%s\")=%s navmode=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(navmode));
    return ok;
}

// Get (GSA) system ID
static bool GetSystemId(NmeaSystemId& system, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {
        switch ((NmeaSystemId)field[0]) {  // clang-format off
            case NmeaSystemId::GPS_SBAS:     system = NmeaSystemId::GPS_SBAS;  break;
            case NmeaSystemId::GLO:          system = NmeaSystemId::GLO;       break;
            case NmeaSystemId::GAL:          system = NmeaSystemId::GAL;       break;
            case NmeaSystemId::BDS:          system = NmeaSystemId::BDS;       break;
            case NmeaSystemId::QZSS:         system = NmeaSystemId::QZSS;      break;
            case NmeaSystemId::NAVIC:        system = NmeaSystemId::NAVIC;     break;
            case NmeaSystemId::UNSPECIFIED:  ok = false; break;
        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetSystemId(\"%s\")=%s system=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(system));
    return ok;
}

// Get system Id
static bool GetSignalId(NmeaSignalId& signalid, const std::string& field, const NmeaSystemId system)
{
    bool ok = true;
    if (!field.empty()) {
        switch (system) {  // clang-format off
            case NmeaSystemId::GPS_SBAS: switch ((int)field[0]) {
                case 0xff & types::EnumToVal(NmeaSignalId::NONE):       signalid = NmeaSignalId::NONE;       break;
                case 0xff & types::EnumToVal(NmeaSignalId::GPS_L1CA):   signalid = NmeaSignalId::GPS_L1CA;   break; // = NmeaSignalId::SBAS_L1CA
                case 0xff & types::EnumToVal(NmeaSignalId::GPS_L2CL):   signalid = NmeaSignalId::GPS_L2CL;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::GPS_L2CM):   signalid = NmeaSignalId::GPS_L2CM;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::GPS_L5I):    signalid = NmeaSignalId::GPS_L5I;    break;
                case 0xff & types::EnumToVal(NmeaSignalId::GPS_L5Q):    signalid = NmeaSignalId::GPS_L5Q;    break;
                default: ok = false; break;
            } break;
            case NmeaSystemId::GLO: switch ((int)field[0]) {
                case 0xff & types::EnumToVal(NmeaSignalId::NONE):       signalid = NmeaSignalId::NONE;       break;
                case 0xff & types::EnumToVal(NmeaSignalId::GLO_L1OF):   signalid = NmeaSignalId::GLO_L1OF;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::GLO_L2OF):   signalid = NmeaSignalId::GLO_L2OF;   break;
                default: ok = false; break;
            } break;
            case NmeaSystemId::GAL: switch ((int_fast16_t)field[0]) {
                case 0xff & types::EnumToVal(NmeaSignalId::NONE):       signalid = NmeaSignalId::NONE;       break;
                case 0xff & types::EnumToVal(NmeaSignalId::GAL_E1):     signalid = NmeaSignalId::GAL_E1;     break;
                case 0xff & types::EnumToVal(NmeaSignalId::GAL_E5A):    signalid = NmeaSignalId::GAL_E5A;    break;
                case 0xff & types::EnumToVal(NmeaSignalId::GAL_E5B):    signalid = NmeaSignalId::GAL_E5B;    break;
                case 0xff & types::EnumToVal(NmeaSignalId::GAL_E6BC):   signalid = NmeaSignalId::GAL_E6BC;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::GAL_E6A):    signalid = NmeaSignalId::GAL_E6A;    break;
                default: ok = false; break;
            } break;
            case NmeaSystemId::BDS: switch ((int)field[0]) {
                case 0xff & types::EnumToVal(NmeaSignalId::NONE):       signalid = NmeaSignalId::NONE;       break;
                case 0xff & types::EnumToVal(NmeaSignalId::BDS_B1ID):   signalid = NmeaSignalId::BDS_B1ID;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::BDS_B2ID):   signalid = NmeaSignalId::BDS_B2ID;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::BDS_B1C):    signalid = NmeaSignalId::BDS_B1C;    break;
                case 0xff & types::EnumToVal(NmeaSignalId::BDS_B2A):    signalid = NmeaSignalId::BDS_B2A;    break;
                case 0xff & types::EnumToVal(NmeaSignalId::BDS_B2B):    signalid = NmeaSignalId::BDS_B2B;    break;
                default: ok = false; break;
            } break;
            case NmeaSystemId::QZSS: switch ((int)field[0]) {
                case 0xff & types::EnumToVal(NmeaSignalId::NONE):       signalid = NmeaSignalId::NONE;       break;
                case 0xff & types::EnumToVal(NmeaSignalId::QZSS_L1CA):  signalid = NmeaSignalId::QZSS_L1CA;  break;
                case 0xff & types::EnumToVal(NmeaSignalId::QZSS_L1S):   signalid = NmeaSignalId::QZSS_L1S;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::QZSS_L2CM):  signalid = NmeaSignalId::QZSS_L2CM;  break;
                case 0xff & types::EnumToVal(NmeaSignalId::QZSS_L2CL):  signalid = NmeaSignalId::QZSS_L2CL;  break;
                case 0xff & types::EnumToVal(NmeaSignalId::QZSS_L5I):   signalid = NmeaSignalId::QZSS_L5I;   break;
                case 0xff & types::EnumToVal(NmeaSignalId::QZSS_L5Q):   signalid = NmeaSignalId::QZSS_L5Q;   break;
                default: ok = false; break;
            } break;
            case NmeaSystemId::NAVIC: switch ((int)field[0]) {
                case 0xff & types::EnumToVal(NmeaSignalId::NONE):       signalid = NmeaSignalId::NONE;       break;
                case 0xff & types::EnumToVal(NmeaSignalId::NAVIC_L5A):  signalid = NmeaSignalId::NAVIC_L5A;  break;
                default: ok = false; break;
            } break;
            case NmeaSystemId::UNSPECIFIED:
                ok = false;
                break;

        }  // clang-format on
    } else {
        ok = false;
    }
    NMEA_TRACE("GetSignalId(\"%s\", '%c')=%s signalid=%c", field.c_str(), types::EnumToVal(system), string::ToStr(ok),
        types::EnumToVal(signalid));
    return ok;
}

// Convert talker ID to system ID
static NmeaSystemId TalkerIdToSystemId(const NmeaTalkerId talkerid)
{
    NmeaSystemId systemid = NmeaSystemId::UNSPECIFIED;
    switch (talkerid) {  // clang-format off
        case NmeaTalkerId::GPS_SBAS:     systemid = NmeaSystemId::GPS_SBAS;  break;
        case NmeaTalkerId::GLO:          systemid = NmeaSystemId::GLO;       break;
        case NmeaTalkerId::GAL:          systemid = NmeaSystemId::GAL;       break;
        case NmeaTalkerId::BDS:          systemid = NmeaSystemId::BDS;       break;
        case NmeaTalkerId::NAVIC:        systemid = NmeaSystemId::NAVIC;     break;
        case NmeaTalkerId::QZSS:         systemid = NmeaSystemId::QZSS;      break;
        case NmeaTalkerId::GNSS:
        case NmeaTalkerId::UNSPECIFIED:
        case NmeaTalkerId::PROPRIETARY:  break;
    }  // clang-format on
    NMEA_TRACE("TalkerIdToSystemId(%c)=%c", types::EnumToVal(talkerid), types::EnumToVal(systemid));
    return systemid;
}

static constexpr int INAN = std::numeric_limits<int>::max();

// Get integer value
static bool GetInt(
    NmeaInt& nmeaint, const std::string& field, const bool required, const int min = INAN, const int max = INAN)
{
    bool ok = true;
    // Null field may be okay
    if (field.empty()) {
        if (required) {
            ok = false;
        }
    }
    // Otherwise we require a value
    else {
        int value = 0;
        if (StrToInt(field, value)) {
            // Range limits?
            if ((min != INAN) && (value < min)) {
                ok = false;
            }
            if ((max != INAN) && (value > max)) {
                ok = false;
            }
            if (ok) {
                nmeaint.value = value;
                nmeaint.valid = true;
            }
        } else {
            ok = false;
        }
    }

    NMEA_TRACE("GetInt(\"%s\", %s, %d, %d)=%s value=%d/%s", field.c_str(), string::ToStr(required), min, max,
        string::ToStr(ok), nmeaint.value, string::ToStr(nmeaint.valid));
    return ok;
}

// Get float value
static bool GetFloat(
    NmeaFloat& nmeafloat, const std::string& field, const bool required, const double min = NAN, const double max = NAN)
{
    bool ok = true;
    // Null field may be okay
    if (field.empty()) {
        if (required) {
            ok = false;
        }
    }
    // Otherwise we require a value
    else {
        double value = 0.0;
        if (StrToFloat(field, value)) {
            // Range limits?
            if (std::isfinite(min) && (value < min)) {
                ok = false;
            }
            if (std::isfinite(max) && (value > max)) {
                ok = false;
            }
            if (ok) {
                nmeafloat.value = value;
                nmeafloat.valid = true;
            }
        } else {
            ok = false;
        }
    }

    NMEA_TRACE("GetFloat(\"%s\", %s, %g, %g)=%s value=%g/%s", field.c_str(), string::ToStr(required), min, max,
        string::ToStr(ok), nmeafloat.value, string::ToStr(nmeafloat.valid));
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGgaPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GNGGA,092207.400,4724.017956,N,00827.022383,E,2,41,0.43,411.542,M,47.989,M,,*79\r\n
    //             0        1        2     3        4 5 6  7    8       9 10    11 12 13
    // $GNGGA,235943.812,,,,,0,00,99.99,,M,,M,,*49
    // 14 or 12 fields as diff station info is optional
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && GetTalker(talker_, m.meta_.talker_) &&
        ((m.fields_.size() == 14) || (m.fields_.size() == 12))) {
        ok = (GetTime(time, m.fields_[0]) && GetQualityGga(quality, m.fields_[5]) &&
              GetLlh(llh, m.fields_, 1, 3, 8, 10, quality != NmeaQualityGga::NOFIX) &&
              GetFloat(height_msl, m.fields_[8], quality != NmeaQualityGga::NOFIX) &&
              GetInt(num_sv, m.fields_[6], false, 0, 200) &&
              ((quality == NmeaQualityGga::NOFIX) || GetFloat(hdop, m.fields_[7], true, 0.0)) &&
              ((m.fields_.size() == 12) ||
                  (GetFloat(diff_age, m.fields_[12], false, 0, 1023) && GetInt(diff_sta, m.fields_[13], false, 0.0))));
    }
    NMEA_TRACE("NmeaGgaPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::GGA;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGllPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GNGLL,4724.018931,N,00827.023090,E,110546.800,A,D*4F    $GNGLL,,,,,235943.612,V,N*6B
    //           0        1     2        3      4     5 6
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && GetTalker(talker_, m.meta_.talker_) && (m.fields_.size() == 7)) {
        ok = (GetTime(time, m.fields_[4]) && GetStatusGllRmc(status, m.fields_[5]) &&
              GetModeGllVtg(mode, m.fields_[6]) &&
              GetLlh(ll, m.fields_, 0, 2, -1, -1, mode != NmeaModeGllVtg::INVALID));
    }
    NMEA_TRACE("NmeaGllPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::GLL;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaRmcPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GPRMC,094821.199,A,4724.017904,N,00827.021943,E,0.001,250.72,080125,,,R*73       < NMEA 4.10
    // $GNRMC,110546.800,A,4724.018931,N,00827.023090,E,0.015,139.17,231024,,,D,V*3D    >= NMEA 4.10
    //             0     1    2        3     4        5 6     7      8    9 10 11 12
    // $GNRMC,235943.412,V,,,,,,,050180,,,N,V*28
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && GetTalker(talker_, m.meta_.talker_) &&
        ((m.fields_.size() == 12) || (m.fields_.size() == 13))) {
        // Ignore magnetic variation fields_[9] and fields_[10], and fields_[12] is optional
        ok = (GetTime(time, m.fields_[0]) && GetStatusGllRmc(status, m.fields_[1]) &&
              GetFloat(speed, m.fields_[6], false) && GetFloat(course, m.fields_[7], false, 0.0, 360.0) &&
              GetDateDdMmYy(date, m.fields_[8]) && GetModeRmcGns(mode, m.fields_[11]) &&
              GetLlh(ll, m.fields_, 2, 4, -1, -1, mode != NmeaModeRmcGns::INVALID) &&
              ((m.fields_.size() == 12) || GetNavStatusRmc(navstatus, m.fields_[12])));
    }
    NMEA_TRACE("NmeaRmcPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::RMC;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaVtgPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GNVTG,139.17,T,,M,0.015,N,0.027,K,D*2A*4F       $GNVTG,,T,,M,,N,,K,N*32
    //        0      1 23 4     5 6     7 8
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && (m.fields_.size() == 9)) {
        // Ignore magnetic course fields_[2] and fields_[3]
        ok = (GetTalker(talker_, m.meta_.talker_) && (m.fields_[1] == "T") && GetFloat(cogt, m.fields_[0], false) &&
              (m.fields_[5] == "N") && GetFloat(sogn, m.fields_[4], false) && (m.fields_[7] == "K") &&
              GetFloat(sogk, m.fields_[6], false) && GetModeGllVtg(mode, m.fields_[8]));
    }
    NMEA_TRACE("NmeaVtgPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::VTG;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGstPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GPGST,132419.0000,,0.0515,0.0188,162.6609,0.0495,0.0236,0.0182*7D
    //        0          1 2      3        4      5      6      7
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && (m.fields_.size() == 8)) {
        ok = (GetTalker(talker_, m.meta_.talker_) && GetTime(time, m.fields_[0]) &&
              GetFloat(rms_range, m.fields_[1], false) && GetFloat(std_major, m.fields_[2], false) &&
              GetFloat(std_minor, m.fields_[3], false) && GetFloat(angle_major, m.fields_[4], false) &&
              GetFloat(std_lat, m.fields_[5], false) && GetFloat(std_lon, m.fields_[6], false) &&
              GetFloat(std_alt, m.fields_[7], false));
    }
    NMEA_TRACE("NmeaGstPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::GST;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaHdtPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GPHDT,61.7,T*05
    //        0    1
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && (m.fields_.size() == 2)) {
        // Ignore true_ind fields_[1]
        ok = (GetTalker(talker_, m.meta_.talker_) && GetFloat(heading, m.fields_[0], false, 0.0, 360.0));
    }
    NMEA_TRACE("NmeaHdtPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::HDT;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaZdaPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GPZDA,090924.00,10,10,2023,00,00*63
    //        0         1  2  3    4  5
    bool ok = false;
    NmeaParts m;
    NmeaInt day;
    NmeaInt month;
    NmeaInt year;
    if (GetParts(m, FORMATTER, msg, msg_size) && (m.fields_.size() == 6)) {
        ok = (GetTalker(talker_, m.meta_.talker_) && GetTime(time, m.fields_[0]) &&
              GetInt(day, m.fields_[1], false, 1, 31) && GetInt(month, m.fields_[2], false, 1, 12) &&
              GetInt(year, m.fields_[3], false, 2001, 2099) && GetInt(local_hr, m.fields_[4], false, -13, 13) &&
              GetInt(local_min, m.fields_[5], false, 0, 59));
        date.valid = day.valid && month.valid && year.valid;
        date.years = year.value;
        date.months = month.value;
        date.days = day.value;
    }
    NMEA_TRACE("NmeaZdaPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::ZDA;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGsaPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $GNGSA,A,3,03,04,06,07,09,11,19,20,26,30,31,,0.79,0.46,0.64,1*0F
    //        0 1 2  3  4  5  6  7  8  9  10 11 12 13 14 15   16   17
    // $GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,5*37
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && GetTalker(talker_, m.meta_.talker_) && (m.fields_.size() == 18)) {
        ok = GetSystemId(system, m.fields_[17]);

        // Satellites
        for (std::size_t field_ix = 2; ok && (field_ix <= 13) && (field_ix < m.fields_.size()); field_ix++) {
            if (!m.fields_[field_ix].empty()) {
                NmeaSat cand;
                if (GetSat(cand, m.fields_[field_ix], system, false) && cand.valid) {
                    sats[num_sats] = cand;
                    num_sats++;
                } else {
                    ok = false;
                }
            }
        }

        // Remaining fields
        if (ok) {
            ok = (GetOpModeGsa(opmode, m.fields_[0]) && GetNavModeGsa(navmode, m.fields_[1]) &&
                  ((navmode == NmeaNavModeGsa::NOFIX) || GetFloat(pdop, m.fields_[14], true, 0.0)) &&
                  ((navmode == NmeaNavModeGsa::NOFIX) || GetFloat(hdop, m.fields_[15], true, 0.0)) &&
                  ((navmode == NmeaNavModeGsa::NOFIX) || GetFloat(vdop, m.fields_[16], true, 0.0)));
        }
    }
    NMEA_TRACE("NmeaGsaPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::GSA;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaGsvPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // Sequence of 4 messages, total 14 sats/sigs = 4 + 4 + 4 + 2
    // $GPGSV,4,2,14,09,84,270,52,11,28,311,46,16,03,080,42,17,03,218,35,1*69
    //        0 1  2  3  4  5   6  7  8  9  10 11 12 13  14 15 16 17  18 19
    //               ^^^^^^^^^^^^ ^^^^^^^^^^^^ ^^^^^^^^^^^^ ^^^^^^^^^^^^  <-- 2nd message: 4 sigs/sats
    // $GPGSV,4,4,14,31,08,034,46,36,31,149,43,1*62
    //        0 1  2 3  4  5   6  7  8  9   10 11
    //               ^^^^^^^^^^^^ ^^^^^^^^^^^^ <-- 4th (last) message: 2 sigs/sats
    //
    // $GAGSV,1,1,01,36,01,124,,0*46..
    //        0 1 2  3  4  5  6 7
    //               ^^^^^^^^^^ <-- 1st of 1 message, 1 sig/sat only
    // $GAGSV,1,1,00,1*75
    //        0 1  2 3
    bool ok = false;
    NmeaParts m;
    if (GetParts(m, FORMATTER, msg, msg_size) && GetTalker(talker_, m.meta_.talker_)) {
        // The number of fields depends on the number of satellites and the sequence of the message
        static constexpr int MAX_MSGS = 9;
        static constexpr int SV_PER_MSG = 4;
        ok = (
            // - At least 4 fields are required
            ((int)m.fields_.size() >= 4) &&
            // - The number of messages in the sequence
            GetInt(num_msgs, m.fields_[0], true, 1, MAX_MSGS) && num_msgs.valid &&
            // - The message number in the sequence
            GetInt(msg_num, m.fields_[1], true, 1, num_msgs.value) && msg_num.valid &&
            // - The total number of satellits in the sequence
            GetInt(tot_num_sat, m.fields_[2], true, 0, MAX_MSGS * SV_PER_MSG) && tot_num_sat.valid);

        // Determine the number of sat/sig in this message. The last message has the remainder, other messages have 4.
        int num_sv = 0;
        if (ok) {
            const bool last_message = (msg_num.value == num_msgs.value);
            num_sv = (last_message ? (tot_num_sat.value - ((msg_num.value - 1) * SV_PER_MSG)) : SV_PER_MSG);
            ok = ((int)m.fields_.size() == (4 + (4 * num_sv)));
        }

        // Now we know the field ix for the signal field. Note that GetSignalId() is only valid for certain talkers. So
        // the checks below should give us an ok for valid talker/system and signal, and false otherwise.
        system = TalkerIdToSystemId(talker_);
        signal = NmeaSignalId::UNSPECIFIED;
        if (ok) {
            ok = (GetSignalId(signal, m.fields_[3 + (num_sv * 4)], system) && (system != NmeaSystemId::UNSPECIFIED) &&
                  (signal != NmeaSignalId::UNSPECIFIED));
        }

        // Read the sat/sig and populate the AzEl and Cno arrays
        for (std::size_t field_offs = 3;
            ok && ((int)field_offs < (3 + (num_sv * 4))) && ((field_offs + 3) < m.fields_.size()); field_offs += 4) {
            NmeaSat sat;
            NmeaInt el;
            NmeaInt az;
            NmeaInt cno;
            if (GetSat(sat, m.fields_[field_offs], system, true) && sat.valid &&
                GetInt(el, m.fields_[field_offs + 1], false, -90, 90) &&
                GetInt(az, m.fields_[field_offs + 2], false, 0, 360) &&
                GetInt(cno, m.fields_[field_offs + 3], false, 0, 100)) {
                // Have az/el
                if (el.valid && az.valid) {
                    azels[num_azels].valid = true;
                    azels[num_azels].system = system;
                    azels[num_azels].svid = sat.svid;
                    azels[num_azels].az = az.value;
                    azels[num_azels].el = el.value;
                    num_azels++;
                }
                // Have cno
                if (cno.valid) {
                    cnos[num_cnos].valid = true;
                    cnos[num_cnos].system = system;
                    cnos[num_cnos].svid = sat.svid;
                    cnos[num_cnos].signal = signal;
                    cnos[num_cnos].cno = cno.value;
                    num_cnos++;
                }
            } else {
                ok = false;
            }
        }
    }
    NMEA_TRACE("NmeaGsvPayload %s", string::ToStr(ok));
    valid_ = ok;
    formatter_ = NmeaFormatter::GSV;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

NmeaPayloadPtr NmeaDecodeMessage(const uint8_t* msg, const std::size_t msg_size)
{
    NmeaMessageMeta meta;
    if (!NmeaGetMessageMeta(meta, msg, msg_size)) {
        return nullptr;
    }

#define _GEN(_a_, _b_, _c_, _type_)                                                                     \
    else if ((meta.formatter_[0] == _a_) && (meta.formatter_[1] == _b_) && (meta.formatter_[2] == _c_)) \
    {                                                                                                   \
        auto payload = std::make_unique<_type_>();                                                      \
        if (payload->SetFromMsg(msg, msg_size)) {                                                       \
            return payload;                                                                             \
        }                                                                                               \
    }

    if (false) {
    }
    _GEN('G', 'G', 'A', NmeaGgaPayload)
    _GEN('G', 'L', 'L', NmeaGllPayload)
    _GEN('R', 'M', 'C', NmeaRmcPayload)
    _GEN('V', 'T', 'G', NmeaVtgPayload)
    _GEN('G', 'S', 'T', NmeaGstPayload)
    _GEN('H', 'D', 'T', NmeaHdtPayload)
    _GEN('Z', 'D', 'A', NmeaZdaPayload)
    _GEN('G', 'S', 'A', NmeaGsaPayload)
    _GEN('G', 'S', 'V', NmeaGsvPayload)

#undef _GEN

    return nullptr;
}

// ---------------------------------------------------------------------------------------------------------------------

bool NmeaCollectGsaGsv::AddGsaAndGsv(const std::vector<NmeaGsaPayload>& gsas, const std::vector<NmeaGsvPayload>& gsvs)
{
    bool ok = true;
    for (auto& gsa : gsas) {
        if (!AddGsa(gsa)) {
            ok = false;
        }
    }
    for (auto& gsv : gsvs) {
        if (!AddGsv(gsv)) {
            ok = false;
        }
    }
    NMEA_TRACE("AddGsaAndGsv(%d, %d)=%s", (int)gsas.size(), (int)gsvs.size(), string::ToStr(ok));
    if (ok) {
        Complete();
    }
    return ok;
}

bool NmeaCollectGsaGsv::AddGsa(const NmeaGsaPayload& gsa)
{
    // Collect all "satellites used" from GSA message. They have no numbering, so we have to rely on the user giving us
    // all messages.
    for (auto& gsa_sat : gsa.sats) {
        if (gsa_sat.valid) {
            NMEA_TRACE("AddGsa() sat %c %d", types::EnumToVal(gsa_sat.system), gsa_sat.svid);
            gsa_sats_.push_back(gsa_sat);
        }
    }

    NMEA_TRACE("AddGsa(%d, %c) gsa_sats_=%d", gsa.num_sats, types::EnumToVal(gsa.system), (int)gsa_sats_.size());

    // @todo: Add checks on expected sequence of messages maybe? How? Maybe check that a message for each GNSS was seen?
    //        Check for duplicate SVs, ...

    return true;
}

bool NmeaCollectGsaGsv::AddGsv(const NmeaGsvPayload& gsv)
{
    // @todo: Add checks on expected sequence of messages maybe? We could use the msg_num and num_msgs fields and check
    //        that a set of messages for each GNSS was seen. Check for duplicate SVs, ...

    // Satellite info.
    for (auto& gsv_azel : gsv.azels) {
        // Also check that we don't have this SV already *sat* info is reported repeatedly in the GSV message*s* if
        // there are multiple signals tracked for the satellite.
        auto entry = std::find_if(sats_.cbegin(), sats_.cend(), [&gsv_azel](const auto& cand) {
            return (cand.system_ == gsv_azel.system) && (cand.svid_ == gsv_azel.svid);
        });
        if (gsv_azel.valid && (entry == sats_.cend())) {
            Sat sat;
            sat.system_ = gsv_azel.system;
            sat.svid_ = gsv_azel.svid;
            sat.az_ = gsv_azel.az;
            sat.el_ = gsv_azel.el;
            NMEA_TRACE("AddGsv() sat %c %d %d %d", types::EnumToVal(sat.system_), sat.svid_, sat.az_, sat.el_);
            sats_.push_back(sat);
        }
    }

    // Signal info
    for (auto& gsv_cno : gsv.cnos) {
        if (gsv_cno.valid) {
            Sig sig;
            sig.system_ = gsv_cno.system;
            sig.svid_ = gsv_cno.svid;
            sig.signal_ = gsv_cno.signal;
            sig.cno_ = gsv_cno.cno;
            // If satellite is listed as used in GSA, assume the signal is used. Even though it may be wrong (e.g. for a
            // sat only L1 is used and L2 is only tracked) this is the best we can do with the NMEA data.
            const auto entry = std::find_if(gsa_sats_.cbegin(), gsa_sats_.cend(),
                [&sig](const auto& cand) { return (cand.system == sig.system_) && (cand.svid == sig.svid_); });
            sig.used_ = (entry != gsa_sats_.cend());
            NMEA_TRACE("AddGsv() sig %c %c %d %.1f %s", types::EnumToVal(sig.system_), types::EnumToVal(sig.signal_),
                sig.svid_, sig.cno_, sig.used_ ? "used" : "-");
            sigs_.push_back(sig);
        }
    }

    NMEA_TRACE("AddGsv(%d/%s, %d/%s, %c, %c, %d, %d) sats_=%d sigs_=%d", gsv.msg_num.value,
        string::ToStr(gsv.msg_num.valid), gsv.num_msgs.value, string::ToStr(gsv.num_msgs.valid),
        types::EnumToVal(gsv.system), types::EnumToVal(gsv.signal), gsv.num_azels, gsv.num_cnos, (int)sats_.size(),
        (int)sigs_.size());

    return true;
}

void NmeaCollectGsaGsv::Complete()
{
    // No longer needed
    gsa_sats_.clear();

    // Order by system and svid
    std::sort(sats_.begin(), sats_.end(), [](const auto& a, const auto& b) {
        return (a.system_ == b.system_) ? (a.svid_ < b.svid_) : (a.system_ < b.system_);
    });
    // Order by system, signal and svid
    std::sort(sigs_.begin(), sigs_.end(), [](const auto& a, const auto& b) {
        return (a.system_ == b.system_) ? ((a.signal_ == b.signal_) ? (a.svid_ < b.svid_) : (a.signal_ < b.signal_))
                                        : (a.system_ < b.system_);
    });

    for (std::size_t ix = 0; ix < sats_.size(); ix++) {
        NMEA_TRACE("NmeaCollectGsaGsv() sats_[%d] %c %d %d %d", (int)ix, types::EnumToVal(sats_[ix].system_),
            sats_[ix].svid_, sats_[ix].az_, sats_[ix].el_);
    }
    for (std::size_t ix = 0; ix < sigs_.size(); ix++) {
        NMEA_TRACE("NmeaCollectGsaGsv() sigs_[%d] %c %c %d %.1f %s", (int)ix, types::EnumToVal(sigs_[ix].system_),
            types::EnumToVal(sigs_[ix].signal_), sigs_[ix].svid_, sigs_[ix].cno_, sigs_[ix].used_ ? "used" : "-");
    }
}

/* ****************************************************************************************************************** */
}  // namespace nmea
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
