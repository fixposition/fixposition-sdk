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
 * @brief Fixposition SDK: Parser FP_A routines and types
 */

/* LIBC/STL */
// #include <algorithm>
#include <cinttypes>
#include <cmath>
// #include <cstddef>
// #include <cstdio>
#include <cstring>
// #include <limits>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/fpa.hpp"
#include "fpsdk_common/parser/nmea.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/time.hpp"
#include "fpsdk_common/types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace fpa {
/* ****************************************************************************************************************** */

bool FpaGetMessageMeta(FpaMessageMeta& meta, const uint8_t* msg, const std::size_t msg_size)
{
    if ((msg == NULL) || (msg_size < 10) || (msg[1] != 'F') || (msg[2] != 'P') || (msg[3] != ',')) {
        return false;
    }
    meta.payload_ix0_ = 0;
    meta.payload_ix1_ = 0;
    meta.msg_version_ = -1;

    // 012345678901234567890123456789
    // $FP,ODOMETRY,1,........*XX\r\n
    // $FP,VERSION*XX\r\n
    // $FP,VERSION,*XX\r\n

    std::size_t i_ix = 4;
    std::size_t o_ix = 0;
    meta.msg_type_[0] = '\0';
    for (; (i_ix < (msg_size - 4)) && (o_ix < sizeof(meta.msg_type_)); i_ix++, o_ix++) {
        if ((msg[i_ix] == ',') || (msg[i_ix] == '*')) {
            meta.msg_type_[o_ix] = '\0';
            break;
        } else {
            meta.msg_type_[o_ix] = msg[i_ix];
        }
    }
    if (meta.msg_type_[0] == '\0') {
        return false;
    }

    if (msg[i_ix] == ',') {
        i_ix++;
    }

    if ((msg[i_ix] >= '0') && (msg[i_ix] <= '9') && ((msg[i_ix + 1] == ',') || (msg[i_ix + 1] == '*'))) {
        meta.msg_version_ = msg[i_ix] - '0';
        i_ix++;
        if (msg[i_ix] == ',') {
            i_ix++;
        }
    }

    if (msg[i_ix] != '*') {
        meta.payload_ix0_ = i_ix;
        meta.payload_ix1_ = msg_size - 5 - 1;  // "*XX\r\n"
    }

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((name == NULL) || (size < 1)) {
        return false;
    }
    name[0] = '\0';

    FpaMessageMeta meta;
    if (!FpaGetMessageMeta(meta, msg, msg_size)) {
        return false;
    }

    return std::snprintf(name, size, "FP_A-%s", meta.msg_type_[0] != '\0' ? meta.msg_type_ : "?") < (int)size;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1)) {
        return false;
    }
    std::size_t len = 0;
    FpaMessageMeta meta;
    if (FpaGetMessageMeta(meta, msg, msg_size) && (meta.payload_ix1_ > meta.payload_ix0_)) {
        char fmt[20];
        snprintf(fmt, sizeof(fmt), "%%.%ds", meta.payload_ix1_ - meta.payload_ix0_ + 1);
        len += snprintf(info, size, fmt, (const char*)&msg[meta.payload_ix0_]);
    } else {
        info[0] = '\0';
    }
    return (len > 0) && (len < size);
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaInitStatusStr(const FpaInitStatus status)
{
    switch (status) {  // clang-format off
        case FpaInitStatus::UNSPECIFIED: return "UNSPECIFIED";
        case FpaInitStatus::NOT_INIT:    return "NOT_INIT";
        case FpaInitStatus::LOCAL_INIT:  return "LOCAL_INIT";
        case FpaInitStatus::GLOBAL_INIT: return "GLOBAL_INIT";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaFusionStatusLegacyStr(const FpaFusionStatusLegacy status)
{
    switch (status) {  // clang-format off
        case FpaFusionStatusLegacy::UNSPECIFIED: return "UNSPECIFIED";
        case FpaFusionStatusLegacy::NONE:        return "NONE";
        case FpaFusionStatusLegacy::VISION:      return "VISION";
        case FpaFusionStatusLegacy::VIO:         return "VIO";
        case FpaFusionStatusLegacy::IMU_GNSS:    return "IMU_GNSS";
        case FpaFusionStatusLegacy::VIO_GNSS:    return "VIO_GNSS";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaMeasStatusStr(const FpaMeasStatus status)
{
    switch (status) {  // clang-format off
        case FpaMeasStatus::UNSPECIFIED: return "UNSPECIFIED";
        case FpaMeasStatus::NOT_USED:    return "NOT_USED";
        case FpaMeasStatus::USED:        return "USED";
        case FpaMeasStatus::DEGRADED:    return "DEGRADED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaImuStatusStr(const FpaImuStatus status)
{
    switch (status) {  // clang-format off
        case FpaImuStatus::UNSPECIFIED:     return "UNSPECIFIED";
        case FpaImuStatus::NOT_CONVERGED:   return "NOT_CONVERGED";
        case FpaImuStatus::WARMSTARTED:     return "WARMSTARTED";
        case FpaImuStatus::ROUGH_CONVERGED: return "ROUGH_CONVERGED";
        case FpaImuStatus::FINE_CONVERGED:  return "FINE_CONVERGED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaImuStatusLegacyStr(const FpaImuStatusLegacy status)
{
    switch (status) {  // clang-format off
        case FpaImuStatusLegacy::UNSPECIFIED:   return "UNSPECIFIED";
        case FpaImuStatusLegacy::NOT_CONVERGED: return "NOT_CONVERGED";
        case FpaImuStatusLegacy::CONVERGED:     return "CONVERGED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaImuNoiseStr(const FpaImuNoise noise)
{
    switch (noise) {  // clang-format off
        case FpaImuNoise::UNSPECIFIED:  return "UNSPECIFIED";
        case FpaImuNoise::LOW_NOISE:    return "LOW_NOISE";
        case FpaImuNoise::MEDIUM_NOISE: return "MEDIUM_NOISE";
        case FpaImuNoise::HIGH_NOISE:   return "HIGH_NOISE";
        case FpaImuNoise::RESERVED4:    return "RESERVED4";
        case FpaImuNoise::RESERVED5:    return "RESERVED5";
        case FpaImuNoise::RESERVED6:    return "RESERVED6";
        case FpaImuNoise::RESERVED7:    return "RESERVED7";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaImuConvStr(const FpaImuConv conv)
{
    switch (conv) {  // clang-format off
        case FpaImuConv::UNSPECIFIED:      return "UNSPECIFIED";
        case FpaImuConv::RESERVED0:        return "RESERVED0";
        case FpaImuConv::WAIT_IMU_MEAS:    return "WAIT_IMU_MEAS";
        case FpaImuConv::WAIT_GLOBAL_MEAS: return "WAIT_GLOBAL_MEAS";
        case FpaImuConv::WAIT_MOTION:      return "WAIT_MOTION";
        case FpaImuConv::CONVERGING:       return "CONVERGING";
        case FpaImuConv::RESERVED5:        return "RESERVED5";
        case FpaImuConv::RESERVED6:        return "RESERVED6";
        case FpaImuConv::IDLE:             return "IDLE";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaGnssStatusStr(const FpaGnssStatus status)
{
    switch (status) {  // clang-format off
        case FpaGnssStatus::UNSPECIFIED: return "UNSPECIFIED";
        case FpaGnssStatus::NO_FIX:      return "NO_FIX";
        case FpaGnssStatus::SPP:         return "SPP";
        case FpaGnssStatus::RTK_MB:      return "RTK_MB";
        case FpaGnssStatus::RESERVED3:   return "RESERVED3";
        case FpaGnssStatus::RESERVED4:   return "RESERVED4";
        case FpaGnssStatus::RTK_FLOAT:   return "RTK_FLOAT";
        case FpaGnssStatus::RESERVED6:   return "RESERVED6";
        case FpaGnssStatus::RESERVED7:   return "RESERVED7";
        case FpaGnssStatus::RTK_FIXED:   return "RTK_FIXED";
        case FpaGnssStatus::RESERVED9:   return "RESERVED9";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaCorrStatusStr(const FpaCorrStatus status)
{
    switch (status) {  // clang-format off
        case FpaCorrStatus::UNSPECIFIED:    return "UNSPECIFIED";
        case FpaCorrStatus::WAITING_FUSION: return "WAITING_FUSION";
        case FpaCorrStatus::NO_GNSS:        return "NO_GNSS";
        case FpaCorrStatus::NO_CORR:        return "NO_CORR";
        case FpaCorrStatus::LIMITED_CORR:   return "LIMITED_CORR";
        case FpaCorrStatus::OLD_CORR:       return "OLD_CORR";
        case FpaCorrStatus::GOOD_CORR:      return "GOOD_CORR";
        case FpaCorrStatus::RESERVED6:      return "RESERVED6";
        case FpaCorrStatus::RESERVED7:      return "RESERVED7";
        case FpaCorrStatus::RESERVED8:      return "RESERVED8";
        case FpaCorrStatus::RESERVED9:      return "RESERVED9";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaBaselineStatusStr(const FpaBaselineStatus status)
{
    switch (status) {  // clang-format off
        case FpaBaselineStatus::UNSPECIFIED:    return "UNSPECIFIED";
        case FpaBaselineStatus::WAITING_FUSION: return "WAITING_FUSION";
        case FpaBaselineStatus::NO_FIX:         return "NO_FIX";
        case FpaBaselineStatus::FAILING:        return "FAILING";
        case FpaBaselineStatus::PASSING:        return "PASSING";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaCamStatusStr(const FpaCamStatus status)
{
    switch (status) {  // clang-format off
        case FpaCamStatus::UNSPECIFIED: return "UNSPECIFIED";
        case FpaCamStatus::CAM_UNAVL:   return "CAM_UNAVL";
        case FpaCamStatus::BAD_FEAT:    return "BAD_FEAT";
        case FpaCamStatus::RESERVED2:   return "RESERVED2";
        case FpaCamStatus::RESERVED3:   return "RESERVED3";
        case FpaCamStatus::RESERVED4:   return "RESERVED4";
        case FpaCamStatus::GOOD:        return "GOOD";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaWsStatusStr(const FpaWsStatus status)
{
    switch (status) {  // clang-format off
        case FpaWsStatus::UNSPECIFIED:    return "UNSPECIFIED";
        case FpaWsStatus::NOT_ENABLED:    return "NOT_ENABLED";
        case FpaWsStatus::MISS_MEAS:      return "MISS_MEAS";
        case FpaWsStatus::NONE_CONVERGED: return "NONE_CONVERGED";
        case FpaWsStatus::ONE_CONVERGED:  return "ONE_CONVERGED";
        case FpaWsStatus::ALL_CONVERGED:  return "ALL_CONVERGED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaWsStatusLegacyStr(const FpaWsStatusLegacy status)
{
    switch (status) {  // clang-format off
        case FpaWsStatusLegacy::UNSPECIFIED:           return "UNSPECIFIED";
        case FpaWsStatusLegacy::NOT_ENABLED:           return "NOT_ENABLED";
        case FpaWsStatusLegacy::NONE_CONVERGED:        return "NONE_CONVERGED";
        case FpaWsStatusLegacy::ONE_OR_MORE_CONVERGED: return "ONE_OR_MORE_CONVERGED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaWsConvStr(const FpaWsConv status)
{
    switch (status) {  // clang-format off
        case FpaWsConv::UNSPECIFIED:      return "UNSPECIFIED";
        case FpaWsConv::WAIT_FUSION:      return "WAIT_FUSION";
        case FpaWsConv::WAIT_WS_MEAS:     return "WAIT_WS_MEAS";
        case FpaWsConv::WAIT_GLOBAL_MEAS: return "WAIT_GLOBAL_MEAS";
        case FpaWsConv::WAIT_MOTION:      return "WAIT_MOTION";
        case FpaWsConv::WAIT_IMU_BIAS:    return "WAIT_IMU_BIAS";
        case FpaWsConv::CONVERGING:       return "CONVERGING";
        case FpaWsConv::IDLE:             return "IDLE";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaMarkersStatusStr(const FpaMarkersStatus status)
{
    switch (status) {  // clang-format off
        case FpaMarkersStatus::UNSPECIFIED:    return "UNSPECIFIED";
        case FpaMarkersStatus::NOT_ENABLED:    return "NOT_ENABLED";
        case FpaMarkersStatus::NONE_CONVERGED: return "NONE_CONVERGED";
        case FpaMarkersStatus::ONE_CONVERGED:  return "ONE_CONVERGED";
        case FpaMarkersStatus::ALL_CONVERGED:  return "ALL_CONVERGED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaMarkersConvStr(const FpaMarkersConv conv)
{
    switch (conv) {  // clang-format off
        case FpaMarkersConv::UNSPECIFIED:      return "UNSPECIFIED";
        case FpaMarkersConv::WAIT_FUSION:      return "WAIT_FUSION";
        case FpaMarkersConv::WAIT_MARKER_MEAS: return "WAIT_MARKER_MEAS";
        case FpaMarkersConv::WAIT_GLOBAL_MEAS: return "WAIT_GLOBAL_MEAS";
        case FpaMarkersConv::CONVERGING:       return "CONVERGING";
        case FpaMarkersConv::IDLE:             return "IDLE";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaGnssFixStr(const FpaGnssFix fix)
{
    switch (fix) {  // clang-format off
        case FpaGnssFix::UNSPECIFIED: return "UNSPECIFIED";
        case FpaGnssFix::UNKNOWN:     return "UNKNOWN";
        case FpaGnssFix::NOFIX:       return "NOFIX";
        case FpaGnssFix::DRONLY:      return "DRONLY";
        case FpaGnssFix::TIME:        return "TIME";
        case FpaGnssFix::S2D:         return "S2D";
        case FpaGnssFix::S3D:         return "S3D";
        case FpaGnssFix::S3D_DR:      return "S3D_DR";
        case FpaGnssFix::RTK_FLOAT:   return "RTK_FLOAT";
        case FpaGnssFix::RTK_FIXED:   return "RTK_FIXED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaEpochStr(const FpaEpoch epoch)
{
    switch (epoch) {  // clang-format off
        case FpaEpoch::UNSPECIFIED: return "UNSPECIFIED";
        case FpaEpoch::GNSS1:       return "GNSS1";
        case FpaEpoch::GNSS2:       return "GNSS2";
        case FpaEpoch::GNSS:        return "GNSS";
        case FpaEpoch::FUSION:      return "FUSION";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaAntStateStr(const FpaAntState state)
{
    switch (state) {  // clang-format off
        case FpaAntState::UNSPECIFIED: return "UNSPECIFIED";
        case FpaAntState::OPEN:        return "OPEN";
        case FpaAntState::OK:          return "OK";
        case FpaAntState::SHORT:       return "SHORT";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaAntPowerStr(const FpaAntPower power)
{
    switch (power) {  // clang-format off
        case FpaAntPower::UNSPECIFIED: return "UNSPECIFIED";
        case FpaAntPower::ON:          return "ON";
        case FpaAntPower::OFF:         return "OFF";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaTextLevelStr(const FpaTextLevel level)
{
    switch (level) {  // clang-format off
        case FpaTextLevel::UNSPECIFIED: return "UNSPECIFIED";
        case FpaTextLevel::ERROR:       return "ERROR";
        case FpaTextLevel::WARNING:     return "WARNING";
        case FpaTextLevel::INFO:        return "INFO";
        case FpaTextLevel::DEBUG:       return "DEBUG";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaTimebaseStr(const FpaTimebase base)
{
    switch (base) {  // clang-format off
        case FpaTimebase::UNSPECIFIED: return "UNSPECIFIED";
        case FpaTimebase::NONE:        return "NONE";
        case FpaTimebase::GNSS:        return "GNSS";
        case FpaTimebase::UTC:         return "UTC";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* FpaTimerefStr(const FpaTimeref ref)
{
    switch (ref) {  // clang-format off
        case FpaTimeref::UNSPECIFIED: return "UNSPECIFIED";
        case FpaTimeref::UTC_NONE:    return "UTC_NONE";
        case FpaTimeref::UTC_CRL:     return "UTC_CRL";
        case FpaTimeref::UTC_NIST:    return "UTC_NIST";
        case FpaTimeref::UTC_USNO:    return "UTC_USNO";
        case FpaTimeref::UTC_BIPM:    return "UTC_BIPM";
        case FpaTimeref::UTC_EU:      return "UTC_EU";
        case FpaTimeref::UTC_SU:      return "UTC_SU";
        case FpaTimeref::UTC_NTSC:    return "UTC_NTSC";
        case FpaTimeref::GNSS_GPS:    return "GNSS_GPS";
        case FpaTimeref::GNSS_GAL:    return "GNSS_GAL";
        case FpaTimeref::GNSS_BDS:    return "GNSS_BDS";
        case FpaTimeref::GNSS_GLO:    return "GNSS_GLO";
        case FpaTimeref::GNSS_NVC:    return "GNSS_NVC";
        case FpaTimeref::OTHER:       return "OTHER";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------
// Various helpers for the SetFromMsg() FP_A decoding functions

// Debug prints, for development
#if 0
#  define FPA_TRACE(fmt, ...) fprintf(stderr, fmt "\n", ##__VA_ARGS__)
#else
#  define FPA_TRACE(fmt, ...) /* nothing */
#endif

// ---------------------------------------------------------------------------------------------------------------------

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

// Split NMEA sentence (message) into its meta data and the payload fields
struct FpaParts
{
    FpaMessageMeta meta_;
    std::vector<std::string> fields_;
};

static bool GetParts(FpaParts& parts, const char* msg_type, const uint8_t* msg, const std::size_t msg_size)
{
    bool ok = true;

    if (!FpaGetMessageMeta(parts.meta_, msg, msg_size) || (parts.meta_.payload_ix0_ <= 0) ||
        (parts.meta_.payload_ix1_ <= 0)) {
        ok = false;
    } else {
        if (std::strcmp(parts.meta_.msg_type_, msg_type) != 0) {
            ok = false;
        }

        std::string payload((const char*)&msg[parts.meta_.payload_ix0_],
            (const char*)&msg[parts.meta_.payload_ix0_] + (parts.meta_.payload_ix1_ - parts.meta_.payload_ix0_ + 1));

        FPA_TRACE("GetParts(...) msg_type=%s msg_version=%d payload=%s ix0=%d ix1=%d size=%" PRIuMAX,
            parts.meta_.msg_type_, parts.meta_.msg_version_, payload.c_str(), parts.meta_.payload_ix0_,
            parts.meta_.payload_ix1_, msg_size);

        std::size_t pos = 0;
        while ((pos = payload.find(",")) != std::string::npos) {
            std::string part = payload.substr(0, pos);
            payload.erase(0, pos + 1);
            parts.fields_.push_back(part);
        }
        parts.fields_.push_back(payload);
    }

    FPA_TRACE("GetParts(..., \"%s\", ..., ...)=%s #fields=%d", msg_type, string::ToStr(ok), (int)parts.fields_.size());
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

static bool GetInitStatus(FpaInitStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaInitStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaInitStatus)field[0]) {  // clang-format off
            case FpaInitStatus::NOT_INIT:     status = FpaInitStatus::NOT_INIT;    ok = true; break;
            case FpaInitStatus::LOCAL_INIT:   status = FpaInitStatus::LOCAL_INIT;  ok = true; break;
            case FpaInitStatus::GLOBAL_INIT:  status = FpaInitStatus::GLOBAL_INIT; ok = true; break;
            case FpaInitStatus::UNSPECIFIED:  break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetInitStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetFusionStatusLegacy(FpaFusionStatusLegacy& status, const std::string& field)
{
    bool ok = false;
    status = FpaFusionStatusLegacy::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaFusionStatusLegacy)field[0]) {  // clang-format off
            case FpaFusionStatusLegacy::NONE:         status = FpaFusionStatusLegacy::NONE;     ok = true; break;
            case FpaFusionStatusLegacy::VISION:       status = FpaFusionStatusLegacy::VISION;   ok = true; break;
            case FpaFusionStatusLegacy::VIO:          status = FpaFusionStatusLegacy::VIO;      ok = true; break;
            case FpaFusionStatusLegacy::IMU_GNSS:     status = FpaFusionStatusLegacy::IMU_GNSS; ok = true; break;
            case FpaFusionStatusLegacy::VIO_GNSS:     status = FpaFusionStatusLegacy::VIO_GNSS; ok = true; break;
            case FpaFusionStatusLegacy::UNSPECIFIED:  break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetFusionStatusLegacy(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetMeasStatus(FpaMeasStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaMeasStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaMeasStatus)field[0]) {  // clang-format off
            case FpaMeasStatus::NOT_USED:     status = FpaMeasStatus::NOT_USED; ok = true; break;
            case FpaMeasStatus::USED:         status = FpaMeasStatus::USED;     ok = true; break;
            case FpaMeasStatus::DEGRADED:     status = FpaMeasStatus::DEGRADED; ok = true; break;
            case FpaMeasStatus::UNSPECIFIED:  break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetMeasStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetImuStatus(FpaImuStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaImuStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaImuStatus)field[0]) {  // clang-format off
            case FpaImuStatus::NOT_CONVERGED:    status = FpaImuStatus::NOT_CONVERGED;   ok = true; break;
            case FpaImuStatus::WARMSTARTED:      status = FpaImuStatus::WARMSTARTED;     ok = true; break;
            case FpaImuStatus::ROUGH_CONVERGED:  status = FpaImuStatus::ROUGH_CONVERGED; ok = true; break;
            case FpaImuStatus::FINE_CONVERGED:   status = FpaImuStatus::FINE_CONVERGED;  ok = true; break;
            case FpaImuStatus::UNSPECIFIED:      break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetImuStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetImuStatusLegacy(FpaImuStatusLegacy& status, const std::string& field)
{
    bool ok = false;
    status = FpaImuStatusLegacy::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaImuStatusLegacy)field[0]) {  // clang-format off
            case FpaImuStatusLegacy::NOT_CONVERGED:  status = FpaImuStatusLegacy::NOT_CONVERGED; ok = true; break;
            case FpaImuStatusLegacy::CONVERGED:      status = FpaImuStatusLegacy::CONVERGED;     ok = true; break;
            case FpaImuStatusLegacy::UNSPECIFIED:    break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetImuStatusLegacy(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetImuNoise(FpaImuNoise& noise, const std::string& field)
{
    bool ok = false;
    noise = FpaImuNoise::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaImuNoise)field[0]) {  // clang-format off
            case FpaImuNoise::LOW_NOISE:     noise = FpaImuNoise::LOW_NOISE;    ok = true; break;
            case FpaImuNoise::MEDIUM_NOISE:  noise = FpaImuNoise::MEDIUM_NOISE; ok = true; break;
            case FpaImuNoise::HIGH_NOISE:    noise = FpaImuNoise::HIGH_NOISE;   ok = true; break;
            case FpaImuNoise::RESERVED4:     noise = FpaImuNoise::RESERVED4;    ok = true; break;
            case FpaImuNoise::RESERVED5:     noise = FpaImuNoise::RESERVED5;    ok = true; break;
            case FpaImuNoise::RESERVED6:     noise = FpaImuNoise::RESERVED6;    ok = true; break;
            case FpaImuNoise::RESERVED7:     noise = FpaImuNoise::RESERVED7;    ok = true; break;
            case FpaImuNoise::UNSPECIFIED:   break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetImuNoise(\"%s\")=%s noise=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(noise));
    return ok;
}

static bool GetImuConv(FpaImuConv& conv, const std::string& field)
{
    bool ok = false;
    conv = FpaImuConv::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaImuConv)field[0]) {  // clang-format off
            case FpaImuConv::RESERVED0:         conv = FpaImuConv::RESERVED0;        ok = true; break;
            case FpaImuConv::WAIT_IMU_MEAS:     conv = FpaImuConv::WAIT_IMU_MEAS;    ok = true; break;
            case FpaImuConv::WAIT_GLOBAL_MEAS:  conv = FpaImuConv::WAIT_GLOBAL_MEAS; ok = true; break;
            case FpaImuConv::WAIT_MOTION:       conv = FpaImuConv::WAIT_MOTION;      ok = true; break;
            case FpaImuConv::CONVERGING:        conv = FpaImuConv::CONVERGING;       ok = true; break;
            case FpaImuConv::RESERVED5:         conv = FpaImuConv::RESERVED5;        ok = true; break;
            case FpaImuConv::RESERVED6:         conv = FpaImuConv::RESERVED6;        ok = true; break;
            case FpaImuConv::IDLE:              conv = FpaImuConv::IDLE;             ok = true; break;
            case FpaImuConv::UNSPECIFIED:       break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetImuConv(\"%s\")=%s conv=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(conv));
    return ok;
}

static bool GetGnssStatus(FpaGnssStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaGnssStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaGnssStatus)field[0]) {  // clang-format off
            case FpaGnssStatus::NO_FIX:       status = FpaGnssStatus::NO_FIX;    ok = true; break;
            case FpaGnssStatus::SPP:          status = FpaGnssStatus::SPP;       ok = true; break;
            case FpaGnssStatus::RTK_MB:       status = FpaGnssStatus::RTK_MB;    ok = true; break;
            case FpaGnssStatus::RESERVED3:    status = FpaGnssStatus::RESERVED3; ok = true; break;
            case FpaGnssStatus::RESERVED4:    status = FpaGnssStatus::RESERVED4; ok = true; break;
            case FpaGnssStatus::RTK_FLOAT:    status = FpaGnssStatus::RTK_FLOAT; ok = true; break;
            case FpaGnssStatus::RESERVED6:    status = FpaGnssStatus::RESERVED6; ok = true; break;
            case FpaGnssStatus::RESERVED7:    status = FpaGnssStatus::RESERVED7; ok = true; break;
            case FpaGnssStatus::RTK_FIXED:    status = FpaGnssStatus::RTK_FIXED; ok = true; break;
            case FpaGnssStatus::RESERVED9:    status = FpaGnssStatus::RESERVED9; ok = true; break;
            case FpaGnssStatus::UNSPECIFIED:  break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetGnssStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetCorrStatus(FpaCorrStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaCorrStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaCorrStatus)field[0]) {  // clang-format off
            case FpaCorrStatus::WAITING_FUSION:  status = FpaCorrStatus::WAITING_FUSION; ok = true; break;
            case FpaCorrStatus::NO_GNSS:         status = FpaCorrStatus::NO_GNSS;        ok = true; break;
            case FpaCorrStatus::NO_CORR:         status = FpaCorrStatus::NO_CORR;        ok = true; break;
            case FpaCorrStatus::LIMITED_CORR:    status = FpaCorrStatus::LIMITED_CORR;   ok = true; break;
            case FpaCorrStatus::OLD_CORR:        status = FpaCorrStatus::OLD_CORR;       ok = true; break;
            case FpaCorrStatus::GOOD_CORR:       status = FpaCorrStatus::GOOD_CORR;      ok = true; break;
            case FpaCorrStatus::RESERVED6:       status = FpaCorrStatus::RESERVED6;      ok = true; break;
            case FpaCorrStatus::RESERVED7:       status = FpaCorrStatus::RESERVED7;      ok = true; break;
            case FpaCorrStatus::RESERVED8:       status = FpaCorrStatus::RESERVED8;      ok = true; break;
            case FpaCorrStatus::RESERVED9:       status = FpaCorrStatus::RESERVED9;      ok = true; break;
            case FpaCorrStatus::UNSPECIFIED:     break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetCorrStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetBaselineStatus(FpaBaselineStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaBaselineStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaBaselineStatus)field[0]) {  // clang-format off
            case FpaBaselineStatus::WAITING_FUSION:  status = FpaBaselineStatus::WAITING_FUSION; ok = true; break;
            case FpaBaselineStatus::NO_FIX:          status = FpaBaselineStatus::NO_FIX;         ok = true; break;
            case FpaBaselineStatus::FAILING:         status = FpaBaselineStatus::FAILING;        ok = true; break;
            case FpaBaselineStatus::PASSING:         status = FpaBaselineStatus::PASSING;        ok = true; break;
            case FpaBaselineStatus::UNSPECIFIED:     break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetBaselineStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetCamStatus(FpaCamStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaCamStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaCamStatus)field[0]) {  // clang-format off
            case FpaCamStatus::CAM_UNAVL:    status = FpaCamStatus::CAM_UNAVL; ok = true; break;
            case FpaCamStatus::BAD_FEAT:     status = FpaCamStatus::BAD_FEAT;  ok = true; break;
            case FpaCamStatus::RESERVED2:    status = FpaCamStatus::RESERVED2; ok = true; break;
            case FpaCamStatus::RESERVED3:    status = FpaCamStatus::RESERVED3; ok = true; break;
            case FpaCamStatus::RESERVED4:    status = FpaCamStatus::RESERVED4; ok = true; break;
            case FpaCamStatus::GOOD:         status = FpaCamStatus::GOOD;      ok = true; break;
            case FpaCamStatus::UNSPECIFIED:  break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetCamStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetWsStatus(FpaWsStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaWsStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaWsStatus)field[0]) {  // clang-format off
            case FpaWsStatus::NOT_ENABLED:     status = FpaWsStatus::NOT_ENABLED;    ok = true; break;
            case FpaWsStatus::MISS_MEAS:       status = FpaWsStatus::MISS_MEAS;      ok = true; break;
            case FpaWsStatus::NONE_CONVERGED:  status = FpaWsStatus::NONE_CONVERGED; ok = true; break;
            case FpaWsStatus::ONE_CONVERGED:   status = FpaWsStatus::ONE_CONVERGED;  ok = true; break;
            case FpaWsStatus::ALL_CONVERGED:   status = FpaWsStatus::ALL_CONVERGED;  ok = true; break;
            case FpaWsStatus::UNSPECIFIED:     break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetWsStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetWsStatusLegacy(FpaWsStatusLegacy& status, const std::string& field)
{
    bool ok = false;
    status = FpaWsStatusLegacy::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaWsStatusLegacy)field[0]) {  // clang-format off
            case FpaWsStatusLegacy::NOT_ENABLED:            status = FpaWsStatusLegacy::NOT_ENABLED;           ok = true; break;
            case FpaWsStatusLegacy::NONE_CONVERGED:         status = FpaWsStatusLegacy::NONE_CONVERGED;        ok = true; break;
            case FpaWsStatusLegacy::ONE_OR_MORE_CONVERGED:  status = FpaWsStatusLegacy::ONE_OR_MORE_CONVERGED; ok = true; break;
            case FpaWsStatusLegacy::UNSPECIFIED:            break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetWsStatusLegacy(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetWsConv(FpaWsConv& conv, const std::string& field)
{
    bool ok = false;
    conv = FpaWsConv::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaWsConv)field[0]) {  // clang-format off
            case FpaWsConv::WAIT_FUSION:       conv = FpaWsConv::WAIT_FUSION;      ok = true; break;
            case FpaWsConv::WAIT_WS_MEAS:      conv = FpaWsConv::WAIT_WS_MEAS;     ok = true; break;
            case FpaWsConv::WAIT_GLOBAL_MEAS:  conv = FpaWsConv::WAIT_GLOBAL_MEAS; ok = true; break;
            case FpaWsConv::WAIT_MOTION:       conv = FpaWsConv::WAIT_MOTION;      ok = true; break;
            case FpaWsConv::WAIT_IMU_BIAS:     conv = FpaWsConv::WAIT_IMU_BIAS;    ok = true; break;
            case FpaWsConv::CONVERGING:        conv = FpaWsConv::CONVERGING;       ok = true; break;
            case FpaWsConv::IDLE:              conv = FpaWsConv::IDLE;             ok = true; break;
            case FpaWsConv::UNSPECIFIED:       break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetWsConv(\"%s\")=%s conv=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(conv));
    return ok;
}

static bool GetMarkersStatus(FpaMarkersStatus& status, const std::string& field)
{
    bool ok = false;
    status = FpaMarkersStatus::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaMarkersStatus)field[0]) {  // clang-format off
            case FpaMarkersStatus::NOT_ENABLED:     status = FpaMarkersStatus::NOT_ENABLED;    ok = true; break;
            case FpaMarkersStatus::NONE_CONVERGED:  status = FpaMarkersStatus::NONE_CONVERGED; ok = true; break;
            case FpaMarkersStatus::ONE_CONVERGED:   status = FpaMarkersStatus::ONE_CONVERGED;  ok = true; break;
            case FpaMarkersStatus::ALL_CONVERGED:   status = FpaMarkersStatus::ALL_CONVERGED;  ok = true; break;
            case FpaMarkersStatus::UNSPECIFIED:     break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetMarkersStatus(\"%s\")=%s status=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(status));
    return ok;
}

static bool GetMarkersConv(FpaMarkersConv& conv, const std::string& field)
{
    bool ok = false;
    conv = FpaMarkersConv::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaMarkersConv)field[0]) {  // clang-format off
            case FpaMarkersConv::WAIT_FUSION:       conv = FpaMarkersConv::WAIT_FUSION;      ok = true; break;
            case FpaMarkersConv::WAIT_MARKER_MEAS:  conv = FpaMarkersConv::WAIT_MARKER_MEAS; ok = true; break;
            case FpaMarkersConv::WAIT_GLOBAL_MEAS:  conv = FpaMarkersConv::WAIT_GLOBAL_MEAS; ok = true; break;
            case FpaMarkersConv::CONVERGING:        conv = FpaMarkersConv::CONVERGING;       ok = true; break;
            case FpaMarkersConv::IDLE:              conv = FpaMarkersConv::IDLE;             ok = true; break;
            case FpaMarkersConv::UNSPECIFIED:       break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetMarkersConv(\"%s\")=%s conv=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(conv));
    return ok;
}

static bool GetGnssFix(FpaGnssFix& fix, const std::string& field)
{
    bool ok = false;
    fix = FpaGnssFix::UNSPECIFIED;
    if (!field.empty()) {
        switch ((FpaGnssFix)field[0]) {  // clang-format off
            case FpaGnssFix::UNKNOWN:      fix = FpaGnssFix::UNKNOWN;   ok = true; break;
            case FpaGnssFix::NOFIX:        fix = FpaGnssFix::NOFIX;     ok = true; break;
            case FpaGnssFix::DRONLY:       fix = FpaGnssFix::DRONLY;    ok = true; break;
            case FpaGnssFix::TIME:         fix = FpaGnssFix::TIME;      ok = true; break;
            case FpaGnssFix::S2D:          fix = FpaGnssFix::S2D;       ok = true; break;
            case FpaGnssFix::S3D:          fix = FpaGnssFix::S3D;       ok = true; break;
            case FpaGnssFix::S3D_DR:       fix = FpaGnssFix::S3D_DR;    ok = true; break;
            case FpaGnssFix::RTK_FLOAT:    fix = FpaGnssFix::RTK_FLOAT; ok = true; break;
            case FpaGnssFix::RTK_FIXED:    fix = FpaGnssFix::RTK_FIXED; ok = true; break;
            case FpaGnssFix::UNSPECIFIED:  break;
        }  // clang-format on
    } else {
        ok = true;
    }
    FPA_TRACE("GetGnssFix(\"%s\")=%s conv=%c", field.c_str(), string::ToStr(ok), types::EnumToVal(fix));
    return ok;
}

static bool GetEpoch(FpaEpoch& epoch, const std::string& field)
{
    bool ok = true;
    if (!field.empty()) {  // clang-format off
        if      (field == "GNSS1")  { epoch = FpaEpoch::GNSS1;  }
        else if (field == "GNSS2")  { epoch = FpaEpoch::GNSS2;  }
        else if (field == "GNSS")   { epoch = FpaEpoch::GNSS;   }
        else if (field == "FUSION") { epoch = FpaEpoch::FUSION; }
        else { ok = false; }  // clang-format on
    } else {
        ok = false;
    }
    FPA_TRACE("GetEpoch(\"%s\")=%s epoch=%d", field.c_str(), string::ToStr(ok), types::EnumToVal(epoch));
    return ok;
}

static bool GetAntState(FpaAntState& state, const std::string& field)
{
    bool ok = true;  // clang-format off
    if      (field == "ok")    { state = FpaAntState::OK;    }
    else if (field == "open")  { state = FpaAntState::OPEN;  }
    else if (field == "short") { state = FpaAntState::SHORT; }
    else                       { state = FpaAntState::UNSPECIFIED; ok = field.empty(); }  // clang-format on
    FPA_TRACE("GetAntState(\"%s\")=%s state=%d", field.c_str(), string::ToStr(ok), types::EnumToVal(state));
    return ok;
}

static bool GetAntPower(FpaAntPower& power, const std::string& field)
{
    bool ok = true;  // clang-format off
    if      (field == "on")  { power = FpaAntPower::ON;  }
    else if (field == "off") { power = FpaAntPower::OFF; }
    else                     { power = FpaAntPower::UNSPECIFIED; ok = field.empty(); }  // clang-format on
    FPA_TRACE("GetAntPower(\"%s\")=%s power=%d", field.c_str(), string::ToStr(ok), types::EnumToVal(power));
    return ok;
}

static bool GetTextLevel(FpaTextLevel& level, const std::string& field)
{
    bool ok = true;  // clang-format off
    if      (field == "ERROR")   { level = FpaTextLevel::ERROR; }
    else if (field == "WARNING") { level = FpaTextLevel::WARNING; }
    else if (field == "INFO")    { level = FpaTextLevel::INFO; }
    else if (field == "DEBUG")   { level = FpaTextLevel::DEBUG; }
    else                         { level = FpaTextLevel::UNSPECIFIED; ok = field.empty(); }  // clang-format on
    FPA_TRACE("GetTextLevel(\"%s\")=%s level=%d", field.c_str(), string::ToStr(ok), types::EnumToVal(level));
    return ok;
}

static bool GetTimebase(FpaTimebase& base, const std::string& field)
{
    bool ok = true;  // clang-format off
    if      (field == "")     { base = FpaTimebase::NONE; }
    else if (field == "GNSS") { base = FpaTimebase::GNSS; }
    else if (field == "UTC")  { base = FpaTimebase::UTC;  }
    else if (field == "NONE") { base = FpaTimebase::NONE; }
    else                      { base = FpaTimebase::UNSPECIFIED; ok = false; }  // clang-format on
    FPA_TRACE("GetTimebase(\"%s\")=%s base=%d", field.c_str(), string::ToStr(ok), types::EnumToVal(base));
    return ok;
}

static bool GetTimeref(FpaTimeref& ref, const std::string& field)
{
    bool ok = true;  // clang-format off
    if      (field == "NONE")  { ref = FpaTimeref::UTC_NONE; }
    else if (field == "CRL")   { ref = FpaTimeref::UTC_CRL;  }
    else if (field == "NIST")  { ref = FpaTimeref::UTC_NIST; }
    else if (field == "USNO")  { ref = FpaTimeref::UTC_USNO; }
    else if (field == "BIPM")  { ref = FpaTimeref::UTC_BIPM; }
    else if (field == "EU")    { ref = FpaTimeref::UTC_EU;   }
    else if (field == "SU")    { ref = FpaTimeref::UTC_SU;   }
    else if (field == "NTSC")  { ref = FpaTimeref::UTC_NTSC; }
    else if (field == "GPS")   { ref = FpaTimeref::GNSS_GPS; }
    else if (field == "GAL")   { ref = FpaTimeref::GNSS_GAL; }
    else if (field == "BDS")   { ref = FpaTimeref::GNSS_BDS; }
    else if (field == "GLO")   { ref = FpaTimeref::GNSS_GLO; }
    else if (field == "NVC")   { ref = FpaTimeref::GNSS_NVC; }
    else if (field == "OTHER") { ref = FpaTimeref::OTHER;    }
    else                       { ref = FpaTimeref::UNSPECIFIED; }  // clang-format on
    FPA_TRACE("GetTimeref(\"%s\")=%s ref=%d", field.c_str(), string::ToStr(ok), types::EnumToVal(ref));
    return ok;
}

static constexpr int INAN = std::numeric_limits<int>::max();

// Get integer value
static bool GetInt(
    FpaInt& fpaint, const std::string& field, const bool required, const int min = INAN, const int max = INAN)
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
                fpaint.value = value;
                fpaint.valid = true;
            }
        } else {
            ok = false;
        }
    }

    FPA_TRACE("GetInt(\"%s\", %s, %d, %d)=%s value=%d/%s", field.c_str(), string::ToStr(required), min, max,
        string::ToStr(ok), fpaint.value, string::ToStr(fpaint.valid));
    return ok;
}

// Get float value
static bool GetFloat(
    FpaFloat& fpafloat, const std::string& field, const bool required, const double min = NAN, const double max = NAN)
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
                fpafloat.value = value;
                fpafloat.valid = true;
            }
        } else {
            ok = false;
        }
    }

    FPA_TRACE("GetFloat(\"%s\", %s, %g, %g)=%s value=%g/%s", field.c_str(), string::ToStr(required), min, max,
        string::ToStr(ok), fpafloat.value, string::ToStr(fpafloat.valid));
    return ok;
}

template <typename T>
static bool GetFloatArr(T& fpafloatx, const std::vector<std::string>& fields, const int offs, const bool required,
    const double min = NAN, const double max = NAN)
{
    bool ok = true;
    bool valid = true;
    static constexpr std::size_t N = std::tuple_size<decltype(T::values)>::value;
    for (std::size_t ix = 0; ix < N; ix++) {
        FpaFloat f;
        ok = ok && GetFloat(f, fields[offs + ix], required, min, max);
        valid = valid && f.valid;
        fpafloatx.values[ix] = f.value;
    }
    fpafloatx.valid = valid;
    FPA_TRACE("GetFloatArr<%d>(..., %s, %g, %g)=%s values=.../%s", (int)N, string::ToStr(required), min, max,
        string::ToStr(ok), string::ToStr(fpafloatx.valid));

    //  fields[offs].c_str(),
    //     fields[offs + 1].c_str(), fields[offs + 2].c_str(), , fpafloat3.values[0], fpafloat3.values[1],
    //     fpafloat3.values[2], string::ToStr(fpafloat3.valid));
    return ok;
}

static bool GetFloat3llh(
    FpaFloat3& fpafloat3, const std::vector<std::string>& fields, const int offs, const bool required)
{
    FpaFloat lat;
    FpaFloat lon;
    FpaFloat height;
    const bool ok = GetFloat(lat, fields[offs], required, -90.0, 90.0) &&
                    GetFloat(lon, fields[offs + 1], required, -180.0, 180.0) &&
                    GetFloat(height, fields[offs + 2], required, -1000.0, 50000.0);
    fpafloat3.valid = lat.valid && lon.valid && height.valid;
    fpafloat3.values[0] = lat.value;
    fpafloat3.values[1] = lon.value;
    fpafloat3.values[2] = height.value;
    FPA_TRACE("GetFloat3llh(\"%s\", \"%s\", \"%s\", %s)=%s values=%g/%g/%g/%s", fields[offs].c_str(),
        fields[offs + 1].c_str(), fields[offs + 2].c_str(), string::ToStr(required), string::ToStr(ok),
        fpafloat3.values[0], fpafloat3.values[1], fpafloat3.values[2], string::ToStr(fpafloat3.valid));
    return ok;
}

// Get GPS week/tow pair
static bool GetGpsTime(
    FpaGpsTime& gps_time, const std::vector<std::string>& fields, const int offs, const bool required = false)
{
    bool ok =
        GetInt(gps_time.week, fields[offs], required, time::WnoTow::SANE_WNO_MIN, time::WnoTow::SANE_WNO_MAX) &&
        GetFloat(gps_time.tow, fields[offs + 1], required, time::WnoTow::SANE_TOW_MIN, time::WnoTow::SANE_TOW_MAX);
    FPA_TRACE("GetGpsTime(\"%s\", \"%s\")=%s week=%d/%s tow=%.6f/%s", fields[offs].c_str(), fields[offs + 1].c_str(),
        string::ToStr(ok), gps_time.week.value, string::ToStr(gps_time.week.valid), gps_time.tow.value,
        string::ToStr(gps_time.tow.valid));
    return ok;
}

bool FpaGpsTime::operator==(const FpaGpsTime& rhs) const
{
    return (week.valid == rhs.week.valid) && (tow.valid == rhs.tow.valid) && (week.value == rhs.week.value) &&
           (std::fabs(tow.value - rhs.tow.value) < 1e-6);
}

bool FpaGpsTime::operator!=(const FpaGpsTime& rhs) const
{
    return !(*this == rhs);
}

bool FpaGpsTime::operator>(const FpaGpsTime& rhs) const
{
    return (week.valid == rhs.week.valid) && (tow.valid == rhs.tow.valid) &&
           ((week.value > rhs.week.value) || ((week.value == rhs.week.value) && (tow.value > rhs.tow.value)));
}

bool FpaGpsTime::operator<(const FpaGpsTime& rhs) const
{
    return (week.valid == rhs.week.valid) && (tow.valid == rhs.tow.valid) &&
           ((week.value < rhs.week.value) || ((week.value == rhs.week.value) && (tow.value < rhs.tow.value)));
}

bool FpaGpsTime::operator>=(const FpaGpsTime& rhs) const
{
    return (week.valid == rhs.week.valid) && (tow.valid == rhs.tow.valid) &&
           ((week.value > rhs.week.value) || ((week.value == rhs.week.value) && (tow.value >= rhs.tow.value)));
}

bool FpaGpsTime::operator<=(const FpaGpsTime& rhs) const
{
    return (week.valid == rhs.week.valid) && (tow.valid == rhs.tow.valid) &&
           ((week.value < rhs.week.value) || ((week.value == rhs.week.value) && (tow.value <= rhs.tow.value)));
}

static bool GetText(char* str, const std::size_t size, const std::string& field, const bool required = false)
{
    bool ok = true;
    if (field.empty()) {
        if (required) {
            ok = false;
        } else {
            str[0] = '\0';
        }
    } else if (field.size() > (size - 1)) {
        ok = false;
    } else {
        std::memcpy(str, field.data(), field.size() + 1);
    }
    FPA_TRACE("GetText(..., %" PRIuMAX ", \"%s\", %s)=%s str=%s", size, field.c_str(), string::ToStr(required),
        string::ToStr(ok), str);
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaEoePayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $FP,EOE,1,2322,309663.800000,FUSION*62
    //             0       1        2
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "EOE", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 3)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetEpoch(epoch, m.fields_[2]));
    }
    FPA_TRACE("FpaEoePayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::EOE;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaGnssantPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $FP,GNSSANT,1,2234,305129.200151,short,off,0,open,on,28*65\r\n
    //               0       1            2    3  4   5  6  7
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "GNSSANT", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 8)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetAntState(gnss1_state, m.fields_[2]) &&
              GetAntPower(gnss1_power, m.fields_[3]) && GetInt(gnss1_age, m.fields_[4], false, 0) &&
              GetAntState(gnss2_state, m.fields_[5]) && GetAntPower(gnss2_power, m.fields_[6]) &&
              GetInt(gnss2_age, m.fields_[7], false, 0));
    }
    FPA_TRACE("FpaGnssantPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::GNSSANT;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaGnsscorrPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // $FP,GNSSCORR,1,2237,250035.999865,8,25,18,8,25,18,0.2,1.0,0.8,5.3,2,47.366986804,8.532965023,481.1094,7254*3F
    //                  0       1        2  3 4  5 6  7  8   9   10  11 12 13           14          15       16
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "GNSSCORR", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 17)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetGnssFix(gnss1_fix, m.fields_[2]) &&
              GetInt(gnss1_nsig_l1, m.fields_[3], false, 0) && GetInt(gnss1_nsig_l2, m.fields_[4], false, 0) &&
              GetGnssFix(gnss2_fix, m.fields_[5]) && GetInt(gnss2_nsig_l1, m.fields_[6], false, 0) &&
              GetInt(gnss2_nsig_l2, m.fields_[7], false, 0) && GetFloat(corr_latency, m.fields_[8], false) &&
              GetFloat(corr_update_rate, m.fields_[9], false, 0.0) &&
              GetFloat(corr_data_rate, m.fields_[10], false, 0.0) &&
              GetFloat(corr_msg_rate, m.fields_[11], false, 0.0) && GetInt(sta_id, m.fields_[12], false, 0, 4095) &&
              GetFloat3llh(sta_llh, m.fields_, 13, false) && GetInt(sta_dist, m.fields_[16], false, 0));
    }
    FPA_TRACE("FpaGnsscorrPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::GNSSCORR;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaRawimuPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // v1
    // $FP,RAWIMU,1,2197,126191.777855,-0.199914,0.472851,9.917973,0.023436,0.007723,0.002131*34
    //              0    1             2         3        4        5        6        7
    // v2
    // $FP,RAWIMU,2,2368,228734.073983,-0.118514,-0.082600,9.894035,0.014381,0.004794,-0.003196,0,1,*1C
    //               0    1             2         3         4        5        6        7        8 9 10
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "RAWIMU", msg, msg_size)) {
        if ((m.meta_.msg_version_ == 1) && (m.fields_.size() == 8)) {
            ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(acc, m.fields_, 2, false) &&
                  GetFloatArr(rot, m.fields_, 5, false));
        } else if ((m.meta_.msg_version_ == 2) && (m.fields_.size() == 11)) {
            FpaInt bias;
            ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(acc, m.fields_, 2, false) &&
                  GetFloatArr(rot, m.fields_, 5, false) && GetInt(bias, m.fields_[8], true, 0, 1) && bias.valid &&
                  GetImuStatus(imu_status, m.fields_[9]));
            bias_comp = (bias.value == 1);
        }
    }
    FPA_TRACE("FpaRawimuPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::RAWIMU;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaCorrimuPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // v1
    // $FP,CORRIMU,1,2197,126191.777855,-0.195224,0.393969,9.869998,0.013342,-0.004620,-0.000728*7D
    //             0      1             2         3        4        5        6         7
    // v2
    // $FP,CORRIMU,2,2368,228734.073983,-0.102908,-0.096532,9.732782,0.002208,0.001499,-0.001206,1,1,*54
    //             0      1             2         3        4        5        6         7         8 9 10
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "CORRIMU", msg, msg_size)) {
        if ((m.meta_.msg_version_ == 1) && (m.fields_.size() == 8)) {
            ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(acc, m.fields_, 2, false) &&
                  GetFloatArr(rot, m.fields_, 5, false));
        } else if ((m.meta_.msg_version_ == 2) && (m.fields_.size() == 11)) {
            FpaInt bias;
            ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(acc, m.fields_, 2, false) &&
                  GetFloatArr(rot, m.fields_, 5, false) && GetInt(bias, m.fields_[8], true, 0, 1) && bias.valid &&
                  GetImuStatus(imu_status, m.fields_[9]));
            bias_comp = (bias.value == 1);
        }
    }
    FPA_TRACE("FpaCorrimuPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::CORRIMU;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaImubiasPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,IMUBIAS,1,2342,321247.000000,2,1,1,3,0.008914,0.019806,0.150631,0.027202,0.010599,0.011393,0.00001,0.00001,0.00001,0.00001,0.00001,0.00001*4C\r\n
    //               0      1           2 3 4 5 6        7        8        9        10       11       12      13      14      15      16      17
    // clang-format on
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "IMUBIAS", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 18)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetMeasStatus(fusion_imu, m.fields_[2]) &&
              GetImuStatus(imu_status, m.fields_[3]) && GetImuNoise(imu_noise, m.fields_[4]) &&
              GetImuConv(imu_conv, m.fields_[5]) && GetFloatArr(bias_acc, m.fields_, 6, false) &&
              GetFloatArr(bias_gyr, m.fields_, 9, false) && GetFloatArr(bias_cov_acc, m.fields_, 12, false) &&
              GetFloatArr(bias_cov_gyr, m.fields_, 15, false));
    }
    FPA_TRACE("FpaImubiasPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::IMUBIAS;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaLlhPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,LLH,1,2231,227563.250000,47.392357470,8.448121451,473.5857,0.04533,0.03363,0.02884,0.00417,0.00086,-0.00136*62\r\n
    //           0    1             2            3           4        5       6       7       8       9       10
    // clang-format on
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "LLH", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 11)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloat3llh(llh, m.fields_, 2, false) &&
              GetFloatArr(cov_enu, m.fields_, 5, false));
    }
    FPA_TRACE("FpaLlhPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::LLH;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaOdometryPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "ODOMETRY", msg, msg_size) && (m.meta_.msg_version_ == 2) && (m.fields_.size() == 42)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(pos, m.fields_, 2, false) &&
              GetFloatArr(orientation, m.fields_, 5, false) && GetFloatArr(vel, m.fields_, 9, false) &&
              GetFloatArr(rot, m.fields_, 12, false) && GetFloatArr(acc, m.fields_, 15, false) &&
              GetFusionStatusLegacy(fusion_status, m.fields_[18]) &&
              GetImuStatusLegacy(imu_bias_status, m.fields_[19]) && GetGnssFix(gnss1_fix, m.fields_[20]) &&
              GetGnssFix(gnss2_fix, m.fields_[21]) && GetWsStatusLegacy(wheelspeed_status, m.fields_[22]) &&
              GetFloatArr(pos_cov, m.fields_, 23, false) && GetFloatArr(orientation_cov, m.fields_, 29, false) &&
              GetFloatArr(vel_cov, m.fields_, 35, false) && GetText(version, sizeof(version), m.fields_[41]));
    }
    FPA_TRACE("FpaOdometryPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::ODOMETRY;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaOdomenuPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "ODOMENU", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 41)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(pos, m.fields_, 2, false) &&
              GetFloatArr(orientation, m.fields_, 5, false) && GetFloatArr(vel, m.fields_, 9, false) &&
              GetFloatArr(rot, m.fields_, 12, false) && GetFloatArr(acc, m.fields_, 15, false) &&
              GetFusionStatusLegacy(fusion_status, m.fields_[18]) &&
              GetImuStatusLegacy(imu_bias_status, m.fields_[19]) && GetGnssFix(gnss1_fix, m.fields_[20]) &&
              GetGnssFix(gnss2_fix, m.fields_[21]) && GetWsStatusLegacy(wheelspeed_status, m.fields_[22]) &&
              GetFloatArr(pos_cov, m.fields_, 23, false) && GetFloatArr(orientation_cov, m.fields_, 29, false) &&
              GetFloatArr(vel_cov, m.fields_, 35, false));
    }
    FPA_TRACE("FpaOdomenuPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::ODOMENU;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaOdomshPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "ODOMSH", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 41)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetFloatArr(pos, m.fields_, 2, false) &&
              GetFloatArr(orientation, m.fields_, 5, false) && GetFloatArr(vel, m.fields_, 9, false) &&
              GetFloatArr(rot, m.fields_, 12, false) && GetFloatArr(acc, m.fields_, 15, false) &&
              GetFusionStatusLegacy(fusion_status, m.fields_[18]) &&
              GetImuStatusLegacy(imu_bias_status, m.fields_[19]) && GetGnssFix(gnss1_fix, m.fields_[20]) &&
              GetGnssFix(gnss2_fix, m.fields_[21]) && GetWsStatusLegacy(wheelspeed_status, m.fields_[22]) &&
              GetFloatArr(pos_cov, m.fields_, 23, false) && GetFloatArr(orientation_cov, m.fields_, 29, false) &&
              GetFloatArr(vel_cov, m.fields_, 35, false));
    }
    FPA_TRACE("FpaOdomshPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::ODOMSH;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaOdomstatusPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,ODOMSTATUS,1,2342,321241.350000,2,2,1,1,1,1,,0,,,,,,1,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*24
    //                  0    1             2 3 4 5 6 7  9      1516171819202122 2425           37
    // clang-format on

    bool ok = false;
    FpaParts m;
    if (GetParts(m, "ODOMSTATUS", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 38)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetInitStatus(init_status, m.fields_[2]) &&
              GetMeasStatus(fusion_imu, m.fields_[3]) && GetMeasStatus(fusion_gnss1, m.fields_[4]) &&
              GetMeasStatus(fusion_gnss2, m.fields_[5]) && GetMeasStatus(fusion_corr, m.fields_[6]) &&
              GetMeasStatus(fusion_cam1, m.fields_[7]) && GetMeasStatus(fusion_ws, m.fields_[9]) &&
              GetMeasStatus(fusion_markers, m.fields_[10]) && GetImuStatus(imu_status, m.fields_[15]) &&
              GetImuNoise(imu_noise, m.fields_[16]) && GetImuConv(imu_conv, m.fields_[17]) &&
              GetGnssStatus(gnss1_status, m.fields_[18]) && GetGnssStatus(gnss2_status, m.fields_[19]) &&
              GetBaselineStatus(baseline_status, m.fields_[20]) && GetCorrStatus(corr_status, m.fields_[21]) &&
              GetCamStatus(cam1_status, m.fields_[22]) && GetWsStatus(ws_status, m.fields_[24]) &&
              GetWsConv(ws_conv, m.fields_[25]) && GetMarkersStatus(markers_status, m.fields_[26]) &&
              GetMarkersConv(markers_conv, m.fields_[27]));
    }
    FPA_TRACE("FpaOdomstatusPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::ODOMSTATUS;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaTextPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,TEXT,1,INFO,Fixposition AG - www.fixposition.com*09\r\n
    //            0    1
    // clang-format on
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "TEXT", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 2)) {
        ok = (GetTextLevel(level, m.fields_[0]) && GetText(text, sizeof(text), m.fields_[1], false));
    }
    FPA_TRACE("FpaTextPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::TEXT;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaTfPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,TF,2,2233,315835.000000,VRTK,CAM,-0.00000,-0.00000,-0.00000,1.000000,0.000000,0.000000,0.000000*6B\r\n
    //          0    1             2    3   4        5        6        7        8        9        10
    // clang-format on
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "TF", msg, msg_size) && (m.meta_.msg_version_ == 2) && (m.fields_.size() == 11)) {
        ok = (GetGpsTime(gps_time, m.fields_, 0, false) && GetText(frame_a, sizeof(frame_a), m.fields_[2], true) &&
              GetText(frame_b, sizeof(frame_b), m.fields_[3], true) && GetFloatArr(translation, m.fields_, 4, true) &&
              GetFloatArr(orientation, m.fields_, 7, true));
    }
    FPA_TRACE("FpaTfPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::TF;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaTpPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,TP,1,GNSS1,UTC,USNO,195391,0.000000000000,18*4F\r\n           version 1
    // $FP,TP,2,GNSS1,UTC,NONE,124512,0.000000000000,18,2349*66\r\n      version 2
    //          0     1   2    3      4              5  6
    // clang-format on
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "TP", msg, msg_size) && (((m.meta_.msg_version_ == 1) && (m.fields_.size() == 6)) ||
                                                ((m.meta_.msg_version_ == 2) && (m.fields_.size() == 7)))) {
        ok = (GetText(tp_name, sizeof(tp_name), m.fields_[0], true) && GetTimebase(timebase, m.fields_[1]) &&
              GetTimeref(timeref, m.fields_[2]) && GetInt(tp_tow_sec, m.fields_[3], false, 0) &&
              GetFloat(tp_tow_psec, m.fields_[4], false, 0.0, 0.999999999) &&
              GetInt(gps_leaps, m.fields_[5], false, 0) &&
              ((m.meta_.msg_version_ == 1) || GetInt(tp_week, m.fields_[6], false, 0, 9999)));
    }
    FPA_TRACE("FpaTpPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::TP;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FpaVersionPayload::SetFromMsg(const uint8_t* msg, const std::size_t msg_size)
{
    // clang-format off
    // $FP,VERSION,1,fp_vrtk2-release-vr2_2.123.0-456,NAV_VR2,v1.3b,fp-5d6f64,VRTK2_STK,*3F\r\n
    //               0                                1       2     3         4         5
    // clang-format on
    bool ok = false;
    FpaParts m;
    if (GetParts(m, "VERSION", msg, msg_size) && (m.meta_.msg_version_ == 1) && (m.fields_.size() == 6)) {
        ok = GetText(sw_version, sizeof(sw_version), m.fields_[0]) && GetText(hw_name, sizeof(hw_name), m.fields_[1]) &&
             GetText(hw_ver, sizeof(hw_ver), m.fields_[2]) && GetText(hw_uid, sizeof(hw_uid), m.fields_[3]) &&
             GetText(product_model, sizeof(product_model), m.fields_[4]);
    }
    FPA_TRACE("FpaVersionPayload %s", string::ToStr(ok));
    valid_ = ok;
    msg_type_ = FpaMessageType::VERSION;
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

FpaPayloadPtr FpaDecodeMessage(const uint8_t* msg, const std::size_t msg_size)
{
    FpaMessageMeta meta;
    if (!FpaGetMessageMeta(meta, msg, msg_size)) {
        return nullptr;
    }

#define _GEN(_msg_type_, _type_)                           \
    else if (std::strcmp(meta.msg_type_, _msg_type_) == 0) \
    {                                                      \
        auto payload = std::make_unique<_type_>();         \
        if (payload->SetFromMsg(msg, msg_size)) {          \
            return payload;                                \
        }                                                  \
    }

    if (false) {
    }  // clang-format off
    _GEN("EOE",        FpaEoePayload)
    _GEN("GNSSANT",    FpaGnssantPayload)
    _GEN("GNSSCORR",   FpaGnsscorrPayload)
    _GEN("RAWIMU",     FpaRawimuPayload)
    _GEN("CORRIMU",    FpaCorrimuPayload)
    _GEN("IMUBIAS",    FpaImubiasPayload)
    _GEN("LLH",        FpaLlhPayload)
    _GEN("ODOMETRY",   FpaOdometryPayload)
    _GEN("ODOMENU",    FpaOdomenuPayload)
    _GEN("ODOMSH",     FpaOdomshPayload)
    _GEN("ODOMSTATUS", FpaOdomstatusPayload)
    _GEN("TEXT",       FpaTextPayload)
    _GEN("TF",         FpaTfPayload)
    _GEN("TP",         FpaTpPayload)
    _GEN("VERSION",    FpaVersionPayload)
    // clang-format on

#undef _GEN

    return nullptr;
}

/* ****************************************************************************************************************** */
}  // namespace fpa
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
