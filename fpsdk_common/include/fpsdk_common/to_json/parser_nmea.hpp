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
 * @brief Fixposition SDK: to_json() helpers for some fpsdk::common::parser::nmea messages
 */
#ifndef __FPSDK_COMMON_TO_JSON_NMEA_HPP__
#define __FPSDK_COMMON_TO_JSON_NMEA_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <nlohmann/json.hpp>

/* PACKAGE */
#include "../parser/nmea.hpp"

#ifndef _DOXYGEN_  // not documenting these
/* ****************************************************************************************************************** */
namespace fpsdk::common::parser::nmea {

inline void to_json(nlohmann::json& j, const NmeaTalkerId e)
{
    j = NmeaTalkerIdStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaFormatter e)
{
    j = NmeaFormatterStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaQualityGga e)
{
    j = NmeaQualityGgaStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaStatusGllRmc e)
{
    j = NmeaStatusGllRmcStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaModeGllVtg e)
{
    j = NmeaModeGllVtgStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaModeRmcGns e)
{
    j = NmeaModeRmcGnsStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaNavStatusRmc e)
{
    j = NmeaNavStatusRmcStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaOpModeGsa e)
{
    j = NmeaOpModeGsaStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaNavModeGsa e)
{
    j = NmeaNavModeGsaStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaSystemId e)
{
    j = NmeaSystemIdStr(e);
}

inline void to_json(nlohmann::json& j, const NmeaSignalId e)
{
    j = NmeaSignalIdStr(e);
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaTime& m)
{
    if (m.valid) {
        j = nlohmann::json::object({
            { "hours", m.hours },
            { "mins", m.mins },
            { "secs", m.secs },
        });
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaDate& m)
{
    if (m.valid) {
        j = nlohmann::json::object({
            { "years", m.years },
            { "months", m.months },
            { "days", m.days },
        });
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaLlh& m)
{
    if (m.latlon_valid) {
        j = nlohmann::json::object({
            { "lat", m.lat },
            { "lon", m.lon },
        });
        if (m.height_valid) {
            j["height"] = m.height;
        }
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaSat& m)
{
    if (m.valid) {
        j = nlohmann::json::object({
            { "system", m.system },
            { "svid", m.svid },
        });
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaAzEl& m)
{
    if (m.valid) {
        j = nlohmann::json::object({
            { "system", m.system },
            { "svid", m.svid },
            { "el", m.el },
            { "az", m.az },
        });
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaCno& m)
{
    if (m.valid) {
        j = nlohmann::json::object({
            { "system", m.system },
            { "svid", m.svid },
            { "signal", m.signal },
            { "cno", m.cno },
        });
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaInt& m)
{
    if (m.valid) {
        j = m.value;
    } else {
        j = nullptr;
    }
}

inline void to_json(nlohmann::json& j, const NmeaFloat& m)
{
    if (m.valid) {
        j = m.value;
    } else {
        j = nullptr;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaGgaPayload& m)
{
    j = nlohmann::json::object({
        { "time", m.time },
        { "llh", m.llh },
        { "height_msl", m.height_msl },
        { "quality", m.quality },
        { "num_sv", m.num_sv },
        { "hdop", m.hdop },
        { "diff_age", m.diff_age },
        { "diff_sta", m.diff_sta },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaGllPayload& m)
{
    j = nlohmann::json::object({
        { "ll", m.ll },
        { "time", m.time },
        { "status", m.status },
        { "mode", m.mode },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaRmcPayload& m)
{
    j = nlohmann::json::object({
        { "time", m.time },
        { "ll", m.ll },
        { "speed", m.speed },
        { "course", m.course },
        { "date", m.date },
        { "mode", m.mode },
        { "navstatus", m.navstatus },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaVtgPayload& m)
{
    j = nlohmann::json::object({
        { "cogt", m.cogt },
        { "cogm", m.cogm },
        { "sogn", m.sogn },
        { "sogk", m.sogk },
        { "mode", m.mode },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaGstPayload& m)
{
    j = nlohmann::json::object({
        { "time", m.time },
        { "rms_range", m.rms_range },
        { "std_major", m.std_major },
        { "std_minor", m.std_minor },
        { "angle_major", m.angle_major },
        { "std_lat", m.std_lat },
        { "std_lon", m.std_lon },
        { "std_alt", m.std_alt },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaHdtPayload& m)
{
    j = nlohmann::json::object({
        { "heading", m.heading },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaZdaPayload& m)
{
    j = nlohmann::json::object({
        { "time", m.time },
        { "date", m.date },
        { "local_hr", m.local_hr },
        { "local_min", m.local_min },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaGsaPayload& m)
{
    j = nlohmann::json::object({
        { "opmode", m.opmode },
        { "navmode", m.navmode },
        { "sats", m.sats },
        { "num_sats", m.num_sats },
        { "pdop", m.pdop },
        { "hdop", m.hdop },
        { "vdop", m.vdop },
        { "system", m.system },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaGsvPayload& m)
{
    j = nlohmann::json::object({
        { "num_msgs", m.num_msgs },
        { "msg_num", m.msg_num },
        { "tot_num_sat", m.tot_num_sat },
        { "azels", m.azels },
        { "num_azels", m.num_azels },
        { "num_cnos", m.num_cnos },
        { "system", m.system },
        { "signal", m.signal },
    });
}

// ---------------------------------------------------------------------------------------------------------------------

inline void to_json(nlohmann::json& j, const NmeaPayloadPtr& nmea)
{
    j = nlohmann::json::object();
    if (!nmea) {
        return;
    }
    j["_valid"] = nmea->valid_;
    j["_talker"] = nmea->talker_;
    j["_formatter"] = nmea->formatter_;
    if (nmea) {
        switch (nmea->formatter_) {  // clang-format off
            case NmeaFormatter::UNSPECIFIED: break;
            case NmeaFormatter::GGA:         j.update(dynamic_cast<const NmeaGgaPayload&>(*nmea)); break;
            case NmeaFormatter::GLL:         j.update(dynamic_cast<const NmeaGllPayload&>(*nmea)); break;
            case NmeaFormatter::RMC:         j.update(dynamic_cast<const NmeaRmcPayload&>(*nmea)); break;
            case NmeaFormatter::VTG:         j.update(dynamic_cast<const NmeaVtgPayload&>(*nmea)); break;
            case NmeaFormatter::GST:         j.update(dynamic_cast<const NmeaGstPayload&>(*nmea)); break;
            case NmeaFormatter::HDT:         j.update(dynamic_cast<const NmeaHdtPayload&>(*nmea)); break;
            case NmeaFormatter::ZDA:         j.update(dynamic_cast<const NmeaZdaPayload&>(*nmea)); break;
            case NmeaFormatter::GSA:         j.update(dynamic_cast<const NmeaGsaPayload&>(*nmea)); break;
            case NmeaFormatter::GSV:         j.update(dynamic_cast<const NmeaGsvPayload&>(*nmea)); break;
        }  // clang-format on
    }
}

}  // namespace fpsdk::common::parser::nmea
/* ****************************************************************************************************************** */
#endif  // !_DOXYGEN_
#endif  // __FPSDK_COMMON_TO_JSON_NMEA_HPP__
