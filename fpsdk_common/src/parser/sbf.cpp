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
 * @brief Fixposition SDK: Parser SBF routines and types
 */

/* LIBC/STL */
#include <array>
#include <cinttypes>
#include <cstdio>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/parser/sbf.hpp"

namespace fpsdk {
namespace common {
namespace parser {
namespace sbf {
/* ****************************************************************************************************************** */

// Lookup table entry
struct MsgInfo
{
    uint16_t block;
    const char* name;
    const char* desc;
};

// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_SBF_MSGINFO}
static constexpr std::array<MsgInfo, 95> MSG_INFO =
{{
    { SBF_MEASEPOCH_MSGID,             SBF_MEASEPOCH_STRID,             "Measurement set of one epoch" },
    { SBF_MEASEXTRA_MSGID,             SBF_MEASEXTRA_STRID,             "Additional info such as observable variance" },
    { SBF_ENDOFMEAS_MSGID,             SBF_ENDOFMEAS_STRID,             "Measurement epoch marker" },
    { SBF_GPSRAWCA_MSGID,              SBF_GPSRAWCA_STRID,              "GPS CA navigation subframe" },
    { SBF_GPSRAWL2C_MSGID,             SBF_GPSRAWL2C_STRID,             "GPS L2C navigation frame" },
    { SBF_GPSRAWL5_MSGID,              SBF_GPSRAWL5_STRID,              "GPS L5 navigation frame" },
    { SBF_GPSRAWL1C_MSGID,             SBF_GPSRAWL1C_STRID,             "GPS L1C navigation frame" },
    { SBF_GLORAWCA_MSGID,              SBF_GLORAWCA_STRID,              "GLONASS CA navigation string" },
    { SBF_GALRAWFNAV_MSGID,            SBF_GALRAWFNAV_STRID,            "Galileo F/NAV navigation page" },
    { SBF_GALRAWINAV_MSGID,            SBF_GALRAWINAV_STRID,            "Galileo I/NAV navigation page" },
    { SBF_GALRAWCNAV_MSGID,            SBF_GALRAWCNAV_STRID,            "Galileo C/NAV navigation page" },
    { SBF_GEORAWL1_MSGID,              SBF_GEORAWL1_STRID,              "SBAS L1 navigation message" },
    { SBF_GEORAWL5_MSGID,              SBF_GEORAWL5_STRID,              "SBAS L5 navigation message" },
    { SBF_BDSRAW_MSGID,                SBF_BDSRAW_STRID,                "BeiDou navigation page" },
    { SBF_BDSRAWB1C_MSGID,             SBF_BDSRAWB1C_STRID,             "BeiDou B1C navigation frame" },
    { SBF_BDSRAWB2A_MSGID,             SBF_BDSRAWB2A_STRID,             "BeiDou B2a navigation frame" },
    { SBF_BDSRAWB2B_MSGID,             SBF_BDSRAWB2B_STRID,             "BeiDou B2b navigation frame" },
    { SBF_QZSRAWL1CA_MSGID,            SBF_QZSRAWL1CA_STRID,            "QZSS L1C/A or L1C/B navigation frame" },
    { SBF_QZSRAWL2C_MSGID,             SBF_QZSRAWL2C_STRID,             "QZSS L2C navigation frame" },
    { SBF_QZSRAWL5_MSGID,              SBF_QZSRAWL5_STRID,              "QZSS L5 navigation frame" },
    { SBF_QZSRAWL6D_MSGID,             SBF_QZSRAWL6D_STRID,             "QZSS L6D navigation message" },
    { SBF_QZSRAWL6E_MSGID,             SBF_QZSRAWL6E_STRID,             "QZSS L6E navigation message" },
    { SBF_QZSRAWL1C_MSGID,             SBF_QZSRAWL1C_STRID,             "QZSS L1C navigation frame" },
    { SBF_QZSRAWL1S_MSGID,             SBF_QZSRAWL1S_STRID,             "QZSS L1S navigation message" },
    { SBF_QZSRAWL5S_MSGID,             SBF_QZSRAWL5S_STRID,             "QZSS L5S navigation message" },
    { SBF_NAVICRAW_MSGID,              SBF_NAVICRAW_STRID,              "NavIC/IRNSS L5 subframe" },
    { SBF_GPSNAV_MSGID,                SBF_GPSNAV_STRID,                "GPS ephemeris and clock" },
    { SBF_GPSALM_MSGID,                SBF_GPSALM_STRID,                "Almanac data for a GPS satellite" },
    { SBF_GPSION_MSGID,                SBF_GPSION_STRID,                "Ionosphere data from the GPS subframe 5" },
    { SBF_GPSUTC_MSGID,                SBF_GPSUTC_STRID,                "GPS-UTC data from GPS subframe 5" },
    { SBF_GPSCNAV_MSGID,               SBF_GPSCNAV_STRID,               "CNAV Ephemeris data for one GPS satellite" },
    { SBF_GLONAV_MSGID,                SBF_GLONAV_STRID,                "GLONASS ephemeris and clock" },
    { SBF_GLOALM_MSGID,                SBF_GLOALM_STRID,                "Almanac data for a GLONASS satellite" },
    { SBF_GLOTIME_MSGID,               SBF_GLOTIME_STRID,               "GLO-UTC, GLO-GPS and GLO-UT1 data" },
    { SBF_GALNAV_MSGID,                SBF_GALNAV_STRID,                "Galileo ephemeris, clock, health and BGD" },
    { SBF_GALALM_MSGID,                SBF_GALALM_STRID,                "Almanac data for a Galileo satellite" },
    { SBF_GALION_MSGID,                SBF_GALION_STRID,                "NeQuick Ionosphere model parameters" },
    { SBF_GALUTC_MSGID,                SBF_GALUTC_STRID,                "GST-UTC data" },
    { SBF_GALGSTGPS_MSGID,             SBF_GALGSTGPS_STRID,             "GST-GPS data" },
    { SBF_GALSARRLM_MSGID,             SBF_GALSARRLM_STRID,             "Search-and-rescue return link message" },
    { SBF_BDSNAV_MSGID,                SBF_BDSNAV_STRID,                "BeiDou ephemeris and clock" },
    { SBF_BDSCNAV1_MSGID,              SBF_BDSCNAV1_STRID,              "BeiDou B-CNAV1 ephemeris data for one satellite" },
    { SBF_BDSCNAV2_MSGID,              SBF_BDSCNAV2_STRID,              "BeiDou B-CNAV2 ephemeris data for one satellite" },
    { SBF_BDSCNAV3_MSGID,              SBF_BDSCNAV3_STRID,              "BeiDou B-CNAV3 ephemeris data for one satellite" },
    { SBF_BDSALM_MSGID,                SBF_BDSALM_STRID,                "Almanac data for a BeiDou satellite" },
    { SBF_BDSION_MSGID,                SBF_BDSION_STRID,                "BeiDou Ionospheric delay model parameters" },
    { SBF_BDSUTC_MSGID,                SBF_BDSUTC_STRID,                "BDT-UTC data" },
    { SBF_QZSNAV_MSGID,                SBF_QZSNAV_STRID,                "QZSS ephemeris and clock" },
    { SBF_QZSALM_MSGID,                SBF_QZSALM_STRID,                "Almanac data for a QZSS satellite" },
    { SBF_NAVICLNAV_MSGID,             SBF_NAVICLNAV_STRID,             "NavIC/IRNSS ephemeris and clock" },
    { SBF_GEONAV_MSGID,                SBF_GEONAV_STRID,                "MT09 : SBAS navigation message" },
    { SBF_GEOALM_MSGID,                SBF_GEOALM_STRID,                "MT17 : SBAS satellite almanac" },
    { SBF_PVTCARTESIAN_MSGID,          SBF_PVTCARTESIAN_STRID,          "GNSS position, velocity, and time in Cartesian coordinates" },
    { SBF_PVTGEODETIC_MSGID,           SBF_PVTGEODETIC_STRID,           "GNSS position, velocity, and time in geodetic coordinates" },
    { SBF_POSCOVCARTESIAN_MSGID,       SBF_POSCOVCARTESIAN_STRID,       "Position covariance matrix (X,Y, Z)" },
    { SBF_POSCOVGEODETIC_MSGID,        SBF_POSCOVGEODETIC_STRID,        "Position covariance matrix (Lat, Lon, Alt)" },
    { SBF_VELCOVCARTESIAN_MSGID,       SBF_VELCOVCARTESIAN_STRID,       "Velocity covariance matrix (X, Y, Z)" },
    { SBF_VELCOVGEODETIC_MSGID,        SBF_VELCOVGEODETIC_STRID,        "Velocity covariance matrix (North, East, Up)" },
    { SBF_DOP_MSGID,                   SBF_DOP_STRID,                   "Dilution of precision" },
    { SBF_BASEVECTORCART_MSGID,        SBF_BASEVECTORCART_STRID,        "XYZ relative position and velocity with respect to base(s)" },
    { SBF_BASEVECTORGEOD_MSGID,        SBF_BASEVECTORGEOD_STRID,        "ENU relative position and velocity with respect to base(s)" },
    { SBF_PVTSUPPORT_MSGID,            SBF_PVTSUPPORT_STRID,            "Internal parameters for maintenance and support" },
    { SBF_PVTSUPPORTA_MSGID,           SBF_PVTSUPPORTA_STRID,           "Internal parameters for maintenance and support" },
    { SBF_ENDOFPVT_MSGID,              SBF_ENDOFPVT_STRID,              "PVT epoch marker" },
    { SBF_NAVCART_MSGID,               SBF_NAVCART_STRID,               "Full GNSS position, velocity, attitude, DOP and UTC time in Cartesian coordinates." },
    { SBF_ATTEULER_MSGID,              SBF_ATTEULER_STRID,              "GNSS attitude expressed as Euler angles" },
    { SBF_ATTCOVEULER_MSGID,           SBF_ATTCOVEULER_STRID,           "Covariance matrix of attitude" },
    { SBF_AUXANTPOSITIONS_MSGID,       SBF_AUXANTPOSITIONS_STRID,       "Relative position and velocity estimates of auxiliary antennas" },
    { SBF_ENDOFATT_MSGID,              SBF_ENDOFATT_STRID,              "GNSS attitude epoch marker" },
    { SBF_RECEIVERTIME_MSGID,          SBF_RECEIVERTIME_STRID,          "Current receiver and UTC time" },
    { SBF_XPPSOFFSET_MSGID,            SBF_XPPSOFFSET_STRID,            "Offset of the xPPS pulse with respect to GNSS time" },
    { SBF_EXTEVENT_MSGID,              SBF_EXTEVENT_STRID,              "Time at the instant of an external event" },
    { SBF_EXTEVENTPVTCARTESIAN_MSGID,  SBF_EXTEVENTPVTCARTESIAN_STRID,  "Cartesian position at the instant of an event" },
    { SBF_EXTEVENTPVTGEODETIC_MSGID,   SBF_EXTEVENTPVTGEODETIC_STRID,   "Geodetic position at the instant of an event" },
    { SBF_EXTEVENTBASEVECTGEOD_MSGID,  SBF_EXTEVENTBASEVECTGEOD_STRID,  "ENU relative position with respect to base(s) at the instant of an event" },
    { SBF_EXTEVENTATTEULER_MSGID,      SBF_EXTEVENTATTEULER_STRID,      "GNSS attitude expressed as Euler angles at the instant of an event" },
    { SBF_DIFFCORRIN_MSGID,            SBF_DIFFCORRIN_STRID,            "Incoming RTCM or CMR message" },
    { SBF_BASESTATION_MSGID,           SBF_BASESTATION_STRID,           "Base station coordinates" },
    { SBF_LBANDTRACKERSTATUS_MSGID,    SBF_LBANDTRACKERSTATUS_STRID,    "Status of the L-band signal tracking" },
    { SBF_LBANDRAW_MSGID,              SBF_LBANDRAW_STRID,              "L-Band raw user data" },
    { SBF_CHANNELSTATUS_MSGID,         SBF_CHANNELSTATUS_STRID,         "Status of the tracking for all receiver channels" },
    { SBF_RECEIVERSTATUS_MSGID,        SBF_RECEIVERSTATUS_STRID,        "Overall status information of the receiver" },
    { SBF_SATVISIBILITY_MSGID,         SBF_SATVISIBILITY_STRID,         "Azimuth/elevation of visible satellites" },
    { SBF_INPUTLINK_MSGID,             SBF_INPUTLINK_STRID,             "Statistics on input streams" },
    { SBF_OUTPUTLINK_MSGID,            SBF_OUTPUTLINK_STRID,            "Statistics on output streams" },
    { SBF_QUALITYIND_MSGID,            SBF_QUALITYIND_STRID,            "Quality indicators" },
    { SBF_DISKSTATUS_MSGID,            SBF_DISKSTATUS_STRID,            "Internal logging status" },
    { SBF_RFSTATUS_MSGID,              SBF_RFSTATUS_STRID,              "Radio-frequency interference mitigation status" },
    { SBF_GALAUTHSTATUS_MSGID,         SBF_GALAUTHSTATUS_STRID,         "Galileo OSNMA authentication status" },
    { SBF_RECEIVERSETUP_MSGID,         SBF_RECEIVERSETUP_STRID,         "General information about the receiver installation" },
    { SBF_RXMESSAGE_MSGID,             SBF_RXMESSAGE_STRID,             "Receiver message" },
    { SBF_COMMANDS_MSGID,              SBF_COMMANDS_STRID,              "Commands entered by the user" },
    { SBF_COMMENT_MSGID,               SBF_COMMENT_STRID,               "Comment entered by the user" },
    { SBF_BBSAMPLES_MSGID,             SBF_BBSAMPLES_STRID,             "Baseband samples" },
    { SBF_ASCIIIN_MSGID,               SBF_ASCIIIN_STRID,               "ASCII input from external sensor" },
}};
// @fp_codegen_end{FPSDK_COMMON_PARSER_SBF_MSGINFO}
// clang-format on

// ---------------------------------------------------------------------------------------------------------------------

bool SbfGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    // Check arguments
    if ((name == NULL) || (size < 1)) {
        return false;
    }

    name[0] = '\0';

    if ((msg == NULL) || (msg_size < SBF_HEAD_SIZE)) {
        return false;
    }

    const uint16_t block = SbfBlockType(msg);

    // Try the message name lookup table
    for (auto& info : MSG_INFO) {
        if (info.block == block) {
            return std::snprintf(name, size, "%s", info.name) < (int)size;
        }
    }

    // If that failed, stringify the message ID
    return std::snprintf(name, size, "SBF-BLOCK%05" PRIu16, block) < (int)size;
}

// ---------------------------------------------------------------------------------------------------------------------

bool SbfGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size)
{
    if ((info == NULL) || (size < 1) || (msg == NULL) || (msg_size < SBF_HEAD_SIZE)) {
        return false;
    }

    info[0] = '\0';

    if ((msg == NULL) || (msg_size < SBF_HEAD_SIZE)) {
        return false;
    }

    const uint16_t block = SbfBlockType(msg);

    // Try the message name lookup table
    for (auto& mi : MSG_INFO) {
        if (mi.block == block) {
            return std::snprintf(info, size, "%s", mi.desc) < (int)size;
        }
    }

    return false;
}

/* ****************************************************************************************************************** */
}  // namespace sbf
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
