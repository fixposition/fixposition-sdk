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
 * @brief Fixposition SDK: parsertool documentation
 */

#ifndef __FPSDK_APPS_PARSERTOOL_PARSERTOOL_DOC_HPP__
#define __FPSDK_APPS_PARSERTOOL_PARSERTOOL_DOC_HPP__

namespace fpsdk {
namespace apps {
/**
 * @brief parsertool
 */
namespace parsertool {
// clang-format off
/* ****************************************************************************************************************** */

/*!
    @page FPSDK_APPS_PARSERTOOL Parser tool

    @section FPSDK_APPS_PARSERTOOL_OVERVIEW Overview

    This demonstrates the use of the @ref FPSDK_COMMON_PARSER

    See @ref FPSDK_BUILD_DOC on how to build and run the apps.

    @section FPSDK_APPS_PARSERTOOL_HELP Command line help

    @include parsertool_helpscreen.txt

    @section FPSDK_APPS_PARSERTOOL_EXAMPLES Examples

    @verbatim
    $ parsertool fpsdk_common/test/data/mixed.bin
    @endverbatim

    Outputs:

    @verbatim
    Reading from fpsdk_common/test/data/mixed.bin
    ------- Seq#    Size Protocol Message
    message 000001   109 FP_A     FP_A-ODOMETRY
    message 000002    70 NMEA     NMEA-GN-RMC
    message 000003   104 NOV_B    NOV_B-BESTPOS
    message 000004    42 OTHER    OTHER
    message 000005    28 RTCM3    RTCM3-TYPE1107
    message 000006    48 SPARTN   SPARTN-OCB-GLO
    message 000007    24 UBX      UBX-TIM-TP
    message 000008    20 OTHER    OTHER
    message 000009   104 FP_A     FP_A-TF
    message 000010    82 NMEA     NMEA-GN-GGA
    message 000011   144 NOV_B    NOV_B-BESTXYZ
    message 000012   295 RTCM3    RTCM3-TYPE1077
    message 000013   259 SPARTN   SPARTN-OCB-GPS
    message 000014    68 UBX      UBX-MON-HW
    message 000015    40 OTHER    OTHER
    message 000016    56 NOV_B    NOV_B-RAWIMUSX
    message 000017    52 RTCM3    RTCM3-TYPE1033
    message 000018    28 SPARTN   SPARTN-GAD-GAD
    message 000019   256 OTHER    OTHER
    message 000020    24 OTHER    OTHER
    message 000021    36 UBX      UBX-MON-HW2
    message 000022   192 SPARTN   SPARTN-HPAC-GLO
    message 000023    72 UBX      UBX-NAV-COV
    message 000024   256 OTHER    OTHER
    message 000025   256 OTHER    OTHER
    message 000026    68 OTHER    OTHER
    Stats:     Messages               Bytes
    Total            26 (100.0%)       2733 (100.0%)
    FP_A              2 (  7.7%)        213 (  7.8%)
    FP_B              0 (  0.0%)          0 (  0.0%)
    NMEA              2 (  7.7%)        152 (  5.6%)
    UBX               4 ( 15.4%)        200 (  7.3%)
    RTCM3             3 ( 11.5%)        375 ( 13.7%)
    NOV_B             3 ( 11.5%)        304 ( 11.1%)
    SPARTN            4 ( 15.4%)        527 ( 19.3%)
    OTHER             8 ( 30.8%)        962 ( 35.2%)
    @endverbatim

    */

/* ****************************************************************************************************************** */
// clang-format on
}  // namespace parsertool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_PARSERTOOL_PARSERTOOL_DOC_HPP__
