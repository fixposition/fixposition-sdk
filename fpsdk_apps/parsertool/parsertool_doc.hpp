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

    See @ref FPSDK_BUILD_DOC on how to build and run this app, or @ref FPSDK_RUN_DOC on how to run it using pre-built
    Docker images.

    @section FPSDK_APPS_PARSERTOOL_HELP Command line help

    @include parsertool_helpscreen.txt

    @section FPSDK_APPS_PARSERTOOL_EXAMPLES Examples

    <b>Print info on all messages found in a file:</b>

    @verbatim
    $ parsertool fpsdk_common/test/data/test_data_mixed.bin
    @endverbatim

    Outputs:

    @verbatim
    ------- Seq#     Offset  Size Protocol Message                        Info
    message 000001        0   109 FP_A     FP_A-ODOMETRY                  2253,323299.100000,,,,,,,,,,,,,,,,,0,0,8,8,-1,,,,,,,,,,,,,,,,,,,fp_release_vr2_2.63.1_204
    message 000002      109    70 NMEA     NMEA-GN-RMC                    174644.20,A,4724.01818,N,00827.02244,E,0.013,,150323,,,R,V
    message 000003      179   104 NOV_B    NOV_B-BESTPOS                  l 208198.500
    message 000004      283    42 OTHER    OTHER                          e5 60 12 2f 97 8c 5e 1c 51 01 b8 d1 e2 33 15 ed ...
    message 000005      325    28 RTCM3    RTCM3-TYPE1107                 (#0) 323233.000 (0 * 0, more) - SBAS MSM7 (ext full C, ext full L, S, D)
    message 000006      353    48 SPARTN   SPARTN-OCB-GLO                 Orbits, clock, bias (OCB) GLONASS
    message 000007      401    24 UBX      UBX-TIM-TP                     323205.000000000000 UTC avail USNO 0 Y L
    message 000008      425    20 OTHER    OTHER                          d3 00 16 45 30 00 4d 10 93 a2 00 00 00 00 00 00 ...
    message 000009      445   104 FP_A     FP_A-TF                        2253,323299.000000,VRTK,CAM,-0.03400,0.00200,-0.01600,-0.000000,0.000000,0.000000,1.000000
    message 000010      549    82 NMEA     NMEA-GN-GGA                    174644.20,4724.01818,N,00827.02244,E,4,12,0.58,412.1,M,47.3,M,1.2,0000
    message 000011      631   144 NOV_B    NOV_B-BESTXYZ                  l 208198.500
    message 000012      775   295 RTCM3    RTCM3-TYPE1077                 (#0) 323233.000 (12 * 2, more) - GPS MSM7 (ext full C, ext full L, S, D)
    message 000013     1070   259 SPARTN   SPARTN-OCB-GPS                 Orbits, clock, bias (OCB) GPS
    message 000014     1329    68 UBX      UBX-MON-HW                     -
    message 000015     1397    40 OTHER    OTHER                          b5 62 0a 09 3c 00 c1 81 00 00 1e 00 01 00 00 80 ...
    message 000016     1437    56 NOV_B    NOV_B-RAWIMUSX                 s 208198.491
    message 000017     1493    52 RTCM3    RTCM3-TYPE1033                 (#0) [NULLANTENNA] [] 0 [RTKBase Ublox_ZED-F9P] [2.3.0] [] - Receiver and antenna descriptors
    message 000018     1545    28 SPARTN   SPARTN-GAD-GAD                 Geographic area definition (GAD)
    message 000019     1573   256 OTHER    OTHER                          78 47 51 dd e0 dd 5f b9 68 fc 77 81 65 b9 fb cc ...
    message 000020     1829    24 OTHER    OTHER                          73 60 3c a1 81 c7 aa 2d 1f 2e 5a 43 11 6f 14 1c ...
    message 000021     1853    36 UBX      UBX-MON-HW2                    -
    message 000022     1889   192 SPARTN   SPARTN-HPAC-GLO                High-precision atmosphere correction (HPAC) GLONASS
    message 000023     2081    72 UBX      UBX-NAV-COV                    323222.200
    message 000024     2153   104 FP_B     FP_B-MEASUREMENTS              07d1@0 v1 [3] / 1 1 1 10 12345678 10 20 -30 / 1 2 1 10 12345678 11 21 -29 / 1 3 1 10 12345678 12 22 -28
    message 000025     2257   256 OTHER    OTHER                          b5 62 01 35 84 02 b8 fa 43 13 01 35 00 00 00 01 ...
    message 000026     2513   256 OTHER    OTHER                          29 17 49 00 ff ff 5f 19 32 00 02 0d 2a 27 43 00 ...
    message 000027     2769    68 OTHER    OTHER                          00 00 10 12 00 00 05 03 00 a5 00 00 00 00 01 00 ...
    Stats:     Messages               Bytes
    Total            27 (100.0%)       2837 (100.0%)
    FP_A              2 (  7.4%)        213 (  7.5%)
    FP_B              1 (  3.7%)        104 (  3.7%)
    NMEA              2 (  7.4%)        152 (  5.4%)
    UBX               4 ( 14.8%)        200 (  7.0%)
    RTCM3             3 ( 11.1%)        375 ( 13.2%)
    NOV_B             3 ( 11.1%)        304 ( 10.7%)
    UNI_B             0 (  0.0%)        304 (  0.0%)
    SPARTN            4 ( 14.8%)        527 ( 18.6%)
    OTHER             8 ( 29.6%)        962 ( 33.9%)
    @endverbatim


    <b>Print info on selected messages found in a file:</b>

    @verbatim
    $ parsertool -f NMEA,FP_A fpsdk_common/test/data/mixed.bin
    @endverbatim

    Outputs:

    @verbatim
    ------- Seq#     Offset  Size Protocol Message                        Info
    message 000001        0   109 FP_A     FP_A-ODOMETRY                  2253,323299.100000,,,,,,,,,,,,,,,,,0,0,8,8,-1,,,,,,,,,,,,,,,,,,,fp_release_vr2_2.63.1_204
    message 000002      109    70 NMEA     NMEA-GN-RMC                    174644.20,A,4724.01818,N,00827.02244,E,0.013,,150323,,,R,V
    message 000009      179   104 FP_A     FP_A-TF                        2253,323299.000000,VRTK,CAM,-0.03400,0.00200,-0.01600,-0.000000,0.000000,0.000000,1.000000
    message 000010      283    82 NMEA     NMEA-GN-GGA                    174644.20,4724.01818,N,00827.02244,E,4,12,0.58,412.1,M,47.3,M,1.2,0000
    Stats:     Messages               Bytes
    Total             4 (100.0%)        365 (100.0%)
    FP_A              2 ( 50.0%)        213 ( 58.4%)
    FP_B              0 (  0.0%)          0 (  0.0%)
    NMEA              2 ( 50.0%)        152 ( 41.6%)
    UBX               0 (  0.0%)          0 (  0.0%)
    RTCM3             0 (  0.0%)          0 (  0.0%)
    NOV_B             0 (  0.0%)          0 (  0.0%)
    UNI_B             0 (  0.0%)          0 (  0.0%)
    SPARTN            0 (  0.0%)          0 (  0.0%)
    OTHER             0 (  0.0%)          0 (  0.0%)
    @endverbatim

*/

/* ****************************************************************************************************************** */
// clang-format on
}  // namespace parsertool
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_PARSERTOOL_PARSERTOOL_DOC_HPP__
