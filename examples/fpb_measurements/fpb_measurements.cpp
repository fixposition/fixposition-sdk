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
 * @brief Fixposition SDK examples: Making a FP_B-MEASUREMENTS message
 *
 * To build and run:
 *
 *     make
 *     ./build/fpb_measurements
 *
 * This program shows how a FP_B-MEASUREMENTS message can be made.
 *
 * This file is both the source code of this example as well as the documentation on how it works.
 */

/* LIBC/STL */
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

/* EXTERNAL */
#include <unistd.h>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/fpb.hpp>
#include <fpsdk_common/types.hpp>

/* PACKAGE */

/* ****************************************************************************************************************** */

using namespace fpsdk::common::time;
using namespace fpsdk::common::app;
using namespace fpsdk::common::parser::fpb;
using namespace fpsdk::common::logging;
using namespace fpsdk::common::types;

// ---------------------------------------------------------------------------------------------------------------------

int main(int /*argc*/, char** /*argv*/)
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    LoggingSetParams({ LoggingLevel::TRACE });

    // The speed measurement in XYZ in [m/s], two wheels
    const double speed_FL[3] = { 5.422, -0.02, 0.4 };
    const double speed_FR[3] = { 6.543, +0.03, 0.5 };

    // The FP_B-MEASUREMENTS message payload consists of two parts:
    // - 1. The header
    FpbMeasurementsHead head;
    head.num_meas = 2;
    // - 2. The measurements. In this example we have two measurements:
    FpbMeasurementsMeas meas[2];
    // - FL wheel
    meas[0].meas_x = std::floor(speed_FL[0] * 1e3);  // [m/s] -> [mm/s]
    meas[0].meas_y = std::floor(speed_FL[1] * 1e3);  // [m/s] -> [mm/s]
    meas[0].meas_z = std::floor(speed_FL[2] * 1e3);  // [m/s] -> [mm/s]
    meas[0].meas_x_valid = 1;
    meas[0].meas_y_valid = 1;
    meas[0].meas_z_valid = 1;
    meas[0].meas_type = EnumToVal(FpbMeasurementsMeasType::VELOCITY);
    meas[0].meas_loc = EnumToVal(FpbMeasurementsMeasLoc::FL);
    meas[0].timestamp_type = EnumToVal(FpbMeasurementsTimestampType::TIMEOFARRIVAL);
    // - FR wheel
    meas[1].meas_x = std::floor(speed_FR[0] * 1e3);  // [m/s] -> [mm/s]
    meas[1].meas_y = std::floor(speed_FR[1] * 1e3);  // [m/s] -> [mm/s]
    meas[1].meas_z = std::floor(speed_FR[2] * 1e3);  // [m/s] -> [mm/s]
    meas[1].meas_x_valid = 1;
    meas[1].meas_y_valid = 1;
    meas[1].meas_z_valid = 1;
    meas[1].meas_type = EnumToVal(FpbMeasurementsMeasType::VELOCITY);
    meas[1].meas_loc = EnumToVal(FpbMeasurementsMeasLoc::FR);
    meas[1].timestamp_type = EnumToVal(FpbMeasurementsTimestampType::TIMEOFARRIVAL);

    // Create the raw payload by copying the structs above to the right place.
    uint8_t payload[FP_B_MEASUREMENTS_HEAD_SIZE + (FP_B_MEASUREMENTS_MAX_NUM_MEAS * FP_B_MEASUREMENTS_MEAS_SIZE)];
    std::size_t payload_size = 0;
    // - The header
    std::memcpy(&payload[payload_size], &head, sizeof(head));
    payload_size += sizeof(head);
    // - The data
    for (std::size_t ix = 0; (ix < head.num_meas) && (ix < FP_B_MEASUREMENTS_MAX_NUM_MEAS); ix++) {
        std::memcpy(&payload[payload_size], &meas[ix], sizeof(meas[ix]));
        payload_size += sizeof(meas[ix]);
    }

    INFO("num_meas=%" PRIu8 " payload_size=%" PRIuMAX, head.num_meas, payload_size);
    DEBUG_HEXDUMP(payload, payload_size, NULL, NULL);

    // Create FP_B-MEASUREMENTS message
    std::vector<uint8_t> message;
    if (FpbMakeMessage(message, FP_B_MEASUREMENTS_MSGID, 0, payload, payload_size)) {
        INFO("Message successfully made");
        DEBUG_HEXDUMP(message.data(), message.size(), NULL, NULL);

        // Print to stdio unless that's a terminal
        if (isatty(fileno(stdout)) == 0) {
            write(fileno(stdout), message.data(), message.size());
        }
    } else {
        WARNING("Failed making FP_B-MEASUREMENTS message");
    }

    // clang-format off
    // This should output:
    //
    //     num_meas=2 payload_size=64
    //     0x0000 00000  01 02 00 00 00 00 00 00  2e 15 00 00 ec ff ff ff  |................|
    //     0x0010 00016  90 01 00 00 01 01 01 01  03 00 00 00 00 01 00 00  |................|
    //     0x0020 00032  00 00 00 00 8f 19 00 00  1e 00 00 00 f4 01 00 00  |................|
    //     0x0030 00048  01 01 01 01 02 00 00 00  00 01 00 00 00 00 00 00  |................|
    //     Message successfully made
    //     0x0000 00000  66 21 d1 07 40 00 00 00  01 02 00 00 00 00 00 00  |f!..@...........|
    //     0x0010 00016  2e 15 00 00 ec ff ff ff  90 01 00 00 01 01 01 01  |................|
    //     0x0020 00032  03 00 00 00 00 01 00 00  00 00 00 00 8f 19 00 00  |................|
    //     0x0030 00048  1e 00 00 00 f4 01 00 00  01 01 01 01 02 00 00 00  |................|
    //     0x0040 00064  00 01 00 00 00 00 00 00  22 59 21 e2
    //
    // The message can be stored to a file:
    //
    //     ./build/fpb_measurements > fpbmeasurements.bin
    //
    // Which can be parsed by the parsertool from the Fixposition SDK:
    //
    //     parsertool fpbmeasurements.bin
    //
    // Which prints something like this:
    //
    //     ------- Seq#     Offset  Size Protocol Message                        Info
    //     Reading from fpbmeasurements.bin
    //     message 000001        0    76 FP_B     FP_B-MEASUREMENTS              07d1@0 v1 [2] / 1 3 1 0 0 5422 -20 400 / 1 2 1 0 0 6543 30 500
    //     Stats:     Messages               Bytes
    //     Total             1 (100.0%)         76 (100.0%)
    //     FP_A              0 (  0.0%)          0 (  0.0%)
    //     FP_B              1 (100.0%)         76 (100.0%)
    //     NMEA              0 (  0.0%)          0 (  0.0%)
    //     UBX               0 (  0.0%)          0 (  0.0%)
    //     RTCM3             0 (  0.0%)          0 (  0.0%)
    //     NOV_B             0 (  0.0%)          0 (  0.0%)
    //     UNI_B             0 (  0.0%)          0 (  0.0%)
    //     SPARTN            0 (  0.0%)          0 (  0.0%)
    //     OTHER             0 (  0.0%)          0 (  0.0%)
    //     Done
    // clang-format on

    return EXIT_SUCCESS;
}

/* ****************************************************************************************************************** */
