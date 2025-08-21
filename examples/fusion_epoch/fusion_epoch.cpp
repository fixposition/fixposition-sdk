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
 * @brief Fixposition SDK examples: collecting fusion epoch data from FP_A messages
 *
 * To build and run:
 *
 *     make
 *     ./build/fusion_epoch
 *
 * This program builds on the "parser_intro" example and details how to collect the FP_A fusion messages into
 * fusion epochs for further processing.
 *
 * This file is both the source code of this example as well as the documentation on how it works.
 */

/* LIBC/STL */
#include <cinttypes>
#include <cstdint>
#include <cstdlib>

/* EXTERNAL */
#include "fpsdk_common/ext/eigen_core.hpp"

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/trafo.hpp>

/* PACKAGE */

/* ****************************************************************************************************************** */

using namespace fpsdk::common::time;
using namespace fpsdk::common::app;
using namespace fpsdk::common::parser;
using namespace fpsdk::common::parser::fpa;
using namespace fpsdk::common::logging;
using namespace fpsdk::common::trafo;

// ---------------------------------------------------------------------------------------------------------------------

// The example data is a short sequence of output from the Vision-RTK 2 sensor.
// clang-format off
const uint8_t SAMPLE_DATA[] = {
    // (1)
    "$FP,ODOMSTATUS,1,2349,59920.000000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*18\r\n"
    "$FP,EOE,1,2349,59920.000000,FUSION*59\r\n"
    // (2)
    "$FP,ODOMETRY,2,2349,59921.000000,4278387.6882,635620.5002,4672339.9313,-0.580560,0.326972,-0.184160,0.722582,"
    "-0.0008,0.0001,-0.0003,0.00190,-0.00021,-0.00018,0.1875,-0.1978,9.8054,4,0,8,8,-1,0.00139,0.00459,0.00201,0.0024"
    "4,-0.00294,-0.00160,0.03639,0.00068,0.04631,0.00467,-0.00524,-0.04100,0.00175,0.00055,0.00065,-0.00005,0.0000"
    "0,0.00045,fp_vrtk2-integ_6912e460-1703*20\r\n"
    "$FP,ODOMSTATUS,1,2349,59921.000000,1,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*1A\r\n"
    "$FP,EOE,1,2349,59921.000000,FUSION*58\r\n"
    // (3)
    "$FP,ODOMETRY,2,2349,59922.000000,,,,-0.580477,0.327111,-0.184310,0.722547,"
    "-0.0012,0.0001,-0.0010,-0.00048,-0.00065,-0.00063,0.1939,-0.2015,9.8043,4,0,8,8,-1,0.00139,0.00458,0.00200,0.002"
    "44,-0.00293,-0.00160,0.03636,0.00067,0.04631,0.00466,-0.00524,-0.04098,0.00171,0.00055,0.00064,-0.00005,0.000"
    "00,0.00043,fp_vrtk2-integ_6912e460-1703*29\r\n"
    "$FP,ODOMSTATUS,1,2349,59922.000000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*1A\r\n"
    "$FP,EOE,1,2349,59922.000000,FUSION*5B\r\n"
    // (4)
    "$FP,ODOMETRY,2,2349,59923.000000,4278387.6888,635620.5014,4672339.9313,-0.580321,0.327140,-0.184394,0.722638,"
    "-0.0023,0.0005,-0.0003,0.00038,0.00123,0.00023,0.1910,-0.2060,9.7975,4,0,8,8,-1,0.00139,0.00458,0.00200,0.00244,"
    "-0.00293,-0.00160,0.03633,0.00068,0.04629,0.00467,-0.00524,-0.04096,0.00171,0.00055,0.00064,-0.00005,0.00000,"
    "0.00043,fp_vrtk2-integ_6912e460-1703*27\r\n"
    "$FP,ODOMSTATUS,1,2349,59923.000000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*1B\r\n"
    "$FP,EOE,1,2349,59923.000000,FUSION*5A\r\n"
};
const std::size_t SAMPLE_SIZE = sizeof(SAMPLE_DATA) - 1; // -1 because "" adds \0
// clang-format on

// We want to collect the output messages into a "fusion epoch" so that we can further process the data. Specifically,
// we collect the FP_A-ODOMETRY and FP_A-ODOMSTATUS into matching pairs of messages. We'll use the FP_A-EOE message to
// detect when an epoch is complete.
struct CollectedMsgs
{
    FpaOdometryPayload fpa_odometry_;
    FpaOdomstatusPayload fpa_odomstatus_;
    FpaEoePayload fpa_eoe_;
};

// The coordinate reference system (CRS) of the position output of the sensor depends on the correction data being used.
// And for the application we want the to use a particular local coordinate reference system. In this example we assume
// the raw sensor positions are in ETRS89 and we're in Switzerland, so we Swiss coordinates. See the documentation
// of the Transformer in fpskd_common.hpp
static constexpr const char* source_crs = "EPSG:4936";  // ETRS89 x/y/z
static constexpr const char* target_crs = "EPSG:2056";  // Swiss CH1903+ / LV95 east/north/up

// We'll process the data received from the sensor, for which we'll need a a coordinate transformer
static void ProcessCollectedMsgs(const CollectedMsgs& collected_msgs, Transformer& trafo);

int main(int /*argc*/, char** /*argv*/)
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    LoggingSetParams({ LoggingLevel::TRACE });

    // Initialise the coordinate transformer
    Transformer trafo;
    if (!trafo.Init(source_crs, target_crs)) {
        ERROR("Failed creating coordinate transformer!");
        return EXIT_FAILURE;
    }

    // We parse the input data, decode the messages and process the data as we go. See the parser_intro example for
    // details on the parsing and decoding the messages.
    Parser parser;
    ParserMsg msg;
    if (!parser.Add(SAMPLE_DATA, SAMPLE_SIZE)) {
        WARNING("Parser overflow, SAMPLE_DATA is too large!");
    }

    // The collector of messages
    CollectedMsgs collected_msgs;
    while (parser.Process(msg)) {
        DEBUG("Message %-s (size %" PRIuMAX " bytes)", msg.name_.c_str(), msg.Size());
        bool msg_ok = true;

        // Collect FP_A-ODOMETRY
        if (msg.name_ == FpaOdometryPayload::MSG_NAME) {
            msg_ok = collected_msgs.fpa_odometry_.SetFromMsg(msg.Data(), msg.Size());
        }
        // Collect FP_A-ODOMSTATUS
        else if (msg.name_ == FpaOdomstatusPayload::MSG_NAME) {
            msg_ok = collected_msgs.fpa_odomstatus_.SetFromMsg(msg.Data(), msg.Size());
        }
        // FP_A-EOE: Check if it the *fusion* end of epoch, otherwise ignore
        else if (msg.name_ == FpaEoePayload::MSG_NAME) {
            FpaEoePayload fpa_eoe;
            msg_ok = fpa_eoe.SetFromMsg(msg.Data(), msg.Size());
            if (fpa_eoe.epoch == FpaEpoch::FUSION) {
                // Add it to the collection and process the data
                collected_msgs.fpa_eoe_ = fpa_eoe;
                ProcessCollectedMsgs(collected_msgs, trafo);

                // Clear the collection for the next epoch
                collected_msgs = CollectedMsgs();
            }
        }
        // else: ignore any other message

        // Carp if decoding a message failed. But otherwise ignore it. We'll handle missing messages below.
        if (!msg_ok) {
            msg.MakeInfo();
            WARNING("Failed decoding %s: %s", msg.name_.c_str(), msg.info_.c_str());
            // DEBUG_HEXDUMP(msg.Data(), msg.Size(), NULL, NULL);  // Hexdump of the raw message data
        }
    }

    // This should output:
    //
    //     Message FP_A-ODOMSTATUS (size 92 bytes)
    //     Message FP_A-EOE (size 39 bytes)
    //     Warning: Ignoring incomplete epoch at 2349:059920.000                                 (1)
    //     Message FP_A-ODOMETRY (size 373 bytes)
    //     Message FP_A-ODOMSTATUS (size 92 bytes)
    //     Message FP_A-EOE (size 39 bytes)
    //     Warning: Ignoring uninitialised fusion at 2349:059921.000                             (2)
    //     Message OTHER (size 256 bytes)
    //     Message OTHER (size 83 bytes)
    //     Message FP_A-ODOMSTATUS (size 92 bytes)
    //     Message FP_A-EOE (size 39 bytes)
    //     Warning: Ignoring unusable epoch at 2349:059922.000                                   (3)
    //     Message FP_A-ODOMETRY (size 371 bytes)
    //     Message FP_A-ODOMSTATUS (size 92 bytes)
    //     Message FP_A-EOE (size 39 bytes)
    //     Using position at 2349:059923.000: [ 4278387.7, 635620.5, 4672339.9 ]                 (4)
    //     Local coordinates: 2676373.0 1250433.7 459.5
    //
    // - (1) The first epoch is incomplete as the FP_A-ODOMETRY for that epoch was not present
    // - (2) The second epoch is complete (all messages present and matching), but FP_A-ODOMSTATUS.init_status is
    //       LOCAL_INIT and we want status GLOBAL_INIT in order to use the data)
    // - (3) The third epoch is complete, but the FP_A-ODOMETRY.pos_{x,y,z} fields are empty (and we want the position)
    // - (4) The fourth epoch is complete and passes all checks and we can transform the position to the local CRS

    return EXIT_SUCCESS;
}

static void ProcessCollectedMsgs(const CollectedMsgs& collected_msgs, Transformer& trafo)
{
    auto& fpa_odometry = collected_msgs.fpa_odometry_;
    auto& fpa_odomstatus = collected_msgs.fpa_odomstatus_;
    auto& fpa_eoe = collected_msgs.fpa_eoe_;

    // The epoch is complete if all messages are present and their timestamps are valid and match
    if (!(fpa_odometry.valid_ && fpa_odomstatus.valid_ && fpa_eoe.valid_ && fpa_odometry.gps_time.week.valid &&
            fpa_odometry.gps_time.tow.valid && (fpa_odometry.gps_time == fpa_odomstatus.gps_time) &&
            (fpa_odometry.gps_time == fpa_eoe.gps_time))) {
        WARNING("Ignoring incomplete epoch at %04d:%010.3f", fpa_eoe.gps_time.week.value, fpa_eoe.gps_time.tow.value);
        return;
    }

    // Epoch is complete. However, we'll have to check if all the data we need is present. We want the position and the
    // fusion initialisation status.
    if (!fpa_odometry.pos.valid || (fpa_odomstatus.init_status == FpaInitStatus::UNSPECIFIED)) {
        WARNING("Ignoring unusable epoch at %04d:%010.3f", fpa_eoe.gps_time.week.value, fpa_eoe.gps_time.tow.value);
        return;
    }

    // Now we should have all the data we want. For this example we only use the position if fusion is fully
    // initialised.
    if (fpa_odomstatus.init_status != FpaInitStatus::GLOBAL_INIT) {
        WARNING(
            "Ignoring uninitialised fusion at %04d:%010.3f", fpa_eoe.gps_time.week.value, fpa_eoe.gps_time.tow.value);
        return;
    }

    // We have a usable position
    INFO("Using position at %04d:%010.3f: [ %.1f, %.1f, %.1f ]", fpa_eoe.gps_time.week.value,
        fpa_eoe.gps_time.tow.value, fpa_odometry.pos.values[0], fpa_odometry.pos.values[1], fpa_odometry.pos.values[2]);

    // Transform to local CRS
    Eigen::Vector3d pos = { fpa_odometry.pos.values[0], fpa_odometry.pos.values[1], fpa_odometry.pos.values[2] };
    if (trafo.Transform(pos)) {
        INFO("Local coordinates: %.1f %.1f %.1f", pos(0), pos(1), pos(2));
    }

    // Notes:
    // - The same approach can be used to collect NMEA messages into an epoch. For example, using the FP_A-EOE for GNSS,
    //   this lets one reliably combine all NMEA messages for GNSS, including those that do not have a timestamp, into
    //   a consistent set of messages that are part of the same naviation epoch.
}

/* ****************************************************************************************************************** */
