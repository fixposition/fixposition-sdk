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
 * @brief Fixposition SDK examples: introduction to the parser and FP_A message decoding
 *
 * To build and run:
 *
 *     make
 *     ./build/parser_intro
 *
 * This program demonstrates how to use the fpsdk::common::parser functions to parse FP_A protocol
 * messages from data received from the Vision-RTK 2 sensor as well as how to decode those messages
 * for futher processing.
 *
 * This file is both the source code of this example as well as the documentation on how it works.
 */

/* LIBC/STL */
#include <cinttypes>
#include <cstdint>
#include <cstdlib>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/math.hpp>
#include <fpsdk_common/parser.hpp>
#include <fpsdk_common/parser/fpa.hpp>
#include <fpsdk_common/time.hpp>
#include <fpsdk_common/trafo.hpp>

/* PACKAGE */

/* ****************************************************************************************************************** */

using namespace fpsdk::common::time;
using namespace fpsdk::common::app;
using namespace fpsdk::common::parser;
using namespace fpsdk::common::parser::fpa;
using namespace fpsdk::common::logging;
using namespace fpsdk::common::trafo;
using namespace fpsdk::common::math;

// ---------------------------------------------------------------------------------------------------------------------

// This is an example of data received from the Vision-RTK 2 sensor. Note that the data is a long sequence of bytes.
// Even though it looks like strings, do not treat data received from the sensor as strings. Always treat it as a
// sequence of bytes that may have any value, including the string nul terminator (0x00) character. For readability
// the data here is formatted as one FP_A message per line. Do not assume this is how a program would receive the data
// from the sensor in a real application. Depending on the connection and implementation the app receives the data
// in chunks, perhaps as small as one byte at a time. Or there may be errors (some detailed below) in transmissions.
// To extract complete message frames from such a stream of data we can use the *parser*.
// clang-format off
const uint8_t SAMPLE_DATA_1[] = {
    // The data starts with what looks like an incomplete FP_A-ODOMETRY...
    /*  (1) */ "2650,0.00283,-0.00305,-0.02449,0.00108,0.00054,0.00057,-0.00009,-0.00002,0.00024,"
               "fp_vrtk2-integ_6912e460-1703*3C\r\n"
    // Then it seems we're receiving correct FP_A messages as expected...
    /*  (2) */ "$FP,ODOMSTATUS,1,2348,574451.500000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*2D\r\n"
    /*  (3) */ "$FP,EOE,1,2348,574451.500000,FUSION*6C\r\n"
    // ..and more messages...
    /*  (4) */ "$FP,ODOMETRY,2,2348,574452.000000,4278387.7000,635620.5134,4672339.9355,-0.603913,0.313038,-0.179283,"
               "0.710741,0.0003,-0.0009,0.0012,-0.00110,0.00128,0.00026,0.2101,0.1256,9.8099,4,0,8,8,-1,0.00073,"
               "0.00294,0.00107,0.00141,-0.00170,-0.00083,0.02271,0.00042,0.02650,0.00283,-0.00305,-0.02448,0.00101,"
               "0.00053,0.00056,-0.00008,-0.00002,0.00021,fp_vrtk2-integ_6912e460-1703*17\r\n"
    /*  (5) */ "$FP,ODOMSTATUS,1,2348,574452.000000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*2B\r\n"
    /*  (6) */ "$FP,EOE,1,2348,574452.000000,FUSION*6A\r\n"
    // ..and more messages...
    /*  (7) */ "$FP,ODOMETRY,2,2348,574452.500000,4278387.6984,635620.5126,4672339.9366,0.313004,-0.179369,0.710598,"
               "0.0002,-0.0008,0.0009,0.00060,-0.00117,0.00038,0.2129,0.1266,9.8185,4,0,8,8,-1,0.00073,0.00294,0.00107,"
               "0.00141,-0.00169,-0.00083,0.02270,0.00042,0.02650,0.00283,-0.00305,-0.02448,0.00098,0.00053,0.00056,"
               "-0.00008,-0.00002,0.00019,fp_vrtk2-integ_6912e460-1703*01\r\n"
    /*  (8) */ "$FP,ODOMSTATUS,1,2348,574452.500000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*2E\r\n"
    /*  (9) */ "$FP,EOE,1,2348,574452.500000,FUSION*6F\r\n"
    // Hmmm... there seems to be some kind of errors in the reception of the data. Perhaps a bad cable or a shaky connector?
    /* (10) */ "$FP,ODOM3TRY,2,2348,574453.000000,4278387.6988,6\xaa\x55\xaa\x55.604077,0.312982\xba\xad\xc0\xff\xee.71"
               "0587,0.0010,-0.0005,0.0018,0.00368,-0.00044,0.00092,0.2163,0.1214,9.8162,4,0,8,8,-1,0.00073,0.00294,"
               "0.00107,0.00141,-0.00170,-0.00083,0.02269,0.00042,0.02650,0.00283,-0.00305,-0.02448,0.00100,0.00053,"
               "0.00056,-0.00008,-0.00002,0.00021,fp_vrtk2-integ_6912e460-1703*12\r\n"
    // Ok, back to normal it seems..
    /* (11) */ "$FP,ODOMSTATUS,1,2348,574453.000000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*2A\r\n"
    /* (12) */ "$FP,EOE,1,2348,574453.000000,FUSION*6B\r\n"
    // The data ends in an incomplete FP_A-ODOMETRY message...
    /* (13) */ "$FP,ODOMETRY,2,2348,579519.500000,4278387.7342,635620.5918,4672339.8925,-0.700505,0.286"
};
const std::size_t SAMPLE_SIZE_1 = sizeof(SAMPLE_DATA_1) - 1; // -1 because "" adds \0
// A second chunk of data, which is the continuation from the above
const uint8_t SAMPLE_DATA_2[] = {
    /* (13) */"891,-0.222201,0.614502,0.0002,0.0000,-0.0002,-0.00071,-0.00146,-0.00001,0.2974,0.0215,9.8060,4,0,8,8,-1,"
              "0.00025,0.00493,0.00060,0.00099,-0.00161,-0.00033,0.02743,0.00044,0.03268,0.00316,-0.00342,-0.02989,"
              "0.00094,0.00066,0.00049,-0.00023,-0.00003,0.00007,fp_vrtk2-integ_6912e460-1703*18\r\n"
    /* (14) */"$FP,ODOMSTATUS,1,2348,579519.500000,2,2,1,1,1,1,,0,,,,,,0,1,3,8,8,3,5,5,,0,6,,,,,,,,,,,,*2D\r\n"
    /* (15) */"$FP,EOE,1,2348,579519.500000,FUSION*6C\r\n"
    /* (16) */"$FP,ODOMETRY,2,2348,579520.000000,,,,-0.700563,0.286828,-0.222129,0.614491,0.0013,-0.0012,0.0004,"
              "-0.00513,-0.00055,-0.00092,0.2984,0.0225,9.8047,4,0,8,8,-1,0.00025,0.00493,0.00060,0.00099,-0.00161,"
              "-0.00033,0.02743,0.00044,0.03268,0.00316,-0.00342,-0.02989,0.00094,0.00066,0.00048,-0.00024,-0.00003,"
              "0.00007,fp_vrtk2-integ_6912e460-1703*35\r\n"
    /* (17) */"$FP,ODOMSTATUS,1,2348,579520.00000"
};
const std::size_t SAMPLE_SIZE_2 = sizeof(SAMPLE_DATA_2) - 1; // -1 because "" adds \0
// clang-format on

int main(int /*argc*/, char** /*argv*/)
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    LoggingSetParams({ LoggingLevel::TRACE });

    // -----------------------------------------------------------------------------------------------------------------
    // Experiment 1
    // -----------------------------------------------------------------------------------------------------------------
    NOTICE("----- Parsing the data into messages -----");

    // The first step is to *parse* the data. This process splits the data into message frames. Note that "parsing" is
    // not the same as "decoding" the messages. "Parsing" (as per Fixposition SDK definition) refers to splitting a
    // stream of data into individual messages (als known as "frames"). The Parser recgonises a variety of protocols and
    // it can reliably split data into messages of all the protocols it understands. It can do so in streams of mixed
    // data, such as FP_A, NMEA and NOV_B messages in the same stream.

    // Create a parser instance. Note that one parser should be used per stream of data. If multiple streams (inputs)
    // are handled, they should all be processed by a separate Parser instance.
    Parser parser;

    // Feed data to the parser. We'll have to check that we don't overwhelm the parser by feeding too much data.
    // The details for this are explained in the fpsdk::common::parser documentation in fpsdk_common/parser.hpp.
    INFO("Adding SAMPLE_DATA_1 (%" PRIuMAX " bytes)", SAMPLE_SIZE_1);
    if (!parser.Add(SAMPLE_DATA_1, SAMPLE_SIZE_1)) {
        WARNING("Parser overflow, SAMPLE_DATA_1 is too large!");
    }

    // Now we can process the data in the parser and we get back the data we fed in above message by message. On
    // success, the ParserMsg object is populated with the part of the input data that corresponds to a message along
    // with some meta data, such as a message name. The naming of the messages is explained in the fpsdk::common::parser
    // documentation.
    ParserMsg msg;
    while (parser.Process(msg)) {
        INFO("Message %-20s (size %" PRIuMAX " bytes)", msg.name_.c_str(), msg.Size());
        // DEBUG_HEXDUMP(msg.Data(), msg.Size(), NULL, NULL);  // Hexdump of the raw message data
    }
    // This should print the following output:
    //
    //     Adding SAMPLE_DATA_1 (1810 bytes)
    //     Message OTHER                (size 114 bytes)                      (1)
    //     Message FP_A-ODOMSTATUS      (size 93 bytes)                       (2)
    //     Message FP_A-EOE             (size 40 bytes)                       (3)
    //     Message FP_A-ODOMETRY        (size 372 bytes)                      (4)
    //     Message FP_A-ODOMSTATUS      (size 93 bytes)                       (5)
    //     Message FP_A-EOE             (size 40 bytes)                       (6)
    //     Message FP_A-ODOMETRY        (size 362 bytes)                      (7)
    //     Message FP_A-ODOMSTATUS      (size 93 bytes)                       (8)
    //     Message FP_A-EOE             (size 40 bytes)                       (9)
    //     Message OTHER                (size 256 bytes)                     (10)
    //     Message OTHER                (size 87 bytes)                      (10)
    //     Message FP_A-ODOMSTATUS      (size 93 bytes)                      (11)
    //     Message FP_A-EOE             (size 40 bytes)                      (12)
    //
    // We can observe the following parser behaviour:
    //
    // - A total of 1723 bytes of the 1810 input bytes are output in 13 messages (1) - (13)
    // - Message (1) is of the "fake" type Protocol::OTHER, which means it is data that doesn't match any of the known
    //   protocols. Even though it is a partial FP_A message, the parser cannot recognise this as the frame is
    //   incomplete and therefore does not pass the "looks like an FP_A message" check.
    // - Similarly, the corrupt message (13) is output as OTHER messages. Since a OTHER message is limited to 256 bytes,
    //   we see two such messages being emitted from the parser.
    // - Message (14) is missing. This is expected as it is the beginning of a FP_A message and the parser cannot yet
    //   determine that is _not_ such a message. For this decision it would need more data.
    // - That is, 87 byte (1810 bytes input minus 1723 bytes output) are "stuck" in the parser.

    // Let's add more data, namely the rest of the partial FP_A-ODOMETRY now "stuck" in the parser, and process more
    INFO("Adding SAMPLE_DATA_2 (%" PRIuMAX " bytes)", SAMPLE_SIZE_2);
    if (!parser.Add(SAMPLE_DATA_2, SAMPLE_SIZE_2)) {
        WARNING("Parser overflow, SAMPLE_DATA_2 is too large!");
    }
    while (parser.Process(msg)) {
        INFO("Message %-20s (size %" PRIuMAX " bytes)", msg.name_.c_str(), msg.Size());
        // DEBUG_HEXDUMP(msg.Data(), msg.Size(), NULL, NULL);  // Hexdump of the raw message data
    }
    // This should print the following output:
    //
    //     Adding SAMPLE_DATA_2 (793 bytes)
    //     Message FP_A-ODOMETRY        (size 374 bytes)                    (13)
    //     Message FP_A-ODOMSTATUS      (size 93 bytes)                     (14)
    //     Message FP_A-EOE             (size 40 bytes)                     (15)
    //     Message FP_A-ODOMETRY        (size 339 bytes)                    (16)
    //
    // We can observe the following parser behaviour:
    //
    // - A total of 846 byte are output in messages. Note that at the time we added the second chunk of data (793 bytes)
    //   there were still 87 bytes from the first chunk left. So in total the parser had 880 bytes of data.
    // - The parser now had enough data to output message (13)
    // - Also there was data for messages (14) - (16)
    // - There must be 34 (880 - 846) bytes "stuck" in the parser now

    // The parser has a "flush" mode that forces "stuck" bytes to be output even though they look like the beginning of
    // a valid message:
    INFO("Flushing parser");
    while (parser.Flush(msg)) {
        INFO("Message %-20s (size %" PRIuMAX " bytes)", msg.name_.c_str(), msg.Size());
        DEBUG_HEXDUMP(msg.Data(), msg.Size(), NULL, NULL);
    }
    // This should print:
    //
    //     Message OTHER                (size 34 bytes)                     (17)
    //     0x0000 00000  24 46 50 2c 4f 44 4f 4d  53 54 41 54 55 53 2c 31  |$FP,ODOMSTATUS,1|
    //     0x0010 00016  2c 32 33 34 38 2c 35 37  39 35 32 30 2e 30 30 30  |,2348,579520.000|
    //     0x0020 00032  30 30                                             |00              |
    //
    // And indeed it returned the 34 bytes that were stuck in the parser and indeed they are the incomplete
    // FP_A-ODOMSTATUS message at the end of the second junk.

    // -----------------------------------------------------------------------------------------------------------------
    // Experiment 2
    // -----------------------------------------------------------------------------------------------------------------
    NOTICE("----- Decoding the data in the messages -----");

    // We have now seen how to *parse* the messages. Let's have a closer look at how to *decode* the payload (contents)
    // of some of the messages.

    // Reset the parser to assert it's empty and back to the initial state. Then, add all data we have.
    INFO("Adding SAMPLE_DATA_1+SAMPLE_DATA_2 (%" PRIuMAX " bytes)", SAMPLE_SIZE_1 + SAMPLE_SIZE_2);
    parser.Reset();
    if (!parser.Add(SAMPLE_DATA_1, SAMPLE_SIZE_1) || !parser.Add(SAMPLE_DATA_2, SAMPLE_SIZE_2)) {
        WARNING("Parser overflow, SAMPLE_DATA_1+SAMPLE_DATA_2 is too large!");
    }

    // Loop though all messages and have a closer look at FP_A-ODOMETRY
    while (parser.Process(msg)) {
        // Is it a FP_A-ODOMETRY?
        if (msg.name_ == FpaOdometryPayload::MSG_NAME) {
            INFO("Message %-20s (size %" PRIuMAX " bytes)", msg.name_.c_str(), msg.Size());
            // We can now try to decode it
            FpaOdometryPayload payload;
            if (payload.SetFromMsg(msg.Data(), msg.Size())) {
                INFO("Decode OK");
            }
            // Decoding failed
            else {
                INFO("Decode fail");
            }
        }
    }
    // We should get:
    //
    //     Adding SAMPLE_DATA_1+SAMPLE_DATA_2 (2603 bytes)
    //     Message FP_A-ODOMETRY        (size 372 bytes)                     (4)
    //     Message OK
    //     Message FP_A-ODOMETRY        (size 362 bytes)                     (7)
    //     Message decode fail                                                  <---- !!!
    //     Message FP_A-ODOMETRY        (size 374 bytes)                    (13)
    //     Message OK
    //     Message FP_A-ODOMETRY        (size 339 bytes)                    (16)
    //     Message OK
    //
    // We again see all four FP_A-ODOMETRY messges. However, the second messge (7) failed to decode. If we have a close
    // look at SAMPLE_DATA_1 we can see that even though it is a valid FP_A message frame (the checksum is correct) and
    // the message type ("ODOMETRY") is present, the number of fields does not match the specification for FP_A-ODOMETRY
    // and therefore FpaOdometryPayload::SetFromMsg() complains. This method does various checks to make sure the data
    // it decodes matches the corresponding message specification.

    // -----------------------------------------------------------------------------------------------------------------
    // Experiment 3
    // -----------------------------------------------------------------------------------------------------------------
    NOTICE("----- Using decoded data in the messages -----");

    // We repeat the parsing and decoding and now try to *use* some of the data.

    // Reset the parser to assert it's empty and back to the initial state. Then, add all data we have.
    INFO("Adding SAMPLE_DATA_1+SAMPLE_DATA_2 (%" PRIuMAX " bytes)", SAMPLE_SIZE_1 + SAMPLE_SIZE_2);
    parser.Reset();
    if (!parser.Add(SAMPLE_DATA_1, SAMPLE_SIZE_1) || !parser.Add(SAMPLE_DATA_2, SAMPLE_SIZE_2)) {
        WARNING("Parser overflow, SAMPLE_DATA_1+SAMPLE_DATA_2 is too large!");
    }

    // Loop though all messages and have a closer look at some of the data in the valid FP_A-ODOMETRY messages
    while (parser.Process(msg)) {
        // Ignore messages other than FP_A-ODOMETRY
        if (msg.name_ != FpaOdometryPayload::MSG_NAME) {
            continue;
        }
        // Ignore invalid FP_A-ODOMETRY messages
        FpaOdometryPayload payload;
        if (!payload.SetFromMsg(msg.Data(), msg.Size())) {
            continue;
        }

        INFO("Message %-20s (size %" PRIuMAX " bytes)", msg.name_.c_str(), msg.Size());

        // Some the message fields are optional and in principle any field can be a "null" (empty) field. It is highly
        // recommended to *always* check if the desired fields are valid.

        // For example we can check if the time is available. It it is, we can convert the GPS time to UTC for the
        // debugging purposes here. (Note that you should probably not use UTC time for anything other than to display
        // time to humans.)

        // GPS week number and time of week can be indpendently valid or invalid
        if (payload.gps_time.week.valid && payload.gps_time.tow.valid) {
            Time time;
            // We should also handle the week number and/or time of week values not being in range
            if (time.SetWnoTow({ payload.gps_time.week.value, payload.gps_time.tow.value, WnoTow::Sys::GPS })) {
                INFO("GPS time %04d:%010.3f (%s)", payload.gps_time.week.value, payload.gps_time.tow.value,
                    time.StrUtcTime().c_str());
            } else {
                INFO("GPS time %04d:%010.3f is bad", payload.gps_time.week.value, payload.gps_time.tow.value);
            }
        } else {
            INFO("GPS time not available");
        }

        // The position data can should be checked for availability, too
        if (payload.pos.valid) {
            INFO("Position: [ %.1f,  %.1f,  %.1f ]", payload.pos.values[0], payload.pos.values[1],
                payload.pos.values[2]);
        } else {
            INFO("Position not available");
        }

        // We can transform the position to latititude, longitude and height (and lat/lon to degrees). Note that
        // TfWgs84LlhEcef() should not be used for anything other than debugging. Instead, an appropriately configured
        // fpsdk::common::trafo::Transformer instance should be used to transform from the input coordinate reference
        // system (determined by the correction data service used with the sensor) to the desired output coordinate
        // reference system (which may or may not be WGS-84). See the fusion_epoch example.
        if (payload.pos.valid) {
            const auto llh = TfWgs84LlhEcef({ payload.pos.values[0], payload.pos.values[1], payload.pos.values[2] });
            INFO("LLH: %.6f %.6f %.1f", RadToDeg(llh(0)), RadToDeg(llh(1)), llh(2));
        }
    }
    // This should output:
    //     Adding SAMPLE_DATA_1+SAMPLE_DATA_2 (2603 bytes)
    //     Message FP_A-ODOMETRY        (size 372 bytes)                     (4)
    //     GPS time 2348:574452.000 (2025-01-11 15:33:54.000)
    //     Position: [ 4278387.7,  635620.5,  4672339.9 ]
    //     LLH: 47.400296 8.450363 459.5
    //     Message FP_A-ODOMETRY        (size 374 bytes)                    (13)
    //     GPS time 2348:579519.500 (2025-01-11 16:58:21.500)
    //     Position: [ 4278387.7,  635620.6,  4672339.9 ]
    //     LLH: 47.400296 8.450364 459.5
    //     Message FP_A-ODOMETRY        (size 339 bytes)                    (16)
    //     GPS time 2348:579520.000: 2025-01-11 16:58:22.000
    //     Position not available

    return EXIT_SUCCESS;
}

/* ****************************************************************************************************************** */
