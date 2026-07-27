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
 * @brief Fixposition SDK example: Fixposition SDK example: camera streaming from PBx-A1 sensor
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
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

/* EXTERNAL */
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/extended_p_square.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/sum.hpp>
#include <boost/accumulators/statistics/variance.hpp>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/cam.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>
#include <fpsdk_common/types.hpp>
#include <fpsdk_common/video.hpp>

/* PACKAGE */

/* ****************************************************************************************************************** */

using namespace fpsdk::common::app;
using namespace fpsdk::common::cam;
using namespace fpsdk::common::logging;
using namespace fpsdk::common::string;
using namespace fpsdk::common::time;
using namespace fpsdk::common::video;

// ---------------------------------------------------------------------------------------------------------------------

// Statistics
struct Stats
{
    // clang-format off
    static constexpr std::array<double, 4> PROB = {{ 0.5, 0.68, 0.95, 0.997 }};
    using Accumulator = boost::accumulators::accumulator_set<double, boost::accumulators::stats<
        boost::accumulators::tag::count,
        boost::accumulators::tag::mean,
        boost::accumulators::tag::min,
        boost::accumulators::tag::max,
        boost::accumulators::tag::sum,
        boost::accumulators::tag::variance,
        boost::accumulators::tag::extended_p_square>>;
    Accumulator lat  { boost::accumulators::extended_p_square_probabilities = PROB };  // Latency receiving the data
    Accumulator size { boost::accumulators::extended_p_square_probabilities = PROB };  // Size of the data
    Accumulator exp  { boost::accumulators::extended_p_square_probabilities = PROB };  // Exposure duration
    Accumulator dec  { boost::accumulators::extended_p_square_probabilities = PROB };  // Time decoding video
    // clang-format on
};

// ---------------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif

    // We need three things (see CamStreamParams):
    //
    // 1. The sensor (hostname, IP address)
    // 2. Which camera (CAM1, CAM2, ...)
    // 3. Which data (HIRES_VID, LORES_IMG, ...)
    // 4. Throttle value
    //
    if (argc != 5) {
        ERROR("Missing arguments!");
        INFO(
            "Usage: camera_streaming <sensor> <cam> <type> <rate>\n"
            "Where: <sensor> is the sensor's hostname or IP address, <cam> is CAM1 or CAM2, <type> is\n"
            "HIRES_VID, LORES_VID, HIRES_IMG or LORES_IMG, and <rate> is the throttling rate (must be\n"
            "1 for encoded video)\n"
            "Examples:\n"
            "    camera_streaming 10.0.2.1 CAM1 HIRES_VID 1\n"
            "    camera_streaming 10.0.2.1 CAM1 HIRES_IMG 10\n"
            "    timeout -s SIGINT 60 camera_streaming ...");
        return EXIT_FAILURE;
    }

#if 0  // Set to 1 (and build with CMAKE_BUILD_TYPE=Debug) for debugging
    LoggingSetParams({ LoggingLevel::TRACE, LoggingColour::AUTO, LoggingTimestamps::RELATIVE });
#else
    LoggingSetParams({ LoggingLevel::INFO, LoggingColour::AUTO, LoggingTimestamps::RELATIVE });
#endif

    const std::string sensor = argv[1];
    const CamId cam_id = CamIdFromStrOr(argv[2], CamId::UNSPECIFIED);
    const CamDataType type = CamDataTypeFromStrOr(argv[3], CamDataType::UNSPECIFIED);
    int rate = 0;
    StrToValue(argv[4], rate);

    NOTICE("camera_streaming %s %s %s %d", sensor.c_str(), CamIdToStr(cam_id), CamDataTypeToStr(type), rate);

    // Create stream handle (this formally verifies the arguments)
    auto stream = CreateCamStream({ "cam", sensor, cam_id, type, rate });
    if (!stream) {
        return EXIT_FAILURE;
    }

    // Connect to sensor
    if (!stream->Connect()) {
        return EXIT_FAILURE;
    }

    // We'll decode encoded video frames
    VideoFrameDecoderPtr decoder;

    // Statistics
    Stats stats;

    // Stream data until we get SIGINT (CTRL-c)
    SigIntHelper sigint;
    CamData data;
    std::size_t n_frames = 0;
    char info[1000];
    bool ok = true;
    const auto t_start = Time::FromClockRealtime();
    while (ok && !sigint.ShouldAbort() && stream->NextFrame(data)) {
        // We'll calculate the latency from the time we received the data (now) and the data reference time. Note that
        // this requires the computer that runs this program being timesynced with the sensor (and the sensor being
        // timesynced, too, ideally with GNSS). Any error in timesync of the sensor or the computer affects the measured
        // latency (could be to the better or to the worse). Also, we'll use the end of exposure as the reference time
        // instead of the middle of exposure.
        const auto t_recv = Time::FromClockRealtime();
        const auto t_expo = Duration::FromNSec(data.dt_);
        const auto t_data = Time::FromPosixNs(data.ts_) + (t_expo * 0.5);
        n_frames++;

        // Generic info
        std::size_t len =
            std::snprintf(info, sizeof(info), "Frame %6" PRIuMAX ": %-7s %-5s %-10s %-9s %-7s %6" PRIuMAX " %s %4.1f",
                n_frames, data.valid_ ? "valid" : "invalid", CamIdToStr(data.cam_id_), CamDataTypeToStr(data.type_),
                CamDataFmtToStr(data.fmt_), CamDataFrmToStr(data.frm_), data.data_.size(), t_data.StrIsoTime(3).c_str(),
                Duration::FromNSec(data.dt_).GetSec() * 1e3);

        // Update statistics
        stats.size((double)data.data_.size() / 1024.0);  // [KiB]
        stats.exp((double)t_expo.GetSec() * 1e3);        // [ms]

        // Latency
        if (data.valid_) {
            const double lat = (t_recv - t_data).GetSec() * 1e3;  // [ms]
            len += std::snprintf(&info[len], sizeof(info) - len, " -- latency: %+5.1f", lat);
            stats.lat(lat);
        }

        // Decode video
        if (data.fmt_ == CamDataFmt::H265_NAL) {
            // We can only start decoding from the first I-frame onwards
            if (!decoder && (data.frm_ == CamDataFrm::I_FRAME)) {
                decoder = CreateVideoFrameDecoder({ "dec", VideoCodec::H265, PixelFmt::RGB24 });
            }
            if (decoder) {
                TicToc tt;
                const auto img = decoder->DecodeFrame(data.data_);
                if (!img) {
                    ok = false;
                    break;
                }

                // --------------------------------------------------------------------------------------
                // Now we have the decoded image in "img" and all the raw data and meta data in "data"...
                // --------------------------------------------------------------------------------------

                // Decoding adds latency
                const auto lat = tt.Toc().GetSec() * 1e3;  // [ms]
                len += std::snprintf(&info[len], sizeof(info) - len, " -- decode: %+5.1f", lat);
                stats.dec(lat);
            }
        }

        INFO("%s", info);
    }
    const auto t_end = Time::FromClockRealtime();
    const auto t_str = (t_end - t_start).GetSec();

    stream->Disconnect();

    // Print stats
    NOTICE("Streamed %s %s %s %d for %.0fs (%" PRIuMAX " frames)", sensor.c_str(), CamIdToStr(cam_id),
        CamDataTypeToStr(type), rate, t_str, boost::accumulators::count(stats.size));
    if (boost::accumulators::count(stats.lat) > 10) {  // clang-format off
        INFO("Latency receiving data [ms]: mean %4.1f (std %4.1f) min/0.5/0.68/0.95/0.997/max %4.1f %4.1f %4.1f %4.1f %4.1f %4.1f",  // clang-format on
            boost::accumulators::mean(stats.lat), std::sqrt(boost::accumulators::variance(stats.lat)),
            boost::accumulators::min(stats.lat), boost::accumulators::extended_p_square(stats.lat)[0],
            boost::accumulators::extended_p_square(stats.lat)[1], boost::accumulators::extended_p_square(stats.lat)[2],
            boost::accumulators::extended_p_square(stats.lat)[3], boost::accumulators::max(stats.lat));
    }
    if (boost::accumulators::count(stats.dec) > 10) {  // clang-format off
        INFO("Latency decoding video [ms]: mean %4.1f (std %4.1f) min/0.5/0.68/0.95/0.997/max %4.1f %4.1f %4.1f %4.1f %4.1f %4.1f",  // clang-format on
            boost::accumulators::mean(stats.dec), std::sqrt(boost::accumulators::variance(stats.dec)),
            boost::accumulators::min(stats.dec), boost::accumulators::extended_p_square(stats.dec)[0],
            boost::accumulators::extended_p_square(stats.dec)[1], boost::accumulators::extended_p_square(stats.dec)[2],
            boost::accumulators::extended_p_square(stats.dec)[3], boost::accumulators::max(stats.dec));
    }
    if (boost::accumulators::count(stats.exp) > 10) {  // clang-format off
        INFO("Exposure duration [ms]:      mean %4.1f (std %4.1f) min/0.5/0.68/0.95/0.997/max %4.1f %4.1f %4.1f %4.1f %4.1f %4.1f",  // clang-format on
            boost::accumulators::mean(stats.exp), std::sqrt(boost::accumulators::variance(stats.exp)),
            boost::accumulators::min(stats.exp), boost::accumulators::extended_p_square(stats.exp)[0],
            boost::accumulators::extended_p_square(stats.exp)[1], boost::accumulators::extended_p_square(stats.exp)[2],
            boost::accumulators::extended_p_square(stats.exp)[3], boost::accumulators::max(stats.exp));
    }
    if ((boost::accumulators::count(stats.size) > 10) && (t_str > 1.0)) {  // clang-format off
        INFO("Data size per frame [KiB]:   mean %4.0f (std %4.0f) min/0.5/0.68/0.95/0.997/max %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f",  // clang-format on
            boost::accumulators::mean(stats.size), std::sqrt(boost::accumulators::variance(stats.size)),
            boost::accumulators::min(stats.size), boost::accumulators::extended_p_square(stats.size)[0],
            boost::accumulators::extended_p_square(stats.size)[1],
            boost::accumulators::extended_p_square(stats.size)[2],
            boost::accumulators::extended_p_square(stats.size)[3], boost::accumulators::max(stats.size));
        const double avg = boost::accumulators::sum(stats.size) / t_str;
        // Assuming GibE link TCP socket ~115MB/s = ~120500 KiB/s
        INFO("Average data rate [KiB/s]: %.0f (~%.1f%%GigE)", avg, avg / 120500.0 * 1e2);
    }

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

/* ****************************************************************************************************************** */
