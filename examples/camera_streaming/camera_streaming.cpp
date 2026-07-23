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
#include <cstdint>
#include <cstring>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/cam.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>
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
            "For example: camera_streaming 10.0.2.1 CAM1 HIRES_VID 1");
        return EXIT_FAILURE;
    }

    // LoggingSetParams({ LoggingLevel::TRACE, LoggingColour::AUTO, LoggingTimestamps::RELATIVE });
    LoggingSetParams({ LoggingLevel::INFO, LoggingColour::AUTO, LoggingTimestamps::RELATIVE });

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

    // Stream data until we get SIGINT (CTRL-c)
    SigIntHelper sigint;
    CamData data;
    std::size_t n_frames = 0;
    char info[1000];
    bool ok = true;
    while (ok && !sigint.ShouldAbort() && stream->NextFrame(data)) {
        // We'll calculate the latency from the time we received the data (now) and the data reference time. Note that
        // this requires the computer that runs this program being timesynced with the sensor (and the sensor being
        // timesynced, too, ideally with GNSS). Any error in timesync of the sensor or the computer affects the measured
        // latency (could be to the better or to the worse).
        const auto t_recv = Time::FromClockRealtime();
        const auto t_data = Time::FromPosixNs(data.ts_);
        const auto dt = Duration::FromNSec(data.dt_);  // exposure duration
        n_frames++;

        // Generic info
        std::size_t len = std::snprintf(info, sizeof(info), "Frame %6" PRIuMAX ": %-7s %-5s %-10s %-9s %-7s %s %4.1f",
            n_frames, data.valid_ ? "valid" : "invalid", CamIdToStr(data.cam_id_), CamDataTypeToStr(data.type_),
            CamDataFmtToStr(data.fmt_), CamDataFrmToStr(data.frm_), t_data.StrIsoTime(3).c_str(), dt.GetSec() * 1e3);

        // Latency
        if (data.valid_) {
            const auto lat = t_recv - t_data;
            len += std::snprintf(&info[len], sizeof(info) - len, " -- %+5.1f", lat.GetSec() * 1e3 + 0.1);
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

                len += std::snprintf(&info[len], sizeof(info) - len, " -- %+4.1f", tt.TocMs());
            }
        }

        INFO("%s", info);
    }

    stream->Disconnect();

    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

/* ****************************************************************************************************************** */
