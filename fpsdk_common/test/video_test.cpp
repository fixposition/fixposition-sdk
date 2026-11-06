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
 * @brief Fixposition SDK: tests for fpsdk::common::video
 */

/* LIBC/STL */
#include <algorithm>
#include <cstdlib>
#include <map>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/video.hpp>

namespace {
/* ****************************************************************************************************************** */
#if FPSDK_USE_FFMPEG

#  include "data/hevc_gop.cpp"
#  include "data/hevc_i_frame.cpp"

using namespace fpsdk::common::video;
using namespace fpsdk::common::path;

// Split H.265 NAL stream (with 4-byte start codes) into frames. Each frame ends with a VCL NALU (slice, type < 32),
// preceded by any non-VCL NALUs (VPS, SPS, PPS, ...)
static std::vector<std::vector<uint8_t>> SplitHevcFrames(const std::vector<uint8_t>& data)
{
    std::vector<std::size_t> offs;
    for (std::size_t ix = 0; (ix + 4) < data.size(); ix++) {
        if ((data[ix] == 0x00) && (data[ix + 1] == 0x00) && (data[ix + 2] == 0x00) && (data[ix + 3] == 0x01)) {
            offs.push_back(ix);
        }
    }
    offs.push_back(data.size());

    std::vector<std::vector<uint8_t>> frames;
    std::size_t frame_start = 0;
    for (std::size_t ix = 0; (ix + 1) < offs.size(); ix++) {
        const int nalu_type = (data[offs[ix] + 4] >> 1) & 0x3f;
        if (nalu_type < 32) {
            frames.emplace_back(data.begin() + frame_start, data.begin() + offs[ix + 1]);
            frame_start = offs[ix + 1];
        }
    }
    return frames;
}

TEST(VideoTest, DecodeFrame_RGB24)
{
    auto dec = CreateVideoFrameDecoder({ "test", VideoCodec::H265, PixelFmt::RGB24, 1.0, HwAccel::SW });
    EXPECT_NE(dec, nullptr);
    EXPECT_TRUE(dec->IsOkay());

    auto img = dec->DecodeFrame(HEVC_I_FRAME_BIN);
    EXPECT_TRUE(img);
    EXPECT_TRUE(dec->IsOkay());

    EXPECT_EQ(img->fmt_, PixelFmt::RGB24);
    EXPECT_EQ(img->width_, 1024);
    EXPECT_EQ(img->height_, 768);
    EXPECT_EQ(img->data_.size(), 1024 * 768 * 3);

    // FileSpew("test-rgb24.bin", img->data_);
    // $ convert -depth 8 -size 1024x768+0 rgb:test-rgb24.bin test-rgb24.png
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(VideoTest, DecodeFrame_Y8)
{
    auto dec = CreateVideoFrameDecoder({ "test", VideoCodec::H265, PixelFmt::Y8, 1.0, HwAccel::SW });
    EXPECT_NE(dec, nullptr);
    EXPECT_TRUE(dec->IsOkay());

    auto img = dec->DecodeFrame(HEVC_I_FRAME_BIN);
    EXPECT_TRUE(img);
    EXPECT_TRUE(dec->IsOkay());

    EXPECT_EQ(img->fmt_, PixelFmt::Y8);
    EXPECT_EQ(img->width_, 1024);
    EXPECT_EQ(img->height_, 768);
    EXPECT_EQ(img->data_.size(), 1024 * 768);

    // FileSpew("test-y8.bin", img->data_);
    // $ convert -depth 8 -size 1024x768+0 gray:test-y8.bin test-y8.png
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(VideoTest, DecodeFrame_RGB24_05)
{
    auto dec = CreateVideoFrameDecoder({ "test", VideoCodec::H265, PixelFmt::RGB24, 0.5, HwAccel::SW });
    EXPECT_NE(dec, nullptr);
    EXPECT_TRUE(dec->IsOkay());

    auto img = dec->DecodeFrame(HEVC_I_FRAME_BIN);
    EXPECT_TRUE(img);
    EXPECT_TRUE(dec->IsOkay());

    EXPECT_EQ(img->fmt_, PixelFmt::RGB24);
    EXPECT_EQ(img->width_, 1024 / 2);
    EXPECT_EQ(img->height_, 768 / 2);
    EXPECT_EQ(img->data_.size(), (1024 / 2) * (768 / 2) * 3);

    // FileSpew("test-rgb24-05.bin", img->data_);
    // $ convert -depth 8 -size 512x384+0 rgb:test-rgb24-05.bin test-rgb24-05.png
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(VideoTest, DecodeGop)
{
    // One GOP: I frame (VPS, SPS, PPS, IDR) followed by 9 P frames (TRAIL_R)
    const auto frames = SplitHevcFrames(HEVC_GOP_BIN);
    ASSERT_EQ(frames.size(), 10);

    std::map<PixelFmt, std::vector<ImageData>> imgs;
    for (const auto fmt : { PixelFmt::Y8, PixelFmt::RGB24, PixelFmt::GBRP }) {
        SCOPED_TRACE(PixelFmtToStr(fmt));
        auto dec = CreateVideoFrameDecoder({ "test", VideoCodec::H265, fmt, 1.0, HwAccel::SW });
        ASSERT_NE(dec, nullptr);
        EXPECT_TRUE(dec->IsOkay());

        const std::size_t bpp = (fmt == PixelFmt::Y8 ? 1 : 3);
        for (std::size_t ix = 0; ix < frames.size(); ix++) {
            SCOPED_TRACE(ix);
            auto img = dec->DecodeFrame(frames[ix]);
            ASSERT_TRUE(img);
            EXPECT_TRUE(dec->IsOkay());
            EXPECT_EQ(img->fmt_, fmt);
            EXPECT_EQ(img->width_, 1024);
            EXPECT_EQ(img->height_, 768);
            ASSERT_EQ(img->data_.size(), 1024 * 768 * bpp);
            // Not a blank (e.g. all black) image
            const auto [min, max] = std::minmax_element(img->data_.begin(), img->data_.end());
            EXPECT_GT(*max - *min, 100);
            imgs[fmt].push_back(std::move(*img));
        }
    }

    // RGB24 (packed RGBRGB...) and GBRP (planar GGG...BBB...RRR...) should be the same pixels
    ASSERT_EQ(imgs[PixelFmt::RGB24].size(), imgs[PixelFmt::GBRP].size());
    for (std::size_t ix = 0; ix < imgs[PixelFmt::RGB24].size(); ix++) {
        SCOPED_TRACE(ix);
        const auto& rgb = imgs[PixelFmt::RGB24][ix].data_;
        const auto& gbr = imgs[PixelFmt::GBRP][ix].data_;
        const std::size_t n_px = rgb.size() / 3;
        std::size_t n_diff = 0;
        for (std::size_t px = 0; px < n_px; px++) {
            const int dr = std::abs((int)rgb[(px * 3) + 0] - (int)gbr[(2 * n_px) + px]);
            const int dg = std::abs((int)rgb[(px * 3) + 1] - (int)gbr[(0 * n_px) + px]);
            const int db = std::abs((int)rgb[(px * 3) + 2] - (int)gbr[(1 * n_px) + px]);
            if ((dr > 2) || (dg > 2) || (db > 2)) {
                n_diff++;
            }
        }
        EXPECT_EQ(n_diff, 0);
    }
}

#endif  // FPSDK_USE_FFMPEG
/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
