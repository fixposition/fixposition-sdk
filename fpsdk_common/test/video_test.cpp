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

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/path.hpp>
#include <fpsdk_common/video.hpp>

namespace {
/* ****************************************************************************************************************** */
#if FPSDK_USE_FFMPEG

#  include "data/hevc_i_frame.cpp"

using namespace fpsdk::common::video;
using namespace fpsdk::common::path;

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
