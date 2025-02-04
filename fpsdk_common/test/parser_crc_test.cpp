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
 * @brief Fixposition SDK: tests for fpsdk::common::parser::crc
 */

/* LIBC/STL */
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/crc.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::parser::crc;

static const uint8_t RANDOM_1_BYTE[] = { 0xb1 };
static const uint8_t RANDOM_2_BYTES[] = { 0x32, 0x43 };
static const uint8_t RANDOM_3_BYTES[] = { 0x13, 0xd3, 0xe8 };
static const uint8_t RANDOM_4_BYTES[] = { 0xc4, 0xd2, 0xee, 0xac };
static const uint8_t RANDOM_5_BYTES[] = { 0x6d, 0xb3, 0x5b, 0xc6, 0xe1 };
static const uint8_t RANDOM_6_BYTES[] = { 0x85, 0xb5, 0xaf, 0xf8, 0x00, 0x89 };
static const uint8_t RANDOM_7_BYTES[] = { 0x1b, 0xa7, 0x24, 0xc3, 0x3a, 0xb7, 0x77 };
static const uint8_t RANDOM_8_BYTES[] = { 0x25, 0x49, 0xfe, 0x66, 0x14, 0xb1, 0x86, 0x4d };
static const uint8_t RANDOM_9_BYTES[] = { 0x36, 0x6b, 0x6b, 0x4a, 0xa7, 0xfb, 0xdd, 0xa9, 0xc2 };
static const uint8_t RANDOM_10_BYTES[] = { 0x62, 0x61, 0x73, 0xb8, 0xa8, 0xc4, 0x89, 0xda, 0xc9, 0xcc };
static const uint8_t RANDOM_11_BYTES[] = { 0x7f, 0xe8, 0x33, 0x74, 0xd4, 0xab, 0x8a, 0xb0, 0xf7, 0xd8, 0x8d };
static const uint8_t RANDOM_12_BYTES[] = { 0x1a, 0x48, 0x8a, 0xe1, 0xa9, 0xf7, 0x15, 0xce, 0x62, 0xf3, 0xd5, 0x21 };
static const uint8_t RANDOM_13_BYTES[] = { 0xb4, 0xa9, 0x71, 0x80, 0x60, 0x7f, 0xec, 0xa1, 0x87, 0x94, 0xe0, 0x77,
    0x6a };
static const uint8_t RANDOM_14_BYTES[] = { 0xca, 0x5b, 0x01, 0xb4, 0x21, 0xb9, 0xd8, 0xea, 0xa7, 0x4b, 0xb0, 0xfa, 0xaf,
    0xc6 };
static const uint8_t RANDOM_15_BYTES[] = { 0x4c, 0x7b, 0xe8, 0x14, 0x68, 0x23, 0x85, 0x4f, 0xf6, 0x10, 0x27, 0x6e, 0x6c,
    0x6d, 0x85 };
static const uint8_t RANDOM_16_BYTES[] = { 0x4c, 0x7b, 0xe8, 0x14, 0x68, 0x23, 0x85, 0x4f, 0xf6, 0x10, 0x27, 0x6e, 0x6c,
    0x6d, 0x85, 0x66 };
static const uint8_t RANDOM_29_BYTES[] = { 0x63, 0xe1, 0x7a, 0x23, 0x21, 0x5e, 0x0d, 0x54, 0x90, 0x77, 0xfe, 0xda, 0x12,
    0x20, 0x63, 0x03, 0x93, 0xe8, 0x1f, 0x65, 0x2c, 0x07, 0x4b, 0x9a, 0x8d, 0x10, 0x46, 0xc6, 0x61 };
static const uint8_t RANDOM_30_BYTES[] = { 0xd6, 0xda, 0x6b, 0xf8, 0x53, 0xc1, 0x35, 0xfa, 0xbe, 0x9e, 0x30, 0xc9, 0x48,
    0x57, 0x25, 0xde, 0x88, 0x74, 0x9e, 0x97, 0x3d, 0x42, 0x0e, 0xfa, 0xf1, 0x9e, 0x0d, 0xa8, 0x65, 0x5d };
static const uint8_t RANDOM_31_BYTES[] = { 0xe2, 0xe9, 0xea, 0x65, 0x49, 0xd5, 0xee, 0x9d, 0xba, 0x63, 0x6a, 0xbb, 0x17,
    0xb8, 0x5b, 0x90, 0xb8, 0xcc, 0xb7, 0x44, 0x71, 0x09, 0xa7, 0xc4, 0x8c, 0xf5, 0x69, 0xbe, 0x64, 0x68, 0xdf };
static const uint8_t RANDOM_32_BYTES[] = { 0xe4, 0x97, 0x5f, 0x0d, 0x23, 0x44, 0xb3, 0xf0, 0x14, 0x07, 0x17, 0xe6, 0x16,
    0x2e, 0xc6, 0xfd, 0x40, 0xff, 0x76, 0x47, 0xd7, 0xaa, 0xdf, 0xb8, 0x7d, 0x60, 0xe8, 0x75, 0x50, 0x49, 0x8f, 0xcf };
static const uint8_t RANDOM_64_BYTES[] = { 0xfc, 0xf3, 0xbb, 0x2b, 0x6f, 0xf0, 0x77, 0xc9, 0x03, 0x5f, 0x43, 0xb3, 0xc6,
    0xb3, 0x51, 0xbf, 0xbb, 0x44, 0x3d, 0x11, 0x58, 0xf1, 0x47, 0x1d, 0xcc, 0xa8, 0x3f, 0x87, 0x33, 0x50, 0x57, 0xe0,
    0xf8, 0xa0, 0x4d, 0x88, 0x82, 0x7a, 0xd6, 0x5c, 0x56, 0x72, 0x28, 0xa0, 0x82, 0x7f, 0xba, 0xfa, 0xaf, 0x2a, 0x61,
    0xed, 0xfa, 0x60, 0xa4, 0xa6, 0x27, 0x07, 0xe9, 0x5b, 0x32, 0xd1, 0xb7, 0x9a };

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Data)
{
    EXPECT_EQ(sizeof(RANDOM_1_BYTE), (std::size_t)1);
    EXPECT_EQ(sizeof(RANDOM_2_BYTES), (std::size_t)2);
    EXPECT_EQ(sizeof(RANDOM_3_BYTES), (std::size_t)3);
    EXPECT_EQ(sizeof(RANDOM_4_BYTES), (std::size_t)4);
    EXPECT_EQ(sizeof(RANDOM_5_BYTES), (std::size_t)5);
    EXPECT_EQ(sizeof(RANDOM_6_BYTES), (std::size_t)6);
    EXPECT_EQ(sizeof(RANDOM_7_BYTES), (std::size_t)7);
    EXPECT_EQ(sizeof(RANDOM_8_BYTES), (std::size_t)8);
    EXPECT_EQ(sizeof(RANDOM_9_BYTES), (std::size_t)9);
    EXPECT_EQ(sizeof(RANDOM_10_BYTES), (std::size_t)10);
    EXPECT_EQ(sizeof(RANDOM_11_BYTES), (std::size_t)11);
    EXPECT_EQ(sizeof(RANDOM_12_BYTES), (std::size_t)12);
    EXPECT_EQ(sizeof(RANDOM_13_BYTES), (std::size_t)13);
    EXPECT_EQ(sizeof(RANDOM_14_BYTES), (std::size_t)14);
    EXPECT_EQ(sizeof(RANDOM_15_BYTES), (std::size_t)15);
    EXPECT_EQ(sizeof(RANDOM_16_BYTES), (std::size_t)16);
    EXPECT_EQ(sizeof(RANDOM_29_BYTES), (std::size_t)29);
    EXPECT_EQ(sizeof(RANDOM_30_BYTES), (std::size_t)30);
    EXPECT_EQ(sizeof(RANDOM_31_BYTES), (std::size_t)31);
    EXPECT_EQ(sizeof(RANDOM_32_BYTES), (std::size_t)32);
    EXPECT_EQ(sizeof(RANDOM_64_BYTES), (std::size_t)64);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc32fpb)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc32fpb(NULL, 0), (uint32_t)0x00000000);
    EXPECT_EQ(Crc32fpb(NULL, 1), (uint32_t)0x00000000);
    EXPECT_EQ(Crc32fpb(RANDOM_64_BYTES, 0), (uint32_t)0x00000000);
    // Actual data
    EXPECT_EQ(Crc32fpb(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint32_t)0x3243baa7);
    EXPECT_EQ(Crc32fpb(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint32_t)0xc8e898ff);
    EXPECT_EQ(Crc32fpb(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint32_t)0x9301f69e);
    EXPECT_EQ(Crc32fpb(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint32_t)0x7b42e10b);
    EXPECT_EQ(Crc32fpb(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint32_t)0x708c17e9);
    EXPECT_EQ(Crc32fpb(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint32_t)0xaa9195ba);
    EXPECT_EQ(Crc32fpb(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint32_t)0x7de7be0d);
    EXPECT_EQ(Crc32fpb(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint32_t)0xd91fc4f1);
    EXPECT_EQ(Crc32fpb(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint32_t)0xd4b803fc);
    EXPECT_EQ(Crc32fpb(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint32_t)0xbb8e4f8d);
    EXPECT_EQ(Crc32fpb(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint32_t)0x19082804);
    EXPECT_EQ(Crc32fpb(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint32_t)0xe6e82b53);
    EXPECT_EQ(Crc32fpb(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint32_t)0x80f65532);
    EXPECT_EQ(Crc32fpb(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint32_t)0xe182e0d6);
    EXPECT_EQ(Crc32fpb(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint32_t)0x048fe6a3);
    EXPECT_EQ(Crc32fpb(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint32_t)0xc327e431);
    EXPECT_EQ(Crc32fpb(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint32_t)0xbc0293a0);
    EXPECT_EQ(Crc32fpb(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint32_t)0x32b413b3);
    EXPECT_EQ(Crc32fpb(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint32_t)0x8e1c0611);
    EXPECT_EQ(Crc32fpb(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint32_t)0xd5307d0d);
    EXPECT_EQ(Crc32fpb(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint32_t)0xabbbb8f9);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc24rtcm3)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc24rtcm3(NULL, 0), (uint32_t)0x000000);
    EXPECT_EQ(Crc24rtcm3(NULL, 1), (uint32_t)0x000000);
    EXPECT_EQ(Crc24rtcm3(RANDOM_64_BYTES, 0), (uint32_t)0x000000);
    // Actual data
    EXPECT_EQ(Crc24rtcm3(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint32_t)0x007c943f);
    EXPECT_EQ(Crc24rtcm3(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint32_t)0x004a6d00);
    EXPECT_EQ(Crc24rtcm3(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint32_t)0x000c5b04);
    EXPECT_EQ(Crc24rtcm3(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint32_t)0x0082465c);
    EXPECT_EQ(Crc24rtcm3(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint32_t)0x007b800f);
    EXPECT_EQ(Crc24rtcm3(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint32_t)0x00993123);
    EXPECT_EQ(Crc24rtcm3(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint32_t)0x001056bb);
    EXPECT_EQ(Crc24rtcm3(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint32_t)0x00bfdb4a);
    EXPECT_EQ(Crc24rtcm3(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint32_t)0x005f3ea2);
    EXPECT_EQ(Crc24rtcm3(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint32_t)0x00ec5fbd);
    EXPECT_EQ(Crc24rtcm3(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint32_t)0x00e4e1e0);
    EXPECT_EQ(Crc24rtcm3(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint32_t)0x001e46af);
    EXPECT_EQ(Crc24rtcm3(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint32_t)0x00f68a9b);
    EXPECT_EQ(Crc24rtcm3(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint32_t)0x00dd2d74);
    EXPECT_EQ(Crc24rtcm3(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint32_t)0x001df608);
    EXPECT_EQ(Crc24rtcm3(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint32_t)0x008b2c7d);
    EXPECT_EQ(Crc24rtcm3(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint32_t)0x00ff1037);
    EXPECT_EQ(Crc24rtcm3(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint32_t)0x00a5b132);
    EXPECT_EQ(Crc24rtcm3(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint32_t)0x003e9621);
    EXPECT_EQ(Crc24rtcm3(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint32_t)0x00ef5580);
    EXPECT_EQ(Crc24rtcm3(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint32_t)0x005d4034);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc32novb)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc32novb(NULL, 0), (uint32_t)0x00000000);
    EXPECT_EQ(Crc32novb(NULL, 1), (uint32_t)0x00000000);
    EXPECT_EQ(Crc32novb(RANDOM_64_BYTES, 0), (uint32_t)0x00000000);
    // Actual data
    EXPECT_EQ(Crc32novb(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint32_t)0xbc66831a);
    EXPECT_EQ(Crc32novb(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint32_t)0x02a5445b);
    EXPECT_EQ(Crc32novb(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint32_t)0x1aa4e1bf);
    EXPECT_EQ(Crc32novb(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint32_t)0x68bfa9ae);
    EXPECT_EQ(Crc32novb(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint32_t)0x442131ff);
    EXPECT_EQ(Crc32novb(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint32_t)0xace112b0);
    EXPECT_EQ(Crc32novb(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint32_t)0x7d609de8);
    EXPECT_EQ(Crc32novb(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint32_t)0x27bcf013);
    EXPECT_EQ(Crc32novb(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint32_t)0xeb46e4df);
    EXPECT_EQ(Crc32novb(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint32_t)0xde30a04d);
    EXPECT_EQ(Crc32novb(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint32_t)0xd6597dc0);
    EXPECT_EQ(Crc32novb(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint32_t)0xd1a01f30);
    EXPECT_EQ(Crc32novb(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint32_t)0x31100903);
    EXPECT_EQ(Crc32novb(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint32_t)0x69bab6dd);
    EXPECT_EQ(Crc32novb(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint32_t)0xe3f3d424);
    EXPECT_EQ(Crc32novb(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint32_t)0x9831d368);
    EXPECT_EQ(Crc32novb(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint32_t)0x6cf9627d);
    EXPECT_EQ(Crc32novb(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint32_t)0xaf485c94);
    EXPECT_EQ(Crc32novb(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint32_t)0x87fd4627);
    EXPECT_EQ(Crc32novb(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint32_t)0x063804b7);
    EXPECT_EQ(Crc32novb(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint32_t)0xe898a950);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc4spartn)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc4spartn(NULL, 0), (uint8_t)0x0);
    EXPECT_EQ(Crc4spartn(NULL, 1), (uint8_t)0x0);
    EXPECT_EQ(Crc4spartn(RANDOM_64_BYTES, 0), (uint8_t)0x0);
    // Actual data
    EXPECT_EQ(Crc4spartn(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint8_t)0x3);
    EXPECT_EQ(Crc4spartn(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint8_t)0x9);
    EXPECT_EQ(Crc4spartn(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint8_t)0x6);
    EXPECT_EQ(Crc4spartn(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint8_t)0xf);
    EXPECT_EQ(Crc4spartn(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint8_t)0x3);
    EXPECT_EQ(Crc4spartn(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint8_t)0x7);
    EXPECT_EQ(Crc4spartn(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint8_t)0x7);
    EXPECT_EQ(Crc4spartn(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint8_t)0x1);
    EXPECT_EQ(Crc4spartn(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint8_t)0x1);
    EXPECT_EQ(Crc4spartn(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint8_t)0x9);
    EXPECT_EQ(Crc4spartn(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint8_t)0x6);
    EXPECT_EQ(Crc4spartn(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint8_t)0x7);
    EXPECT_EQ(Crc4spartn(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint8_t)0xc);
    EXPECT_EQ(Crc4spartn(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint8_t)0x6);
    EXPECT_EQ(Crc4spartn(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint8_t)0x5);
    EXPECT_EQ(Crc4spartn(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint8_t)0xc);
    EXPECT_EQ(Crc4spartn(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint8_t)0x4);
    EXPECT_EQ(Crc4spartn(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint8_t)0xb);
    EXPECT_EQ(Crc4spartn(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint8_t)0x2);
    EXPECT_EQ(Crc4spartn(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint8_t)0xe);
    EXPECT_EQ(Crc4spartn(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint8_t)0xb);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc8spartn)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc8spartn(NULL, 0), (uint8_t)0x00);
    EXPECT_EQ(Crc8spartn(NULL, 1), (uint8_t)0x00);
    EXPECT_EQ(Crc8spartn(RANDOM_64_BYTES, 0), (uint8_t)0x00);
    // Actual data
    EXPECT_EQ(Crc8spartn(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint8_t)0x1e);
    EXPECT_EQ(Crc8spartn(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint8_t)0x1d);
    EXPECT_EQ(Crc8spartn(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint8_t)0x0c);
    EXPECT_EQ(Crc8spartn(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint8_t)0xd3);
    EXPECT_EQ(Crc8spartn(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint8_t)0x08);
    EXPECT_EQ(Crc8spartn(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint8_t)0xeb);
    EXPECT_EQ(Crc8spartn(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint8_t)0x2c);
    EXPECT_EQ(Crc8spartn(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint8_t)0xb2);
    EXPECT_EQ(Crc8spartn(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint8_t)0x8d);
    EXPECT_EQ(Crc8spartn(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint8_t)0x34);
    EXPECT_EQ(Crc8spartn(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint8_t)0x9e);
    EXPECT_EQ(Crc8spartn(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint8_t)0xab);
    EXPECT_EQ(Crc8spartn(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint8_t)0xcc);
    EXPECT_EQ(Crc8spartn(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint8_t)0xe2);
    EXPECT_EQ(Crc8spartn(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint8_t)0x20);
    EXPECT_EQ(Crc8spartn(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint8_t)0xd5);
    EXPECT_EQ(Crc8spartn(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint8_t)0xaa);
    EXPECT_EQ(Crc8spartn(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint8_t)0xb5);
    EXPECT_EQ(Crc8spartn(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint8_t)0x13);
    EXPECT_EQ(Crc8spartn(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint8_t)0x71);
    EXPECT_EQ(Crc8spartn(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint8_t)0xe6);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc16spartn)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc16spartn(NULL, 0), (uint16_t)0x0000);
    EXPECT_EQ(Crc16spartn(NULL, 1), (uint16_t)0x0000);
    EXPECT_EQ(Crc16spartn(RANDOM_64_BYTES, 0), (uint16_t)0x0000);
    // Actual data
    EXPECT_EQ(Crc16spartn(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint16_t)0xb7fa);
    EXPECT_EQ(Crc16spartn(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint16_t)0x1b50);
    EXPECT_EQ(Crc16spartn(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint16_t)0x2661);
    EXPECT_EQ(Crc16spartn(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint16_t)0x357a);
    EXPECT_EQ(Crc16spartn(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint16_t)0xad69);
    EXPECT_EQ(Crc16spartn(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint16_t)0xee82);
    EXPECT_EQ(Crc16spartn(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint16_t)0x7779);
    EXPECT_EQ(Crc16spartn(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint16_t)0x307e);
    EXPECT_EQ(Crc16spartn(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint16_t)0x5475);
    EXPECT_EQ(Crc16spartn(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint16_t)0xf1f9);
    EXPECT_EQ(Crc16spartn(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint16_t)0x3e0a);
    EXPECT_EQ(Crc16spartn(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint16_t)0x8c88);
    EXPECT_EQ(Crc16spartn(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint16_t)0x1a70);
    EXPECT_EQ(Crc16spartn(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint16_t)0x64ec);
    EXPECT_EQ(Crc16spartn(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint16_t)0x0278);
    EXPECT_EQ(Crc16spartn(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint16_t)0x5422);
    EXPECT_EQ(Crc16spartn(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint16_t)0x30d4);
    EXPECT_EQ(Crc16spartn(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint16_t)0x349c);
    EXPECT_EQ(Crc16spartn(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint16_t)0xb846);
    EXPECT_EQ(Crc16spartn(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint16_t)0xa5e1);
    EXPECT_EQ(Crc16spartn(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint16_t)0xc5f4);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, Crc32spartn)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(Crc32spartn(NULL, 0), (uint32_t)0x00000000);
    EXPECT_EQ(Crc32spartn(NULL, 1), (uint32_t)0x00000000);
    EXPECT_EQ(Crc32spartn(RANDOM_64_BYTES, 0), (uint32_t)0x00000000);
    // Actual data
    EXPECT_EQ(Crc32spartn(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint32_t)0x0808d082);
    EXPECT_EQ(Crc32spartn(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint32_t)0x568dfe16);
    EXPECT_EQ(Crc32spartn(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint32_t)0x15a904d3);
    EXPECT_EQ(Crc32spartn(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint32_t)0xff69839b);
    EXPECT_EQ(Crc32spartn(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint32_t)0xce450906);
    EXPECT_EQ(Crc32spartn(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint32_t)0xa399ff53);
    EXPECT_EQ(Crc32spartn(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint32_t)0xd2a22d5b);
    EXPECT_EQ(Crc32spartn(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint32_t)0xedd76950);
    EXPECT_EQ(Crc32spartn(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint32_t)0x47184985);
    EXPECT_EQ(Crc32spartn(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint32_t)0x3fefe99a);
    EXPECT_EQ(Crc32spartn(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint32_t)0x76678f10);
    EXPECT_EQ(Crc32spartn(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint32_t)0xe861a39d);
    EXPECT_EQ(Crc32spartn(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint32_t)0x587d9e0f);
    EXPECT_EQ(Crc32spartn(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint32_t)0xa7510e5d);
    EXPECT_EQ(Crc32spartn(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint32_t)0xd296dbc2);
    EXPECT_EQ(Crc32spartn(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint32_t)0x891679e9);
    EXPECT_EQ(Crc32spartn(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint32_t)0x21a40a93);
    EXPECT_EQ(Crc32spartn(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint32_t)0xf16fb9b3);
    EXPECT_EQ(Crc32spartn(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint32_t)0xab12b0b9);
    EXPECT_EQ(Crc32spartn(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint32_t)0x7ffb3664);
    EXPECT_EQ(Crc32spartn(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint32_t)0xf2f3867a);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ParserCrcTest, ChecksumUbx)
{
    // Bad input should lead to 0 CRC
    EXPECT_EQ(ChecksumUbx(NULL, 0), (uint16_t)0x0000);
    EXPECT_EQ(ChecksumUbx(NULL, 1), (uint16_t)0x0000);
    EXPECT_EQ(ChecksumUbx(RANDOM_64_BYTES, 0), (uint16_t)0x0000);
    // Actual data
    EXPECT_EQ(ChecksumUbx(RANDOM_1_BYTE, sizeof(RANDOM_1_BYTE)), (uint16_t)0xb1b1);
    EXPECT_EQ(ChecksumUbx(RANDOM_2_BYTES, sizeof(RANDOM_2_BYTES)), (uint16_t)0xa775);
    EXPECT_EQ(ChecksumUbx(RANDOM_3_BYTES, sizeof(RANDOM_3_BYTES)), (uint16_t)0xc7ce);
    EXPECT_EQ(ChecksumUbx(RANDOM_4_BYTES, sizeof(RANDOM_4_BYTES)), (uint16_t)0x0e30);
    EXPECT_EQ(ChecksumUbx(RANDOM_5_BYTES, sizeof(RANDOM_5_BYTES)), (uint16_t)0x6b22);
    EXPECT_EQ(ChecksumUbx(RANDOM_6_BYTES, sizeof(RANDOM_6_BYTES)), (uint16_t)0xd46a);
    EXPECT_EQ(ChecksumUbx(RANDOM_7_BYTES, sizeof(RANDOM_7_BYTES)), (uint16_t)0xfa11);
    EXPECT_EQ(ChecksumUbx(RANDOM_8_BYTES, sizeof(RANDOM_8_BYTES)), (uint16_t)0xd56a);
    EXPECT_EQ(ChecksumUbx(RANDOM_9_BYTES, sizeof(RANDOM_9_BYTES)), (uint16_t)0xc140);
    EXPECT_EQ(ChecksumUbx(RANDOM_10_BYTES, sizeof(RANDOM_10_BYTES)), (uint16_t)0xb152);
    EXPECT_EQ(ChecksumUbx(RANDOM_11_BYTES, sizeof(RANDOM_11_BYTES)), (uint16_t)0x5223);
    EXPECT_EQ(ChecksumUbx(RANDOM_12_BYTES, sizeof(RANDOM_12_BYTES)), (uint16_t)0x569b);
    EXPECT_EQ(ChecksumUbx(RANDOM_13_BYTES, sizeof(RANDOM_13_BYTES)), (uint16_t)0x6896);
    EXPECT_EQ(ChecksumUbx(RANDOM_14_BYTES, sizeof(RANDOM_14_BYTES)), (uint16_t)0x9787);
    EXPECT_EQ(ChecksumUbx(RANDOM_15_BYTES, sizeof(RANDOM_15_BYTES)), (uint16_t)0x191b);
    EXPECT_EQ(ChecksumUbx(RANDOM_16_BYTES, sizeof(RANDOM_16_BYTES)), (uint16_t)0x9a81);
    EXPECT_EQ(ChecksumUbx(RANDOM_29_BYTES, sizeof(RANDOM_29_BYTES)), (uint16_t)0xe959);
    EXPECT_EQ(ChecksumUbx(RANDOM_30_BYTES, sizeof(RANDOM_30_BYTES)), (uint16_t)0xb70b);
    EXPECT_EQ(ChecksumUbx(RANDOM_31_BYTES, sizeof(RANDOM_31_BYTES)), (uint16_t)0xcc76);
    EXPECT_EQ(ChecksumUbx(RANDOM_32_BYTES, sizeof(RANDOM_32_BYTES)), (uint16_t)0x0f55);
    EXPECT_EQ(ChecksumUbx(RANDOM_64_BYTES, sizeof(RANDOM_64_BYTES)), (uint16_t)0xa2b4);
}

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
