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
 * @brief Fixposition SDK: tests for fpsdk::common::utils
 */

/* LIBC/STL */
#include <string>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/utils.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::utils;
using namespace fpsdk::common::string;

TEST(UtilsTest, GetVersionString)
{
    const std::string version_string = GetVersionString();
    DEBUG("version_string=[%s]", version_string.c_str());
    EXPECT_GT(version_string.size(), 0);
    EXPECT_LT(version_string.size(), 80);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(UtilsTest, GetCopyrightString)
{
    const std::string copyright_string = GetCopyrightString();
    DEBUG("copyright_string=[%s]", copyright_string.c_str());
    EXPECT_GT(copyright_string.size(), 0);
    EXPECT_LT(copyright_string.size(), 80);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(UtilsTest, GetLicenseString)
{
    const std::string license_string = GetLicenseString();
    DEBUG("license_string=[%s]", license_string.c_str());
    EXPECT_GT(license_string.size(), 0);
    EXPECT_LT(license_string.size(), 80);
}

// ---------------------------------------------------------------------------------------------------------------------

#if 0  // Must also make "private:" stuff "public:" in CircularBuffer for this to work...
#  define HD_CB(label)                                                                                     \
      DEBUG_HEXDUMP(cb.buf_.data(), cb.buf_.size(), "", label " read=%d write=%d full=%s Used=%d Free=%d", \
          (int)cb.read_, (int)cb.write_, ToStr(cb.full_), (int)cb.Used(), (int)cb.Avail())
#else
#  define HD_CB(...) /* nothing */
#endif

TEST(UtilsTest, CircularBuffer)
{
    CircularBuffer cb(16);

    // init, cb: ----------
    HD_CB("empty");
    EXPECT_EQ(cb.Size(), 16);
    EXPECT_TRUE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 0);
    EXPECT_EQ(cb.Avail(), 16);

    // write 3, cb: 333-------------
    const uint8_t w3[] = { '3', '3', '3' };
    EXPECT_TRUE(cb.Write(w3, sizeof(w3)));
    HD_CB("w3");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 3);
    EXPECT_EQ(cb.Avail(), 13);

    // write 7, cb: 3337777777------
    const uint8_t w7[] = { '7', '7', '7', '7', '7', '7', '7' };
    EXPECT_TRUE(cb.Write(w7, sizeof(w7)));
    HD_CB("w7");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 10);
    EXPECT_EQ(cb.Avail(), 6);

    // write NULL or 0, not allowed
    EXPECT_FALSE(cb.Write(NULL, 0));
    EXPECT_FALSE(cb.Write(NULL, 2));
    EXPECT_FALSE(cb.Write(w7, 0));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 10);
    EXPECT_EQ(cb.Avail(), 6);

    // write 7, no space, cb unchanged
    EXPECT_FALSE(cb.Write(w7, sizeof(w7)));
    HD_CB("w7");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 10);
    EXPECT_EQ(cb.Avail(), 6);

    // write 6, cb: 3337777777666666, now full
    const uint8_t w6[] = { '6', '6', '6', '6', '6', '6' };
    EXPECT_TRUE(cb.Write(w6, sizeof(w6)));
    HD_CB("w6");
    EXPECT_FALSE(cb.Empty());
    EXPECT_TRUE(cb.Full());
    EXPECT_EQ(cb.Used(), 16);
    EXPECT_EQ(cb.Avail(), 0);

    // clear, cb: ----------------
    cb.Reset();
    HD_CB("reset");
    EXPECT_EQ(cb.Size(), 16);
    EXPECT_TRUE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 0);
    EXPECT_EQ(cb.Avail(), 16);

    // write 6, cb: 666666----------
    EXPECT_TRUE(cb.Write(w6, sizeof(w6)));
    HD_CB("w6");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 6);
    EXPECT_EQ(cb.Avail(), 10);

    // write 7, cb: 6666667777777---
    EXPECT_TRUE(cb.Write(w7, sizeof(w7)));
    HD_CB("w7");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 13);
    EXPECT_EQ(cb.Avail(), 3);

    // read 14, should fail
    uint8_t r14[14];
    EXPECT_FALSE(cb.Read(r14, sizeof(r14)));
    HD_CB("r14");

    // peek 10, cb: 6666667777777---
    uint8_t p10[10];
    EXPECT_TRUE(cb.Peek(p10, sizeof(p10)));
    HD_CB("p10");
    DEBUG_HEXDUMP(p10, sizeof(p10), "", "p10");
    EXPECT_EQ(std::vector<uint8_t>(p10, p10 + sizeof(p10)),
        std::vector<uint8_t>({ '6', '6', '6', '6', '6', '6', '7', '7', '7', '7' }));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 13);
    EXPECT_EQ(cb.Avail(), 3);

    // read 10, cb: ----------777---
    uint8_t r10[10];
    EXPECT_TRUE(cb.Read(r10, sizeof(r10)));
    HD_CB("r10");
    DEBUG_HEXDUMP(r10, sizeof(r10), "", "r10");
    EXPECT_EQ(std::vector<uint8_t>(r10, r10 + sizeof(r10)),
        std::vector<uint8_t>({ '6', '6', '6', '6', '6', '6', '7', '7', '7', '7' }));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 3);
    EXPECT_EQ(cb.Avail(), 13);

    // read 0 or NULL not allowed
    EXPECT_FALSE(cb.Read(NULL, 0));
    EXPECT_FALSE(cb.Read(NULL, 10));
    EXPECT_FALSE(cb.Read(r10, 0));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 3);
    EXPECT_EQ(cb.Avail(), 13);

    // read 4, not enough data, cb unchanged
    uint8_t r4[4];
    EXPECT_FALSE(cb.Read(r4, sizeof(r4)));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 3);
    EXPECT_EQ(cb.Avail(), 13);

    // read 3, cb: ----------------
    uint8_t r3[3];
    EXPECT_TRUE(cb.Read(r3, sizeof(r3)));
    HD_CB("r3");
    DEBUG_HEXDUMP(r3, sizeof(r3), "", "r3");
    EXPECT_EQ(std::vector<uint8_t>(r3, r3 + sizeof(r3)), std::vector<uint8_t>({ '7', '7', '7' }));
    EXPECT_TRUE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 0);
    EXPECT_EQ(cb.Avail(), 16);

    // write 5, cb: 55-----------555
    const uint8_t w5[] = { '5', '5', '5', '5', '5' };
    EXPECT_TRUE(cb.Write(w5, sizeof(w5)));
    HD_CB("w5");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 5);
    EXPECT_EQ(cb.Avail(), 11);

    // write 8, cb: 5588888888---555
    const uint8_t w8[] = { '8', '8', '8', '8', '8', '8', '8', '8' };
    EXPECT_TRUE(cb.Write(w8, sizeof(w8)));
    HD_CB("w8");
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 13);
    EXPECT_EQ(cb.Avail(), 3);

    // read 15, not enough data, cb unchanged
    uint8_t r15[15];
    EXPECT_FALSE(cb.Read(r15, sizeof(r15)));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 13);
    EXPECT_EQ(cb.Avail(), 3);

    // peek 9, cb: 5588888888---555
    uint8_t p9[9];
    EXPECT_TRUE(cb.Peek(p9, sizeof(p9)));
    HD_CB("p9");
    DEBUG_HEXDUMP(p9, sizeof(p9), "", "p9");
    EXPECT_EQ(std::vector<uint8_t>(p9, p9 + sizeof(p9)),
        std::vector<uint8_t>({ '5', '5', '5', '5', '5', '8', '8', '8', '8' }));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 13);
    EXPECT_EQ(cb.Avail(), 3);

    // read 9, cb: ------8888------
    uint8_t r9[9];
    EXPECT_TRUE(cb.Read(r9, sizeof(r9)));
    HD_CB("r9");
    DEBUG_HEXDUMP(r9, sizeof(r9), "", "r9");
    EXPECT_EQ(std::vector<uint8_t>(r9, r9 + sizeof(r9)),
        std::vector<uint8_t>({ '5', '5', '5', '5', '5', '8', '8', '8', '8' }));
    EXPECT_FALSE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 4);
    EXPECT_EQ(cb.Avail(), 12);

    // read 4, cb: ----------------
    EXPECT_TRUE(cb.Read(r4, sizeof(r4)));
    HD_CB("r4");
    DEBUG_HEXDUMP(r4, sizeof(r4), "", "r4");
    EXPECT_EQ(std::vector<uint8_t>(r4, r4 + sizeof(r4)), std::vector<uint8_t>({ '8', '8', '8', '8' }));
    EXPECT_TRUE(cb.Empty());
    EXPECT_FALSE(cb.Full());
    EXPECT_EQ(cb.Used(), 0);
    EXPECT_EQ(cb.Avail(), 16);
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
