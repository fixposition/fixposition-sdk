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

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/utils.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::utils;

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
