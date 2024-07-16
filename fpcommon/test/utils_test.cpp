/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: tests for fp::common::utils
 */

/* LIBC/STL */
#include <string>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpcommon/logging.hpp>
#include <fpcommon/utils.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fp::common::utils;

TEST(UtilsTest, GetVersionString)
{
    const std::string version_string = GetVersionString();
    EXPECT_GT(version_string.size(), 0);
    EXPECT_LT(version_string.size(), 50);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(UtilsTest, GetCopyrightString)
{
    const std::string copyright_string = GetCopyrightString();
    EXPECT_GT(copyright_string.size(), 0);
    EXPECT_LT(copyright_string.size(), 80);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(UtilsTest, GetLicenseString)
{
    const std::string license_string = GetLicenseString();
    EXPECT_GT(license_string.size(), 0);
    EXPECT_LT(license_string.size(), 80);
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fp::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fp::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
