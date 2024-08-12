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
 * @brief Fixposition SDK: tests for fpsdk::common::logging
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::logging;

TEST(LoggingTest, LoggingPrint)
{
    DEBUG("This is fpsdk_common debug...");
    INFO("This is fpsdk_common info...");
    // WARNING("This is fpsdk_common warning...");

    DEBUG_S("This"
            << " is"
            << " fpsdk_common"
            << " debug");
    INFO_S("This"
           << " is"
           << " fpsdk_common"
           << " debug");
    // WARNING_S("This" << " is" << " fpsdk_common" << " warning");
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
