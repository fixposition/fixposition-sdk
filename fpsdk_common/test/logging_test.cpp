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
#include <fpsdk_common/time.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::logging;
using namespace fpsdk::common::time;

TEST(LoggingTest, LoggingPrint)
{
    DEBUG("This is fpsdk_common debug...");
    INFO("This is fpsdk_common info...");
    // WARNING("This is fpsdk_common warning...");
    DEBUG_THR(1234, "This is fpsdk_common debug thr...");

    int n = 5;
    while (n > 0) {
        INFO_THR(10, "This is fpsdk_common info thr... %d", 42);
        if (n == 2) {
            Duration::FromSec(0.05).Sleep();
        }
        n--;
    }

    // WARNING_THR(1234, "This is fpsdk_common warning...");

    DEBUG_S("This"              // thank you, clang-format
            << " is"            // thank you, clang-format
            << " fpsdk_common"  // thank you, clang-format
            << " debug str");
    INFO_S("This"              // thank you, clang-format
           << " is"            // thank you, clang-format
           << " fpsdk_common"  // thank you, clang-format
           << " debug str");
    // WARNING_S("This" << " is" << " fpsdk_common" << " warning");
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    // auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            //      level++;
        }
    }
    // fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
