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
 * @brief Fixposition SDK: tests for fp::common::logging
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpcommon/logging.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fp::common::logging;

TEST(LoggingTest, LoggingPrint)
{
    DEBUG("This is fpcommon debug...");
    INFO("This is fpcommon info...");
    // WARNING("This is fpcommon warning...");

    DEBUG_S("This"
            << " is"
            << " fpcommon"
            << " debug");
    INFO_S("This"
           << " is"
           << " fpcommon"
           << " debug");
    // WARNING_S("This" << " is" << " fpcommon" << " warning");
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
