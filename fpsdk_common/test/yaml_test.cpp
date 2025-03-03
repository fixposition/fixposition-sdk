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
 * @brief Fixposition SDK: tests for fpsdk::common::yaml
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/yaml.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::yaml;

static const char* yaml_str_1 = R"yaml(
.red:   &red   "#ff0000"
.green: &green "#00ff00"
.blue:  &blue  "#0000ff"
.one-two-three: &one-two-three
    - 1
    - &two 2
    - 3
.params: &params
    foo: 0
    bar: &bar "nope"
apple: *green
cherry: *red
ocean: *blue
a_list: *one-two-three
a_number: *two
a_string: *bar
)yaml";

TEST(YamlTest, StringToYaml)
{
    YAML::Node node;
    EXPECT_TRUE(StringToYaml(yaml_str_1, node));
    EXPECT_EQ(node["apple"].as<std::string>(), "#00ff00");
    EXPECT_EQ(node["cherry"].as<std::string>(), "#ff0000");
    EXPECT_EQ(node["ocean"].as<std::string>(), "#0000ff");
    EXPECT_EQ(node["a_list"][0].as<int>(), 1);
    EXPECT_EQ(node["a_list"][1].as<int>(), 2);
    EXPECT_EQ(node["a_list"][2].as<int>(), 3);
    EXPECT_EQ(node["a_number"].as<int>(), 2);
    EXPECT_EQ(node["a_string"].as<std::string>(), "nope");
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
