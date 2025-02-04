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
 * @brief Fixposition SDK: YAML utilities
 */

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/yaml.hpp"

namespace fpsdk {
namespace common {
namespace yaml {
/* ****************************************************************************************************************** */

bool StringToYaml(const std::string& yaml_str, YAML::Node& yaml_node)
{
    bool ok = true;
    try {
        yaml_node = YAML::Load(yaml_str);
    } catch (std::exception& ex) {
        WARNING("StringToYaml: bad YAML: %s", ex.what());
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

std::string YamlToString(const YAML::Node& yaml_node)
{
    YAML::Emitter emitter;
    emitter.SetIndent(4);
    emitter << yaml_node;
    return std::string(emitter.c_str()) + "\n";
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<std::string> GetKeysFromYaml(const YAML::Node& yaml_node, const std::string& key)
{
    std::vector<std::string> keys;
    try {
        for (auto& entry : key.empty() ? yaml_node : yaml_node[key]) {
            keys.push_back(entry.first.as<std::string>().c_str());
        }

    } catch (YAML::Exception& e) {
        WARNING("GetKeysFromYaml: bad yaml: %s", e.what());
    }
    return keys;
}

/* ****************************************************************************************************************** */
}  // namespace yaml
}  // namespace common
}  // namespace fpsdk
