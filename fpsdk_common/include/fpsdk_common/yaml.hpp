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
 *
 * @page FPSDK_COMMON_YAML YAML utilities
 *
 * **API**: fpsdk_common/yaml.hpp and fpsdk::common::yaml
 *
 */
#ifndef __FPSDK_COMMON_YAML_HPP__
#define __FPSDK_COMMON_YAML_HPP__

/* LIBC/STL */
#include <string>

/* EXTERNAL */
#include <yaml-cpp/yaml.h>

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief YAML utilities
 */
namespace yaml {
/* ****************************************************************************************************************** */

/**
 * @brief Parse YAML string
 *
 * @param[in]   yaml_str   The YAML string
 * @param[out]  yaml_node  The parsed YAML node
 *
 * @returns true if string was successfully parsed into a YAML node, false otherwise
 */
bool StringToYaml(const std::string& yaml_str, YAML::Node& yaml_node);

/**
 * @brief Stringify YAML node
 *
 * @param[in]  yaml_node  The YAML node to stringfy
 *
 * @returns a, possibly empty, stringification of the YAML node
 */
std::string YamlToString(const YAML::Node& yaml_node);

/**
 * @brief Get keys in YAML
 *
 * For example, for the yaml node:
 *     key:
 *        foo: 1
 *        bar:
 *           - 1
 *           - 2
 *        baz:
 *             bla: 42
 *
 *  GetKeysFromYaml(node, "key") would return `[ "bar", "baz", "foo" ]`.
 *
 * @param[in]  yaml_node  The YAML node
 * @param[in]  key        The entry in the node to use, can be empty to use the node itself
 *
 * @returns an alphabetically ordered list of all keys found in the parameter space, resp. an empty list if the param is
 *          an empty map or not a map
 */
std::vector<std::string> GetKeysFromYaml(const YAML::Node& yaml_node, const std::string& key = "");

/* ****************************************************************************************************************** */
}  // namespace yaml
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_YAML_HPP__
