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
 * @brief Fixposition SDK: Common types
 *
 * @page FPSDK_COMMON_TYPES Common types
 *
 * **API**: fpsdk_common/types.hpp and fpsdk::common::types
 *
 */
#ifndef __FPSDK_COMMON_TYPES_HPP__
#define __FPSDK_COMMON_TYPES_HPP__

/* LIBC/STL */
#include <cstdint>
#include <type_traits>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Common types
 */
namespace types {
/* ****************************************************************************************************************** */

/**
 * @brief Convert enum class constant to the underlying integral type value
 *
 * @tparam    T         The enum class type
 * @param[in] enum_val  The enum constant
 *
 * @returns the integral value of the enum constant as the value of the underlying type
 */
template <typename T, typename = typename std::enable_if<std::is_enum<T>::value, T>::type>
constexpr typename std::underlying_type<T>::type EnumToVal(T enum_val)
{
    return static_cast<typename std::underlying_type<T>::type>(enum_val);
}

/* ****************************************************************************************************************** */
}  // namespace types
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_TYPES_HPP__
