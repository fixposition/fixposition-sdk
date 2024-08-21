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

/**
 * @brief GNSS fix types
 */
enum class GnssFixType : int8_t {
    // clang-format off
    FIX_UNKNOWN        =  0,  //!< Unknown fix
    FIX_NOFIX          =  1,  //!< No fix
    FIX_DRONLY         =  2,  //!< Dead-reckoning only fix
    FIX_TIME           =  3,  //!< Time only fix
    FIX_2D             =  4,  //!< 2D fix
    FIX_3D             =  5,  //!< 3D fix
    FIX_3D_DR          =  6,  //!< 3D + dead-reckoning fix
    FIX_RTK_FLOAT      =  7,  //!< RTK float fix (implies 3D fix)
    FIX_RTK_FIXED      =  8,  //!< RTK fixed fix (implies 3D fix)
    FIX_RTK_FLOAT_DR   =  9,  //!< RTK float fix + dead-reckoning (implies 3D_DR fix)
    FIX_RTK_FIXED_DR   = 10,  //!< RTK fixed fix + dead-reckoning (implies 3D_DR fix)
    // clang-format on
};

/**
 * @brief Stringify GNSS fix type
 *
 * @param[in]  fix_type  The fix type
 *
 * @returns a concise and unique string for the fix types, "?" for bad values
 */
const char* GnssFixTypeStr(const GnssFixType fix_type);

/* ****************************************************************************************************************** */
}  // namespace types
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_TYPES_HPP__
