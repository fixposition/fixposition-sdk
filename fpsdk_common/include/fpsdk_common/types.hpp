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
 * @brief Fixposition SDK: Common types and type helpers
 *
 * @page FPSDK_COMMON_TYPES Common types and type helpers
 *
 * **API**: fpsdk_common/types.hpp and fpsdk::common::types
 *
 */
#ifndef __FPSDK_COMMON_TYPES_HPP__
#define __FPSDK_COMMON_TYPES_HPP__

/* LIBC/STL */
#include <array>
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
 * @brief Get number of elements in array variable
 *
 * @tparam    T    Array type
 * @param[in] arr  The array
 *
 * @returns the number of elements in arr
 *
 * @code{cpp}
 * std::array<int, 5> a5;
 * const std::size_t n6 = NumOf(a6); // = 5, same as a.size(), but works compile-time
 * int a3[3];
 * const std::size_t n3 = NumOf(a3); // = 3, works compile-time
 * @endcode
 */
template <typename T>
constexpr std::size_t NumOf(const T& arr)
{
    return sizeof(arr) / sizeof(arr[0]);
}

/**
 * @brief Get number of elements in array type
 *
 * @tparam  T  Array type
 *
 * @returns the number of elements in the array
 *
 * @code{cpp}
 * using Arr5 = std::array<int, 5>;
 * const std::size_t n6 = NumOf<Arr5>(); // = 5, works compile-time
 * @endcode
 */
template <typename T>
constexpr std::size_t NumOf()
{
    return std::tuple_size<T>{};
}

/**
 * @brief Base class to prevent copy or move
 *
 * Other classes can be derived from this to prevent copying of moving instances.
 */
class NoCopyNoMove  // clang-format off
{
   public:
    NoCopyNoMove() { }
    ~NoCopyNoMove() { }
   protected:
    NoCopyNoMove& operator=(const NoCopyNoMove&) = delete;
    NoCopyNoMove(const NoCopyNoMove&)            = delete;
    NoCopyNoMove(NoCopyNoMove&&)                 = delete;
    NoCopyNoMove& operator=(NoCopyNoMove&&)      = delete;
};  // clang-format on

//! Mark variable as unused to silence compiler warnings                        \hideinitializer
#ifndef UNUSED
#  define UNUSED(thing) (void)thing
#endif

//! Preprocessor stringification                                                \hideinitializer
#ifndef STRINGIFY
#  define STRINGIFY(x) _STRINGIFY(x)
#endif

//! Preprocessor concatenation                                                  \hideinitializer
#ifndef CONCAT
#  define CONCAT(a, b) _CONCAT(a, b)
#endif

//! Size of struct member
#ifndef SIZEOF_FIELD
#  define SIZEOF_FIELD(_type_, _member_) sizeof((((_type_*)NULL)->_member_))
#endif

// Helpers, which we don't want to document in Doxygen, for some of the above
#ifndef _DOXYGEN_
#  define _STRINGIFY(x) #x
#  define _CONCAT(a, b) a##b
#endif

/* ****************************************************************************************************************** */
}  // namespace types
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_TYPES_HPP__
