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
 * @brief Fixposition SDK: Math utilities
 *
 * @page FPSDK_COMMON_MATH Math utilities
 *
 * **API**: fpsdk_common/math.hpp and fpsdk::common::math
 *
 */
#ifndef __FPSDK_COMMON_MATH_HPP__
#define __FPSDK_COMMON_MATH_HPP__

/* LIBC/STL */
#include <algorithm>
#include <cmath>
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Math utilities
 */
namespace math {
/* ****************************************************************************************************************** */

/**
 * @brief Convert degrees to radians
 *
 * @tparam  T  value type
 * @param[in] degrees  Angle in degrees
 *
 * @returns the angle in radians
 */
template <typename T>
constexpr inline T DegToRad(T degrees)
{
    static_assert(::std::is_floating_point<T>::value, "Value must be float or double");
    return degrees * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees
 *
 * @tparam  T  value type
 * @param[in] radians  Angle in radians
 * @returns the angle in radians
 */
template <typename T>
constexpr inline T RadToDeg(T radians)
{
    static_assert(::std::is_floating_point<T>::value, "Value must be float or double");
    return radians * 180.0 / M_PI;
}

/**
 * @brief Integer degrees and minutes with fractional seconds
 */
struct DegMinSec
{
    DegMinSec() = default;  //!< Default ctor

    /**
     * @brief Set from fractional degrees
     *
     * @param[in]  deg  Fractional degrees
     */
    DegMinSec(const double deg);
    int deg_ = 0;       //!< Integer degrees
    int min_ = 0;       //!< Integer minutes
    double sec_ = 0.0;  //!< Fractional seconds
};

/**
 * @brief Round to desired number of fractional digits (of precision)
 *
 * @param[in]  value   The value
 * @param[in]  digits  Number of digits (0-12), param clamped to range
 *
 * @returns the value rounded to the given number of fractional digits, or the original value if it is not finite
 */
double RoundToFracDigits(const double value, const int digits);

/**
 * @brief Clip to desired number of fractional digits (of precision)
 *
 * @param[in]  value   The value
 * @param[in]  digits  Number of digits (0-12), param clamped to range
 *
 * @returns the value clipped to the given number of fractional digits, or the original value if it is not finite
 */
double ClipToFracDigits(const double value, const int digits);

// ---------------------------------------------------------------------------------------------------------------------
/**
 * @name Bit manipulation functions
 *
 * Examples:
 *
 * @code{cpp}
 * uint8_t mask = 0;
 * SetBits(mask, Bit(0) | Bit(1) | Bit(7));  // mask is now 0x83
 * const bool bit_7_is_set = CheckBitsAll(mask, Bit(7));  // true
 * @endcode
 *
 * @{
 */
/**
 * @brief Return a number with the given bit set to 1 (i.e. 2^bit)
 *
 * @tparam     T    Numerical type
 * @param[in]  bit  bit to be set to 1 (0-63, depending on T, 0 = LSB)
 *
 * @returns the mask (value) with the desired bit set
 */
template <typename T>
constexpr T Bit(const std::size_t bit)
{
    return static_cast<T>(static_cast<uint64_t>(1) << bit);
}

/**
 * @brief Checks if all bits are set
 *
 * @tparam      T     Numerical type
 * @param[in]   mask  Mask (value) that should be checked
 * @param[in]   bits  Bit(s) to be checked
 *
 * @returns true if all bit(s) is (are) set, false otherwise
 */
template <typename T>
constexpr bool CheckBitsAll(const T mask, const T bits)
{
    return (mask & bits) == bits;
}

/**
 * @brief Checks if any bits are set
 *
 * @tparam     T     Numerical type
 * @param[in]  mask  Mask (value) to be checked
 * @param[in]  bits  Bit(s) to be checked
 *
 * @returns true if any bit(s) is (are) set, false otherwise
 */
template <typename T>
constexpr bool CheckBitsAny(const T mask, const T bits)
{
    return (mask & bits) != 0;
}

/**
 * @brief Extracts bits
 *
 * @tparam     T      Numerical type
 * @param[in]  value  The bitfield value
 * @param[in]  mask   Mask of bits that should be extracted
 *
 * @returns the extracted bits
 */
template <typename T>
constexpr T GetBits(const T value, const T mask)
{
    return (value & mask);
}

/**
 * @brief Sets the bits
 *
 * @tparam         T     Numerical type
 * @param[in,out]  mask  Mask (value) to be modified
 * @param[in]      bits  Bit(s) to be set
 */
template <typename T>
inline void SetBits(T& mask, const T bits)
{
    mask |= bits;
}

/**
 * @brief Clears the bits
 *
 * @tparam         T     Numerical type
 * @param[in,out]  mask  Mask (value) to be modified
 * @param[in]      bits  Bit(s) to be cleared
 */
template <typename T>
inline void ClearBits(T& mask, const T bits)
{
    mask &= ~bits;
}

/**
 * @brief Toggles the bits
 *
 * @tparam         T     Numerical type
 * @param[in,out]  mask  Mask (value) to be modified
 * @param[in]      bits  Bit(s) to be toggled
 */
template <typename T>
inline void ToggleBits(T& mask, const T bits)
{
    mask ^= bits;
}

///@}
// ---------------------------------------------------------------------------------------------------------------------

/* ****************************************************************************************************************** */
}  // namespace math
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_MATH_HPP__
