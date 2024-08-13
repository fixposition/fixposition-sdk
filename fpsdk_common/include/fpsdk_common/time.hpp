/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 *  Parts copyright (c) 2008, Willow Garage, Inc., see time.cpp for details
 *  Parts copyright (c) 2007-2020, T. Takasu, see time.cpp for details
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Time utilities
 *
 * @page FPSDK_COMMON_TIME Time utilities
 *
 * API: fpsdk::common::time
 *
 */
#ifndef __FPSDK_COMMON_TIME_HPP__
#define __FPSDK_COMMON_TIME_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Time utilities
 */
namespace time {
/* ****************************************************************************************************************** */

/**
 * @brief Get ticks [ms], monotonic time
 *
 * @returns the number of ticks
 */
uint64_t GetTicks();

/**
 * @brief Get seconds [s], monotonic time
 *
 * @returns the number of seconds
 */
double GetSecs();

/**
 * @brief Sleep for a bit
 *
 * @note The effective sleep duration is approximate only
 *
 * @param[in]  duration  Duration in [ms]
 */
void Sleep(const uint32_t duration);

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Minimal ros::Time() / rplcpp::Time implementation (that doesn't throw)
 */
struct RosTime
{
    RosTime();
    /**
     * @brief Constructor
     *
     * @param[in]  sec   Time value seconds
     * @param[in]  nsec   Time value nanoseconds
     */
    RosTime(const uint32_t sec, const uint32_t nsec);

    /**
     * @brief Convert to seconds
     *
     * @returns the time value (time since epoch) in [s]
     */
    double ToSec() const;

    /**
     * @brief Check if time is zero (invalid, unset)
     *
     * @returns true if time is zero (invalid, unset)
     */
    bool IsZero() const;

    uint32_t sec_;   //!< Seconds part of time
    uint32_t nsec_;  //!< Nanoseconds part of time (*should* be in range 0-999999999)
};

/* ****************************************************************************************************************** */
}  // namespace time
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_TIME_HPP__
