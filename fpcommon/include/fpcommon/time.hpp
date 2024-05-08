/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: time utilities
 */
#ifndef __FPCOMMON_TIME_HPP__
#define __FPCOMMON_TIME_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fp {
namespace common {
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
 * @brief Minimal ros::Time() implementation
 */
struct RosTime {
    RosTime();
    RosTime(const uint32_t sec, const uint32_t nsec);
    double ToSec() const;
    bool IsZero() const;
    uint32_t sec_;
    uint32_t nsec_;
};

/* ****************************************************************************************************************** */
}  // namespace time
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_TIME_HPP__
