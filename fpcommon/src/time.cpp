/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com)
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: time utilities
 */

/* LIBC/STL */
#include <cstddef>
#include <ctime>

/* EXTERNAL */
#include <sys/stat.h>
#include <unistd.h>

/* PACKAGE */
#include "fpcommon/time.hpp"

namespace fp {
namespace common {
namespace time {
/* ****************************************************************************************************************** */

uint64_t GetTicks()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (tp.tv_sec * 1000) + (tp.tv_nsec / 1000000);
}

// ---------------------------------------------------------------------------------------------------------------------

double GetSecs()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (double)tp.tv_sec + ((double)tp.tv_nsec * 1e-9);
}

// ---------------------------------------------------------------------------------------------------------------------

void Sleep(const uint32_t duration)
{
    usleep(duration * 1000);
}

/* ****************************************************************************************************************** */

RosTime::RosTime() : sec_{0}, nsec_{0}
{
}

RosTime::RosTime(const uint32_t sec, const uint32_t nsec) : sec_{sec}, nsec_{nsec}
{
}

double RosTime::ToSec() const
{
    return (double)sec_ + ((double)nsec_ * 1e-9);
}

bool RosTime::IsZero() const
{
    return (sec_ == 0) && (nsec_ == 0);
}

static_assert(sizeof(RosTime) == (sizeof(uint32_t) + sizeof(uint32_t)), "");
static_assert(offsetof(RosTime, sec_) == 0, "");
static_assert(offsetof(RosTime, nsec_) == sizeof(uint32_t), "");

/* ****************************************************************************************************************** */
}  // namespace time
}  // namespace common
}  // namespace fp
