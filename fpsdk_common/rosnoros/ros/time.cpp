/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file
 *  @brief ROS time polyfill (implementation)
 *
 *  @details Files impoorted:
 *              - ros/src/duration.cpp
 *              - ros/src/time.cpp
 *
 *  @details Unlike the real thing there is no simulation time here. ros::Time::now() always is the wall clock time and
 *           Time::init()/shutdown()/setNow() do nothing. The sleep(), toBoost() and fromBoost() methods and the Rate
 *           class are not implemented (they are declared in time.h, but never defined). This file is only compiled in
 *           the non-ROS1 case (see ../../CMakeLists.txt), where the real librostime is not available.
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#include <ctime>
#include <iomanip>
#include <limits>
#include <ostream>
#include <stdexcept>

#include "time.h"

namespace ros {

/**********************************************************************************************************************/
// ros/src/duration.cpp
/**********************************************************************************************************************/

void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec) {
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + (nsec / 1000000000L);
    if (nsec_part < 0) {
        nsec_part += 1000000000L;
        --sec_part;
    }
    if ((sec_part < std::numeric_limits<int32_t>::min()) || (sec_part > std::numeric_limits<int32_t>::max())) {
        throw std::runtime_error("Duration is out of dual 32-bit range");
    }
    sec = sec_part;
    nsec = nsec_part;
}

void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec) {
    int64_t sec64 = sec;
    int64_t nsec64 = nsec;
    normalizeSecNSecSigned(sec64, nsec64);
    sec = static_cast<int32_t>(sec64);
    nsec = static_cast<int32_t>(nsec64);
}

/**********************************************************************************************************************/
// ros/src/time.cpp
/**********************************************************************************************************************/

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec) {
    const uint64_t nsec_part = nsec % 1000000000UL;
    const uint64_t sec_part = nsec / 1000000000UL;
    if ((sec + sec_part) > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("Time is out of dual 32-bit range");
    }
    sec += sec_part;
    nsec = nsec_part;
}

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec) {
    uint64_t sec64 = sec;
    uint64_t nsec64 = nsec;
    normalizeSecNSec(sec64, nsec64);
    sec = static_cast<uint32_t>(sec64);
    nsec = static_cast<uint32_t>(nsec64);
}

void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec) {
    int64_t nsec_part = nsec % 1000000000L;
    int64_t sec_part = sec + (nsec / 1000000000L);
    if (nsec_part < 0) {
        nsec_part += 1000000000L;
        --sec_part;
    }
    if ((sec_part < 0) || (sec_part > static_cast<int64_t>(std::numeric_limits<uint32_t>::max()))) {
        throw std::runtime_error("Time is out of dual 32-bit range");
    }
    sec = sec_part;
    nsec = nsec_part;
}

// ---------------------------------------------------------------------------------------------------------------------

// Note: the order of these definitions matters (static initialisation order within this translation unit)

const Duration DURATION_MAX(std::numeric_limits<int32_t>::max(), 999999999);
const Duration DURATION_MIN(std::numeric_limits<int32_t>::min(), 0);

template <>
const Duration DurationBase<Duration>::MAX = DURATION_MAX;
template <>
const Duration DurationBase<Duration>::MIN = DURATION_MIN;
template <>
const Duration DurationBase<Duration>::ZERO = Duration(0, 0);
template <>
const Duration DurationBase<Duration>::NANOSECOND = Duration(0, 1);
template <>
const Duration DurationBase<Duration>::MICROSECOND = Duration(0, 1000);
template <>
const Duration DurationBase<Duration>::MILLISECOND = Duration(0, 1000000);
template <>
const Duration DurationBase<Duration>::SECOND = Duration(1, 0);
template <>
const Duration DurationBase<Duration>::MINUTE = Duration(60, 0);
template <>
const Duration DurationBase<Duration>::HOUR = Duration(60 * 60, 0);
template <>
const Duration DurationBase<Duration>::DAY = Duration(60 * 60 * 24, 0);

template <>
const WallDuration DurationBase<WallDuration>::MAX = WallDuration(Duration::MAX.sec, Duration::MAX.nsec);
template <>
const WallDuration DurationBase<WallDuration>::MIN = WallDuration(Duration::MIN.sec, Duration::MIN.nsec);
template <>
const WallDuration DurationBase<WallDuration>::ZERO = WallDuration(Duration::ZERO.sec, Duration::ZERO.nsec);
template <>
const WallDuration DurationBase<WallDuration>::DAY = WallDuration(Duration::DAY.sec, Duration::DAY.nsec);
template <>
const WallDuration DurationBase<WallDuration>::HOUR = WallDuration(Duration::HOUR.sec, Duration::HOUR.nsec);
template <>
const WallDuration DurationBase<WallDuration>::MINUTE = WallDuration(Duration::MINUTE.sec, Duration::MINUTE.nsec);
template <>
const WallDuration DurationBase<WallDuration>::SECOND = WallDuration(Duration::SECOND.sec, Duration::SECOND.nsec);
template <>
const WallDuration DurationBase<WallDuration>::MILLISECOND =
    WallDuration(Duration::MILLISECOND.sec, Duration::MILLISECOND.nsec);
template <>
const WallDuration DurationBase<WallDuration>::MICROSECOND =
    WallDuration(Duration::MICROSECOND.sec, Duration::MICROSECOND.nsec);
template <>
const WallDuration DurationBase<WallDuration>::NANOSECOND =
    WallDuration(Duration::NANOSECOND.sec, Duration::NANOSECOND.nsec);

const Time TIME_MAX(std::numeric_limits<uint32_t>::max(), 999999999);
const Time TIME_MIN(0, 1);

template <>
const Time TimeBase<Time, Duration>::MAX = TIME_MAX;
template <>
const Time TimeBase<Time, Duration>::MIN = TIME_MIN;
template <>
const Time TimeBase<Time, Duration>::ZERO = Time(0, 0);
template <>
const Time TimeBase<Time, Duration>::UNINITIALIZED = Time::ZERO;

template <>
const WallTime TimeBase<WallTime, WallDuration>::MAX = WallTime(Time::MAX.sec, Time::MAX.nsec);
template <>
const WallTime TimeBase<WallTime, WallDuration>::MIN = WallTime(Time::MIN.sec, Time::MIN.nsec);
template <>
const WallTime TimeBase<WallTime, WallDuration>::ZERO = WallTime(Time::ZERO.sec, Time::ZERO.nsec);
template <>
const WallTime TimeBase<WallTime, WallDuration>::UNINITIALIZED =
    WallTime(Time::UNINITIALIZED.sec, Time::UNINITIALIZED.nsec);

template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::MAX = SteadyTime(Time::MAX.sec, Time::MAX.nsec);
template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::MIN = SteadyTime(Time::MIN.sec, Time::MIN.nsec);
template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::ZERO = SteadyTime(Time::ZERO.sec, Time::ZERO.nsec);
template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::UNINITIALIZED =
    SteadyTime(Time::UNINITIALIZED.sec, Time::UNINITIALIZED.nsec);

// ---------------------------------------------------------------------------------------------------------------------

void ros_walltime(uint32_t& sec, uint32_t& nsec) {
    struct timespec start;
    clock_gettime(CLOCK_REALTIME, &start);
    if ((start.tv_sec < 0) || (start.tv_sec > static_cast<time_t>(std::numeric_limits<uint32_t>::max()))) {
        throw std::runtime_error("Timespec is out of dual 32-bit range");
    }
    sec = static_cast<uint32_t>(start.tv_sec);
    nsec = static_cast<uint32_t>(start.tv_nsec);
}

void ros_steadytime(uint32_t& sec, uint32_t& nsec) {
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    sec = static_cast<uint32_t>(start.tv_sec);
    nsec = static_cast<uint32_t>(start.tv_nsec);
}

// ---------------------------------------------------------------------------------------------------------------------

// There is no simulation time here, it's always the wall clock

bool Time::useSystemTime() {
    return true;
}

bool Time::isSimTime() {
    return false;
}

bool Time::isSystemTime() {
    return true;
}

bool Time::isValid() {
    return true;
}

bool Time::waitForValid() {
    return true;
}

bool Time::waitForValid(const ros::WallDuration& timeout) {
    (void)timeout;
    return true;
}

void Time::init() {
}

void Time::shutdown() {
}

void Time::setNow(const Time& new_now) {
    (void)new_now;
}

Time Time::now() {
    Time t;
    ros_walltime(t.sec, t.nsec);
    return t;
}

WallTime WallTime::now() {
    WallTime t;
    ros_walltime(t.sec, t.nsec);
    return t;
}

SteadyTime SteadyTime::now() {
    SteadyTime t;
    ros_steadytime(t.sec, t.nsec);
    return t;
}

// ---------------------------------------------------------------------------------------------------------------------

namespace {
// Print sec.nsec, restoring the stream flags afterwards
template <typename T>
std::ostream& printTime(std::ostream& os, const T& rhs) {
    const std::ios_base::fmtflags flags = os.flags();
    const char fill = os.fill();
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    os.fill(fill);
    os.flags(flags);
    return os;
}

template <typename T>
std::ostream& printDuration(std::ostream& os, const T& rhs) {
    const std::ios_base::fmtflags flags = os.flags();
    const char fill = os.fill();
    if ((rhs.sec >= 0) || (rhs.nsec == 0)) {
        os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    } else {
        os << (rhs.sec == -1 ? "-" : "") << (rhs.sec + 1) << "." << std::setw(9) << std::setfill('0')
           << (1000000000 - rhs.nsec);
    }
    os.fill(fill);
    os.flags(flags);
    return os;
}
}  // namespace

std::ostream& operator<<(std::ostream& os, const Time& rhs) {
    return printTime(os, rhs);
}

std::ostream& operator<<(std::ostream& os, const WallTime& rhs) {
    return printTime(os, rhs);
}

std::ostream& operator<<(std::ostream& os, const SteadyTime& rhs) {
    return printTime(os, rhs);
}

std::ostream& operator<<(std::ostream& os, const Duration& rhs) {
    return printDuration(os, rhs);
}

std::ostream& operator<<(std::ostream& os, const WallDuration& rhs) {
    return printDuration(os, rhs);
}

}  // namespace ros
