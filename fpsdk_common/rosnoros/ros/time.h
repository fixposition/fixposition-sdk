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
 *  @brief ROS time polyfill
 *
 *  @details Files impoorted:
 *              - ros/duration.h
 *              - ros/time.h
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __ROSNOROS__TIME_H__
#define __ROSNOROS__TIME_H__

/**********************************************************************************************************************/
// COMMON
/**********************************************************************************************************************/
#include <climits>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>

namespace boost {
namespace posix_time {
class ptime;
class time_duration;
}  // namespace posix_time
}  // namespace boost

#define ROSTIME_DECL

/**********************************************************************************************************************/
// ros/duration.h
/**********************************************************************************************************************/
namespace ros {

ROSTIME_DECL void normalizeSecNSecSigned(int64_t& sec, int64_t& nsec);
ROSTIME_DECL void normalizeSecNSecSigned(int32_t& sec, int32_t& nsec);

/**
 * \brief Base class for Duration implementations.  Provides storage, common functions and operator overloads.
 * This should not need to be used directly.
 */
template <class T>
class DurationBase {
   public:
    int32_t sec, nsec;
    DurationBase() : sec(0), nsec(0) {}
    DurationBase(int32_t _sec, int32_t _nsec);
    explicit DurationBase(double t) { fromSec(t); };
    T operator+(const T& rhs) const;
    T operator-(const T& rhs) const;
    T operator-() const;
    T operator*(double scale) const;
    T& operator+=(const T& rhs);
    T& operator-=(const T& rhs);
    T& operator*=(double scale);
    bool operator==(const T& rhs) const;
    inline bool operator!=(const T& rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T& rhs) const;
    bool operator<(const T& rhs) const;
    bool operator>=(const T& rhs) const;
    bool operator<=(const T& rhs) const;
    double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); };
    int64_t toNSec() const { return static_cast<int64_t>(sec) * 1000000000ll + static_cast<int64_t>(nsec); };
    T& fromSec(double t);
    T& fromNSec(int64_t t);
    bool isZero() const;
    boost::posix_time::time_duration toBoost() const;
    static const T MIN;          //!< Minimum representable duration (negative)
    static const T MAX;          //!< Maximum representable duration
    static const T ZERO;         //!< Zero duration
    static const T DAY;          //!< One day duration
    static const T HOUR;         //!< One hour duration
    static const T MINUTE;       //!< One minute duration
    static const T SECOND;       //!< One second duration
    static const T MILLISECOND;  //!< One millisecond duration
    static const T MICROSECOND;  //!< One microsecond duration
    static const T NANOSECOND;   //!< One nanosecond duration
};

class Rate;

/**
 * \brief Duration representation for use with the Time class.
 *
 * ros::DurationBase provides most of its functionality.
 */
class ROSTIME_DECL Duration : public DurationBase<Duration> {
   public:
    Duration() : DurationBase<Duration>() {}

    Duration(int32_t _sec, int32_t _nsec) : DurationBase<Duration>(_sec, _nsec) {}

    explicit Duration(double t) { fromSec(t); }
    explicit Duration(const Rate&);
    /**
     * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for
     * the time remaining.
     * @return True if the desired sleep duration was met, false otherwise.
     */
    bool sleep() const;
};

extern ROSTIME_DECL const Duration DURATION_MAX;
extern ROSTIME_DECL const Duration DURATION_MIN;
template <>
const Duration DurationBase<Duration>::MAX;
template <>
const Duration DurationBase<Duration>::MIN;
template <>
const Duration DurationBase<Duration>::ZERO;
template <>
const Duration DurationBase<Duration>::DAY;
template <>
const Duration DurationBase<Duration>::HOUR;
template <>
const Duration DurationBase<Duration>::MINUTE;
template <>
const Duration DurationBase<Duration>::SECOND;
template <>
const Duration DurationBase<Duration>::MILLISECOND;
template <>
const Duration DurationBase<Duration>::MICROSECOND;
template <>
const Duration DurationBase<Duration>::NANOSECOND;

/**
 * \brief Duration representation for use with the WallTime class.
 *
 * ros::DurationBase provides most of its functionality.
 */
class ROSTIME_DECL WallDuration : public DurationBase<WallDuration> {
   public:
    WallDuration() : DurationBase<WallDuration>() {}

    WallDuration(int32_t _sec, int32_t _nsec) : DurationBase<WallDuration>(_sec, _nsec) {}

    explicit WallDuration(double t) { fromSec(t); }
    explicit WallDuration(const Rate&);
    /**
     * \brief sleep for the amount of time specified by this Duration.  If a signal interrupts the sleep, resleeps for
     * the time remaining.
     * @return True if the desired sleep duration was met, false otherwise.
     */
    bool sleep() const;
};

template <>
const WallDuration DurationBase<WallDuration>::MAX;
template <>
const WallDuration DurationBase<WallDuration>::MIN;
template <>
const WallDuration DurationBase<WallDuration>::ZERO;
template <>
const WallDuration DurationBase<WallDuration>::DAY;
template <>
const WallDuration DurationBase<WallDuration>::HOUR;
template <>
const WallDuration DurationBase<WallDuration>::MINUTE;
template <>
const WallDuration DurationBase<WallDuration>::SECOND;
template <>
const WallDuration DurationBase<WallDuration>::MILLISECOND;
template <>
const WallDuration DurationBase<WallDuration>::MICROSECOND;
template <>
const WallDuration DurationBase<WallDuration>::NANOSECOND;

ROSTIME_DECL std::ostream& operator<<(std::ostream& os, const Duration& rhs);
ROSTIME_DECL std::ostream& operator<<(std::ostream& os, const WallDuration& rhs);

}  // namespace ros

/**********************************************************************************************************************/
// ros/time.h
/**********************************************************************************************************************/
#include "utils.h"

namespace ros {

/*********************************************************************
 ** Exceptions
 *********************************************************************/

/**
 * @brief Thrown if the ros subsystem hasn't been initialised before use.
 */
class ROSTIME_DECL TimeNotInitializedException : public Exception {
   public:
    TimeNotInitializedException()
        : Exception(
              "Cannot use ros::Time::now() before the first NodeHandle has "
              "been created or ros::start() has been called.  "
              "If this is a standalone app or test that just uses "
              "ros::Time and does not communicate over ROS, you may also "
              "call ros::Time::init()") {}
};

/**
 * @brief Thrown if windows high perf. timestamping is unavailable.
 *
 * @sa getWallTime
 */
class ROSTIME_DECL NoHighPerformanceTimersException : public Exception {
   public:
    NoHighPerformanceTimersException()
        : Exception(
              "This windows platform does not "
              "support the high-performance timing api.") {}
};

/*********************************************************************
 ** Functions
 *********************************************************************/

ROSTIME_DECL void normalizeSecNSec(uint64_t& sec, uint64_t& nsec);
ROSTIME_DECL void normalizeSecNSec(uint32_t& sec, uint32_t& nsec);
ROSTIME_DECL void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);
ROSTIME_DECL void ros_walltime(uint32_t& sec, uint32_t& nsec);
ROSTIME_DECL void ros_steadytime(uint32_t& sec, uint32_t& nsec);

/*********************************************************************
 ** Time Classes
 *********************************************************************/

/**
 * \brief Base class for Time implementations.  Provides storage, common
 * functions and operator overloads. This should not need to be used directly.
 */
template <class T, class D>
class TimeBase {
   public:
    uint32_t sec, nsec;

    TimeBase() : sec(0), nsec(0) {}
    TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec) { normalizeSecNSec(sec, nsec); }
    explicit TimeBase(double t) { fromSec(t); }
    D operator-(const T& rhs) const;
    T operator+(const D& rhs) const;
    T operator-(const D& rhs) const;
    T& operator+=(const D& rhs);
    T& operator-=(const D& rhs);
    bool operator==(const T& rhs) const;
    inline bool operator!=(const T& rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T& rhs) const;
    bool operator<(const T& rhs) const;
    bool operator>=(const T& rhs) const;
    bool operator<=(const T& rhs) const;

    double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); };
    T& fromSec(double t);

    uint64_t toNSec() const { return static_cast<uint64_t>(sec) * 1000000000ull + static_cast<uint64_t>(nsec); }
    T& fromNSec(uint64_t t);

    inline bool isZero() const { return sec == 0 && nsec == 0; }
    inline bool is_zero() const { return isZero(); }
    boost::posix_time::ptime toBoost() const;

    static const T MIN;            //!< Minimum representable time
    static const T MAX;            //!< Maximum representable time
    static const T ZERO;           //!< Zero (invalid) time
    static const T UNINITIALIZED;  //!< Uninitialized time
};

/**
 * \brief Time representation.  May either represent wall clock time or ROS
 * clock time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class ROSTIME_DECL Time : public TimeBase<Time, Duration> {
   public:
    Time() : TimeBase<Time, Duration>() {}

    Time(uint32_t _sec, uint32_t _nsec) : TimeBase<Time, Duration>(_sec, _nsec) {}

    explicit Time(double t) { fromSec(t); }

    /**
     * \brief Retrieve the current time.  If ROS clock time is in use, this
     * returns the time according to the ROS clock.  Otherwise returns the current
     * wall clock time.
     */
    static Time now();
    /**
     * \brief Sleep until a specific time has been reached.
     * @return True if the desired sleep time was met, false otherwise.
     */
    static bool sleepUntil(const Time& end);

    static void init();
    static void shutdown();
    static void setNow(const Time& new_now);
    static bool useSystemTime();
    static bool isSimTime();
    static bool isSystemTime();

    /**
     * \brief Returns whether or not the current time source is valid.  Simulation
     * time is valid if it is non-zero.
     */
    static bool isValid();
    /**
     * \brief Wait for time source to become valid
     */
    static bool waitForValid();
    /**
     * \brief Wait for time source to become valid, with timeout
     */
    static bool waitForValid(const ros::WallDuration& timeout);

    static Time fromBoost(const boost::posix_time::ptime& t);
    static Time fromBoost(const boost::posix_time::time_duration& d);
};

extern ROSTIME_DECL const Time TIME_MAX;
extern ROSTIME_DECL const Time TIME_MIN;
template <>
const Time TimeBase<Time, Duration>::MAX;
template <>
const Time TimeBase<Time, Duration>::MIN;
template <>
const Time TimeBase<Time, Duration>::ZERO;
template <>
const Time TimeBase<Time, Duration>::UNINITIALIZED;

/**
 * \brief Time representation.  Always wall-clock time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class ROSTIME_DECL WallTime : public TimeBase<WallTime, WallDuration> {
   public:
    WallTime() : TimeBase<WallTime, WallDuration>() {}

    WallTime(uint32_t _sec, uint32_t _nsec) : TimeBase<WallTime, WallDuration>(_sec, _nsec) {}

    explicit WallTime(double t) { fromSec(t); }

    /**
     * \brief Returns the current wall clock time.
     */
    static WallTime now();

    /**
     * \brief Sleep until a specific time has been reached.
     * @return True if the desired sleep time was met, false otherwise.
     */
    static bool sleepUntil(const WallTime& end);

    static bool isSystemTime() { return true; }
};

template <>
const WallTime TimeBase<WallTime, WallDuration>::MAX;
template <>
const WallTime TimeBase<WallTime, WallDuration>::MIN;
template <>
const WallTime TimeBase<WallTime, WallDuration>::ZERO;
template <>
const WallTime TimeBase<WallTime, WallDuration>::UNINITIALIZED;

/**
 * \brief Time representation.  Always steady-clock time.
 *
 * Not affected by ROS time.
 *
 * ros::TimeBase provides most of its functionality.
 */
class ROSTIME_DECL SteadyTime : public TimeBase<SteadyTime, WallDuration> {
   public:
    SteadyTime() : TimeBase<SteadyTime, WallDuration>() {}

    SteadyTime(uint32_t _sec, uint32_t _nsec) : TimeBase<SteadyTime, WallDuration>(_sec, _nsec) {}

    explicit SteadyTime(double t) { fromSec(t); }

    /**
     * \brief Returns the current steady (monotonic) clock time.
     */
    static SteadyTime now();

    /**
     * \brief Sleep until a specific time has been reached.
     * @return True if the desired sleep time was met, false otherwise.
     */
    static bool sleepUntil(const SteadyTime& end);

    static bool isSystemTime() { return true; }
};

template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::MAX;
template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::MIN;
template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::ZERO;
template <>
const SteadyTime TimeBase<SteadyTime, WallDuration>::UNINITIALIZED;

ROSTIME_DECL std::ostream& operator<<(std::ostream& os, const Time& rhs);
ROSTIME_DECL std::ostream& operator<<(std::ostream& os, const WallTime& rhs);
ROSTIME_DECL std::ostream& operator<<(std::ostream& os, const SteadyTime& rhs);
}  // namespace ros

/**********************************************************************************************************************/
// ros/....h
/**********************************************************************************************************************/

#endif  // __ROSNOROS__TIME_H__
