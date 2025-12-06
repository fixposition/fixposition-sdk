/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Parts copyright (c) 2008, Willow Garage, Inc., see time.cpp and the LICENSE file for details
 * Written by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Time utilities
 *
 * @page FPSDK_COMMON_TIME Time utilities
 *
 * **API**: fpsdk_common/time.hpp and fpsdk::common::time
 *
 */
#ifndef __FPSDK_COMMON_TIME_HPP__
#define __FPSDK_COMMON_TIME_HPP__

/* LIBC/STL */
#include <chrono>
#include <cstdint>
#include <ctime>
#include <string>

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
 * @brief Constants
 * @{
 */

// clang-format off
static constexpr int    SEC_IN_MIN_I   = 60;                                  //!< Number of seconds in a minute (integer) = 60
static constexpr int    SEC_IN_HOUR_I  = 60 * SEC_IN_MIN_I;                   //!< Number of seconds in an hour (integer)  = 3600
static constexpr int    SEC_IN_DAY_I   = 24 * SEC_IN_HOUR_I;                  //!< Number of seconds in a day (integer)    = 86400
static constexpr int    SEC_IN_WEEK_I  =  7 * SEC_IN_DAY_I;                   //!< Number of seconds in a week (integer)   = 604800
static constexpr double SEC_IN_MIN_D   = static_cast<double>(SEC_IN_MIN_I);   //!< Number of seconds in a minute (double)  = 60.0
static constexpr double SEC_IN_HOUR_D  = static_cast<double>(SEC_IN_HOUR_I);  //!< Number of seconds in an hour (double)   = 3600.0
static constexpr double SEC_IN_DAY_D   = static_cast<double>(SEC_IN_DAY_I);   //!< Number of seconds in a day (double)     = 86400.0
static constexpr double SEC_IN_WEEK_D  = static_cast<double>(SEC_IN_WEEK_I);  //!< Number of seconds in a week (double)    = 604800.0
// clang-format on

/**
 * @brief Convert seconds to nanoseconds
 *
 * @note Depending on the \c ns value this can be lossy.
 *
 * @param[in]  ns  Nanoseconds alue
 *
 * @returns the seconds value
 */
static constexpr double NsecToSec(const uint64_t ns)
{
    return (double)ns * 1e-9;  // (double)(ns % (uint64_t)1000000000) + ((double)(ns / (uint64_t)1000000000) * 1e-9);
}

/**
 * @brief Convert nanoseconds to seconds
 *
 * @param[in]  sec  Seconds value
 *
 * @returns the nanoseconds value
 */
static constexpr uint64_t SecToNsec(const double sec)
{
    return sec * 1e9;
}

///@}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Get milliseconds [ms]
 *
 * Time is monotonic time since first call to GetMillis() or GetSecs().
 *
 * @returns the number of milliseconds (> 0)
 */
uint64_t GetMillis();

/**
 * @brief Get seconds [s]
 *
 * Time is monotonic time since first call to GetMillis() or GetSecs().
 *
 * @returns the number of seconds
 */
double GetSecs();

// ---------------------------------------------------------------------------------------------------------------------

class Duration;  // forward declaration

/**
 * @brief Helper to measure wallclock time
 */
class TicToc
{
   public:
    /**
     * @brief Constructor
     *
     * This also starts the measurements, like Tic().
     */
    TicToc();

    /**
     * @brief (Re-)start measurement
     */
    void Tic();

    /**
     * @brief Get elapsed wallclock time
     *
     * @param[in]  reset  Reset (restart) measurements
     *
     * @returns the elapsed time since object constructed resp. last call to Tic()
     */
    Duration Toc(const bool reset = false);

    /**
     * @brief Get elapsed wallclock time in [ms]
     *
     * @param[in]  reset  Reset (restart) measurements
     *
     * @returns the elapsed time since object constructed resp. last call to Tic() in [ms]
     */
    double TocMs(const bool reset = false);

   private:
    std::chrono::time_point<std::chrono::steady_clock> t0_;
};

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
     * @param[in]  nsec  Time value nanoseconds
     */
    RosTime(const uint32_t sec, const uint32_t nsec);

    /**
     * @brief Convert to seconds
     *
     * @returns the time value (time since epoch) in [s]
     */
    double ToSec() const;

    /**
     * @brief Convert to nanoseconds
     *
     * @returns the time value (time since epoch) in [ns]
     */
    uint64_t ToNSec() const;

    /**
     * @brief Check if time is zero (invalid, unset)
     *
     * @returns true if time is zero (invalid, unset)
     */
    bool IsZero() const;

    uint32_t sec_;   //!< Seconds part of time
    uint32_t nsec_;  //!< Nanoseconds part of time (*should* be in range 0-999999999)

    bool operator==(const RosTime& rhs) const;  //!< Equal
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Time duration
 *
 * This is similar (and binary compatible) to ros::Duration and rpclpp::Duration. While some of the constructors and
 * operators can throw, it also provides non-throwing methods to manipulate the duration.
 */
class Duration
{
   public:
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Make Duration object
     *
     * @note These throw std::runtime_error if the values are out of range.
     * @{
     */

    /**
     * @brief Constructor, zero duration
     */
    Duration();

    /**
     * @brief Make Duration from seconds and nanoseconds
     *
     * @param[in]  sec   Duration value seconds
     * @param[in]  nsec  Duration value nanoseconds
     */
    static Duration FromSecNSec(const int32_t sec, const int32_t nsec);

    /**
     * @brief Make Duration from nanoseconds
     *
     * @param[in]  nsec  Duration value nanoseconds
     */
    static Duration FromNSec(const int64_t nsec);

    /**
     * @brief Make Duration from seconds
     *
     * @param[in]  sec   Duration value seconds
     */
    static Duration FromSec(const double sec);

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Set duration
     *
     * These do not throw and instead return true/false.
     * @{
     */

    /**
     * @brief Set duration from seconds and nanoseconds
     *
     * @param[in]  sec   Duration value seconds
     * @param[in]  nsec  Duration value nanoseconds
     *
     * @returns true if successful (values in range), false otherwise (bad values)
     */
    bool SetSecNSec(const int32_t sec, const int32_t nsec);

    /**
     * @brief Set duration from nanoseconds
     *
     * @param[in]  nsec   Duration value nanoseconds
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool SetNSec(const int64_t nsec);

    /**
     * @brief Set duration from seconds
     *
     * @param[in]  sec  Duration value seconds
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool SetSec(const double sec);

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Get duration
     * @{
     */

    /**
     * @brief Get duration as nanoseconds
     *
     * @returns the duration as nanoseconds
     */
    int64_t GetNSec() const;

    /**
     * @brief Get duration as seconds
     *
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the duration as seconds
     */
    double GetSec(const int prec = 9) const;

    /**
     * @brief Get duration as std::chrono::milliseconds
     *
     * @returns the duration as std::chrono::milliseconds
     */
    std::chrono::milliseconds GetChronoMilli() const;

    /**
     * @brief Get duration as std::chrono::nanoseconds
     *
     * @returns the duration as std::chrono::nanoseconds
     */
    std::chrono::nanoseconds GetChronoNano() const;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Misc
     * @{
     */

    /**
     * @brief Check if duration is zero
     *
     * @returns true if duration is exactly zero
     */
    bool IsZero() const;

    /**
     * @brief Stringify duration, for debugging
     *
     * @param[in]  prec  Number of fractional digits for the seconds (0-9)
     *
     * @returns The stringified duration ("HH:MM:SS.SSS", resp. "DDd HH:MM:SS.SSS" if duration > 1d)
     */
    std::string Stringify(const int prec = 3) const;

    /**
     * @brief Sleep for the duration (if > 0)
     */
    void Sleep() const;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Arithmetic methods
     *
     * These do not throw and instead return true/false.
     * @{
     */

    /**
     * @brief Add duration to duration
     *
     * @param[in]  dur  Duration to add
     *
     * @returns true if successful (dur value in range), false otherwise (bad sec value)
     */
    bool AddDur(const Duration& dur);

    /**
     * @brief Add nanoseconds to duration
     *
     * @param[in]  nsec  Nanoseconds to add
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool AddNSec(const int64_t nsec);

    /**
     * @brief Add seconds to duration
     *
     * @param[in]  sec  Seconds to add
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool AddSec(const double sec);

    /**
     * @brief Substract duration from duration
     *
     * @param[in]  dur  Duration to substract
     *
     * @returns true if successful (dur value in range), false otherwise (bad sec value)
     */
    bool SubDur(const Duration& dur);

    /**
     * @brief Substract nanoseconds from duration
     *
     * @param[in]  nsec  Nanoseconds to substract
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool SubNSec(const int64_t nsec);

    /**
     * @brief Substract seconds from duration
     *
     * @param[in]  sec  Seconds to substract
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool SubSec(const double sec);

    /**
     * @brief Scale (multiply) duration
     *
     * @param[in]  sec  Scale factor
     *
     * @returns true if successful (scale value in range), false otherwise (bad scale value)
     */
    bool Scale(const double sec);

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Arithmetic operators
     *
     * @note These throw std::runtime_error if the values are out of range.
     * @{
     */

    Duration operator+(const Duration& rhs) const;  //!< Sum duration and duration
    Duration operator+(const int64_t nsec) const;   //!< Sum duration and nanoseconds
    Duration operator+(const double sec) const;     //!< Sum duration and seconds
    Duration& operator+=(const Duration& rhs);      //!< Add duration to duration
    Duration& operator+=(const int64_t nsec);       //!< Add nanoseconds to durationn
    Duration& operator+=(const double sec);         //!< Add seconds to duration
    Duration operator-(const Duration& rhs) const;  //!< Subtract duration and duration
    Duration operator-(const int64_t nsec) const;   //!< Subtract duration and nanoseconds
    Duration operator-(const double sec) const;     //!< Subtract duration and seconds
    Duration& operator-=(const Duration& rhs);      //!< Subtract duration from duration
    Duration& operator-=(const int64_t nsec);       //!< Subtract nanoseconds from duration
    Duration& operator-=(const double sec);         //!< Subtract seconds from durationn
    Duration operator-() const;                     //!< Reverse sign
    Duration operator*(double scale) const;         //!< Multiply (scale) duration
    Duration& operator*=(double scale);             //!< Multiply (scale) duration

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Logic operators
     * @{
     */

    bool operator==(const Duration& rhs) const;  //!< Equal
    bool operator!=(const Duration& rhs) const;  //!< Not equal
    bool operator>(const Duration& rhs) const;   //!< Greater than
    bool operator<(const Duration& rhs) const;   //!< Smaller than
    bool operator>=(const Duration& rhs) const;  //!< Greater or equal than
    bool operator<=(const Duration& rhs) const;  //!< Smaller or equal than

    ///@}
    // -----------------------------------------------------------------------------------------------------------------

    // Storage deliberately public
    int32_t sec_;   //!< Seconds part of duration
    int32_t nsec_;  //!< Nanoseconds part of duration (*should* be in range 0-999999999)

    ///@
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief GNSS atomic time representation: week number (wno) and time of week (tow) used by GPS, Galileo and BeiDou
 */
struct WnoTow
{
    /**
     * @brief GNSS time system
     */
    enum class Sys
    {
        GPS,  //!< GPS (also: SBAS, QZSS)
        GAL,  //!< Galileo system time (GST)
        BDS,  //!< BeiDou time
    };

    /**
     * @brief Constructor
     *
     * @param[in]  sys  GNSS
     */
    WnoTow(const Sys sys = Sys::GPS);

    /**
     * @brief Constructor
     *
     * @note Input values are not normalised nor range checked here. See comments in Time::GetWnoTow() and
     *       Time::SetWnoTow()
     *
     * @param[in]  wno  Week number [-] (>= SANE_WNO_MIN)
     * @param[in]  tow  Time of week [s] (SANE_TOW_MIN - SANE_TOW_MAX)
     * @param[in]  sys  GNSS
     */
    WnoTow(const int wno, const double tow, const Sys sys = Sys::GPS);

    // clang-format off
    static constexpr int    SANE_WNO_MIN = 0;    //!< Minimum sane wno_ value
    static constexpr int    SANE_WNO_MAX = 9999; //!< Maximum sane wno_ value, good enough for the next centuries
    static constexpr double SANE_TOW_MIN = 0.0;  //!< Minimum sane tow_ value
    static constexpr double SANE_TOW_MAX = SEC_IN_WEEK_D - std::numeric_limits<double>::epsilon();  //!< Maximum sane tow_ value

    int    wno_ = 0;    //!< Week number [-] (>= SANE_WNO_MIN, if normalised)
    double tow_ = 0.0;  //!< Time of week [s] (SANE_TOW_MIN - SANE_TOW_MAX, if normalised)
    Sys    sys_;        //!< Time system
    // clang-format on

    /**
     * @brief Stringify time system (for debugging)
     *
     * @returns a string identifying the time system
     */
    const char* SysStr() const;
};

/**
 * @brief GLONASS time
 */
struct GloTime
{
    GloTime() = default;  //!< Ctor

    /**
     * @brief Constructor
     *
     * @note Input values are not normalised nor range checked here. See comments in Time::GetGloTime() and
     *       Time::SetGloTime()
     *
     * @param[in]  N4   Four-year interval, 1=1996..1999, 2=2000..2003, ... (SANE_N4_MIN - SANE_N4_MAX)
     * @param[in]  Nt   Day in four-year interval (SANE_NT_MIN - SANE_NT_MAX)
     * @param[in]  TOD  Time of day (in Москва time zone) [s] (SANE_TOD_MIN - SANE_TOD_MAX)
     */
    GloTime(const int N4, const int Nt, const double TOD);

    // clang-format off
    static constexpr int    SANE_N4_MIN  = 1;     //!< Minimum sane N4_ value
    static constexpr int    SANE_N4_MAX  = 99;    //!< Maximum sane N4_ value, good enough for the next centuries
    static constexpr int    SANE_NT_MIN  = 1;     //!< Minimum sane Nt_ value
    static constexpr int    SANE_NT_MAX  = 1461;  //!< Maximum sane Nt_ value
    static constexpr double SANE_TOD_MIN = 0.0;   //!< Minimum sane TOD_ value
    static constexpr double SANE_TOD_MAX = SEC_IN_DAY_D - std::numeric_limits<double>::epsilon();  //!< Maximum sane TOD_ value

    int    N4_  = 0;    //!< Four-year interval, 1=1996..1999, 2=2000..2003, ... (SANE_N4_MIN - SANE_N4_MAX)
    int    Nt_  = 0;    //!< Day in four-year interval (SANE_NT_MIN - SANE_NT_MAX)
    double TOD_ = 0.0;  //!< Time of day (in Москва time zone) [s] (SANE_TOD_MIN - SANE_TOD_MAX)
    // clang-format on
};

/**
 * @brief UTC time representation
 */
struct UtcTime
{
    UtcTime() = default;  //!< Ctor
    /**

     * @brief Constructor
     *
     * @param[in]   year   Year
     * @param[in]   month  Month (1-12)
     * @param[in]   day    Day (1-31)
     * @param[in]   hour   Hour (0-23)
     * @param[in]   min    Minute (0-59)
     * @param[in]   sec    Second (0-60)
     */
    UtcTime(const int year, const int month, const int day, const int hour, const int min, const double sec);

    // clang-format off
    int    year_  = 0;    //!< Year
    int    month_ = 0;    //!< Month (1-12)
    int    day_   = 0;    //!< Day (1-31)
    int    hour_  = 0;    //!< Hour (0-23)
    int    min_   = 0;    //!< Minute (0-59)
    double sec_   = 0.0;  //!< Second (0-60)
    // clang-format on
};

// clang-format off
/**
 * @brief Time
 *
 * This class implements conversion from and to different time systems and time artithmetics. Internally it uses an
 * *atomic* time in seconds and nanoseconds since 1970 as its representation of time. Conversion from and to UTC time or
 * POSIX time are available. No timezones or local time are supported (use the ctime API for that).
 *
 * Some of the constructors and operators can throw. Non-throwing methods to manipulate the time are provided as well.
 *
 * This time object can represent the time from T_MIN to T_MAX, which is (the old) 32bit POSIX time. Note that for the
 * different GNSS times you will get negative week numbers (GPS, Galileo, BeiDou) or a negative offset "M4" value
 * (GLONASS) for timestamps before the beginning of respective timescales.
 *
 *                           ---1970-------------1980-------1996-1999----2006--------------------2106---//--fuuuuuture----> time
 *
 *     1970-01-01 00:00:00.0 UTC |<----- std::time_t ---------------------------------------------------//----->|
 *                               |<----- ros::Time/rclcpp::Time----------------------------------->|
 *     1980-01-06 00:00:00.0 UTC                  |<---- GPS time---------------------------------------//----->
 *     1996-01-01 00:00:00.0 UTC(SU)                         |<---- GLONASS time -----------------------//----->
 *     1999-08-21 23:59:47.0 UTC                                  |<---- Galileo time ------------------//----->
 *     2006-01-01 00:00:00.0 UTC                                          |<---- BeiDou time -----------//----->
 *
 *                               |<======================= Time object ===========================>|
 *                             T_MIN                                                             T_MAX
 *                  1970-01-01 00:00:0.0 UTC                                   2106-02-06 06:28:16.0 UTC (approximately!)
 *
 * Some notes:
 *
 * - "Strict" POSIX time is used for the methods marked with "POSIX". This time has a discontinuity and/or ambiguity
 *   at/during leapsecond events. It does not use the "Mills" or NTP style of handling these periods and is therefore,
 *   for those timestamps, likely not compatible with the system time (CLOCK_REALTIME) of a typical Linux system. See
 *   references below.
 * - The from/to UTC calculations are not fully correct for times before 1972.
 * - FromClockTai() and SetClockTai() likely will not give the expected result unless your system (Linux kernel) is
 *   configured for the correct leapsecond.
 * - The precision of all integer getters, setters and operators should be [ns] in all cases
 * - The precision of all double getters, setters and operators should be [ns] in most cases
 * - The internal representation of time (the sec_ value) is atomic, but it is not CLOCK_TAI (which already "has" 10
 *   leapseconds at sec_ = 0).
 *
 * Some references:
 *
 * - https://en.wikipedia.org/wiki/Unix_time
 * - https://en.wikipedia.org/wiki/Atomic_clock
 * - https://en.wikipedia.org/wiki/Coordinated_Universal_Time
 * - https://docs.ntpsec.org/latest/leapsmear.html
 * - https://www.eecis.udel.edu/~mills/leap.html
 * - https://manpages.org/adjtimex/2
 *
 */
// clang-format on
class Time
{
   public:
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Make Time object
     *
     * @note These throw std::runtime_error if the time (the arguments) is out of range
     * @{
     */

    /**
     * @brief Constructor, empty (invalid) time
     */
    Time();

    /**
     * @brief Time from seconds and nanoseconds (atomic)
     *
     * @param[in]  sec   Time value seconds
     * @param[in]  nsec  Time value nanoseconds
     *
     * @returns the Time object
     */
    static Time FromSecNSec(const uint32_t sec, const uint32_t nsec);

    /**
     * @brief Time from nanoseconds (atomic)
     *
     * @param[in]  nsec  Time value nanoseconds (> 0)
     *
     * @returns the Time object
     */
    static Time FromNSec(const uint64_t nsec);

    /**
     * @brief From seconds (atomic)
     *
     * @param[in]  sec  Time value seconds (> 0.0)
     *
     * @returns the Time object
     */
    static Time FromSec(const double sec);

    /**
     * @brief From POSIX time (POSIX, seconds)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  posix  Time value seconds (> 0)
     *
     * @returns the Time object
     */
    static Time FromPosix(const std::time_t posix);

    /**
     * @brief From POSIX seconds (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  posix_sec  Time value seconds (> 0.0)
     *
     * @returns the Time object
     */
    static Time FromPosixSec(const double posix_sec);

    /**
     * @brief From POSIX nanoseconds (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  posix_ns  Time value nanoseconds (> 0)
     *
     * @returns the Time object
     */
    static Time FromPosixNs(const uint64_t posix_ns);

    /**
     * @brief From ROS time (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  rostime  Time value
     *
     * @returns the Time object
     */
    static Time FromRosTime(const RosTime& rostime);

    /**
     * @brief From GNSS time (atomic)
     *
     * @param[in]  wnotow  GNSS time
     *
     * @returns the Time object
     */
    static Time FromWnoTow(const WnoTow& wnotow);

    /**
     * @brief From GLONASS time (UTC + 3h)
     *
     * @param[in]  glotime  GLONASS time
     *
     * @returns the Time object
     */
    static Time FromGloTime(const GloTime& glotime);

    /**
     * @brief From UTC time (UTC)
     *
     * @param[in]  utctime  UTC time
     *
     * @returns the Time object
     */
    static Time FromUtcTime(const UtcTime& utctime);

    /**
     * @brief From TAI time (CLOCK_TAI)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    static Time FromTai(const std::time_t tai);

    /**
     * @brief From TAI seconds (CLOCK_TAI)
     *
     * @param[in]  tai_sec  Time value seconds (> 0.0)
     *
     * @returns the Time object
     */
    static Time FromTaiSec(const double tai_sec);

    /**
     * @brief From TAI nanoseconds (CLOCK_TAI)
     *
     * @param[in]  tai_ns  Time value nanoseconds (> 0)
     *
     * @returns the Time object
     */
    static Time FromTaiNs(const uint64_t tai_ns);

    /**
     * @brief From system clock current (now) system time (CLOCK_REALTIME)
     *
     * @returns the Time object
     */
    static Time FromClockRealtime();

#ifdef CLOCK_TAI
    /**
     * @brief From system clock current (now) atomic time (CLOCK_TAI)
     *
     * @note This will only produce the right result if your system is configured accordingly (which may not be the
     *       case). See comments in the Time class description.
     *
     * @returns the Time object
     */
    static Time FromClockTai();
#endif

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Set time
     *
     * These do not throw and instead return true/false.
     * @{
     */

    /**
     * @brief Set time from seconds and nanoseconds (atomic)
     *
     * @param[in]  sec   Time value seconds
     * @param[in]  nsec  Time value nanoseconds
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetSecNSec(const uint32_t sec, const uint32_t nsec);

    /**
     * @brief Set time from nanoseconds (atomic)
     *
     * @param[in]  nsec   Time value nanoseconds (> 0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetNSec(const uint64_t nsec);

    /**
     * @brief Set time from seconds (atomic)
     *
     * @param[in]  sec  Time value seconds (> 0.0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetSec(const double sec);

    /**
     * @brief Set time from POSIX time (POSIX, seconds)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  posix  Time value seconds (> 0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetPosix(const std::time_t posix);

    /**
     * @brief Set time from POSIX seconds (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  posix_sec  Time value seconds (> 0.0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetPosixSec(const double posix_sec);

    /**
     * @brief Set time from POSIX nanoseconds (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  posix_ns  Time value nanoseconds (> 0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetPosixNs(const uint64_t posix_ns);

    /**
     * @brief Set time from ROS time (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  rostime  Time value
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetRosTime(const RosTime& rostime);

    /**
     * @brief Set time from GNSS (GPS, Galileo, BeiDou) time (atomic)
     *
     * @note Input values are normalised, so tow < SANE_TOW_MIN, tow > SANE_TOW_MAX, and wno < SANE_WNO_MIN are
     *       acceptable. Compare GetWnoTow().
     *
     * @param[in]  wnotow  GNSS time
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetWnoTow(const WnoTow& wnotow);

    /**
     * @brief Set time from GLONASS time (UTC + 3h)
     *
     * @note Input values will be normalised, so e.g. Nt_ < SANE_NT_MIN or N4_ > SANE_N4_MAX, ... are acceptable.
     *       Compare GetGloTime().
     *
     * @param[in]  glotime  GLONASS time
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetGloTime(const GloTime& glotime);

    /**
     * @brief Set time from UTC time (UTC)
     *
     * @param[in]  utctime  UTC time
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetUtcTime(const UtcTime& utctime);

    /**
     * @brief Set time from TAI time (CLOCK_TAI)
     *
     * @param[in]  tai  The TAI time
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetTai(const std::time_t tai);

    /**
     * @brief Set time from TAI seconds (CLOCK_TAI)
     *
     * @param[in]  tai_sec  Time value seconds (> 0.0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetTaiSec(const double tai_sec);

    /**
     * @brief Set time from TAI nanoseconds (CLOCK_TAI)
     *
     * @param[in]  tai_ns  Time value nanoseconds (> 0)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetTaiNs(const uint64_t tai_ns);

    /**
     * @brief Set time from system clock current (now) system time (CLOCK_REALTIME)
     *
     * @returns true if successful, false otherwise (bad time)
     */
    bool SetClockRealtime();

#ifdef CLOCK_TAI
    /**
     * @brief Set time from system clock current (now) TAI time (CLOCK_TAI)
     *
     * @note This will only produce the right result if your system is configured accordingly (which is unlikely). See
     *       comments in the Time class description.
     *
     * @returns the Time object
     */

    bool SetClockTai();
#endif

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Get time
     * @{
     */

    /**
     * @brief Get time as nanoseconds (atomic)
     *
     * @returns the time as nanoseconds
     */
    uint64_t GetNSec() const;

    /**
     * @brief Get time as seconds (atomic)
     *
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the time as seconds
     */
    double GetSec(const int prec = 9) const;

    /**
     * @brief Get time as POSIX time (POSIX, seconds)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @returns the POSIX time, truncated (rounded down, sub-seconds ignored)
     */
    std::time_t GetPosix() const;

    /**
     * @brief Get time as POSIX seconds (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the POSIX time in seconds
     */
    double GetPosixSec(const int prec = 9) const;

    /**
     * @brief Get time as POSIX nanoseconds (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @returns the POSIX time in nanoseconds
     */
    uint64_t GetPosixNs() const;

    /**
     * @brief Get time as ROS time (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @returns the time
     */
    RosTime GetRosTime() const;

    /**
     * @brief Get time as GNSS (GPS, Galileo, BeiDou) time (atomic)
     *
     * @note For times before the respective GNSS epoch the result is not usable (e.g. wno_ may be < SANE_WNO_MIN).
     *       Output tow_ is guaranteed to be within sane range (i.e. SANE_TOW_MIN <= tow_ < SANE_TOW_MAX). Compare
     *       SetWnoTow()
     *
     * @param[in]  sys   GNSS
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the GNSS time for the selected GNSS
     */
    WnoTow GetWnoTow(const WnoTow::Sys sys = WnoTow::Sys::GPS, const int prec = 9) const;

    /**
     * @brief Get time as GLONASS time (UTC + 3h)
     *
     * @note For times before the respective GLONASS epoch the result is not usable (e.g. N4_ may be < SANE_N4_MIN).
     *       Output Nt_ and tod_ are guaranteed to be within sane range (e.g. SANE_NT_MIN <= Nt_ < SANE_NT_MAX). Compare
     *       SetGloTime()
     *
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the GLONASS time
     */
    GloTime GetGloTime(const int prec = 9) const;

    /**
     * @brief Get time as UTC time (UTC)
     *
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the UTC time
     */
    UtcTime GetUtcTime(const int prec = 9) const;

    /**
     * @brief Get day of year
     *
     * @param[in]  prec  Round to this many fractional digits (0-12)
     *
     * @returns the day of the year
     */
    double GetDayOfYear(const int prec = 12) const;

    /**
     * @brief Get time as TAI time (CLOCK_TAI)
     *
     * @returns the TAI time, truncated (rounded down, sub-seconds ignored)
     */
    std::time_t GetTai() const;

    /**
     * @brief Get time as TAI seconds (CLOCK_TAI)
     *
     * @param[in]  prec  Round the seconds to this many fractional digits (0-9)
     *
     * @returns the TAI time in seconds
     */
    double GetTaiSec(const int prec = 9) const;

    /**
     * @brief Get time as TAI nanoseconds (CLOCK_TAI)
     *
     * @returns the TAI time in nanoseconds
     */
    uint64_t GetTaiNs() const;

    /**
     * @brief Get time as std::chrono::duration (milliseconds since epoch) (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @returns the time as std::chrono::milliseconds
     */
    std::chrono::milliseconds GetChronoMilli() const;

    /**
     * @brief Get time as std::chrono::nanoseconds (nanoseconds since epoch) (POSIX)
     *
     * @note See comments in the class description regarding POSIX time!
     *
     * @returns the time as std::chrono::nanoseconds
     */
    std::chrono::nanoseconds GetChronoNano() const;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Misc
     * @{
     */

    /**
     * @brief Check if time is zero (invalid)
     *
     * @returns true if time is exactly zero
     */
    bool IsZero() const;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Arithmetic methods
     *
     * These do not throw and instead return true/false.
     * @{
     */

    /**
     * @brief Add duration to time
     *
     * @param[in]  dur  Duration to add
     *
     * @returns true if successful (dur value in range), false otherwise (bad sec value)
     */
    bool AddDur(const Duration& dur);

    /**
     * @brief Add seconds to time
     *
     * @param[in]  sec  Seconds to add
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool AddSec(const double sec);

    /**
     * @brief Add nanoseconds to time
     *
     * @param[in]  nsec  Nanoseconds to add
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool AddNSec(const int64_t nsec);

    /**
     * @brief Substract duration from time
     *
     * @param[in]  dur  Duration to substract
     *
     * @returns true if successful (dur value in range), false otherwise (bad sec value)
     */
    bool SubDur(const Duration& dur);

    /**
     * @brief Substract nanoseconds from time
     *
     * @param[in]  nsec  Nanoseconds to substract
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool SubNSec(const int64_t nsec);

    /**
     * @brief Substract seconds from time
     *
     * @param[in]  sec  Seconds to substract
     *
     * @returns true if successful (sec value in range), false otherwise (bad sec value)
     */
    bool SubSec(const double sec);

    /**
     * @brief Calculate difference between times
     *
     * @param[in]   other  The other time
     * @param[out]  diff   The difference to the other time (time - other)
     *
     * @returns true if the times and the difference are within range, false otherwise
     */
    bool Diff(const Time& other, Duration& diff) const;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Arithmetic operators
     *
     * @note These throw std::runtime_error if the arguments (the time) are out of range.
     * @{
     */

    Time operator+(const Duration& rhs) const;  //!< Sum time and duration
    Time operator+(const int64_t nsec) const;   //!< Sum time and nanoseconds
    Time operator+(const double sec) const;     //!< Sum time and seconds
    Time& operator+=(const Duration& rhs);      //!< Add duration to time
    Time& operator+=(const int64_t nsec);       //!< Add nanoseconds to time
    Time& operator+=(const double sec);         //!< Add seconds to time
    Time operator-(const Duration& rhs) const;  //!< Subtract time and duration
    Time operator-(const int64_t nsec) const;   //!< Subtract time and nanoseconds
    Time operator-(const double sec) const;     //!< Subtract time and seconds
    Time& operator-=(const Duration& rhs);      //!< Subtract duration from time
    Time& operator-=(const int64_t nsec);       //!< Subtract nanoseconds from time
    Time& operator-=(const double sec);         //!< Subtract seconds from time
    Duration operator-(const Time& rhs) const;  //!< Subtract time and time

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Logic operators
     * @{
     */

    bool operator==(const Time& rhs) const;  //!< Equal
    bool operator!=(const Time& rhs) const;  //!< Not equal
    bool operator>(const Time& rhs) const;   //!< Greater than
    bool operator<(const Time& rhs) const;   //!< Smaller than
    bool operator>=(const Time& rhs) const;  //!< Greater or equal than
    bool operator<=(const Time& rhs) const;  //!< Smaller or equal than

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Stringification
     *
     * See also fpsdk::common::string::Strftime().
     *
     * @{
     */

    /**
     * @brief Stringify as GNSS time (atomic)
     *
     * @param[in]  sys   The desired GNSS time system
     * @param[in]  prec  Number of fractional digits for the seconds (0-9)
     *
     * @returns a string with formatted week-number and time-of-week ("wwww:tttttt.ttt")
     */
    std::string StrWnoTow(const WnoTow::Sys sys = WnoTow::Sys::GPS, const int prec = 3) const;

    /**
     * @brief Stringify as year, month, day, hour, minute and second time (UTC)
     *
     * @param[in]  prec  Number of fractional digits for the seconds (0-9)
     *
     * @returns a string with formatted UTC time ("yyyy-mm-dd hh:mm:ss.sss")
     */
    std::string StrUtcTime(const int prec = 3) const;

    /**
     * @brief Stringify as ISO 8601 time (UTC)
     *
     * @param[in]  prec  Number of fractional digits for the seconds (0-9)
     *
     * @returns a string with formatted UTC time ("yyyy-dd-mmThh:mm:ssZ")
     */
    std::string StrIsoTime(const int prec = 0) const;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------
    /**
     * @name Leapseconds
     *
     * @{
     */

    /**
     * @brief Set or change current leapseconds
     *
     * This extends the build-in leapseconds table with the current value. The given value is assumed to be the
     * leapseconds value (TAI - UTC) that is valid from the object's time (truncated to integer seconds). The object's
     * time must be past (later than) and the value must be greater or equal than the information in the latest entry in
     * the built-in table.
     *
     * The built-in table and the "current leapseconds" information are global per process. Setting or changing the
     * current leapseconds value affects all existing and future Time objects immediately.
     *
     * For example, the latest entry in the built-in table may be 2017-01-01 00:00:00 UTC with TAI-UTC = 37. So it could
     * be updated with a time of 2035-06-01 00:00:00 UTC and a value of 38.
     *
     * The idea is to use this in real-time applications where an upcoming leapseconds event can be learned from on-line
     * data, such as GNSS navigation data. Calling this method at and appropriate time with appropriate arguments is up
     * to the application. For example, if multiple future leapseconds events are known, the right event must be
     * "activated" at the right time.
     *
     * @param[in]  value  The current leapseconds value
     *
     * @returns true if the objects time and the leapseconds value were acceptable and set, false otherwise
     */
    bool SetCurrentLeapseconds(const int value) const;

    /**
     * @brief Get leapseconds (TAI - UTC)
     *
     * @returns the leapseconds for the time
     */
    int GetLeapseconds() const;

    /**
     * @brief Offset between GPS leapseconds and TAI-UTC leapseconds
     */
    static constexpr int GPS_LEAPSECONDS_OFFS = 19;

    ///@}
    // -----------------------------------------------------------------------------------------------------------------

    static const Time MIN;   //!< Minimum representable time
    static const Time MAX;   //!< Maximum representable time
    static const Time ZERO;  //!< Zero (invalid, uninitialised) time

    // Storage deliberately public
    uint32_t sec_;   //!< Seconds part of time (atomic seconds since 1970-01-01 00:00:00 UTC)
    uint32_t nsec_;  //!< Nanoseconds part of time (*should* be in range 0-999999999)
};

/* ****************************************************************************************************************** */
}  // namespace time
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_TIME_HPP__
