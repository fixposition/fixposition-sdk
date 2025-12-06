/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Parts copyright (c) 2008, Willow Garage, Inc., look for "[ros-code]" below
 * Written by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Time utilities
 */

/* LIBC/STL */
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <stdexcept>
#include <tuple>

/* EXTERNAL */
#include <sys/stat.h>
#include <unistd.h>

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/math.hpp"
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/time.hpp"

namespace fpsdk {
namespace common {
namespace time {
/* ****************************************************************************************************************** */

uint64_t GetMillis()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    static uint64_t t0 = 0;
    uint64_t t = (tp.tv_sec * 1000) + (tp.tv_nsec / 1000000);
    if (t0 == 0) {
        t0 = t - 1;
        return 1;
    }
    return t - t0;
}

// ---------------------------------------------------------------------------------------------------------------------

double GetSecs()
{
    return (double)GetMillis() * 1e-3;
}

/* ****************************************************************************************************************** */
#if 1

TicToc::TicToc()
{
    Tic();
}

void TicToc::Tic()
{
    t0_ = std::chrono::steady_clock::now();
}

Duration TicToc::Toc(const bool reset)
{
    const std::chrono::time_point<std::chrono::steady_clock> t1 = std::chrono::steady_clock::now();
    const std::uint64_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0_).count();
    Duration dur;
    dur.SetNSec(nsec);
    if (reset) {
        t0_ = t1;
    }
    return dur;
}

double TicToc::TocMs(const bool reset)
{
    return Toc(reset).GetSec() * 1e-3;
}

#endif
/* ****************************************************************************************************************** */
#if 1

RosTime::RosTime() : sec_{ 0 }, nsec_{ 0 }
{
}

RosTime::RosTime(const uint32_t sec, const uint32_t nsec) : sec_{ sec }, nsec_{ nsec }
{
    while (nsec_ > 999999999) {
        nsec_ -= 1000000000;
        sec_ += 1;
    }
}

double RosTime::ToSec() const
{
    return (double)sec_ + ((double)nsec_ * 1e-9);
}

uint64_t RosTime::ToNSec() const
{
    return (static_cast<uint64_t>(sec_) * (uint64_t)1000000000) + static_cast<uint64_t>(nsec_);
}

bool RosTime::IsZero() const
{
    return (sec_ == 0) && (nsec_ == 0);
}

bool RosTime::operator==(const RosTime& rhs) const
{
    return (sec_ == rhs.sec_) && (nsec_ == rhs.nsec_);
}

static_assert(sizeof(RosTime) == (sizeof(uint32_t) + sizeof(uint32_t)), "");
static_assert(offsetof(RosTime, sec_) == 0, "");
static_assert(offsetof(RosTime, nsec_) == sizeof(uint32_t), "");

#endif
/* ****************************************************************************************************************** */
#if 1

#  define TIME_TRACE(...) /* nothing */
// #  define TIME_TRACE(...) TRACE(__VA_ARGS__)

// based on [ros-code]: roscpp_core/rostime
static bool durationNormalizeSecNSecSigned(int64_t& sec, int64_t& nsec)
{
    int64_t nsec_part = nsec % (int64_t)1000000000;
    int64_t sec_part = sec + (nsec / (int64_t)1000000000);
    if (nsec_part < 0) {
        nsec_part += (int64_t)1000000000;
        sec_part--;
    }

    if ((sec_part < std::numeric_limits<int32_t>::min()) || (sec_part > std::numeric_limits<int32_t>::max())) {
        return false;
    }

    sec = sec_part;
    nsec = nsec_part;

    return true;
}

// based on [ros-code]: roscpp_core/rostime
static bool durationNormalizeSecNSecSigned(int32_t& sec, int32_t& nsec)
{
    int64_t sec64 = sec;
    int64_t nsec64 = nsec;

    if (!durationNormalizeSecNSecSigned(sec64, nsec64)) {
        return false;
    }

    sec = static_cast<int32_t>(sec64);
    nsec = static_cast<int32_t>(nsec64);

    return true;
}

// based on [ros-code]: roscpp_core/rostime
static bool durationSecToIsecNsec(const double sec, int32_t& isec, int32_t& nsec)
{
    if (!std::isfinite(sec) || (sec <= static_cast<double>(std::numeric_limits<int64_t>::min())) ||
        (sec >= static_cast<double>(std::numeric_limits<int64_t>::max()))) {
        return false;
    }

    int64_t sec64 = static_cast<int64_t>(std::floor(sec));

    if ((sec64 < std::numeric_limits<int32_t>::min()) || (sec64 > std::numeric_limits<int32_t>::max())) {
        return false;
    }

    isec = static_cast<int32_t>(sec64);
    nsec = static_cast<int32_t>(std::round((sec - isec) * 1e9));
    int32_t rollover = nsec / (int32_t)1000000000;
    isec += rollover;
    nsec %= (int32_t)1000000000;
    return true;
}

static constexpr const char* DURATION_THROW_MSG = "Duration out of range";

// ---------------------------------------------------------------------------------------------------------------------

Duration::Duration() : sec_{ 0 }, nsec_{ 0 }
{
    // We want to be binary compatible with ros::Duration
    static_assert(sizeof(Duration) == (sizeof(int32_t) + sizeof(int32_t)), "");
    static_assert(offsetof(Duration, sec_) == 0, "");
    static_assert(offsetof(Duration, nsec_) == sizeof(int32_t), "");
}

/*static*/ Duration Duration::FromSecNSec(const int32_t sec, const int32_t nsec)
{
    Duration dur;
    if (!dur.SetSecNSec(sec, nsec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

/*static*/ Duration Duration::FromNSec(const int64_t nsec)
{
    Duration dur;
    if (!dur.SetNSec(nsec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

/*static*/ Duration Duration::FromSec(const double sec)
{
    Duration dur;
    if (!dur.SetSec(sec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Duration::SetSecNSec(const int32_t sec, const int32_t nsec)
{
    int32_t maybe_sec = sec;
    int32_t maybe_nsec = nsec;

    if (!durationNormalizeSecNSecSigned(maybe_sec, maybe_nsec)) {
        return false;
    }

    sec_ = maybe_sec;
    nsec_ = maybe_nsec;

    return true;
}

bool Duration::SetNSec(const int64_t nsec)
{
    const int64_t sec64 = nsec / (int64_t)1000000000;
    if ((sec64 < std::numeric_limits<int32_t>::min()) || (sec64 > std::numeric_limits<int32_t>::max())) {
        return false;
    }

    int32_t maybe_sec = static_cast<int32_t>(sec64);
    int32_t maybe_nsec = static_cast<int32_t>(nsec % (int64_t)1000000000);

    if (!durationNormalizeSecNSecSigned(maybe_sec, maybe_nsec)) {
        return false;
    }

    sec_ = maybe_sec;
    nsec_ = maybe_nsec;

    return true;
}

bool Duration::SetSec(const double sec)
{
    return durationSecToIsecNsec(sec, sec_, nsec_);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Duration::IsZero() const
{
    return (sec_ == 0) && (nsec_ == 0);
}

// ---------------------------------------------------------------------------------------------------------------------

int64_t Duration::GetNSec() const
{
    return (static_cast<int64_t>(sec_) * (int64_t)1000000000) + static_cast<int64_t>(nsec_);
}

double Duration::GetSec(const int prec) const
{
    return math::RoundToFracDigits(
        static_cast<double>(sec_) + (1e-9 * static_cast<double>(nsec_)), std::clamp(prec, 0, 9));
}

std::chrono::milliseconds Duration::GetChronoMilli() const
{
    const int64_t nsec = GetNSec();
    return std::chrono::milliseconds(
        (nsec < 0 ? (nsec - (int64_t)500000) : (nsec + (int64_t)500000)) / (int64_t)1000000);
}

std::chrono::nanoseconds Duration::GetChronoNano() const
{
    return std::chrono::nanoseconds(GetNSec());
}

// ---------------------------------------------------------------------------------------------------------------------

bool Duration::AddDur(const Duration& dur)
{
    int64_t maybe_sec = static_cast<int64_t>(sec_ + dur.sec_);
    int64_t maybe_nsec = static_cast<int64_t>(nsec_ + dur.nsec_);
    if (!durationNormalizeSecNSecSigned(maybe_sec, maybe_nsec)) {
        return false;
    }
    sec_ = maybe_sec;
    nsec_ = maybe_nsec;
    return true;
}

bool Duration::AddNSec(const int64_t nsec)
{
    const int64_t cur = GetNSec();
    if ((nsec > 0) && (cur > 0)) {
        const int64_t rem = std::numeric_limits<int64_t>::max() - cur;
        TIME_TRACE("AddNSec 1 %" PRIi64 " %" PRIi64 " %" PRIi64, cur, nsec, rem);
        if (nsec > rem) {
            return false;
        }
    } else if ((nsec < 0) && (cur < 0)) {
        const int64_t rem = std::numeric_limits<int64_t>::min() - cur;
        TIME_TRACE("AddNSec 2 %" PRIi64 " %" PRIi64 " %" PRIi64, cur, nsec, rem);
        if (nsec < rem) {
            return false;
        }
    } else {
        TIME_TRACE("AddNSec 3 %" PRIi64 " %" PRIi64, cur, nsec);
    }
    return SetNSec(cur + nsec);
}

bool Duration::AddSec(const double sec)
{
    int32_t isec;
    int32_t nsec;
    if (!durationSecToIsecNsec(sec, isec, nsec)) {
        return false;
    }
    int64_t maybe_sec = static_cast<int64_t>(sec_ + isec);
    int64_t maybe_nsec = static_cast<int64_t>(nsec_ + nsec);
    if (!durationNormalizeSecNSecSigned(maybe_sec, maybe_nsec)) {
        return false;
    }
    sec_ = maybe_sec;
    nsec_ = maybe_nsec;
    TIME_TRACE("AddSec %" PRIi32 " %" PRIi32 " %.9f -> %" PRIi32 " %" PRIi32, sec_, nsec_, sec, isec, nsec);
    return true;
}

Duration Duration::operator+(const Duration& rhs) const
{
    Duration dur = *this;
    if (!dur.AddDur(rhs)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration Duration::operator+(const int64_t nsec) const
{
    Duration dur = *this;
    if (!dur.AddNSec(nsec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration Duration::operator+(const double sec) const
{
    Duration dur = *this;
    if (!dur.AddSec(sec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration& Duration::operator+=(const Duration& rhs)
{
    *this = *this + rhs;
    return *static_cast<Duration*>(this);
}

Duration& Duration::operator+=(const int64_t nsec)
{
    *this = *this + nsec;
    return *static_cast<Duration*>(this);
}

Duration& Duration::operator+=(const double sec)
{
    *this = *this + sec;
    return *static_cast<Duration*>(this);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Duration::SubDur(const Duration& dur)
{
    return AddDur(-dur);
}

bool Duration::SubNSec(const int64_t nsec)
{
    return AddNSec(-nsec);
}

bool Duration::SubSec(const double sec)
{
    return AddSec(-sec);
}

Duration Duration::operator-(const Duration& rhs) const
{
    Duration dur = *this;
    if (!dur.SubDur(rhs)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration Duration::operator-(const int64_t nsec) const
{
    Duration dur = *this;
    if (!dur.SubNSec(nsec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration Duration::operator-(const double sec) const
{
    Duration dur = *this;
    if (!dur.SubSec(sec)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration& Duration::operator-=(const Duration& rhs)
{
    *this = *this - rhs;
    return *static_cast<Duration*>(this);
}

Duration& Duration::operator-=(const int64_t nsec)
{
    *this = *this - nsec;
    return *static_cast<Duration*>(this);
}

Duration& Duration::operator-=(const double sec)
{
    *this = *this - sec;
    return *static_cast<Duration*>(this);
}

Duration Duration::operator-() const
{
    Duration t;
    if (!t.SetNSec(-GetNSec())) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return t;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Duration::Scale(const double sec)
{
    const double new_sec = GetSec() * sec;
    if (!SetSec(new_sec)) {
        return false;
    }
    return true;
}

Duration Duration::operator*(double scale) const
{
    Duration dur = *this;
    if (!dur.Scale(scale)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return dur;
}

Duration& Duration::operator*=(double scale)
{
    if (!Scale(scale)) {
        throw std::runtime_error(DURATION_THROW_MSG);
    }
    return *static_cast<Duration*>(this);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Duration::operator==(const Duration& rhs) const
{
    return (sec_ == rhs.sec_) && (nsec_ == rhs.nsec_);
}

bool Duration::operator!=(const Duration& rhs) const
{
    return (sec_ != rhs.sec_) || (nsec_ != rhs.nsec_);
}

bool Duration::operator>(const Duration& rhs) const
{
    if (sec_ > rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ > rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

bool Duration::operator<(const Duration& rhs) const
{
    if (sec_ < rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ < rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

bool Duration::operator>=(const Duration& rhs) const
{
    if (sec_ > rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ >= rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

bool Duration::operator<=(const Duration& rhs) const
{
    if (sec_ < rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ <= rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::string Duration::Stringify(const int prec) const
{
    const int n_frac = std::clamp(prec, 0, 9);

    std::string str;
    double sec = GetSec();
    if (sec < 0.0) {
        sec = -sec;
        str = "-";
    }

    // Round to desired number of fractional digits
    sec = math::RoundToFracDigits(sec, n_frac);

    if (sec > (SEC_IN_DAY_D - 1e-9)) {
        const double days = std::floor(sec / SEC_IN_DAY_D);
        str += string::Sprintf("%.0fd ", days);
        sec -= (days * SEC_IN_DAY_D);
    }

    const double hours = std::floor(sec / SEC_IN_HOUR_D);
    sec -= (hours * SEC_IN_HOUR_D);
    const double mins = std::floor(sec / SEC_IN_MIN_D);
    sec -= (mins * SEC_IN_MIN_D);

    str += string::Sprintf("%02.0f:%02.0f:%0*.*f", hours, mins, n_frac > 0 ? (n_frac + 3) : (n_frac + 2), n_frac, sec);

    return str;
}

// ---------------------------------------------------------------------------------------------------------------------

void Duration::Sleep() const
{
    if (sec_ < 0) {
        return;
    }
    timespec req = { sec_, nsec_ };
    timespec rem = { 0, 0 };
    while (nanosleep(&req, &rem)) {
        req = rem;
    }
}

#endif

/* ****************************************************************************************************************** */
#if 1

// based on [ros-code]: roscpp_core/rostime
static bool timeNormalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
    uint64_t nsec_part = nsec % (uint64_t)1000000000;
    uint64_t sec_part = nsec / (uint64_t)1000000000;

    if ((sec + sec_part) > std::numeric_limits<uint32_t>::max()) {
        return false;
    }

    sec += sec_part;
    nsec = nsec_part;
    return true;
}

// based on [ros-code]: roscpp_core/rostime
static bool timeNormalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
    uint64_t sec64 = sec;
    uint64_t nsec64 = nsec;

    if (!timeNormalizeSecNSec(sec64, nsec64)) {
        return false;
    }

    sec = static_cast<uint32_t>(sec64);
    nsec = static_cast<uint32_t>(nsec64);
    return true;
}

// based on [ros-code]: roscpp_core/rostime
static bool timeNormalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec)
{
    int64_t nsec_part = nsec % (int64_t)1000000000;
    int64_t sec_part = sec + (nsec / (int64_t)1000000000);
    if (nsec_part < 0) {
        nsec_part += (int64_t)1000000000;
        sec_part--;
    }

    if ((sec_part < 0) || (sec_part > std::numeric_limits<uint32_t>::max())) {
        return false;
    }

    sec = sec_part;
    nsec = nsec_part;
    return true;
}

// based on [ros-code]: roscpp_core/rostime
static bool timeSecToIsecNsec(const double sec, uint32_t& isec, uint32_t& nsec)
{
    if (!std::isfinite(sec) || (sec < 0.0) || (sec >= static_cast<double>(std::numeric_limits<int64_t>::max()))) {
        return false;
    }

    int64_t sec64 = static_cast<int64_t>(std::floor(sec));
    if (sec64 > std::numeric_limits<uint32_t>::max()) {
        return false;
    }

    isec = static_cast<uint32_t>(sec64);
    nsec = static_cast<uint32_t>(std::round((sec - isec) * 1e9));
    isec += (nsec / (uint32_t)1000000000);
    nsec %= (uint32_t)1000000000;
    return true;
}

// Public domain from: http://howardhinnant.github.io/date_algorithms.html#civil_from_days
// Returns year/month/day triple in civil calendar
// Preconditions:  z is number of days since 1970-01-01 and is in the range:
//                   [numeric_limits<Int>::min(), numeric_limits<Int>::max()-719468].
template <class Int>
constexpr std::tuple<Int, unsigned, unsigned> civil_from_days(Int z) noexcept
{
    static_assert(
        std::numeric_limits<unsigned>::digits >= 18, "This algorithm has not been ported to a 16 bit unsigned integer");
    static_assert(
        std::numeric_limits<Int>::digits >= 20, "This algorithm has not been ported to a 16 bit signed integer");
    z += 719468;
    const Int era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);                // [0, 146096]
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;  // [0, 399]
    const Int y = static_cast<Int>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);  // [0, 365]
    const unsigned mp = (5 * doy + 2) / 153;                       // [0, 11]
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;               // [1, 31]
    const unsigned m = mp < 10 ? mp + 3 : mp - 9;                  // [1, 12]
    return std::tuple<Int, unsigned, unsigned>(y + (m <= 2), m, d);
}

// Public domain from: http://howardhinnant.github.io/date_algorithms.html#days_from_civil
// Returns number of days since civil 1970-01-01.  Negative values indicate
//    days prior to 1970-01-01.
// Preconditions:  y-m-d represents a date in the civil (Gregorian) calendar
//                 m is in [1, 12]
//                 d is in [1, last_day_of_month(y, m)]
//                 y is "approximately" in
//                   [numeric_limits<Int>::min()/366, numeric_limits<Int>::max()/366]
//                 Exact range of validity is:
//                 [civil_from_days(numeric_limits<Int>::min()),
//                  civil_from_days(numeric_limits<Int>::max()-719468)]
template <class Int>
constexpr Int days_from_civil(Int y, unsigned m, unsigned d) noexcept
{
    static_assert(
        std::numeric_limits<unsigned>::digits >= 18, "This algorithm has not been ported to a 16 bit unsigned integer");
    static_assert(
        std::numeric_limits<Int>::digits >= 20, "This algorithm has not been ported to a 16 bit signed integer");
    y -= m <= 2;
    const Int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);             // [0, 399]
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;  // [0, 365]
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;            // [0, 146096]
    return era * 146097 + static_cast<Int>(doe) - 719468;
}

// ---------------------------------------------------------------------------------------------------------------------

struct LeapSecInfo
{
    LeapSecInfo(const int leapsec_value, const bool at_leapsec)
        : leapsec_value_{ leapsec_value }, at_leapsec_{ at_leapsec }
    {
    }
    int leapsec_value_;
    bool at_leapsec_;
};

// "Our" internal atomic timescale starts at the same time as POSIX. So at 1970-01-01 both are value 0. At this point
// TAI (CLOCK_TAI) has TAI_OFFS leap seconds.
static constexpr int TAI_OFFS = 10;  // offset of CLOCK_TAI to our atomic time

// TODO: consider making some CI that uses the IERS bulletins or https://data.iana.org/time-zones/data/leap-seconds.list
//       to check, and possibly update, this table.

// See IERS "Bulletin C" #70 July 2025 (https://hpiers.obspm.fr/iers/bul/bulc/bulletinc.dat)
// See also https://data.iana.org/time-zones/data/leap-seconds.list
// See also /usr/share/zoneinfo/{leapseconds,leap-seconds.list}
// The earliest time there can be a change is approximately:
static constexpr uint32_t max_ts = 1782820800;  // TZ=UTC date --date "2026-06-30 12:00:00" +%s
static constexpr uint32_t NUM_LEAPS = 27;
static constexpr std::array<std::array<uint32_t, 2>, NUM_LEAPS> LEAPSECONDS = { {
    // clang-format off
    // POSIX      Atomic
    { 1483228800, 1483228800 + 27 - 1 },  // TAI-UTC = 37 (GPS 18) 2017-01-01 (see IERS "Bulletin C" #52 January 2016)
    { 1435708800, 1435708800 + 26 - 1 },  // TAI-UTC = 36 (GPS 17) 2015-07-01
    { 1341100800, 1341100800 + 25 - 1 },  // TAI-UTC = 35 (GPS 16) 2012-07-01
    { 1230768000, 1230768000 + 24 - 1 },  // TAI-UTC = 34 (GPS 15) 2009-01-01
    { 1136073600, 1136073600 + 23 - 1 },  // TAI-UTC = 33 (GPS 14) 2006-01-01
    {  915148800,  915148800 + 22 - 1 },  // TAI-UTC = 32 (GPS 13) 1999-01-01
    {  867715200,  867715200 + 21 - 1 },  // TAI-UTC = 31 (GPS 12) 1997-07-01
    {  820454400,  820454400 + 20 - 1 },  // TAI-UTC = 30 (GPS 11) 1996-01-01
    {  773020800,  773020800 + 19 - 1 },  // TAI-UTC = 29 (GPS 10) 1994-07-01
    {  741484800,  741484800 + 18 - 1 },  // TAI-UTC = 28 (GPS  9) 1993-07-01
    {  709948800,  709948800 + 17 - 1 },  // TAI-UTC = 27 (GPS  8) 1992-07-01
    {  662688000,  662688000 + 16 - 1 },  // TAI-UTC = 26 (GPS  7) 1991-01-01
    {  631152000,  631152000 + 15 - 1 },  // TAI-UTC = 25 (GPS  6) 1990-01-01
    {  567993600,  567993600 + 14 - 1 },  // TAI-UTC = 24 (GPS  5) 1988-01-01
    {  489024000,  489024000 + 13 - 1 },  // TAI-UTC = 23 (GPS  4) 1985-07-01
    {  425865600,  425865600 + 12 - 1 },  // TAI-UTC = 22 (GPS  3) 1983-07-01
    {  394329600,  394329600 + 11 - 1 },  // TAI-UTC = 21 (GPS  2) 1982-07-01
    {  362793600,  362793600 + 10 - 1 },  // TAI-UTC = 20 (GPS  1) 1981-07-01
    {  315532800,  315532800 +  9 - 1 },  // TAI-UTC = 19 (GPS  0) 1980-01-01
    {  283996800,  283996800 +  8 - 1 },  // TAI-UTC = 18          1979-01-01
    {  252460800,  252460800 +  7 - 1 },  // TAI-UTC = 17          1978-01-01
    {  220924800,  220924800 +  6 - 1 },  // TAI-UTC = 16          1977-01-01
    {  189302400,  189302400 +  5 - 1 },  // TAI-UTC = 15          1976-01-01
    {  157766400,  157766400 +  4 - 1 },  // TAI-UTC = 14          1975-01-01
    {  126230400,  126230400 +  3 - 1 },  // TAI-UTC = 13          1974-01-01
    {   94694400,   94694400 +  2 - 1 },  // TAI-UTC = 12          1973-01-01
    {   78796800,   78796800 +  1 - 1 },  // TAI-UTC = 11          1972-07-01
    //         0           0                 TAI-UTC = 10          1970-01-01 --> TAI_OFFS
}};  // clang-format on

static uint32_t current_leapsec_ts = 0;
static int current_leapsec_value = 0;

LeapSecInfo getLeapSecInfo(const uint32_t ts, const bool posix)
{
    // We may have current leapsecond info
    if (current_leapsec_ts > 0) {
        const uint32_t check = ts + (posix ? (current_leapsec_value - 1) : 0);
        // TIME_TRACE("getLeapSecInfo %" PRIu32 " %" PRIu32 " %" PRIu32, ts, check, current_leapsec_ts);
        if (check >= current_leapsec_ts) {
            return { current_leapsec_value, check == current_leapsec_ts };
        }
    }

    if (ts > max_ts) {
        static bool you_have_been_warned = false;
        if (!you_have_been_warned) {
            WARNING("time::Time() Leapsecond knowledge outdated (at %" PRIu32 "), assuming unchanged since %" PRIu32,
                ts, max_ts);
            you_have_been_warned = true;
        }
        return getLeapSecInfo(max_ts, true);
    }

    int leapsec = LEAPSECONDS.size();
    for (auto entry : LEAPSECONDS) {
        const uint32_t ls_ts = entry[posix ? 0 : 1];
        if (ts == ls_ts) {
            return { leapsec, true };
        } else if (ts > ls_ts) {
            return { leapsec, false };
        }
        leapsec--;
    }
    return { 0, false };
}

bool Time::SetCurrentLeapseconds(const int value) const
{
    if ((sec_ <= LEAPSECONDS[0][0]) || (value < ((int)NUM_LEAPS + TAI_OFFS))) {
        TIME_TRACE("SetCurrentLeapseconds %" PRIu32 " <= %" PRIu32 " or %d < %d", sec_, LEAPSECONDS[0][0], value,
            (int)NUM_LEAPS + TAI_OFFS);
        return false;
    }

    current_leapsec_ts = sec_;
    current_leapsec_value = value - (int)TAI_OFFS;
    TIME_TRACE("SetCurrentLeapseconds %" PRIu32 " > %" PRIu32 " and %d >= %d", current_leapsec_ts, LEAPSECONDS[0][0],
        current_leapsec_value + (int)TAI_OFFS, (int)NUM_LEAPS + (int)TAI_OFFS);

    return true;
}

int Time::GetLeapseconds() const
{
    const LeapSecInfo lsinfo = getLeapSecInfo(sec_, false);
    return lsinfo.leapsec_value_ + TAI_OFFS;
}

// ---------------------------------------------------------------------------------------------------------------------

// GPS 0:0 in atomic sec, 1980-01-06 00:00:00.0 UTC
static constexpr uint32_t GPS_OFFS = 315532800 + (5 * SEC_IN_DAY_I) + 9;  // 315964809
static constexpr int32_t GPS_OFFS_WNO = GPS_OFFS / SEC_IN_WEEK_I;         // 522
static constexpr int32_t GPS_OFFS_TOW = GPS_OFFS % SEC_IN_WEEK_I;         // 259209

// GAL 0:0 in atomic sec, 1999-08-21 23:59:47.0 UTC
static constexpr uint32_t GAL_OFFS = GPS_OFFS + (1024 * SEC_IN_WEEK_I);  // 935280009
static constexpr int32_t GAL_OFFS_WNO = GAL_OFFS / SEC_IN_WEEK_I;        // 1546
static constexpr int32_t GAL_OFFS_TOW = GAL_OFFS % SEC_IN_WEEK_I;        // 259209

// BDS 0:0 in atomic sec, 2006-01-01 00:00:00.0 UTC
static constexpr uint32_t BDS_OFFS = GPS_OFFS + (1356 * SEC_IN_WEEK_I) + 14;  // 1136073623
static constexpr int32_t BDS_OFFS_WNO = BDS_OFFS / SEC_IN_WEEK_I;             // 1878
static constexpr int32_t BDS_OFFS_TOW = BDS_OFFS % SEC_IN_WEEK_I;             // 259223

// GLO 1:1:0.0, 1996-01-01 00:00:00.0 UTC in POSIX
static constexpr uint32_t GLO_OFFS_POSIX = 820454400;

static constexpr const char* TIME_THROW_MSG = "Time out of range";

// ---------------------------------------------------------------------------------------------------------------------

WnoTow::WnoTow(const Sys sys) : sys_{ sys }
{
}

WnoTow::WnoTow(const int wno, const double tow, const Sys sys) : wno_{ wno }, tow_{ tow }, sys_{ sys }
{
}

// ---------------------------------------------------------------------------------------------------------------------

GloTime::GloTime(const int N4, const int Nt, const double TOD) : N4_{ N4 }, Nt_{ Nt }, TOD_{ TOD }
{
}

// ---------------------------------------------------------------------------------------------------------------------

UtcTime::UtcTime(const int year, const int month, const int day, const int hour, const int min, const double sec)
    : year_{ year }, month_{ month }, day_{ day }, hour_{ hour }, min_{ min }, sec_{ sec }
{
}

// ---------------------------------------------------------------------------------------------------------------------

Time::Time() : sec_{ 0 }, nsec_{ 0 }
{
    // We want to guarantee the binary structure of the data
    static_assert(sizeof(Time) == (sizeof(uint32_t) + sizeof(uint32_t)), "");
    static_assert(offsetof(Time, sec_) == 0, "");
    static_assert(offsetof(Time, nsec_) == sizeof(uint32_t), "");
}

/*static*/ Time Time::FromSecNSec(const uint32_t sec, const uint32_t nsec)
{
    Time time;
    if (!time.SetSecNSec(sec, nsec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromNSec(const uint64_t nsec)
{
    Time time;
    if (!time.SetNSec(nsec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromSec(const double sec)
{
    Time time;
    if (!time.SetSec(sec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromPosix(const std::time_t posix)
{
    Time time;
    if (!time.SetPosix(posix)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromPosixSec(const double posix_sec)
{
    Time time;
    if (!time.SetPosixSec(posix_sec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromPosixNs(const uint64_t posix_ns)
{
    Time time;
    if (!time.SetPosixNs(posix_ns)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromRosTime(const RosTime& rostime)
{
    Time time;
    if (!time.SetRosTime(rostime)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromWnoTow(const WnoTow& wnotow)
{
    Time time;
    if (!time.SetWnoTow(wnotow)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromGloTime(const GloTime& glotime)
{
    Time time;
    if (!time.SetGloTime(glotime)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromUtcTime(const UtcTime& utctime)
{
    Time time;
    if (!time.SetUtcTime(utctime)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromTai(const std::time_t tai)
{
    Time time;
    if (!time.SetTai(tai)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromTaiSec(const double tai_sec)
{
    Time time;
    if (!time.SetTaiSec(tai_sec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromTaiNs(const uint64_t tai_ns)
{
    Time time;
    if (!time.SetTaiNs(tai_ns)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

/*static*/ Time Time::FromClockRealtime()
{
    Time time;
    if (!time.SetClockRealtime()) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

#  ifdef CLOCK_TAI
/*static*/ Time Time::FromClockTai()
{
    Time time;
    if (!time.SetClockTai()) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}
#  endif  // CLOCK_TAI

// ---------------------------------------------------------------------------------------------------------------------

bool Time::SetSecNSec(const uint32_t sec, const uint32_t nsec)
{
    uint32_t maybe_sec = sec;
    uint32_t maybe_nsec = nsec;
    if (!timeNormalizeSecNSec(maybe_sec, maybe_nsec)) {
        return false;
    }
    sec_ = maybe_sec;
    nsec_ = maybe_nsec;
    return true;
}

bool Time::SetNSec(const uint64_t nsec)
{
    uint64_t sec64 = 0;
    uint64_t nsec64 = nsec;
    if (!timeNormalizeSecNSec(sec64, nsec64)) {
        return false;
    }
    sec_ = static_cast<uint32_t>(sec64);
    nsec_ = static_cast<uint32_t>(nsec64);
    return true;
}

bool Time::SetSec(const double sec)
{
    return timeSecToIsecNsec(sec, sec_, nsec_);
}

bool Time::SetPosix(const std::time_t posix)
{
    if ((posix < 0) || (posix > (std::numeric_limits<int32_t>::max() - NUM_LEAPS))) {
        return false;
    }
    const auto lsinfo = getLeapSecInfo(posix, true);
    uint32_t sec = static_cast<uint32_t>(posix);
    sec += lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        sec -= 1;
    }
    sec_ = sec;
    nsec_ = 0;
    TIME_TRACE("SetPosix %" PRIiMAX " lsinfo %d %s -> %" PRIu32 " %" PRIu32, posix, lsinfo.leapsec_value_,
        string::ToStr(lsinfo.at_leapsec_), sec_, nsec_);
    return true;
}

bool Time::SetPosixSec(const double posix_sec)
{
    uint32_t isec = 0;
    uint32_t nsec = 0;
    if (!timeSecToIsecNsec(posix_sec, isec, nsec)) {
        return false;
    }
    const auto lsinfo = getLeapSecInfo(isec, true);
    isec += lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        isec -= 1;
    }
    sec_ = isec;
    nsec_ = nsec;
    return true;
}

bool Time::SetPosixNs(const uint64_t posix_ns)
{
    uint64_t sec64 = 0;
    uint64_t nsec64 = posix_ns;
    if (timeNormalizeSecNSec(sec64, nsec64) && SetPosix(static_cast<std::time_t>(sec64))) {
        nsec_ = nsec64;
        return true;
    } else {
        return false;
    }
}

bool Time::SetRosTime(const RosTime& rostime)
{
    if ((rostime.sec_ > (std::numeric_limits<uint32_t>::max() - NUM_LEAPS))) {
        return false;
    }
    const auto lsinfo = getLeapSecInfo(rostime.sec_, true);
    uint32_t sec = rostime.sec_;
    sec += lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        sec -= 1;
    }
    sec_ = sec;
    nsec_ = rostime.nsec_;
    TIME_TRACE("SetRosTime %" PRIu32 " %" PRIu32 " lsinfo %d %s -> %" PRIu32 " %" PRIu32, rostime.sec_, rostime.nsec_,
        lsinfo.leapsec_value_, string::ToStr(lsinfo.at_leapsec_), sec_, nsec_);
    return true;
}

bool Time::SetWnoTow(const WnoTow& wnotow)
{
    // @todo could probably use timeSecToIsecNsec()...
    double itow = 0.0;
    double ftow = std::modf(math::RoundToFracDigits(wnotow.tow_, 9), &itow);
    int tow = static_cast<int>(itow);
    int wno = wnotow.wno_;

    switch (wnotow.sys_) {
        case WnoTow::Sys::GPS:
            wno += GPS_OFFS_WNO;
            tow += GPS_OFFS_TOW;
            break;
        case WnoTow::Sys::GAL:
            wno += GAL_OFFS_WNO;
            tow += GAL_OFFS_TOW;
            break;
        case WnoTow::Sys::BDS:
            wno += BDS_OFFS_WNO;
            tow += BDS_OFFS_TOW;
            break;
    }
    int64_t sec = (wno * SEC_IN_WEEK_I) + tow;
    if ((sec < 0) || (sec > std::numeric_limits<uint32_t>::max())) {
        return false;
    }
    sec_ = static_cast<uint32_t>(sec);
    nsec_ = static_cast<uint32_t>(std::round(ftow * 1e9));
    TIME_TRACE("SetWnoTow %d %.9f %d -> %" PRIu32 " %" PRIu32, wnotow.wno_, wnotow.tow_, (int)wnotow.sys_, sec_, nsec_);
    return true;
}

bool Time::SetGloTime(const GloTime& glotime)
{
    // @todo could probably use timeSecToIsecNsec()...
    double itod = 0.0;
    double ftod = std::modf(math::RoundToFracDigits(glotime.TOD_, 9), &itod);

    int64_t sec = ((((glotime.N4_ - 1) * 1461) + (glotime.Nt_ - 1)) * SEC_IN_DAY_I) + static_cast<uint32_t>(itod) +
                  GLO_OFFS_POSIX - (3 * SEC_IN_HOUR_I);
    const uint32_t nsec = static_cast<uint32_t>(std::round(ftod * 1e9));

    const auto lsinfo = getLeapSecInfo(sec, true);
    sec += lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        sec -= 1;
    }

    TIME_TRACE("SetGloTime %d %d %.9f lsinfo %d %s -> %" PRIi64 " %" PRIu32, glotime.N4_, glotime.Nt_, glotime.TOD_,
        lsinfo.leapsec_value_, string::ToStr(lsinfo.at_leapsec_), sec, nsec);

    return (sec >= 0) && (sec <= std::numeric_limits<uint32_t>::max()) && SetSecNSec(static_cast<uint32_t>(sec), nsec);
}

bool Time::SetUtcTime(const UtcTime& utctime)
{
    // The algorithm
    const int days = days_from_civil<int>(utctime.year_, utctime.month_, utctime.day_);

    // Now we know the days since epoch
    if ((days < 0) || (days > (static_cast<int>(std::numeric_limits<uint32_t>::max() / SEC_IN_DAY_I) - 1))) {
        return false;
    }
    uint32_t sec = days * SEC_IN_DAY_I;

    // Time of day
    // @todo could probably use timeSecToIsecNsec()...
    double isec = 0.0;
    const double fsec = std::modf(math::RoundToFracDigits(utctime.sec_, 9), &isec);
    int tod = (utctime.hour_ * SEC_IN_HOUR_I) + (utctime.min_ * SEC_IN_MIN_I) + static_cast<int>(isec);
    if ((tod < 0) || (tod > SEC_IN_DAY_I)) {  // >, not >=, as it can be 86400 on leapsecond event
        return false;
    }
    sec += tod;

    // Leapseconds
    const auto lsinfo = getLeapSecInfo(sec, true);
    sec += lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_ && (tod == SEC_IN_DAY_I)) {
        sec -= 1;
    }

    // Finally
    sec_ = sec;
    nsec_ = static_cast<uint32_t>(std::round(fsec * 1e9));

    TIME_TRACE(/* clang-format off */
        "SetUtcTime %04d-%02d-%02d %02d:%02d:%012.9f -> %d %d -> %d %s -> %" PRIu32 " %" PRIu32,
        utctime.year_, utctime.month_, utctime.day_, utctime.hour_, utctime.min_, utctime.sec_,
        days, tod, lsinfo.leapsec_value_, string::ToStr(lsinfo.at_leapsec_), sec_, nsec_);  // clang-format on

    return true;
}

bool Time::SetTai(const std::time_t tai)
{
    if ((tai < TAI_OFFS) || (tai > (std::numeric_limits<int32_t>::max() - NUM_LEAPS - TAI_OFFS))) {
        return false;
    }
    sec_ = tai - TAI_OFFS;
    nsec_ = 0;
    TIME_TRACE("SetTai %" PRIiMAX " -> %" PRIu32 " %" PRIu32, tai, sec_, nsec_);
    return true;
}

bool Time::SetTaiSec(const double tai_sec)
{
    uint32_t isec = 0;
    uint32_t nsec = 0;
    if (!timeSecToIsecNsec(tai_sec, isec, nsec) || !SetTai(static_cast<std::time_t>(isec))) {
        return false;
    }
    nsec_ = nsec;
    return true;
}

bool Time::SetTaiNs(const uint64_t tai_ns)
{
    uint64_t sec64 = 0;
    uint64_t nsec64 = tai_ns;
    if (!timeNormalizeSecNSec(sec64, nsec64) || !SetTai(static_cast<std::time_t>(sec64))) {
        return false;
    }
    nsec_ = nsec64;
    return true;
}

bool Time::SetClockRealtime()
{
    timespec tsnow;
    clock_gettime(CLOCK_REALTIME, &tsnow);
    TIME_TRACE("SetClockRealtime %.3f", (double)tsnow.tv_sec + ((double)tsnow.tv_nsec * 1e-9));
    return (tsnow.tv_sec >= 0) && (tsnow.tv_sec < (std::numeric_limits<uint32_t>::max() - NUM_LEAPS)) &&
           (tsnow.tv_nsec >= 0) && (tsnow.tv_nsec < 1000000000) &&
           SetRosTime({ static_cast<uint32_t>(tsnow.tv_sec), static_cast<uint32_t>(tsnow.tv_nsec) });
}

#  ifdef CLOCK_TAI
bool Time::SetClockTai()
{
    timespec tsnow;
    clock_gettime(CLOCK_TAI, &tsnow);
    TIME_TRACE("SetClockTai %.3f", (double)tsnow.tv_sec + ((double)tsnow.tv_nsec * 1e-9));
    if ((tsnow.tv_sec >= TAI_OFFS) && (tsnow.tv_sec < std::numeric_limits<uint32_t>::max()) && (tsnow.tv_nsec >= 0) &&
        (tsnow.tv_nsec < 1000000000)) {
        sec_ = static_cast<uint32_t>(tsnow.tv_sec) - TAI_OFFS;
        nsec_ = static_cast<uint32_t>(tsnow.tv_nsec);
        return true;
    } else {
        return false;
    }
}
#  endif  // CLOCK_TAI

// ---------------------------------------------------------------------------------------------------------------------

uint64_t Time::GetNSec() const
{
    return (static_cast<uint64_t>(sec_) * (uint64_t)1000000000) + static_cast<uint64_t>(nsec_);
}

double Time::GetSec(const int prec) const
{
    return math::RoundToFracDigits(
        static_cast<double>(sec_) + (1e-9 * static_cast<double>(nsec_)), std::clamp(prec, 0, 9));
}

std::time_t Time::GetPosix() const
{
    const auto lsinfo = getLeapSecInfo(sec_, false);
    std::time_t posix = static_cast<std::time_t>(sec_);
    posix -= lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        posix += 1;
    }
    TIME_TRACE("GetPosix %" PRIu32 " lsinfo %d %s -> %" PRIiMAX, sec_, lsinfo.leapsec_value_,
        string::ToStr(lsinfo.at_leapsec_), posix);
    return posix;
}

double Time::GetPosixSec(const int prec) const
{
    return math::RoundToFracDigits(
        static_cast<double>(GetPosix()) + (1e-9 * static_cast<double>(nsec_)), std::clamp(prec, 0, 9));
}

uint64_t Time::GetPosixNs() const
{
    return (static_cast<uint64_t>(GetPosix()) * (uint64_t)1000000000) + static_cast<uint64_t>(nsec_);
}

RosTime Time::GetRosTime() const
{
    const auto lsinfo = getLeapSecInfo(sec_, false);
    uint32_t posix = sec_;
    posix -= lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        posix += 1;
    }
    RosTime rostime(posix, nsec_);
    TIME_TRACE("GetRosTime %" PRIu32 " %" PRIu32 " lsinfo %d %s -> %" PRIu32 " %" PRIu32, sec_, nsec_,
        lsinfo.leapsec_value_, string::ToStr(lsinfo.at_leapsec_), rostime.sec_, rostime.nsec_);
    return rostime;
}

WnoTow Time::GetWnoTow(const WnoTow::Sys sys, const int prec) const
{
    int wno = sec_ / SEC_IN_WEEK_I;
    int tow = sec_ % SEC_IN_WEEK_I;
    double towf = math::RoundToFracDigits(static_cast<double>(nsec_) * 1e-9, std::clamp(prec, 0, 9));
    if (towf >= (1.0 - std::numeric_limits<double>::epsilon())) {
        towf -= 1.0;
        tow += 1;
    }

    switch (sys) {
        case WnoTow::Sys::GPS:
            wno -= GPS_OFFS_WNO;
            tow -= GPS_OFFS_TOW;
            break;
        case WnoTow::Sys::GAL:
            wno -= GAL_OFFS_WNO;
            tow -= GAL_OFFS_TOW;
            break;
        case WnoTow::Sys::BDS:
            wno -= BDS_OFFS_WNO;
            tow -= BDS_OFFS_TOW;
            break;
    }
    while (tow < 0) {
        tow += SEC_IN_WEEK_I;
        wno--;
    }
    while (tow >= SEC_IN_WEEK_I) {
        tow -= SEC_IN_WEEK_I;
        wno++;
    }
    WnoTow wnotow(wno, static_cast<double>(tow) + towf, sys);
    TIME_TRACE("GetWnoTow %" PRIu32 " %" PRIu32 " -> %04d:%016.9f %d", sec_, nsec_, wnotow.wno_, wnotow.tow_, (int)sys);
    return wnotow;
}

GloTime Time::GetGloTime(const int prec) const
{
    int64_t sec = static_cast<int64_t>(sec_) - static_cast<int64_t>(GLO_OFFS_POSIX) + (3 * SEC_IN_HOUR_I);
    const auto lsinfo = getLeapSecInfo(sec_, true);
    sec -= lsinfo.leapsec_value_;
    if (lsinfo.at_leapsec_) {
        sec += 1;
    }
    const int glo_days = sec / SEC_IN_DAY_I;
    const int N4 = 1 + (glo_days / 1461);
    const int Nt = 1 + (glo_days % 1461);
    const double TOD = math::RoundToFracDigits(
        static_cast<double>(sec % SEC_IN_DAY_I) + static_cast<double>(nsec_ * 1e-9), std::clamp(prec, 0, 9));
    TIME_TRACE("GetGloTime %" PRIu32 " %" PRIu32 " lsinfo %d %s -> %" PRIi64 " %d -> %d %d %.9f", sec_, nsec_,
        lsinfo.leapsec_value_, string::ToStr(lsinfo.at_leapsec_), sec, glo_days, N4, Nt, TOD);
    return GloTime(N4, Nt, TOD);
}

UtcTime Time::GetUtcTime(const int prec) const
{
    // Will the subsection round to > 1.0s?
    const int n_frac = std::clamp(prec, 0, 9);
    uint32_t isec = sec_;
    double fsec = math::RoundToFracDigits(static_cast<double>(nsec_) * 1e-9, n_frac);
    if (fsec >= (1.0 - std::numeric_limits<double>::epsilon())) {
        fsec -= 1.0;
        isec += 1;
    }

    // Time since epoch w/o leapseconds (i.e., POSIX time)
    const auto lsinfo = getLeapSecInfo(isec, false);
    const uint32_t posix_secs = isec - lsinfo.leapsec_value_;
    const int32_t posix_days = posix_secs / SEC_IN_DAY_I;
    const uint32_t rem_secs = posix_secs % SEC_IN_DAY_I;

    // The algorithm
    auto ymd = civil_from_days<int>(posix_days);
    const int utc_year = std::get<0>(ymd);
    const int utc_month = std::get<1>(ymd);
    const int utc_day = std::get<2>(ymd);

    // Time of day
    int utc_sec = rem_secs;
    int utc_hour = rem_secs / SEC_IN_HOUR_I;
    utc_sec -= utc_hour * SEC_IN_HOUR_I;
    int utc_min = utc_sec / SEC_IN_MIN_I;
    utc_sec -= utc_min * SEC_IN_MIN_I;

    // Correct for leapseconds
    if (lsinfo.at_leapsec_) {
        utc_sec = 60;
    }

    // Finally
    UtcTime utctime(utc_year, utc_month, utc_day, utc_hour, utc_min, static_cast<double>(utc_sec) + fsec);

    TIME_TRACE(/* clang-format off */
        "GetUtcTime %" PRIu32 " %" PRIu32 " %d %s -> %" PRIu32 " %" PRIu32 " %" PRIu32 " -> %04d-%02d-%02d %02d:%02d:%0*.*f",
        sec_, nsec_, lsinfo.leapsec_value_, string::ToStr(lsinfo.at_leapsec_),  posix_secs, posix_days, rem_secs,
        utctime.year_, utctime.month_, utctime.day_, utctime.hour_, utctime.min_,
            n_frac > 0 ? (n_frac + 3) : (n_frac + 2), n_frac, utctime.sec_);  // clang-format on

    return utctime;
}

double Time::GetDayOfYear(const int prec) const
{
    const auto utc = GetUtcTime();
    std::array<int, 12> days = { { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } };
    int doyi = utc.day_;
    for (int ix = 0; ix < (utc.month_ - 1); ix++) {
        doyi += days[ix];
    }
    const bool leap_year = (((utc.year_ % 4) == 0) && (((utc.year_ % 100) != 0) || ((utc.year_ % 400) == 0)));
    if (leap_year && (utc.month_ > 2)) {
        doyi += 1;
    }

    const double doyf = ((double)((utc.hour_ * SEC_IN_HOUR_I) + (utc.min_ + SEC_IN_MIN_I)) + utc.sec_) /
                        (leap_year ? SEC_IN_DAY_D : SEC_IN_DAY_D);
    const double doyfr = math::RoundToFracDigits(doyf, std::clamp(prec, 0, 12));

    TIME_TRACE("GetDayOfYear %d %d %d %d %d %.9f %s -> %d %.12f %.12f", utc.year_, utc.month_, utc.day_, utc.hour_,
        utc.min_, utc.sec_, string::ToStr(leap_year), doyi, doyf, doyfr);

    return (double)doyi + doyfr;
}

std::time_t Time::GetTai() const
{
    std::time_t tai = static_cast<std::time_t>(sec_) + TAI_OFFS;
    TIME_TRACE("GetTai %" PRIu32 " -> %" PRIiMAX, sec_, tai);
    return tai;
}

double Time::GetTaiSec(const int prec) const
{
    return math::RoundToFracDigits(
        static_cast<double>(GetTai()) + (1e-9 * static_cast<double>(nsec_)), std::clamp(prec, 0, 9));
}

uint64_t Time::GetTaiNs() const
{
    return (static_cast<uint64_t>(GetTai()) * (uint64_t)1000000000) + static_cast<uint64_t>(nsec_);
}

std::chrono::milliseconds Time::GetChronoMilli() const
{
    std::time_t posix = GetPosix();
    return std::chrono::milliseconds((static_cast<uint64_t>(posix) * 1000) + ((nsec_ + 500000) / 1000000));
}

std::chrono::nanoseconds Time::GetChronoNano() const
{
    std::time_t posix = GetPosix();
    return std::chrono::nanoseconds((static_cast<uint64_t>(posix) * 1000000000) + nsec_);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Time::IsZero() const
{
    return (sec_ == 0) && (nsec_ == 0);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Time::AddDur(const Duration& dur)
{
    int64_t sec_sum = static_cast<int64_t>(sec_) + static_cast<int64_t>(dur.sec_);
    int64_t nsec_sum = static_cast<int64_t>(nsec_) + static_cast<int64_t>(dur.nsec_);
    if (!timeNormalizeSecNSecUnsigned(sec_sum, nsec_sum)) {
        return false;
    }
    sec_ = sec_sum;
    nsec_ = nsec_sum;
    return true;
}

bool Time::AddNSec(const int64_t nsec)
{
    const uint64_t cur = GetNSec();
    if (((nsec < 0) && (static_cast<uint32_t>(-nsec) > cur)) ||
        ((nsec > 0) && (static_cast<uint32_t>(nsec) > (std::numeric_limits<uint64_t>::max() - cur)))) {
        return false;
    }
    return SetNSec(cur + nsec);
}

bool Time::AddSec(const double sec)
{
    if (sec < 0.0) {
        return SubSec(-sec);
    }
    uint32_t isec;
    uint32_t nsec;
    if (!timeSecToIsecNsec(sec, isec, nsec)) {
        return false;
    }
    int64_t sec64 = static_cast<int64_t>(sec_) + static_cast<int64_t>(isec);
    int64_t nsec64 = static_cast<int64_t>(nsec_) + static_cast<int64_t>(nsec);
    if (!timeNormalizeSecNSecUnsigned(sec64, nsec64)) {
        return false;
    }
    sec_ = static_cast<uint32_t>(sec64);
    nsec_ = static_cast<uint32_t>(nsec64);
    return true;
}

Time Time::operator+(const Duration& dur) const
{
    Time time = *this;
    if (!time.AddDur(dur)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

Time Time::operator+(const int64_t nsec) const
{
    Time time = *this;
    if (!time.AddNSec(nsec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

Time Time::operator+(const double sec) const
{
    Time time = *this;
    if (!time.AddSec(sec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

Time& Time::operator+=(const Duration& dur)
{
    *this = *this + dur;
    return *static_cast<Time*>(this);
}

Time& Time::operator+=(const int64_t nsec)
{
    *this = *this + nsec;
    return *static_cast<Time*>(this);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Time::SubDur(const Duration& dur)
{
    return AddDur(-dur);
}

bool Time::SubNSec(const int64_t nsec)
{
    return AddNSec(-nsec);
}

bool Time::SubSec(const double sec)
{
    if (sec < 0.0) {
        return AddSec(-sec);
    }
    uint32_t isec;
    uint32_t nsec;
    if (!timeSecToIsecNsec(sec, isec, nsec)) {
        return false;
    }
    int64_t sec64 = static_cast<int64_t>(sec_) - static_cast<int64_t>(isec);
    int64_t nsec64 = static_cast<int64_t>(nsec_) - static_cast<int64_t>(nsec);
    if (!timeNormalizeSecNSecUnsigned(sec64, nsec64)) {
        return false;
    }
    sec_ = static_cast<uint32_t>(sec64);
    nsec_ = static_cast<uint32_t>(nsec64);
    return true;
}

Time Time::operator-(const Duration& dur) const
{
    Time time = *this;
    if (!time.SubDur(dur)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

Time Time::operator-(const int64_t nsec) const
{
    Time time = *this;
    if (!time.SubNSec(nsec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

Time Time::operator-(const double sec) const
{
    Time time = *this;
    if (!time.SubSec(sec)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return time;
}

Time& Time::operator-=(const Duration& dur)
{
    *this = *this - dur;
    return *static_cast<Time*>(this);
}

Time& Time::operator-=(const int64_t nsec)
{
    *this = *this - nsec;
    return *static_cast<Time*>(this);
}

Time& Time::operator-=(const double sec)
{
    *this = *this - sec;
    return *static_cast<Time*>(this);
}

Time& Time::operator+=(const double sec)
{
    *this = *this + sec;
    return *static_cast<Time*>(this);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Time::Diff(const Time& other, Duration& diff) const
{
    const bool sign = (*this > other);
    const uint64_t value = (sign ? GetNSec() - other.GetNSec() : other.GetNSec() - GetNSec());
    if (value < (uint64_t)std::numeric_limits<int64_t>::max()) {
        return diff.SetNSec(sign ? value : -value);
    } else {
        return false;
    }
}

Duration Time::operator-(const Time& rhs) const
{
    Duration d;
    if (!Diff(rhs, d)) {
        throw std::runtime_error(TIME_THROW_MSG);
    }
    return d;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Time::operator==(const Time& rhs) const
{
    return (sec_ == rhs.sec_) && (nsec_ == rhs.nsec_);
}

bool Time::operator!=(const Time& rhs) const
{
    return (sec_ != rhs.sec_) || (nsec_ != rhs.nsec_);
}

bool Time::operator>(const Time& rhs) const
{
    if (sec_ > rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ > rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

bool Time::operator<(const Time& rhs) const
{
    if (sec_ < rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ < rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

bool Time::operator>=(const Time& rhs) const
{
    if (sec_ > rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ >= rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

bool Time::operator<=(const Time& rhs) const
{
    if (sec_ < rhs.sec_) {
        return true;
    } else if ((sec_ == rhs.sec_) && (nsec_ <= rhs.nsec_)) {
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::string Time::StrWnoTow(const WnoTow::Sys sys, const int prec) const
{
    const int n_frac = std::clamp(prec, 0, 9);
    const auto wnotow = GetWnoTow(sys, n_frac);
    return string::Sprintf("%04d:%0*.*f", wnotow.wno_, n_frac > 0 ? (n_frac + 7) : (n_frac + 6), n_frac, wnotow.tow_);
}

std::string Time::StrUtcTime(const int prec) const
{
    const int n_frac = std::clamp(prec, 0, 9);
    const auto utctime = GetUtcTime(n_frac);
    return string::Sprintf("%04d-%02d-%02d %02d:%02d:%0*.*f", utctime.year_, utctime.month_, utctime.day_,
        utctime.hour_, utctime.min_, n_frac > 0 ? (n_frac + 3) : (n_frac + 2), n_frac, utctime.sec_);
}

std::string Time::StrIsoTime(const int prec) const
{
    const int n_frac = std::clamp(prec, 0, 9);
    const auto utctime = GetUtcTime(n_frac);
    return string::Sprintf("%04d-%02d-%02dT%02d:%02d:%0*.*fZ", utctime.year_, utctime.month_, utctime.day_,
        utctime.hour_, utctime.min_, n_frac > 0 ? (n_frac + 3) : (n_frac + 2), n_frac, utctime.sec_);
}

// ---------------------------------------------------------------------------------------------------------------------

const Time Time::MAX = Time::FromSecNSec(std::numeric_limits<uint32_t>::max() - 1, 999999999);
const Time Time::MIN = Time::FromSecNSec(0, 1);
const Time Time::ZERO = Time::FromSecNSec(0, 0);

#endif
/* ****************************************************************************************************************** */
}  // namespace time
}  // namespace common
}  // namespace fpsdk
