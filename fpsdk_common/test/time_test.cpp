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
 * @brief Fixposition SDK: tests for fpsdk::common::time
 */

/* LIBC/STL */
#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <ostream>

/* EXTERNAL */
#include <gtest/gtest.h>
#include <sys/timex.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>
#include <fpsdk_common/time.hpp>

/* ****************************************************************************************************************** */

// Some operators and stringifcation functions for the tests below
// clang-format off
namespace fpsdk { namespace common { namespace time {

bool operator==(const UtcTime& lhs, const UtcTime& rhs)
{
    return (lhs.year_ == rhs.year_) && (lhs.month_ == rhs.month_) && (lhs.day_ == rhs.day_) &&
           (lhs.hour_ == rhs.hour_) && (lhs.min_ == rhs.min_) && (std::fabs(lhs.sec_ - rhs.sec_) < 1e-9);
}

bool operator==(const WnoTow& lhs, const WnoTow& rhs)
{
    return (lhs.wno_ == rhs.wno_) && (std::fabs(lhs.tow_ - rhs.tow_) < 1e-9) && (lhs.sys_ == rhs.sys_);
}

bool operator==(const GloTime& lhs, const GloTime& rhs)
{
    return (lhs.N4_ == rhs.N4_) && (lhs.Nt_ == rhs.Nt_) && (std::fabs(lhs.TOD_ - rhs.TOD_) < 1e-9);
}

std::ostream& operator<<(std::ostream& os, const Duration& dur)
{
    return os << fpsdk::common::string::Sprintf("%" PRIi32 " %" PRIi32, dur.sec_, dur.nsec_);
}
void PrintTo(const Duration& dur, ::std::ostream* os) { *os << dur; }

std::ostream& operator<<(std::ostream& os, const RosTime& rostime)
{
    return os << fpsdk::common::string::Sprintf("%" PRIi32 " %" PRIi32, rostime.sec_, rostime.nsec_);
}
void PrintTo(const RosTime& rostime, ::std::ostream* os) { *os << rostime; }

std::ostream& operator<<(std::ostream& os, const UtcTime& utctime)
{
    return os << fpsdk::common::string::Sprintf( "%04d-%02d-%02d %02d:%02d:%012.9f",
        utctime.year_, utctime.month_, utctime.day_, utctime.hour_, utctime.min_, utctime.sec_);
}
void PrintTo(const UtcTime& utctime, ::std::ostream* os) { *os << utctime; }

std::ostream& operator<<(std::ostream& os, const WnoTow& wnotow)
{
    return os << fpsdk::common::string::Sprintf( "%04d:%016.9f", wnotow.wno_, wnotow.tow_);
}
void PrintTo(const WnoTow& wnotow, ::std::ostream* os) { *os << wnotow; }

std::ostream& operator<<(std::ostream& os, const GloTime& glotime)
{
    return os << fpsdk::common::string::Sprintf( "%d:%d:%016.9f", glotime.N4_, glotime.Nt_, glotime.TOD_);
}
void PrintTo(const GloTime& glotime, ::std::ostream* os) { *os << glotime; }

std::ostream& operator<<(std::ostream& os, const Time& time)
{
    return os << fpsdk::common::string::Sprintf( "%" PRIu32 " %" PRIu32, time.sec_, time.nsec_);
}
void PrintTo(const Time& time, ::std::ostream* os) { *os << time; }

} } }
// clang-format on

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::time;
using namespace fpsdk::common::string;

TEST(TimeTest, RosTime)
{
    const RosTime t0;
    EXPECT_TRUE(t0.IsZero());
    EXPECT_EQ(t0.sec_, 0);
    EXPECT_EQ(t0.nsec_, 0);
    EXPECT_NEAR(t0.ToSec(), 0.0, 1e-9);
    EXPECT_EQ(t0.ToNSec(), (uint64_t)0);

    const RosTime t1(123, 456);
    EXPECT_FALSE(t1.IsZero());
    EXPECT_EQ(t1.sec_, 123);
    EXPECT_EQ(t1.nsec_, 456);
    EXPECT_NEAR(t1.ToSec(), 123.000000456, 1e-9);
    EXPECT_EQ(t1.ToNSec(), (uint64_t)123000000456);

    const RosTime t2(123, 999999999);
    EXPECT_FALSE(t2.IsZero());
    EXPECT_EQ(t2.sec_, 123);
    EXPECT_EQ(t2.nsec_, 999999999);
    EXPECT_NEAR(t2.ToSec(), 123.999999999, 1e-9);
    EXPECT_EQ(t2.ToNSec(), (uint64_t)123999999999);

    const RosTime t3(123, 999999999 + 1);
    EXPECT_FALSE(t3.IsZero());
    EXPECT_EQ(t3.sec_, 124);
    EXPECT_EQ(t3.nsec_, 0);
    EXPECT_NEAR(t3.ToSec(), 124.0, 1e-9);
    EXPECT_EQ(t3.ToNSec(), (uint64_t)124000000000);

    const RosTime t4(123, 999999999 + 1 + 1);
    EXPECT_FALSE(t4.IsZero());
    EXPECT_EQ(t4.sec_, 124);
    EXPECT_EQ(t4.nsec_, 1);
    EXPECT_NEAR(t4.ToSec(), 124.000000001, 1e-9);
    EXPECT_EQ(t4.ToNSec(), (uint64_t)124000000001);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, TicToc)
{
    const double s = 0.1;
    const Duration dur = Duration::FromSec(s);
    TicToc tt1;
    TicToc tt2;

    dur.Sleep();

    const auto d1a = tt1.Toc();
    const auto d2a = tt2.Toc(true);
    // DEBUG("d1a=%.3f d2a=%.3f", d1a.GetSec(), d2a.GetSec());
    EXPECT_NEAR(d1a.GetSec(), s, 5e-3);
    EXPECT_NEAR(d2a.GetSec(), s, 5e-3);

    dur.Sleep();

    const auto d1b = tt1.Toc();
    const auto d2b = tt2.Toc(true);
    // DEBUG("d1b=%.3f d2b=%.3f", d1b.GetSec(), d2b.GetSec());
    EXPECT_NEAR(d1b.GetSec(), 2.0 * s, 5e-3);
    EXPECT_NEAR(d2b.GetSec(), s, 5e-3);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_FromSecNSec)
{
    const Duration dur1;  // 0.0
    EXPECT_TRUE(dur1.IsZero());
    EXPECT_EQ(dur1.sec_, 0);
    EXPECT_EQ(dur1.nsec_, 0);

    const Duration dur2 = Duration::FromSecNSec(1, 0);  // 1.000000000
    EXPECT_FALSE(dur2.IsZero());
    EXPECT_EQ(dur2.sec_, 1);
    EXPECT_EQ(dur2.nsec_, 0);

    const Duration dur3 = Duration::FromSecNSec(0, 1);  // 0.000000001
    EXPECT_FALSE(dur3.IsZero());
    EXPECT_EQ(dur3.sec_, 0);
    EXPECT_EQ(dur3.nsec_, 1);

    const Duration dur4 = Duration::FromSecNSec(1, 1);  // 1.000000001
    EXPECT_FALSE(dur4.IsZero());
    EXPECT_EQ(dur4.sec_, 1);
    EXPECT_EQ(dur4.nsec_, 1);

    const Duration dur5 = Duration::FromSecNSec(-1, 1);  // -0.999999999
    EXPECT_FALSE(dur5.IsZero());
    EXPECT_EQ(dur5.sec_, -1);
    EXPECT_EQ(dur5.nsec_, 1);

    const Duration dur6 = Duration::FromSecNSec(1, -1);  // 0.999999999
    EXPECT_FALSE(dur6.IsZero());
    EXPECT_EQ(dur6.sec_, 0);
    EXPECT_EQ(dur6.nsec_, 999999999);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_FromNSec)
{
    const Duration dur1 = Duration::FromNSec(1000000001);  // 1.000000001
    EXPECT_EQ(dur1.sec_, 1);
    EXPECT_EQ(dur1.nsec_, 1);
    EXPECT_EQ(dur1.GetNSec(), (int64_t)1000000001);

    const Duration dur2 = Duration::FromNSec(-1000000001);  // -1.000000001
    EXPECT_EQ(dur2.sec_, -2);
    EXPECT_EQ(dur2.nsec_, 999999999);
    EXPECT_EQ(dur2.GetNSec(), (int64_t)-1000000001);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_FromSec)
{
    const Duration dur1 = Duration::FromSec(1.000000001);
    EXPECT_NEAR(dur1.GetSec(), 1.000000001, 1e-9);

    const Duration dur2 = Duration::FromSec(-1.000000001);
    EXPECT_NEAR(dur2.GetSec(), -1.000000001, 1e-9);

    const Duration dur3 = Duration::FromSec(1.123456789);
    EXPECT_NEAR(dur3.GetSec(), 1.123456789, 1e-9);
    EXPECT_NEAR(dur3.GetSec(0), 1.000000000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(1), 1.100000000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(2), 1.120000000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(3), 1.123000000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(4), 1.123500000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(5), 1.123460000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(6), 1.123457000, 1e-9);
    EXPECT_NEAR(dur3.GetSec(7), 1.123456800, 1e-9);
    EXPECT_NEAR(dur3.GetSec(8), 1.123456790, 1e-9);
    EXPECT_NEAR(dur3.GetSec(9), 1.123456789, 1e-9);

    const Duration dur4 = Duration::FromSec(-1.123456789);
    EXPECT_NEAR(dur4.GetSec(), -1.123456789, 1e-9);
    EXPECT_NEAR(dur4.GetSec(0), -1.000000000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(1), -1.100000000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(2), -1.120000000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(3), -1.123000000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(4), -1.123500000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(5), -1.123460000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(6), -1.123457000, 1e-9);
    EXPECT_NEAR(dur4.GetSec(7), -1.123456800, 1e-9);
    EXPECT_NEAR(dur4.GetSec(8), -1.123456790, 1e-9);
    EXPECT_NEAR(dur4.GetSec(9), -1.123456789, 1e-9);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_Logic)
{
    Duration dur1 = Duration::FromSecNSec(1, 0);
    Duration dur2 = Duration::FromSecNSec(2, 0);
    Duration dur3 = Duration::FromSecNSec(1, 1);

    EXPECT_TRUE(dur1 == dur1);
    EXPECT_FALSE(dur1 != dur1);

    EXPECT_TRUE(dur1 != dur2);
    EXPECT_FALSE(dur1 == dur2);

    EXPECT_TRUE(dur1 != dur3);
    EXPECT_FALSE(dur1 == dur3);

    EXPECT_TRUE(dur1 < dur2);
    EXPECT_TRUE(dur1 <= dur2);
    EXPECT_TRUE(dur1 <= dur1);
    EXPECT_TRUE(dur1 < dur3);
    EXPECT_TRUE(dur1 <= dur3);

    EXPECT_TRUE(dur2 > dur1);
    EXPECT_TRUE(dur2 >= dur1);
    EXPECT_TRUE(dur1 >= dur1);
    EXPECT_TRUE(dur3 > dur1);
    EXPECT_TRUE(dur3 >= dur1);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_Arithmetic)
{
    const int64_t seq[] = {
        (int64_t)1500000000,
        (int64_t)250000000,
        (int64_t)0,
        (int64_t)1750000000,
        (int64_t)3250000000,
        (int64_t)550000000,
        (int64_t)-4500000000,
        (int64_t)-750000000,
        (int64_t)-0,
        (int64_t)-2250000000,
        (int64_t)-1250000000,
        (int64_t)-1550000000,
    };
    // Addition sec
    {
        int64_t sum = 0;
        Duration dur1;
        Duration dur2;
        Duration dur3;
        for (const int64_t v : seq) {
            sum += v;
            const double d = (double)v * 1e-9;
            DEBUG("d=%.9f v=%" PRIi64 " -> sum=%" PRIi64, d, v, sum);
            EXPECT_TRUE(dur1.AddSec(d));
            dur2 = dur2 + d;
            dur3 += d;
            EXPECT_EQ(dur1.GetNSec(), sum);
            EXPECT_EQ(dur2.GetNSec(), sum);
            EXPECT_EQ(dur3.GetNSec(), sum);
        }
    }
    // Addition nsec
    {
        int64_t sum = 0.0;
        Duration dur1;
        Duration dur2;
        Duration dur3;
        for (const int64_t v : seq) {
            sum += v;
            DEBUG("v=%" PRIi64 " -> sum=%" PRIi64, v, sum);
            EXPECT_TRUE(dur1.AddNSec(v));
            dur2 = dur2 + v;
            dur3 += v;
            EXPECT_EQ(dur1.GetNSec(), sum);
            EXPECT_EQ(dur2.GetNSec(), sum);
            EXPECT_EQ(dur3.GetNSec(), sum);
        }
    }
    // Addition Duration
    {
        int64_t sum = 0.0;
        Duration dur1;
        Duration dur2;
        Duration dur3;
        for (const int64_t v : seq) {
            sum += v;
            DEBUG("v=%" PRIi64 " -> sum=%" PRIi64, v, sum);
            const Duration d = Duration::FromNSec(v);
            EXPECT_TRUE(dur1.AddDur(d));
            dur2 = dur2 + d;
            dur3 += d;
            EXPECT_EQ(dur1.GetNSec(), sum);
            EXPECT_EQ(dur2.GetNSec(), sum);
            EXPECT_EQ(dur3.GetNSec(), sum);
        }
    }
    // Substraction sec
    {
        int64_t sum = 0;
        Duration dur1;
        Duration dur2;
        Duration dur3;
        for (const int64_t v : seq) {
            sum -= v;
            const double d = (double)v * 1e-9;
            DEBUG("d=%.9f v=%" PRIi64 " -> sum=%" PRIi64, d, v, sum);
            EXPECT_TRUE(dur1.SubSec(d));
            dur2 = dur2 - d;
            dur3 -= d;
            EXPECT_EQ(dur1.GetNSec(), sum);
            EXPECT_EQ(dur2.GetNSec(), sum);
            EXPECT_EQ(dur3.GetNSec(), sum);
        }
    }
    // Substraction nsec
    {
        int64_t sum = 0.0;
        Duration dur1;
        Duration dur2;
        Duration dur3;
        for (const int64_t v : seq) {
            sum -= v;
            DEBUG("v=%" PRIi64 " -> sum=%" PRIi64, v, sum);
            EXPECT_TRUE(dur1.SubNSec(v));
            dur2 = dur2 - v;
            dur3 -= v;
            EXPECT_EQ(dur1.GetNSec(), sum);
            EXPECT_EQ(dur2.GetNSec(), sum);
            EXPECT_EQ(dur3.GetNSec(), sum);
        }
    }
    // Substraction duration
    {
        int64_t sum = 0.0;
        Duration dur1;
        Duration dur2;
        Duration dur3;
        for (const int64_t v : seq) {
            sum -= v;
            DEBUG("v=%" PRIi64 " -> sum=%" PRIi64, v, sum);
            const Duration d = Duration::FromNSec(v);
            EXPECT_TRUE(dur1.SubDur(d));
            dur2 = dur2 - d;
            dur3 -= d;
            EXPECT_EQ(dur1.GetNSec(), sum);
            EXPECT_EQ(dur2.GetNSec(), sum);
            EXPECT_EQ(dur3.GetNSec(), sum);
        }
    }
    // Scaling
    {
        Duration dur;
        EXPECT_TRUE(dur.SetSec(5.0));
        EXPECT_TRUE(dur.Scale(2.0));
        ASSERT_NEAR(dur.GetSec(), 10.0, 1e-9);
        EXPECT_TRUE(dur.Scale(-1.5));
        ASSERT_NEAR(dur.GetSec(), -15.0, 1e-9);
        dur = dur * 2.0;
        ASSERT_NEAR(dur.GetSec(), -30.0, 1e-9);
        dur *= -2.0;
        ASSERT_NEAR(dur.GetSec(), 60.0, 1e-9);
    }
    // Other arithmetics
    {
        const Duration d1 = Duration::FromSecNSec(1, 1);           // 1.000000001
        const Duration d2 = Duration::FromSecNSec(-2, 999999999);  // -1.000000001
        EXPECT_EQ(-d1, d2);
        EXPECT_EQ(d1, -d2);
        EXPECT_EQ(-d1, d1 * -1.0);
        EXPECT_EQ(-d2, d2 * -1.0);
        EXPECT_EQ(-d1, d2 * 1.0);
        EXPECT_EQ(-d2, d1 * 1.0);
        EXPECT_EQ(d1, d2 * -1.0);
        EXPECT_EQ(d2, d1 * -1.0);
        EXPECT_EQ(d1, -d2 * 1.0);
        EXPECT_EQ(d2, -d1 * 1.0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_Chrono)
{
    const Duration dur1 = Duration::FromSecNSec(1, 123456789);
    EXPECT_EQ(dur1.GetChronoMilli(), std::chrono::milliseconds(1123));
    EXPECT_EQ(dur1.GetChronoNano(), std::chrono::nanoseconds(1123456789));

    const Duration dur2 = Duration::FromSecNSec(1, 555555555);
    EXPECT_EQ(dur2.GetChronoMilli(), std::chrono::milliseconds(1556));
    EXPECT_EQ(dur2.GetChronoNano(), std::chrono::nanoseconds(1555555555));

    const Duration dur3 = -dur1;
    EXPECT_EQ(dur3.GetChronoMilli(), std::chrono::milliseconds(-1123));
    EXPECT_EQ(dur3.GetChronoNano(), std::chrono::nanoseconds(-1123456789));

    const Duration dur4 = -dur2;
    EXPECT_EQ(dur4.GetChronoMilli(), std::chrono::milliseconds(-1556));
    EXPECT_EQ(dur4.GetChronoNano(), std::chrono::nanoseconds(-1555555555));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_Stringify)
{
    // clang-format off
    const struct {
        double sec; int prec; std::string str;
    } tests[] =
    {
        {             ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  0,      "02:03:05"       },
        {             ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  1,      "02:03:04.6"     },
        {             ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  2,      "02:03:04.57"    },
        {             ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  3,      "02:03:04.567"   },
        {             ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  4,      "02:03:04.5670"  },
        {   86400.0 + ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  0,   "1d 02:03:05"       },
        {   86400.0 + ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567,  5,   "1d 02:03:04.56700" },
        { -(86400.0 + ( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567), 5,  "-1d 02:03:04.56700" },
        {           -(( 2.0 * 3600.0) + ( 3.0 * 60.0) + 4.567), 3,     "-02:03:04.567"   },
        {                                                59.9,  1,      "00:00:59.9"     },
        {                                                59.95, 1,      "00:01:00.0"     },
        {                                                60.0,  1,      "00:01:00.0"     },
        {                               (59.0 * 60.0) +  60.0,  1,      "01:00:00.0"     },
        {             (23.0 * 3600.0) + (59.0 * 60.0) +  60.0,  1,   "1d 00:00:00.0"     },
    };
    // clang-format on
    for (auto& test : tests) {
        Duration dur;
        EXPECT_TRUE(dur.SetSec(test.sec));
        EXPECT_EQ(dur.Stringify(test.prec), test.str);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_Sleep)
{
    const Duration dur = Duration::FromSec(0.1);
    const double s0 = GetSecs();
    const uint64_t t0 = GetMillis();
    TicToc tt;
    dur.Sleep();
    const double s1 = GetSecs();
    const uint64_t t1 = GetMillis();
    const Duration dtt = tt.Toc();
    DEBUG("s0=%.3f t0=%" PRIu64, s0, t0);
    DEBUG("s1=%.3f t0=%" PRIu64 " %.3f %" PRIu64 " %.3f", s1, t1, s1 - s0, t1 - t0, dtt.GetSec());
    ASSERT_NEAR(s1 - s0, 0.1, 10e-3);
    ASSERT_NEAR(t1 - t0, 100, 10);
    ASSERT_NEAR(dtt.GetSec(), 0.1, 10e-3);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Duration_Various)
{
    // clang-format off
    const struct {
        int32_t in_sec; int32_t in_nsec; int32_t out_sec; int32_t out_nsec; double out_tosec; int64_t out_tonsec;
    } tests[] = {
        // in sec/nsec         out sec/nsec        GetSec()       GetNSec()
        { 1,           2,      1,          2,      1.000000002,   1000000002 },
        { 1,  1000000000,      2,          0,      2.000000000,   2000000000 },
        { 1,  1000000001,      2,          1,      2.000000001,   2000000001 },
        { 1,  2000000000,      3,          0,      3.000000000,   3000000000 },
        { 1,  2000000001,      3,          1,      3.000000001,   3000000001 },
        { 1,          -1,      0,  999999999,      0.999999999,    999999999 },
        { 1, -1000000000,      0,          0,     -0.000000000,            0 },
        { 1, -1000000001,     -1,  999999999,     -0.000000001,           -1 },
        { 1, -2000000001,     -2,  999999999,     -1.000000001,  -1000000001 },
    };
    // clang-format on
    for (auto& test : tests) {
        Duration dur;
        EXPECT_TRUE(dur.SetSecNSec(test.in_sec, test.in_nsec));
        DEBUG("in: %" PRIi32 " %" PRIi32 " / out: %" PRIi32 " %" PRIi32 " / dur: %" PRIi32 " %" PRIi32 " %.9f %" PRIi64,
            test.in_sec, test.in_nsec, test.out_sec, test.out_nsec, dur.sec_, dur.nsec_, dur.GetSec(), dur.GetNSec());
        EXPECT_EQ(dur.sec_, test.out_sec);
        EXPECT_EQ(dur.nsec_, test.out_nsec);
        ASSERT_NEAR(dur.GetSec(), test.out_tosec, 1e-9);
        EXPECT_EQ(dur.GetNSec(), test.out_tonsec);
    }
}

// @todo add tests for extreme values and other corner cases

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_FirstLeapsecEvent)
{
    // First leapsecond event
    INFO("----- leapsecond event 1972-07-01 -----");
    // clang-format off
    //  1970  1971  Jan  Feb  Mar  Apr  May  Jun
    // (365 + 365 + 31 + 29 + 31 + 30 + 31 + 30) * 86400 = 78796800s = 912d
    //    UTC                                    Atomic          POSIX
    // t1 1972-06-30 23:59:59.0   before leaps   78796799        78796799
    // t2 1972-06-30 23:59:60.0   at leaps       78796800        78796800
    // t3 1972-07-01 00:00:00.0   after leaps    78796801        78796800  Posix doesn't have leapseconds
    // t4 1972-07-01 00:00:00.5   after leaps    78796801.5      78796800.5
    // t5 1972-07-01 00:00:01.0   after leaps    78796802        78796801
    // clang-format on
    {
        // Initialise from atomic time
        const Time t1 = Time::FromSecNSec(78796799, 0);
        const Time t2 = Time::FromSecNSec(78796800, 0);
        const Time t3 = Time::FromSecNSec(78796801, 0);
        const Time t4 = Time::FromSecNSec(78796801, 500000000);
        const Time t5 = Time::FromSecNSec(78796802, 0);

        // From atomic to POSIX time
        EXPECT_EQ(t1.GetPosix(), 78796799);
        EXPECT_EQ(t2.GetPosix(), 78796800);
        EXPECT_EQ(t3.GetPosix(), 78796800);  // No leapsec
        EXPECT_EQ(t4.GetPosix(), 78796800);  // Rounded down to integer POSIX
        EXPECT_EQ(t5.GetPosix(), 78796801);

        // Also POSIX time, but we have nsec, so no rounding down for t4
        EXPECT_EQ(t1.GetRosTime(), RosTime(78796799, 0));
        EXPECT_EQ(t2.GetRosTime(), RosTime(78796800, 0));
        EXPECT_EQ(t3.GetRosTime(), RosTime(78796800, 0));
        EXPECT_EQ(t4.GetRosTime(), RosTime(78796800, 500000000));
        EXPECT_EQ(t5.GetRosTime(), RosTime(78796801, 0));

        // UTC
        EXPECT_EQ(t1.GetUtcTime(9), UtcTime(1972, 6, 30, 23, 59, 59.0));
        EXPECT_EQ(t2.GetUtcTime(9), UtcTime(1972, 6, 30, 23, 59, 60.0));
        EXPECT_EQ(t3.GetUtcTime(9), UtcTime(1972, 7, 1, 0, 0, 0.0));
        EXPECT_EQ(t4.GetUtcTime(9), UtcTime(1972, 7, 1, 0, 0, 0.5));
        EXPECT_EQ(t5.GetUtcTime(9), UtcTime(1972, 7, 1, 0, 0, 1.0));
    }
    {
        // Initialise from POSIX time
        const Time t1 = Time::FromPosix(78796799);
        const Time t2 = Time::FromPosix(78796800);
        // const Time t3 = Time::FromPosix(78796800); // ambiguous
        // const Time t4 = Time::FromPosix(78796800.5); // ambiguous and POSIX has no fractional part
        const Time t5 = Time::FromPosix(78796801);

        EXPECT_EQ(t1, Time::FromSecNSec(78796799, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(78796800, 0));  // +1 in real world and in POSIX
        EXPECT_EQ(t5, Time::FromSecNSec(78796802, 0));  // +2 in real world, only +1 in POSIX
    }
    {
        // Initialise from RosTime time
        const Time t1 = Time::FromRosTime({ 78796799, 0 });
        const Time t2 = Time::FromRosTime({ 78796800, 0 });
        // const Time t3 = Time::FromRosTime({ 78796800, 0 }); // ambiguous
        const Time t4 = Time::FromRosTime({ 78796800, 500000000 });  // ambiguous but RosTime has a fractional part
        const Time t5 = Time::FromRosTime({ 78796801, 0 });

        EXPECT_EQ(t1, Time::FromSecNSec(78796799, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(78796800, 0));          // +1 in real world and in POSIX
        EXPECT_EQ(t4, Time::FromSecNSec(78796800, 500000000));  // +0.5 or +1.5 in real world, + 0.5 in POSIX
        EXPECT_EQ(t5, Time::FromSecNSec(78796802, 0));          // +2 in real world, only +1 in POSIX
    }
    {
        // Initialise from UTC time
        const Time t1 = Time::FromUtcTime({ 1972, 6, 30, 23, 59, 59.0 });  // sec=78710400 911d
        const Time t2 = Time::FromUtcTime({ 1972, 6, 30, 23, 59, 60.0 });
        const Time t3 = Time::FromUtcTime({ 1972, 7, 1, 0, 0, 0.0 });
        const Time t4 = Time::FromUtcTime({ 1972, 7, 1, 0, 0, 0.5 });
        const Time t5 = Time::FromUtcTime({ 1972, 7, 1, 0, 0, 1.0 });

        EXPECT_EQ(t1, Time::FromSecNSec(78796799, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(78796800, 0));
        EXPECT_EQ(t3, Time::FromSecNSec(78796801, 0));
        EXPECT_EQ(t4, Time::FromSecNSec(78796801, 500000000));
        EXPECT_EQ(t5, Time::FromSecNSec(78796802, 0));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_LaterLeapsecEvent)
{
    // A later leapsecond event
    INFO("----- leapsecond event 1982-07-01 -----");
    // clang-format off
    //  1970  1971  1972  1973  1974  1975  1976  1977  1978  1979  1980  1981  Jan  Feb  Mar  Apr  May  Jun
    // (365 + 365 + 366 + 365 + 365 + 365 + 366 + 365 + 365 + 365 + 366 + 365 + 31 + 28 + 31 + 30 + 31 + 30) * 86400 + 10 = 394329610
    //    UTC                                    Atomic        GPS            POSIX
    // t1 1982-06-30 23:59:59.0   before leaps   394329609     129:345600.0   394329599
    // t1 1982-06-30 23:59:60.0   at leaps       394329610     129:345601.0   394329600
    // t1 1982-07-01 00:00:00.0   after leaps    394329611     129:345602.0   394329600  Posix doesn't have leapseconds
    // t1 1982-07-01 00:00:00.5   after leaps    394329611.5   129:345602.5   394329600.5
    // t1 1982-07-01 00:00:01.0   after leaps    394329612     129:345603.0   394329601
    // clang-format on
    {
        // Initialise from atomic time
        const Time t1 = Time::FromSecNSec(394329609, 0);
        const Time t2 = Time::FromSecNSec(394329610, 0);
        const Time t3 = Time::FromSecNSec(394329611, 0);
        const Time t4 = Time::FromSecNSec(394329611, 500000000);
        const Time t5 = Time::FromSecNSec(394329612, 0);

        // From atomic to POSIX time
        EXPECT_EQ(t1.GetPosix(), 394329599);
        EXPECT_EQ(t2.GetPosix(), 394329600);
        EXPECT_EQ(t3.GetPosix(), 394329600);  // No leapsec
        EXPECT_EQ(t4.GetPosix(), 394329600);  // Rounded down to integer POSIX
        EXPECT_EQ(t5.GetPosix(), 394329601);

        // Also system (POSIX) time, but we have nsec, so no rounding down for t4
        EXPECT_EQ(t1.GetRosTime(), RosTime(394329599, 0));
        EXPECT_EQ(t2.GetRosTime(), RosTime(394329600, 0));
        EXPECT_EQ(t3.GetRosTime(), RosTime(394329600, 0));
        EXPECT_EQ(t4.GetRosTime(), RosTime(394329600, 500000000));
        EXPECT_EQ(t5.GetRosTime(), RosTime(394329601, 0));

        // UTC
        EXPECT_EQ(t1.GetUtcTime(9), UtcTime(1982, 6, 30, 23, 59, 59.0));
        EXPECT_EQ(t2.GetUtcTime(9), UtcTime(1982, 6, 30, 23, 59, 60.0));
        EXPECT_EQ(t3.GetUtcTime(9), UtcTime(1982, 7, 1, 0, 0, 0.0));
        EXPECT_EQ(t4.GetUtcTime(9), UtcTime(1982, 7, 1, 0, 0, 0.5));
        EXPECT_EQ(t5.GetUtcTime(9), UtcTime(1982, 7, 1, 0, 0, 1.0));

        // GPS
        // 0:0 1980-01-06 00:00:00.0   315532800 + 9 + 5*86400 = 315964809
        //     1982-07-01 00:00:00.0   (-5+366+365+31+28+31+30+31+30) * 86400 = 78364800 = 907d = 129w4d
        // This seems to be one of the few correct ones on the internets: https://www.andrews.edu/~tzs/timeconv/
        EXPECT_EQ(t1.GetWnoTow(WnoTow::Sys::GPS, 9), WnoTow(129, 345600.0, WnoTow::Sys::GPS));
        EXPECT_EQ(t2.GetWnoTow(WnoTow::Sys::GPS, 9), WnoTow(129, 345601.0, WnoTow::Sys::GPS));
        EXPECT_EQ(t3.GetWnoTow(WnoTow::Sys::GPS, 9), WnoTow(129, 345602.0, WnoTow::Sys::GPS));
        EXPECT_EQ(t4.GetWnoTow(WnoTow::Sys::GPS, 9), WnoTow(129, 345602.5, WnoTow::Sys::GPS));
        EXPECT_EQ(t5.GetWnoTow(WnoTow::Sys::GPS, 9), WnoTow(129, 345603.0, WnoTow::Sys::GPS));
    }
    {
        // Initialise from GPS time
        const Time t1 = Time::FromWnoTow({ 129, 345600.0, WnoTow::Sys::GPS });
        const Time t2 = Time::FromWnoTow({ 129, 345601.0, WnoTow::Sys::GPS });
        const Time t3 = Time::FromWnoTow({ 129, 345602.0, WnoTow::Sys::GPS });
        const Time t4 = Time::FromWnoTow({ 129, 345602.5, WnoTow::Sys::GPS });
        const Time t5 = Time::FromWnoTow({ 129, 345603.0, WnoTow::Sys::GPS });

        EXPECT_EQ(t1, Time::FromSecNSec(394329609, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(394329610, 0));
        EXPECT_EQ(t3, Time::FromSecNSec(394329611, 0));
        EXPECT_EQ(t4, Time::FromSecNSec(394329611, 500000000));
        EXPECT_EQ(t5, Time::FromSecNSec(394329612, 0));
    }
    {
        // Initialise from POSIX time
        const Time t1 = Time::FromPosix(394329599);
        const Time t2 = Time::FromPosix(394329600);
        // const Time t3 = Time::FromPosix(394329600); // ambiguous
        // const Time t4 = Time::FromPosix(394329600.5); // ambiguous and POSIX has no fractional part
        const Time t5 = Time::FromPosix(394329601);

        EXPECT_EQ(t1, Time::FromSecNSec(394329609, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(394329610, 0));  // +1 in real world and in POSIX
        EXPECT_EQ(t5, Time::FromSecNSec(394329612, 0));  // +2 in real world, only +1 in POSIX
    }
    {
        // Initialise from RosTime time
        const Time t1 = Time::FromRosTime({ 394329599, 0 });
        const Time t2 = Time::FromRosTime({ 394329600, 0 });
        // const Time t3 = Time::FromRosTime({ 394329600, 0 }); // ambiguous
        const Time t4 = Time::FromRosTime({ 394329600, 500000000 });  // ambiguous but RosTime has a fractional part
        const Time t5 = Time::FromRosTime({ 394329601, 0 });

        EXPECT_EQ(t1, Time::FromSecNSec(394329609, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(394329610, 0));          // +1 in real world and in POSIX
        EXPECT_EQ(t4, Time::FromSecNSec(394329610, 500000000));  // +0.5 or +1.5 in real world, + 0.5 in POSIX
        EXPECT_EQ(t5, Time::FromSecNSec(394329612, 0));          // +2 in real world, only +1 in POSIX
    }
    {
        // Initialise from UTC time
        const Time t1 = Time::FromUtcTime({ 1982, 6, 30, 23, 59, 59.0 });  // sec=78710400 911d
        const Time t2 = Time::FromUtcTime({ 1982, 6, 30, 23, 59, 60.0 });
        const Time t3 = Time::FromUtcTime({ 1982, 7, 1, 0, 0, 0.0 });
        const Time t4 = Time::FromUtcTime({ 1982, 7, 1, 0, 0, 0.5 });
        const Time t5 = Time::FromUtcTime({ 1982, 7, 1, 0, 0, 1.0 });

        EXPECT_EQ(t1, Time::FromSecNSec(394329609, 0));
        EXPECT_EQ(t2, Time::FromSecNSec(394329610, 0));
        EXPECT_EQ(t3, Time::FromSecNSec(394329611, 0));
        EXPECT_EQ(t4, Time::FromSecNSec(394329611, 500000000));
        EXPECT_EQ(t5, Time::FromSecNSec(394329612, 0));
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_LatestLeapsec)
{
    const Time t0 = Time::FromUtcTime({ 2100, 1, 1, 0, 0, 0.0 });
    const time_t asec = std::floor(t0.GetSec());
    const time_t psec = t0.GetPosix();
    const time_t tsec = t0.GetTai();
    DEBUG("%s asec=%" PRIiMAX " psec=%" PRIiMAX " tsec=%" PRIuMAX " asec-psec=%" PRIiMAX " tsec-psec=%" PRIiMAX,
        t0.StrUtcTime().c_str(), asec, psec, tsec, asec - psec, tsec - psec);
    ASSERT_EQ(asec - psec, 27);  // This fails when the leapseconds table is updated
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_GloTime)
{
    // @todo Is this all correct?

    // GLONASS epoch
    const Time t1 = Time::FromGloTime({ 1, 1, 0.0 });
    EXPECT_EQ(t1.GetUtcTime(), UtcTime({ 1995, 12, 31, 21, 0, 0.0 }));
    EXPECT_EQ(t1, Time::FromUtcTime({ 1995, 12, 31, 21, 0, 0.0 }));
    EXPECT_EQ(t1.GetGloTime(), GloTime({ 1, 1, 0.0 }));

    // With .5 fractional seconds
    const Time t1f = Time::FromGloTime({ 1, 1, 0.5 });
    EXPECT_EQ(t1f.GetUtcTime(), UtcTime({ 1995, 12, 31, 21, 0, 0.5 }));
    EXPECT_EQ(t1f, Time::FromUtcTime({ 1995, 12, 31, 21, 0, 0.5 }));
    EXPECT_EQ(t1f.GetGloTime(), GloTime({ 1, 1, 0.5 }));

    // After 1997-07-01 leapsecond event: 1998-01-01 03:00:00 UTC
    const Time t2 =
        Time::FromGloTime({ 1, 1 + 366 + 365, 43200.0 });  // + 366 days in 1996 + 365.5 days in 1997 + 1 leapsecond
    EXPECT_EQ(t2.GetUtcTime(), UtcTime({ 1998, 1, 1, 9, 0, 0.0 }));
    EXPECT_EQ(t2, Time::FromUtcTime({ 1998, 1, 1, 9, 0, 0.0 }));
    EXPECT_EQ(t2.GetGloTime(), GloTime({ 1, 1 + 366 + 365, 43200.0 }));

    // With .9 fractional seconds
    const Time t2f =
        Time::FromGloTime({ 1, 1 + 366 + 365, 43200.9 });  // + 366 days in 1996 + 365.5 days in 1997 + 1 leapsecond
    EXPECT_EQ(t2f.GetUtcTime(), UtcTime({ 1998, 1, 1, 9, 0, 0.9 }));
    EXPECT_EQ(t2f, Time::FromUtcTime({ 1998, 1, 1, 9, 0, 0.9 }));
    EXPECT_EQ(t2f.GetGloTime(), GloTime({ 1, 1 + 366 + 365, 43200.9 }));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_Chrono)
{
    // Before leapseconds
    const Time t1 = Time::FromSecNSec(1234, 123456789);
    EXPECT_EQ(t1.GetChronoMilli(), std::chrono::milliseconds(1234123));
    EXPECT_EQ(t1.GetChronoNano(), std::chrono::nanoseconds(1234123456789));
    // Rounding
    const Time t2 = Time::FromSecNSec(1234, 123500000);
    EXPECT_EQ(t2.GetChronoMilli(), std::chrono::milliseconds(1234124));

    // After first leapsecond
    const Time t3 = Time::FromSecNSec(78796811, 123456789);
    EXPECT_EQ(t3.GetChronoMilli(), std::chrono::milliseconds(78796810123));
    EXPECT_EQ(t3.GetChronoNano(), std::chrono::nanoseconds(78796810123456789));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_StrWnoTow)
{
    const Time t1 = Time::FromWnoTow({ 1234, 123456.123456789, WnoTow::Sys::GPS });
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 9), std::string("1234:123456.123456789"));
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 8), std::string("1234:123456.12345679"));  // round
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 7), std::string("1234:123456.1234568"));   // round
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 6), std::string("1234:123456.123457"));    // round
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 5), std::string("1234:123456.12346"));     // round
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 4), std::string("1234:123456.1235"));      // round
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 3), std::string("1234:123456.123"));
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 2), std::string("1234:123456.12"));
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 1), std::string("1234:123456.1"));
    EXPECT_EQ(t1.StrWnoTow(WnoTow::Sys::GPS, 0), std::string("1234:123456"));
    // Default system is GPS and default precision is 3
    EXPECT_EQ(t1.StrWnoTow(), std::string("1234:123456.123"));

    const Time t2 = Time::FromWnoTow({ 1234, 123456.1234567899, WnoTow::Sys::GPS });
    EXPECT_EQ(t2.nsec_, 123456790);  // 0.1234567899 rounded in FromWnoTow()
    EXPECT_EQ(t2.StrWnoTow(WnoTow::Sys::GPS, 9), std::string("1234:123456.123456790"));

    // Overflow tow -> wno
    const Time t3 = Time::FromWnoTow({ 1234, 604799.99, WnoTow::Sys::GPS });
    EXPECT_EQ(t3.StrWnoTow(WnoTow::Sys::GPS, 1), std::string("1235:000000.0"));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_StrUtcTime)
{
    const Time t1 = Time::FromUtcTime({ 2024, 1, 2, 3, 4, 5.123456789 });
    EXPECT_EQ(t1.StrUtcTime(9), std::string("2024-01-02 03:04:05.123456789"));
    EXPECT_EQ(t1.StrUtcTime(8), std::string("2024-01-02 03:04:05.12345679"));  // round
    EXPECT_EQ(t1.StrUtcTime(7), std::string("2024-01-02 03:04:05.1234568"));   // round
    EXPECT_EQ(t1.StrUtcTime(6), std::string("2024-01-02 03:04:05.123457"));    // round
    EXPECT_EQ(t1.StrUtcTime(5), std::string("2024-01-02 03:04:05.12346"));     // round
    EXPECT_EQ(t1.StrUtcTime(4), std::string("2024-01-02 03:04:05.1235"));      // round
    EXPECT_EQ(t1.StrUtcTime(3), std::string("2024-01-02 03:04:05.123"));
    EXPECT_EQ(t1.StrUtcTime(2), std::string("2024-01-02 03:04:05.12"));
    EXPECT_EQ(t1.StrUtcTime(1), std::string("2024-01-02 03:04:05.1"));
    EXPECT_EQ(t1.StrUtcTime(0), std::string("2024-01-02 03:04:05"));
    // Default precision is 3
    EXPECT_EQ(t1.StrUtcTime(), std::string("2024-01-02 03:04:05.123"));

    const Time t2 = Time::FromUtcTime({ 2024, 1, 2, 3, 4, 5.1234567899 });
    EXPECT_EQ(t2.nsec_, 123456790);  // 0.1234567899 rounded in FromUtcTime()
    EXPECT_EQ(t2.StrUtcTime(9), std::string("2024-01-02 03:04:05.123456790"));

    // Rollover sec -> min, min -> hour, hour -> day, day -> month, month -> year
    const Time t3 = Time::FromUtcTime({ 2024, 1, 2, 3, 4, 59.99 });
    EXPECT_EQ(t3.StrUtcTime(1), std::string("2024-01-02 03:05:00.0"));
    const Time t4 = Time::FromUtcTime({ 2024, 1, 2, 3, 59, 59.99 });
    EXPECT_EQ(t4.StrUtcTime(1), std::string("2024-01-02 04:00:00.0"));
    const Time t5 = Time::FromUtcTime({ 2024, 1, 2, 23, 59, 59.99 });
    EXPECT_EQ(t5.StrUtcTime(1), std::string("2024-01-03 00:00:00.0"));
    const Time t6 = Time::FromUtcTime({ 2024, 1, 31, 23, 59, 59.99 });
    EXPECT_EQ(t6.StrUtcTime(1), std::string("2024-02-01 00:00:00.0"));
    const Time t7 = Time::FromUtcTime({ 2024, 12, 31, 23, 59, 59.99 });
    EXPECT_EQ(t7.StrUtcTime(1), std::string("2025-01-01 00:00:00.0"));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_StrIsoTime)
{
    const Time t1 = Time::FromUtcTime({ 2024, 1, 2, 3, 4, 5.123456789 });
    EXPECT_EQ(t1.StrIsoTime(9), std::string("2024-01-02T03:04:05.123456789Z"));
    EXPECT_EQ(t1.StrIsoTime(8), std::string("2024-01-02T03:04:05.12345679Z"));  // round
    EXPECT_EQ(t1.StrIsoTime(7), std::string("2024-01-02T03:04:05.1234568Z"));   // round
    EXPECT_EQ(t1.StrIsoTime(6), std::string("2024-01-02T03:04:05.123457Z"));    // round
    EXPECT_EQ(t1.StrIsoTime(5), std::string("2024-01-02T03:04:05.12346Z"));     // round
    EXPECT_EQ(t1.StrIsoTime(4), std::string("2024-01-02T03:04:05.1235Z"));      // round
    EXPECT_EQ(t1.StrIsoTime(3), std::string("2024-01-02T03:04:05.123Z"));
    EXPECT_EQ(t1.StrIsoTime(2), std::string("2024-01-02T03:04:05.12Z"));
    EXPECT_EQ(t1.StrIsoTime(1), std::string("2024-01-02T03:04:05.1Z"));
    EXPECT_EQ(t1.StrIsoTime(0), std::string("2024-01-02T03:04:05Z"));
    // Default precision is 0
    EXPECT_EQ(t1.StrIsoTime(), std::string("2024-01-02T03:04:05Z"));

    const Time t2 = Time::FromUtcTime({ 2024, 1, 2, 3, 4, 5.1234567899 });
    EXPECT_EQ(t2.nsec_, 123456790);  // 0.1234567899 rounded in FromUtcTime()
    EXPECT_EQ(t2.StrIsoTime(9), std::string("2024-01-02T03:04:05.123456790Z"));

    // Rollover sec -> min, min -> hour, hour -> day, day -> month, month -> year
    const Time t3 = Time::FromUtcTime({ 2024, 1, 2, 3, 4, 59.99 });
    EXPECT_EQ(t3.StrIsoTime(1), std::string("2024-01-02T03:05:00.0Z"));
    const Time t4 = Time::FromUtcTime({ 2024, 1, 2, 3, 59, 59.99 });
    EXPECT_EQ(t4.StrIsoTime(1), std::string("2024-01-02T04:00:00.0Z"));
    const Time t5 = Time::FromUtcTime({ 2024, 1, 2, 23, 59, 59.99 });
    EXPECT_EQ(t5.StrIsoTime(1), std::string("2024-01-03T00:00:00.0Z"));
    const Time t6 = Time::FromUtcTime({ 2024, 1, 31, 23, 59, 59.99 });
    EXPECT_EQ(t6.StrIsoTime(1), std::string("2024-02-01T00:00:00.0Z"));
    const Time t7 = Time::FromUtcTime({ 2024, 12, 31, 23, 59, 59.99 });
    EXPECT_EQ(t7.StrIsoTime(1), std::string("2025-01-01T00:00:00.0Z"));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_Logic)
{
    const Time t1 = Time::FromSecNSec(1234, 123456789);
    const Time t2 = Time::FromSecNSec(1234, 123456789);
    const Time t3 = Time::FromSecNSec(1234, 123456790);
    const Time t4 = Time::FromSecNSec(1235, 123456789);

    EXPECT_TRUE(t1 == t2);
    EXPECT_TRUE(t1 != t3);
    EXPECT_TRUE(t2 != t3);
    EXPECT_TRUE(t3 != t4);
    EXPECT_FALSE(t1 < t2);
    EXPECT_FALSE(t1 > t2);
    EXPECT_TRUE(t1 <= t2);
    EXPECT_TRUE(t1 >= t2);
    EXPECT_TRUE(t1 < t3);
    EXPECT_TRUE(t3 > t1);
    EXPECT_FALSE(t1 > t3);
    EXPECT_FALSE(t3 < t1);
    EXPECT_TRUE(t2 < t3);
    EXPECT_TRUE(t3 > t2);
    EXPECT_FALSE(t2 > t3);
    EXPECT_FALSE(t3 < t2);
    EXPECT_TRUE(t3 < t4);
    EXPECT_TRUE(t4 > t3);
    EXPECT_FALSE(t3 > t4);
    EXPECT_FALSE(t4 < t3);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_Arithmetic)
{
    const int64_t seq[] = {
        (int64_t)1500000000,
        (int64_t)250000000,
        (int64_t)0,
        (int64_t)1750000000,
        (int64_t)3250000000,
        (int64_t)550000000,
        (int64_t)-4500000000,
        (int64_t)-750000000,
        (int64_t)-0,
        (int64_t)-2250000000,
        (int64_t)-1250000000,
        (int64_t)-1550000000,
    };
    // Addition sec
    {
        int64_t sum = 1000000000000;
        Time t1 = Time::FromNSec(sum);
        Time t2 = Time::FromNSec(sum);
        Time t3 = Time::FromNSec(sum);
        for (const int64_t v : seq) {
            sum += v;
            DEBUG("add sec: v=%" PRIi64 " (%.12f) -> sum=%" PRIi64 " (%.12f)", v, (double)v * 1e-9, sum,
                (double)sum * 1e-9);
            const double d = (double)v * 1e-9;
            EXPECT_TRUE(t1.AddSec(d));
            t2 = t2 + d;
            t3 += d;
            EXPECT_EQ(t1.GetNSec(), sum);
            EXPECT_EQ(t2.GetNSec(), sum);
            EXPECT_EQ(t3.GetNSec(), sum);
        }
    }
    // Addition nsec
    {
        int64_t sum = 1000000000000;
        Time t1 = Time::FromNSec(sum);
        Time t2 = Time::FromNSec(sum);
        Time t3 = Time::FromNSec(sum);
        for (const int64_t v : seq) {
            sum += v;
            DEBUG("add nsec: v=%" PRIi64 " (%.12f) -> sum=%" PRIi64 " (%.12f)", v, (double)v * 1e-9, sum,
                (double)sum * 1e-9);
            EXPECT_TRUE(t1.AddNSec(v));
            t2 = t2 + v;
            t3 += v;
            EXPECT_EQ(t1.GetNSec(), sum);
            EXPECT_EQ(t2.GetNSec(), sum);
            EXPECT_EQ(t3.GetNSec(), sum);
        }
    }
    // Addition duration
    {
        int64_t sum = 1000000000000;
        Time t1 = Time::FromNSec(sum);
        Time t2 = Time::FromNSec(sum);
        Time t3 = Time::FromNSec(sum);
        for (const int64_t v : seq) {
            sum += v;
            DEBUG("add dur: v=%" PRIi64 " (%.12f) -> sum=%" PRIi64 " (%.12f)", v, (double)v * 1e-9, sum,
                (double)sum * 1e-9);
            const Duration d = Duration::FromNSec(v);
            EXPECT_TRUE(t1.AddDur(d));
            t2 = t2 + d;
            t3 += d;
            EXPECT_EQ(t1.GetNSec(), sum);
            EXPECT_EQ(t2.GetNSec(), sum);
            EXPECT_EQ(t3.GetNSec(), sum);
        }
    }
    // Substraction sec
    {
        int64_t sum = 1000000000000;
        Time t1 = Time::FromNSec(sum);
        Time t2 = Time::FromNSec(sum);
        Time t3 = Time::FromNSec(sum);
        for (const int64_t v : seq) {
            sum -= v;
            DEBUG("sub sec: v=%" PRIi64 " (%.12f) -> sum=%" PRIi64 " (%.12f)", v, (double)v * 1e-9, sum,
                (double)sum * 1e-9);
            const double d = (double)v * 1e-9;
            EXPECT_TRUE(t1.SubSec(d));
            t2 = t2 - d;
            t3 -= d;
            EXPECT_EQ(t1.GetNSec(), sum);
            EXPECT_EQ(t2.GetNSec(), sum);
            EXPECT_EQ(t3.GetNSec(), sum);
        }
    }
    // Substraction nsec
    {
        int64_t sum = 1000000000000;
        Time t1 = Time::FromNSec(sum);
        Time t2 = Time::FromNSec(sum);
        Time t3 = Time::FromNSec(sum);
        for (const int64_t v : seq) {
            sum -= v;
            DEBUG("sub nsec: v=%" PRIi64 " (%.12f) -> sum=%" PRIi64 " (%.12f)", v, (double)v * 1e-9, sum,
                (double)sum * 1e-9);
            EXPECT_TRUE(t1.SubNSec(v));
            t2 = t2 - v;
            t3 -= v;
            EXPECT_EQ(t1.GetNSec(), sum);
            EXPECT_EQ(t2.GetNSec(), sum);
            EXPECT_EQ(t3.GetNSec(), sum);
        }
    }
    // Substraction duration
    {
        int64_t sum = 1000000000000;
        Time t1 = Time::FromNSec(sum);
        Time t2 = Time::FromNSec(sum);
        Time t3 = Time::FromNSec(sum);
        for (const int64_t v : seq) {
            sum -= v;
            DEBUG("sub dur: v=%" PRIi64 " (%.12f) -> sum=%" PRIi64 " (%.12f)", v, (double)v * 1e-9, sum,
                (double)sum * 1e-9);
            const Duration d = Duration::FromNSec(v);
            EXPECT_TRUE(t1.SubDur(d));
            t2 = t2 - d;
            t3 -= d;
            EXPECT_EQ(t1.GetNSec(), sum);
            EXPECT_EQ(t2.GetNSec(), sum);
            EXPECT_EQ(t3.GetNSec(), sum);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Time_FromSecNSec)
{
    // clang-format off
    const struct {
        uint32_t in_sec;
        uint32_t in_nsec;
        uint32_t out_sec;
        uint32_t out_nsec;
    } tests[] = {
        { 1,          1,       1,         1 },
        { 0,          0,       0,         0 },
        { 1,          0,       1,         0 },
        { 0,          1,       0,         1 },
        { 1,  999999999,       1, 999999999 },
        // Overflow
        { 1, 1000000000,       2,         0 },
        { 1, 1000000001,       2,         1 },
        { 1, 2000000001,       3,         1 },
    };
    // clang-format on

    for (auto& test : tests) {
        const Time t = Time::FromSecNSec(test.in_sec, test.in_nsec);
        EXPECT_EQ(t.sec_, test.out_sec);
        EXPECT_EQ(t.nsec_, test.out_nsec);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

#ifdef CLOCK_TAI
TEST(TimeTest, Time_Clock)
{
    const Time t_realtime = Time::FromClockRealtime();
    const Time t_tai = Time::FromClockTai();
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const double now_sys = (double)ts.tv_sec + ((double)ts.tv_nsec * 1e-9);
    clock_gettime(CLOCK_TAI, &ts);
    const double now_tai = (double)ts.tv_sec + ((double)ts.tv_nsec * 1e-9);

    // Get current TAI offset value from kernel
    struct timex tx;
    memset(&tx, 0, sizeof(tx));
    int tx_tai = 0;
    if (adjtimex(&tx) == TIME_OK) {
        tx_tai = tx.tai;
    } else {
        WARNING("adjtimex() fail: %s", StrError(errno).c_str());
    }
    DEBUG("tx_tai=%d", tx_tai);

    DEBUG("now_sys        = %.3f %+.3f", now_sys, now_sys - now_tai);
    DEBUG("now_tai        = %.3f %+.3f", now_tai, now_tai - now_sys);

    // The offset between CLOCK_TAI and Time's atomic scale
    const double tai_offs = Time::FromPosix(100).GetSec() - Time::FromTai(100).GetSec();
    DEBUG("tai_offs=%.3f", tai_offs);

    DEBUG("t_realtime     = %.3f", t_realtime.GetSec());
    DEBUG("t_tai          = %.3f", t_tai.GetSec());
    const double dt_realtime_tai = t_realtime.GetSec() - t_tai.GetSec();
    DEBUG("dt_realtime_tai = %.3f", dt_realtime_tai);
    const double dt_realtime_sys = t_realtime.GetSec() - now_sys;
    DEBUG("dt_realtime_sys = %.3f", dt_realtime_sys);

    // If the system is not configured correctly (TAI offset in kernel is 0), we should see no difference in the value
    // of CLOCK_TAI = CLOCK_REALTIME) and our atomic time from CLOCK_TAI resp. CLOCK_REALTIME should differ by the
    // offset between TAI and our atomic scale.
    // E.g. tx_tai=0
    //      now_sys        = 1731017397.390 +0.000
    //      now_tai        = 1731017397.390 +0.000
    //      SetPosix 100 lsinfo 0 false -> 100 0
    //      SetTai 100 -> 90 0
    //      tai_offs=10.000
    //      t_realtime     = 1731017424.390
    //      t_tai          = 1731017387.390
    //      dt_realtime_tai = 37.000
    //      dt_realtime_sys = 27.000
    if (tx_tai == 0) {
        ASSERT_NEAR(now_sys, now_tai, 1e-3);
        ASSERT_NEAR(dt_realtime_tai, dt_realtime_sys + tai_offs, 1e-3);
    }
    // Leap second *is* set in kernal, so we should see the offset between TAI and our atomic
    // E.g. tx_tai=37
    //      now_sys        = 1731017432.358 -37.000
    //      now_tai        = 1731017469.358 +37.000
    //      SetPosix 100 lsinfo 0 false -> 100 0
    //      SetTai 100 -> 90 0
    //      tai_offs=10.000
    //      t_realtime     = 1731017459.358
    //      t_tai          = 1731017459.358
    //      dt_realtime_tai = -0.000
    //      dt_realtime_sys = 27.000
    else {
        ASSERT_NEAR(dt_realtime_tai, 0.0, 1e-3);
    }
}
#endif  // CLOCK_TAI

// ---------------------------------------------------------------------------------------------------------------------

#define _SCOPE(s1, s2)                                                \
    std::string scope_str = std::string(s1) + ", " + std::string(s2); \
    SCOPED_TRACE(scope_str.c_str())

TEST(TimeTest, Time_Various)
{
    // const uint64_t ls1999m1 = 915148800 + 21; // -1s
    // const uint64_t ls1999t0 = 915148800 + 22; // atomic time for the 1999-01-01 00:00:00 leapsecond
    // const uint64_t ls1999p1 = 915148800 + 23; // +1s
    const uint64_t n = 1000000000;

    // clang-format off
    const struct {
        const char* desc;
        struct { uint32_t sec; uint32_t nsec; } sec_nsec;
        uint64_t nsec;
        double sec;
        time_t posix;
        RosTime rostime;
        WnoTow wnotow;
        GloTime glotime;
        UtcTime utctime;
    } tests[] = {
        // desc                  SecNSec             NSec           sec            POSIX         RosTime            WnoTow                                 GloTime                UtcTime
        { "time 0s",             {          0, 0 },             0,           0.0,           0,  {          0, 0 },  {   -1,      0.0, WnoTow::Sys::GPS },  { -1,    0,     0.0 }, {    0,  0,  0,  0,  0,  0.0 } },
        { "time 1ns",            {          0, 1 },             1,          1e-9,           0,  {          0, 1 },  {   -1,      0.0, WnoTow::Sys::GPS },  { -1,    0,     0.0 }, {    0,  0,  0,  0,  0,  0.0 } },
        { "time 1s",             {          1, 0 },    1000000000,           1.0,           1,  {          1, 0 },  {   -1,      0.0, WnoTow::Sys::GPS },  { -1,    0,     0.0 }, {    0,  0,  0,  0,  0,  0.0 } },
        { "leapsec=0",           {   31536000, 0 },    31536000*n,    31536000.0,    31536000,  {   31536000, 0 },  {   -1,      0.0, WnoTow::Sys::GPS },  { -1,    0,     0.0 }, { 1971,  1,  1,  0,  0,  0.0 } },
        { "leapsec=1",           {   78840001, 0 },    78840001*n,    78840001.0,    78840000,  {   78840000, 0 },  {   -1,      0.0, WnoTow::Sys::GPS },  { -1,    0,     0.0 }, { 1972,  7,  1, 12,  0,  0.0 } },
        { "leapsec=11",          {  829821620, 0 },   829821620*n,   829821620.0,   829821600,  {  829821600, 0 },  {  849, 381611.0, WnoTow::Sys::GPS },  {  1,  109, 46800.0 }, { 1996,  4, 18, 10,  0,  0.0 } },
        { "leapsec=27",          { 1655389303, 0 },  1655389303*n,  1655389303.0,  1655389276,  { 1655389276, 0 },  { 2214, 397294.0, WnoTow::Sys::GPS },  {  7,  898, 62476.0 }, { 2022,  6, 16, 14, 21, 16.0 } },
        { "1980-01-10 12:00:00", {  316353609, 0 },   316353609*n,  316353609.0,    316353600,  {  316353600, 0 },  {    0, 388800.0, WnoTow::Sys::GPS },  { -1,    0,     0.0 }, { 1980,  1, 10, 12,  0,  0.0 } },
        { "GNSS GPS",            { 1641092672, 0 },  1641092672*n,  1641092672.0,  1641092645,  { 1641092645, 0 },  { 2191,  11063.0, WnoTow::Sys::GPS },  {  7,  733, 21845.0 }, { 2022,  1,  2,  3,  4,  5.0 } },
        { "GNSS GAL",            { 1641092672, 0 },  1641092672*n,  1641092672.0,  1641092645,  { 1641092645, 0 },  { 1167,  11063.0, WnoTow::Sys::GAL },  { -1,    0,     0.0 }, { 2022,  1,  2,  3,  4,  5.0 } },
        { "GNSS BDS",            { 1641092672, 0 },  1641092672*n,  1641092672.0,  1641092645,  { 1641092645, 0 },  {  835,  11049.0, WnoTow::Sys::BDS },  { -1,    0,     0.0 }, { 2022,  1,  2,  3,  4,  5.0 } },
        // ublox receiver output
        // message  228, dt   47, size   24, UBX      UBX-NAV-TIMEGPS      312626.000 2326 Y 000312625.999510726 Y 4.000000e-09 18 Y
        // message  229, dt   47, size   28, UBX      UBX-NAV-TIMEGLO      312626.000 008 0220 Y 64207.999510721 Y 5.000000e-09
        // message  230, dt   47, size   28, UBX      UBX-NAV-TIMEUTC      312626.000 Y 2024-08-07 Y 14:50:08.000 Y ?
        { "u-blox rx out",       { 1723042235, 0 },  1723042235*n,  1723042235.0,  1723042208,  { 1723042208, 0 },  { 2326, 312626.0, WnoTow::Sys::GPS },  {  8,  220, 64208.0 }, { 2024,  8,  7, 14, 50,  8.0 } },
    };

    // clang-format on
    for (auto& test : tests) {
        INFO("----- %s: secn_sec %" PRIu32 " %" PRIu32, test.desc, test.sec_nsec.sec, test.sec_nsec.nsec);
        // From sec, nsec
        {
            _SCOPE(test.desc, "FromSecNSec");
            const Time t1 = Time::FromSecNSec(test.sec_nsec.sec, test.sec_nsec.nsec);
            EXPECT_EQ(t1.IsZero(), test.nsec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, test.sec_nsec.nsec);
            EXPECT_EQ(t1.GetNSec(), test.nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), test.rostime);
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
        // From nsec
        {
            _SCOPE(test.desc, "FromNSec");
            Time t1 = Time::FromNSec(test.nsec);
            EXPECT_EQ(t1.IsZero(), test.nsec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, test.sec_nsec.nsec);
            EXPECT_EQ(t1.GetNSec(), test.nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), test.rostime);
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
        // From sec
        {
            _SCOPE(test.desc, "FromSec");
            Time t1 = Time::FromSec(test.sec);
            EXPECT_EQ(t1.IsZero(), test.nsec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, test.sec_nsec.nsec);
            EXPECT_EQ(t1.GetNSec(), test.nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), test.rostime);
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
        // From posix
        {
            _SCOPE(test.desc, "FromPosix");
            Time t1 = Time::FromPosix(test.posix);
            // POSIX is integer only, so nsec_ will always be zeor resp / 1e9
            const uint64_t test_nsec = (test.nsec / (uint64_t)1000000000) * (uint64_t)1000000000;
            EXPECT_EQ(t1.IsZero(), test.sec_nsec.sec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, 0);
            EXPECT_EQ(t1.GetNSec(), test_nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), RosTime(test.rostime.sec_, 0));
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
        // From wnotow
        if (test.wnotow.wno_ >= 0) {
            _SCOPE(test.desc, "FromWnoTow");
            Time t1 = Time::FromWnoTow(test.wnotow);
            EXPECT_EQ(t1.IsZero(), test.nsec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, test.sec_nsec.nsec);
            EXPECT_EQ(t1.GetNSec(), test.nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), test.rostime);
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
        // From utc
        if (test.utctime.year_ > 0) {
            _SCOPE(test.desc, "FromUtcTime");
            Time t1 = Time::FromUtcTime(test.utctime);
            EXPECT_EQ(t1.IsZero(), test.nsec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, test.sec_nsec.nsec);
            EXPECT_EQ(t1.GetNSec(), test.nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), test.rostime);
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
        // From glotime
        if (test.glotime.N4_ >= 0) {
            _SCOPE(test.desc, "FromGloTime");
            Time t1 = Time::FromGloTime(test.glotime);
            EXPECT_EQ(t1.IsZero(), test.nsec == 0);
            EXPECT_EQ(t1.sec_, test.sec_nsec.sec);
            EXPECT_EQ(t1.nsec_, test.sec_nsec.nsec);
            EXPECT_EQ(t1.GetNSec(), test.nsec);
            EXPECT_NEAR(t1.GetSec(), test.sec, 1e-9);
            EXPECT_EQ(t1.GetPosix(), test.posix);
            EXPECT_EQ(t1.GetRosTime(), test.rostime);
            if (test.wnotow.wno_ >= 0) {
                EXPECT_EQ(t1.GetWnoTow(test.wnotow.sys_), test.wnotow);
            }
            if (test.glotime.N4_ >= 0) {
                EXPECT_EQ(t1.GetGloTime(), test.glotime);
            }
            if (test.utctime.year_ > 0) {
                EXPECT_EQ(t1.GetUtcTime(), test.utctime);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, GetDayOfYear)
{
    // clang-format off
    const struct { UtcTime utc; int prec; double doy; double sigma; } tests[] = {

        { { 2023, 1, 1, 0, 0, 0.0 },  0,   1,          1e-12 },
        { { 2023, 2, 1, 0, 0, 0.0 },  0,  31 +  1,     1e-12 },
        { { 2023, 3, 1, 0, 0, 0.0 },  0,  31 + 28 + 1, 1e-12 },

        { { 2024, 1, 1, 0, 0, 0.0 },  0,   1,          1e-12 },
        { { 2024, 2, 1, 0, 0, 0.0 },  0,  31 +  1,     1e-12 },
        { { 2024, 3, 1, 0, 0, 0.0 },  0,  31 + 29 + 1, 1e-12 },

        { { 2000, 1, 1, 0, 0, 0.0 },  0,   1,          1e-12 },
        { { 2000, 2, 1, 0, 0, 0.0 },  0,  31 +  1,     1e-12 },
        { { 2000, 3, 1, 0, 0, 0.0 },  0,  31 + 29 + 1, 1e-12 },

        // { { 2100, 1, 1, 0, 0, 0.0 },  0,   1,          1e-12 },
        // { { 2100, 2, 1, 0, 0, 0.0 },  0,  31 +  1,     1e-12 },
        // { { 2100, 3, 1, 0, 0, 0.0 },  0,  31 + 28 + 1, 1e-12 },

        { { 2024, 1, 1, 11, 59, 59.999 },  0,  1.0,  1e-12 },
        { { 2024, 1, 1, 12,  0,  0.000 },  0,  2.0,  1e-12 },
        { { 2024, 1, 1, 12,  0,  0.000 },  1,  1.5,  1e-12 },

    };  // clang-format on

    for (auto& test : tests) {
        const auto t = Time::FromUtcTime(test.utc);
        EXPECT_NEAR(t.GetDayOfYear(test.prec), test.doy, test.sigma) << test.utc;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, Diff)
{
    // clang-format off
    const struct {
        uint32_t t1_sec; uint32_t t1_nsec; uint32_t t2_sec; uint32_t t2_nsec;
        bool d_ok; int32_t d_sec; int32_t d_nsec; } tests[] =
    {

        { 0, 0,                         0, 0,                          true,  0, 0 },
        { 1, 0,                         0, 0,                          true,  1, 0 },
        { 0, 0,                         1, 0,                          true, -1, 0 },
        { UINT32_MAX, 0,                0, 0,                          false, 0, 0 },
        { 0, 0,                         UINT32_MAX, 0,                 false, 0, 0 },
        { (uint32_t)INT32_MAX + 1, 0,   0, 0,                          false, 0, 0 },
        { 0, 0,                         (uint32_t)INT32_MAX + 2, 0,    false, 0, 0 },
        { INT32_MAX, 0,                 0, 0,                          true, INT32_MAX, 0 },

    };  // clang-format on

    for (auto& test : tests) {
        const auto t1 = Time::FromSecNSec(test.t1_sec, test.t1_nsec);
        const auto t2 = Time::FromSecNSec(test.t2_sec, test.t2_nsec);
        {
            Duration d;
            const bool d_ok = t1.Diff(t2, d);
            DEBUG("t1=%.9f t2=%.9f d_ok=%s d=%.9f", t1.GetSec(), t2.GetSec(), ToStr(d_ok), d.GetSec());
            EXPECT_EQ(d_ok, test.d_ok);
            EXPECT_EQ(d.sec_, test.d_sec);
            EXPECT_EQ(d.nsec_, test.d_nsec);
        }
        if (test.d_ok) {
            const Duration d = t1 - t2;
            EXPECT_EQ(d.sec_, test.d_sec);
            EXPECT_EQ(d.nsec_, test.d_nsec);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, SetCurrentLeapseconds)
{
    const uint32_t sec0 = 4118083225;  // 2100-06-30 23:59:58.0 (at the time of writing)

    // Silence logging and trigger the "leapseconds knowledge outdated" warning, so that the following GetPosix() etc.
    // don't carp again
    {
        using namespace fpsdk::common::logging;
        auto params = LoggingGetParams();
        const auto old_level = params.level_;
        params.level_ = LoggingLevel::ERROR;
        LoggingSetParams(params);
        auto t = Time::FromSecNSec(sec0, 0);
        DEBUG("dummy %s", t.StrUtcTime().c_str());
        params.level_ = old_level;
        LoggingSetParams(params);
    }

    // Without current leapseconds set
    const auto at1 = Time::FromSecNSec(sec0, 0);
    const auto at2 = Time::FromSecNSec(at1.sec_ + 1, 0);
    const auto at3 = Time::FromSecNSec(at1.sec_ + 2, 0);
    const auto at4 = Time::FromSecNSec(at1.sec_ + 3, 0);
    const auto at5 = Time::FromSecNSec(at1.sec_ + 4, 0);

    const int64_t as1 = (int64_t)at1.sec_;
    const int64_t as2 = (int64_t)at2.sec_;
    const int64_t as3 = (int64_t)at3.sec_;
    const int64_t as4 = (int64_t)at4.sec_;
    const int64_t as5 = (int64_t)at5.sec_;
    const int64_t ap1 = (int64_t)at1.GetPosix();
    const int64_t ap2 = (int64_t)at2.GetPosix();
    const int64_t ap3 = (int64_t)at3.GetPosix();
    const int64_t ap4 = (int64_t)at4.GetPosix();
    const int64_t ap5 = (int64_t)at5.GetPosix();

    // Latest leapseconds from built-in table (assuming it ends long before at1)
    const int adiff = as1 - ap1;
    const int aleaps = at1.GetLeapseconds();
    DEBUG("adiff=%d aleaps=%d", adiff, aleaps);  // 27, 37

    // clang-format off
    DEBUG("at1: %s %" PRIi64 " %" PRIi64 " %" PRIi64, at1.StrUtcTime().c_str(), as1, ap1, as1 - ap1);
    DEBUG("at2: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, at2.StrUtcTime().c_str(), as2, ap2, as2 - ap2, as2 - as1, ap2 - ap1);
    DEBUG("at3: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, at3.StrUtcTime().c_str(), as3, ap3, as3 - ap3, as3 - as2, ap3 - ap2);
    DEBUG("at4: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, at4.StrUtcTime().c_str(), as4, ap4, as4 - ap4, as4 - as3, ap4 - ap3);
    DEBUG("at5: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, at5.StrUtcTime().c_str(), as5, ap5, as5 - ap5, as5 - as4, ap5 - ap4);
    // at1: 2100-06-30 23:59:58.000 4118083225 4118083198 27
    // at2: 2100-06-30 23:59:59.000 4118083226 4118083199 27 1 1
    // at3: 2100-07-01 00:00:00.000 4118083227 4118083200 27 1 1
    // at4: 2100-07-01 00:00:01.000 4118083228 4118083201 27 1 1
    // at5: 2100-07-01 00:00:02.000 4118083229 4118083202 27 1 1
    // clang-format on

    // Atomic time increments by 1
    EXPECT_EQ(as2 - as1, 1);
    EXPECT_EQ(as3 - as2, 1);
    EXPECT_EQ(as4 - as3, 1);
    EXPECT_EQ(as5 - as4, 1);

    // POSIX time increments by 1
    EXPECT_EQ(ap2 - ap1, 1);
    EXPECT_EQ(ap3 - ap2, 1);
    EXPECT_EQ(ap4 - ap3, 1);
    EXPECT_EQ(ap5 - ap4, 1);

    // Delta (= leapseconds) is constant
    EXPECT_EQ(as1 - ap1, adiff);
    EXPECT_EQ(as2 - ap2, adiff);
    EXPECT_EQ(as3 - ap3, adiff);
    EXPECT_EQ(as4 - ap4, adiff);
    EXPECT_EQ(as5 - ap5, adiff);

    // With current leapseconds set
    const int bdiff = adiff + 1;
    const int bleaps = aleaps + 1;
    DEBUG("bdiff=%d bleaps=%d", bdiff, bleaps);
    EXPECT_TRUE(at3.SetCurrentLeapseconds(bleaps));

    const auto bt1 = Time::FromSecNSec(sec0, 0);
    const auto bt2 = Time::FromSecNSec(bt1.sec_ + 1, 0);
    const auto bt3 = Time::FromSecNSec(bt1.sec_ + 2, 0);
    const auto bt4 = Time::FromSecNSec(bt1.sec_ + 3, 0);
    const auto bt5 = Time::FromSecNSec(bt1.sec_ + 4, 0);

    const int64_t bs1 = (int64_t)bt1.sec_;
    const int64_t bs2 = (int64_t)bt2.sec_;
    const int64_t bs3 = (int64_t)bt3.sec_;
    const int64_t bs4 = (int64_t)bt4.sec_;
    const int64_t bs5 = (int64_t)bt5.sec_;
    const int64_t bp1 = (int64_t)bt1.GetPosix();
    const int64_t bp2 = (int64_t)bt2.GetPosix();
    const int64_t bp3 = (int64_t)bt3.GetPosix();
    const int64_t bp4 = (int64_t)bt4.GetPosix();
    const int64_t bp5 = (int64_t)bt5.GetPosix();

    // clang-format off
    DEBUG("bt1: %s %" PRIi64 " %" PRIi64 " %" PRIi64, bt1.StrUtcTime().c_str(), bs1, bp1, bs1 - bp1);
    DEBUG("bt2: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, bt2.StrUtcTime().c_str(), bs2, bp2, bs2 - bp2, bs2 - bs1, bp2 - bp1);
    DEBUG("bt3: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, bt3.StrUtcTime().c_str(), bs3, bp3, bs3 - bp3, bs3 - bs2, bp3 - bp2);
    DEBUG("bt4: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, bt4.StrUtcTime().c_str(), bs4, bp4, bs4 - bp4, bs4 - bs3, bp4 - bp3);
    DEBUG("bt5: %s %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64 " %" PRIi64, bt5.StrUtcTime().c_str(), bs5, bp5, bs5 - bp5, bs5 - bs4, bp5 - bp4);
    // bt1: 2100-06-30 23:59:58.000 4118083225 4118083198 27
    // bt2: 2100-06-30 23:59:59.000 4118083226 4118083199 27 1 1
    // bt3: 2100-06-30 23:59:60.000 4118083227 4118083200 27 1 1  // at event
    // bt4: 2100-07-01 00:00:00.000 4118083228 4118083200 28 1 0
    // bt5: 2100-07-01 00:00:01.000 4118083229 4118083201 28 1 1
    // clang-format on

    // Atomic time increments by 1
    EXPECT_EQ(bs2 - bs1, 1);
    EXPECT_EQ(bs3 - bs2, 1);
    EXPECT_EQ(bs4 - bs3, 1);
    EXPECT_EQ(bs5 - bs4, 1);

    // // POSIX time increments by 1, but not at / just after the event
    EXPECT_EQ(bp2 - bp1, 1);
    EXPECT_EQ(bp3 - bp2, 1);
    EXPECT_EQ(bp4 - bp3, 0);
    EXPECT_EQ(bp5 - bp4, 1);

    // Delta (= leapseconds) changes
    EXPECT_EQ(bs1 - bp1, adiff);
    EXPECT_EQ(bs2 - bp2, adiff);
    EXPECT_EQ(bs3 - bp3, adiff);  // at event
    EXPECT_EQ(bs4 - bp4, bdiff);
    EXPECT_EQ(bs5 - bp5, bdiff);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, PosixNs)
{
    const auto t1 = Time::FromSecNSec(800000000, 123456789);
    EXPECT_EQ(t1.GetPosixNs(), (uint64_t)799999981123456789);

    const auto t2 = Time::FromPosixNs(799999981123456789);
    EXPECT_EQ(t2.GetNSec(), 800000000123456789);

    EXPECT_EQ(t1.GetRosTime().ToNSec(), t1.GetPosixNs());
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, PosixSec)
{
    const auto t1 = Time::FromSecNSec(800000000, 123456789);
    EXPECT_NEAR(t1.GetPosixSec(), 799999981.123456789, 1e-9);

    const auto t2 = Time::FromPosixSec(799999981.123456789);
    EXPECT_NEAR(t2.GetSec(), 800000000.123456789, 1e-9);

    EXPECT_EQ(t1.GetRosTime().ToSec(), t1.GetPosixSec());
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, TaiSec)
{
    const auto t1 = Time::FromSecNSec(800000000, 123456789);
    EXPECT_NEAR(t1.GetTaiSec(), 800000010.123456789, 1e-9);

    const auto t2 = Time::FromTaiSec(800000010.123456789);
    EXPECT_NEAR(t2.GetSec(), 800000000.123456789, 1e-9);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, TaiNs)
{
    const auto t1 = Time::FromSecNSec(800000000, 123456789);
    EXPECT_EQ(t1.GetTaiNs(), (uint64_t)800000010123456789);

    const auto t2 = Time::FromTaiNs(800000010123456789);
    EXPECT_EQ(t2.GetNSec(), 800000000123456789);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(TimeTest, SecNsec)
{
    const double s1 = 0.123454321;
    const uint64_t n1 = 123454321;
    EXPECT_EQ(SecToNsec(s1), n1);
    EXPECT_NEAR(NsecToSec(n1), s1, 1e-9);

    const double s2 = 1751348822.123454321;
    const uint64_t n2 = 1751348822123454321;
    const int64_t d2 = SecToNsec(s2) - n2;
    EXPECT_LE(d2 < 0 ? -d2 : d2, 200);
    EXPECT_NEAR(NsecToSec(n2), s2, 1e-9);
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
