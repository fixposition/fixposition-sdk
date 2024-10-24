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
 * @brief Fixposition SDK: tests for fpsdk::common::string
 */

/* LIBC/STL */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/string.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::string;

TEST(StringTest, Sprintf)
{
    const uint32_t v1 = 1234;
    const uint32_t v2 = 42;
    const double v3 = 12.345;
    const std::string str = Sprintf("%" PRIu32 " %05" PRIu32 " %5.2f", v1, v2, v3);
    EXPECT_EQ(str, "1234 00042 12.35");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, Strftime)
{
    EXPECT_EQ(Strftime("%Y-%m-%d %H:%M:%S", 1, true), "1970-01-01 00:00:01");
    EXPECT_EQ(Strftime(NULL, 2, true), "1970-01-01 00:00:02");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrReplace)
{
    {
        std::string str = "foo bar baz foo";
        EXPECT_EQ(StrReplace(str, "xxx", "123"), 0);
        EXPECT_EQ(StrReplace(str, "bar", "123"), 1);
        EXPECT_EQ(str, "foo 123 baz foo");
        EXPECT_EQ(StrReplace(str, "foo", "456"), 2);
        EXPECT_EQ(str, "456 123 baz 456");
    }
    {
        std::string str = "0foo1bar2foo3bar4foo5bar6";
        EXPECT_EQ(StrReplace(str, "foo", ""), 3);
        EXPECT_EQ(str, "01bar23bar45bar6");
        EXPECT_EQ(StrReplace(str, "bar", "", 2), 2);
        EXPECT_EQ(str, "012345bar6");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrTrimLeft)
{
    std::string str = "  foo  ";
    StrTrimLeft(str);
    EXPECT_EQ(str, "foo  ");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrTrimRight)
{
    std::string str = "  foo  ";
    StrTrimRight(str);
    EXPECT_EQ(str, "  foo");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrTrim)
{
    std::string str = " \r \n \t foo \r \n\n \t ";
    StrTrim(str);
    EXPECT_EQ(str, "foo");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrSplit)
{
    {
        const std::string str = "foo,bar,baz";
        const auto split = StrSplit(str, ",");
        EXPECT_EQ(split.size(), (std::size_t)3);
        EXPECT_EQ(split.at(0), "foo");
        EXPECT_EQ(split.at(1), "bar");
        EXPECT_EQ(split.at(2), "baz");
    }
    {
        const std::string str = "foo,bar,baz";
        const auto split = StrSplit(str, ",", 2);
        EXPECT_EQ(split.size(), (std::size_t)2);
        EXPECT_EQ(split.at(0), "foo");
        EXPECT_EQ(split.at(1), "bar,baz");
    }
    {
        const std::string str = "foo,,baz,,,";
        const auto split = StrSplit(str, ",");
        EXPECT_EQ(split.size(), (std::size_t)6);
        EXPECT_EQ(split.at(0), "foo");
        EXPECT_EQ(split.at(1), "");
        EXPECT_EQ(split.at(2), "baz");
        EXPECT_EQ(split.at(3), "");
        EXPECT_EQ(split.at(4), "");
        EXPECT_EQ(split.at(5), "");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrJoin)
{
    const std::vector<std::string> vec = { "foo", "bar", "baz" };
    EXPECT_EQ(StrJoin(vec, ","), "foo,bar,baz");
    EXPECT_EQ(StrJoin(vec, ""), "foobarbaz");
    EXPECT_EQ(StrJoin(vec, "--"), "foo--bar--baz");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrMap)
{
    const auto res = StrMap({ "foo", "bar", "baz" }, [](const auto in) { return "-" + in + "-"; });
    EXPECT_EQ(res.size(), 3);
    EXPECT_EQ(res[0], "-foo-");
    EXPECT_EQ(res[1], "-bar-");
    EXPECT_EQ(res[2], "-baz-");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, MakeUnique)
{
    std::vector<std::string> vec = { "foo", "bar", "", "foo", "baz", "", "abc" };
    MakeUnique(vec);
    EXPECT_EQ(vec.size(), (std::size_t)5);
    EXPECT_EQ(vec.at(0), "foo");
    EXPECT_EQ(vec.at(1), "bar");
    EXPECT_EQ(vec.at(2), "");
    EXPECT_EQ(vec.at(3), "baz");
    EXPECT_EQ(vec.at(4), "abc");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, HexDump)
{
    const std::vector<uint8_t> data = {
        /* clang-format off */
        0xde, 0xad, 0xbe, 0xef, 0xba, 0xad, 0xf0, 0x0d, 0x00, 0xff, 0x55, 0xaa, 0x01, 0x02,
        0x03, 0x04, 0x10, 0x20, 0x30, 0x40, 0xaa, 0xbb, 0xcc, 0xdd, 0x61, 0x62, 0x63, 0x64,
    };  // clang-format on
    const auto dump = HexDump(data);
    EXPECT_EQ(dump.size(), (std::size_t)2);
    EXPECT_EQ(dump.at(0), "0x0000 00000  de ad be ef ba ad f0 0d  00 ff 55 aa 01 02 03 04  |..........U.....|");
    EXPECT_EQ(dump.at(1), "0x0010 00016  10 20 30 40 aa bb cc dd  61 62 63 64              |. 0@....abcd    |");
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrStartsWith)
{
    EXPECT_EQ(StrStartsWith("", ""), false);
    EXPECT_EQ(StrStartsWith("foo", ""), false);
    EXPECT_EQ(StrStartsWith("", "foo"), false);
    EXPECT_EQ(StrStartsWith("foo", "bar"), false);
    EXPECT_EQ(StrStartsWith("foo", "x"), false);
    EXPECT_EQ(StrStartsWith("foo", "fooo"), false);
    EXPECT_EQ(StrStartsWith("foo", "f"), true);
    EXPECT_EQ(StrStartsWith("foo", "fo"), true);
    EXPECT_EQ(StrStartsWith("foo", "foo"), true);
    EXPECT_EQ(StrStartsWith("foo", "fooo"), false);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrEndsWith)
{
    EXPECT_EQ(StrEndsWith("", ""), false);
    EXPECT_EQ(StrEndsWith("foo", ""), false);
    EXPECT_EQ(StrEndsWith("", "foo"), false);
    EXPECT_EQ(StrEndsWith("foo", "bar"), false);
    EXPECT_EQ(StrEndsWith("foo", "x"), false);
    EXPECT_EQ(StrEndsWith("foo", "o"), true);
    EXPECT_EQ(StrEndsWith("foo", "oo"), true);
    EXPECT_EQ(StrEndsWith("foo", "foo"), true);
    EXPECT_EQ(StrEndsWith("foo", "fooo"), false);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrContains)
{
    EXPECT_EQ(StrContains("", ""), false);
    EXPECT_EQ(StrContains("foo", ""), false);
    EXPECT_EQ(StrContains("", "foo"), false);
    EXPECT_EQ(StrContains("foo", "x"), false);
    EXPECT_EQ(StrContains("foo", "xyz"), false);
    EXPECT_EQ(StrContains("foo", "abcde"), false);
    EXPECT_EQ(StrContains("foo", "f"), true);
    EXPECT_EQ(StrContains("foo", "o"), true);
    EXPECT_EQ(StrContains("foo", "fo"), true);
    EXPECT_EQ(StrContains("foo", "oo"), true);
    EXPECT_EQ(StrContains("foo", "foo"), true);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(StringTest, StrToValue_int8)
{
    // clang-format off
    const struct { const char* str; int8_t value; bool res; } tests[] = {
        { "123",          123, true  }, // Valid decimal value
        { "077",          077, true  }, // Valid octal value
        { "0x12",        0x12, true  }, // Valid hex value
        { "-123",        -123, true  }, // Valid negative decimal value
        { "127",          127, true  }, // Max range (INT8_MAX)
        { "-128",        -128, true  }, // Min range (INT8_MIN)
        { "128",            0, false }, // Out of range (INT8_MAX + 1)
        { "-129",           0, false }, // Out of range (INT8_MIN - 1)
        { " 123",           0, false }, // Spurious whitespace
        { "123 ",           0, false }, // Spurious whitespace
        { "080",            0, false }, // Illegal octal
        { "0xag",           0, false }, // Illegal hex
        { "",               0, false }, // Empty string
        { "  ",             0, false }, // No value
        { "abc",            0, false }, // Not a value
    };  // clang-format on

    const int8_t canary = 0x55;
    for (auto& test : tests) {
        int8_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_uint8)
{
    // clang-format off
    const struct { const char* str; uint8_t value; bool res; } tests[] = {
        { "123",          123, true  }, // Valid decimal value
        { "077",          077, true  }, // Valid octal value
        { "0x12",        0x12, true  }, // Valid hex value
        { "255",          255, true  }, // Max range (UINT8_MAX)
        { "0",              0, true  }, // Min range (UINT8_MIN)
        { "256",            0, false }, // Out of range (UINT8_MAX + 1)
        { "-1",             0, false }, // Out of range (UINT8_MIN - 1)
        { "-123",           0, false }, // Negative value
        { " 123",           0, false }, // Spurious whitespace
        { "123 ",           0, false }, // Spurious whitespace
        { "0800",           0, false }, // Illegal octal
        { "0xabg",          0, false }, // Illegal hex
        { "",               0, false }, // Empty string
        { "  ",             0, false }, // No value
        { "abc",            0, false }, // Not a value
    };  // clang-format on

    const uint8_t canary = 0x55;
    for (auto& test : tests) {
        uint8_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_int16)
{
    // clang-format off
    const struct { const char* str; int16_t value; bool res; } tests[] = {
        { "123",          123, true  }, // Valid decimal value
        { "0777",        0777, true  }, // Valid octal value
        { "0x123",      0x123, true  }, // Valid hex value
        { "-123",        -123, true  }, // Valid negative decimal value
        { "32767",      32767, true  }, // Max range (INT16_MAX)
        { "-32768",    -32768, true  }, // Min range (INT16_MIN)
        { "32768",          0, false }, // Out of range (INT16_MAX + 1)
        { "-32769",         0, false }, // Out of range (INT16_MIN - 1)
        { " 123",           0, false }, // Spurious whitespace
        { "123 ",           0, false }, // Spurious whitespace
        { "0800",           0, false }, // Illegal octal
        { "0xabg",          0, false }, // Illegal hex
        { "",               0, false }, // Empty string
        { "  ",             0, false }, // No value
        { "abc",            0, false }, // Not a value
    };  // clang-format on

    const int16_t canary = 0x5555;
    for (auto& test : tests) {
        int16_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_uint16)
{
    // clang-format off
    const struct { const char* str; uint16_t value; bool res; } tests[] = {
        { "123",          123, true  }, // Valid decimal value
        { "0777",        0777, true  }, // Valid octal value
        { "0x123",      0x123, true  }, // Valid hex value
        { "65535",      65535, true  }, // Max range (UINT16_MAX)
        { "0",              0, true  }, // Max range (UINT16_MIN)
        { "65536",          0, false }, // Out of range (UINT16_MAX + 1)
        { "-1",             0, false }, // Out of range (UINT16_MIN - 1)
        { "-123",           0, false }, // Negative value
        { " 123",           0, false }, // Spurious whitespace
        { "123 ",           0, false }, // Spurious whitespace
        { "0800",           0, false }, // Illegal octal
        { "0xabg",          0, false }, // Illegal hex
        { "",               0, false }, // Empty string
        { "  ",             0, false }, // No value
        { "abc",            0, false }, // Not a value
    };  // clang-format on

    const uint16_t canary = 0x5555;
    for (auto& test : tests) {
        uint16_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_int32)
{
    // clang-format off
    const struct { const char* str; int32_t value; bool res; } tests[] = {
        { "123",                123, true  }, // Valid decimal value
        { "0777",              0777, true  }, // Valid octal value
        { "0x123",            0x123, true  }, // Valid hex value
        { "-123",              -123, true  }, // Valid negative decimal value
        { "2147483647",  2147483647, true  }, // Max range (INT32_MAX)
        { "0",                    0, true  }, // Min range (INT32_MIN)
        { "2147483648",           0, false }, // Out of range (INT32_MAX + 1)
        { "-2147483649",          0, false }, // Out of range (INT32_MIN - 1)
        { " 123",                 0, false }, // Spurious whitespace
        { "123 ",                 0, false }, // Spurious whitespace
        { "0800",                 0, false }, // Illegal octal
        { "0xabg",                0, false }, // Illegal hex
        { "",                     0, false }, // Empty string
        { "  ",                   0, false }, // No value
        { "abc",                  0, false }, // Not a value
    };  // clang-format on

    const int32_t canary = 0x55555555;
    for (auto& test : tests) {
        int32_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_uint32)
{
    // clang-format off
    const struct { const char* str; uint32_t value; bool res; } tests[] = {
        { "123",                123, true  }, // Valid decimal value
        { "0777",              0777, true  }, // Valid octal value
        { "0x123",            0x123, true  }, // Valid hex value
        { "4294967295",  4294967295, true  }, // Max range (UINT32_MAX)
        { "0",                    0, true  }, // Min range (UINT32_MIN)
        { "4294967296",           0, false }, // Out of range (UINT32_MAX + 1)
        { "-1",                   0, false }, // Out of range (UINT32_MIN - 1)
        { "-123",                 0, false }, // Negative value
        { " 123",                 0, false }, // Spurious whitespace
        { "123 ",                 0, false }, // Spurious whitespace
        { "0800",                 0, false }, // Illegal octal
        { "0xabg",                0, false }, // Illegal hex
        { "",                     0, false }, // Empty string
        { "  ",                   0, false }, // No value
        { "abc",                  0, false }, // Not a value
    };  // clang-format on

    const uint32_t canary = 0x55555555;
    for (auto& test : tests) {
        uint32_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_int64)
{
    // clang-format off
    const struct { const char* str; int64_t value; bool res; } tests[] = {
        { "123",                                    123, true  }, // Valid decimal value
        { "0777",                                  0777, true  }, // Valid octal value
        { "0x123",                                0x123, true  }, // Valid hex value
        { "-123",                                  -123, true  }, // Valid negative decimal value
        { "9223372036854775806",    9223372036854775806, true  }, // Max range (INT64_MAX - 1)
        { "-9223372036854775807",  -9223372036854775807, true  }, // Min range (UINT8_MIN)
        { "9223372036854775807",                      0, false }, // Out of range (INT64_MAX)
        { "-9223372036854775808",                     0, false }, // Out of range (INT64_MIN)
        { " 123",                                     0, false }, // Spurious whitespace
        { "123 ",                                     0, false }, // Spurious whitespace
        { "0800",                                     0, false }, // Illegal octal
        { "0xabg",                                    0, false }, // Illegal hex
        { "",                                         0, false }, // Empty string
        { "  ",                                       0, false }, // No value
        { "abc",                                      0, false }, // Not a value
        { "17179869184",                    17179869184, true  }, // = 2^34 (larger than 2^32, i.e. int32_t)
        { "-17179869184",                  -17179869184, true  }, // = -2^34 (smaller than -2^32, i.e. int32_t)
    };  // clang-format on

    const int64_t canary = 0x5555555555555555;
    for (auto& test : tests) {
        int64_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_uint64)
{
    // clang-format off
    const struct { const char* str; uint64_t value; bool res; } tests[] = {
        { "123",                                    123, true  }, // Valid decimal value
        { "0777",                                  0777, true  }, // Valid octal value
        { "0x123",                                0x123, true  }, // Valid hex value
        { "18446744073709551614", 18446744073709551614u, true  }, // Max range (UINT64_MAX - 1)
        { "0",                                        0, true  }, // Min range (UINT65_MIN)
        { "18446744073709551615",                     0, false }, // Out of range (UINT64_MAX)
        { "-1",                                       0, false }, // Out of range (UINT64_MIN - 1)
        { "-123",                                     0, false }, // Negative value
        { " 123",                                     0, false }, // Spurious whitespace
        { "123 ",                                     0, false }, // Spurious whitespace
        { "0800",                                     0, false }, // Illegal octal
        { "0xabg",                                    0, false }, // Illegal hex
        { "",                                         0, false }, // Empty string
        { "  ",                                       0, false }, // No value
        { "abc",                                      0, false }, // Not a value
        { "17179869184",                    17179869184, true  }, // = 2^34 (larger than 2^32, i.e. uint32_t)
    };  // clang-format on

    const uint64_t canary = 0x5555555555555555;
    for (auto& test : tests) {
        uint64_t value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_float)
{
    // clang-format off
    const struct { const char* str; float value; bool res; } tests[] = {
        { "123",                123.000f, true  }, // Valid decimal value
        { "123.456",            123.456f, true  }, // Valid decimal value
        { "123456e-3",          123.456f, true  }, // Valid scientific value
        { "-123.456",          -123.456f, true  }, // Valid negative decimal value
        { "-123456e-3",        -123.456f, true  }, // Valid negative scientific value
        { "0",                    0.0f,   true  },
        { "0.0",                  0.0f,   true  },
        { " 123",                 0.0f,   false }, // Spurious whitespace
        { "123 ",                 0.0f,   false }, // Spurious whitespace
        { "",                     0.0f,   false }, // Empty string
        { "  ",                   0.0f,   false }, // No value
        { "abc",                  0.0f,   false }, // Not a value
    };  // clang-format on

    const float canary = 3.14159265359;
    for (auto& test : tests) {
        float value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_DOUBLE_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToValue_double)
{
    // clang-format off
    const struct { const char* str; double value; bool res; } tests[] = {
        { "123",                123.000, true  }, // Valid decimal value
        { "123.456",            123.456, true  }, // Valid decimal value
        { "123456e-3",          123.456, true  }, // Valid scientific value
        { "-123.456",          -123.456, true  }, // Valid negative decimal value
        { "-123456e-3",        -123.456, true  }, // Valid negative scientific value
        { "0",                    0.0,   true  },
        { "0.0",                  0.0,   true  },
        { " 123",                 0.0,   false }, // Spurious whitespace
        { "123 ",                 0.0,   false }, // Spurious whitespace
        { "",                     0.0,   false }, // Empty string
        { "  ",                   0.0,   false }, // No value
        { "abc",                  0.0,   false }, // Not a value
    };  // clang-format on

    const double canary = 3.14159265359;
    for (auto& test : tests) {
        double value = canary;
        const bool res = StrToValue(test.str, value);
        EXPECT_EQ(res, test.res);
        if (res) {
            EXPECT_DOUBLE_EQ(value, test.value);
        } else {
            EXPECT_EQ(value, canary);
        }
    }
}

TEST(StringTest, StrToUpper)
{
    EXPECT_EQ(StrToUpper("foobar 123#%^"), std::string("FOOBAR 123#%^"));
    EXPECT_EQ(StrToUpper("FOOBAR 123#%^"), std::string("FOOBAR 123#%^"));
    EXPECT_EQ(StrToUpper("FooBar 123#%^"), std::string("FOOBAR 123#%^"));
}

TEST(StringTest, StrToLower)
{
    EXPECT_EQ(StrToLower("foobar 123#%^"), std::string("foobar 123#%^"));
    EXPECT_EQ(StrToLower("FOOBAR 123#%^"), std::string("foobar 123#%^"));
    EXPECT_EQ(StrToLower("FooBar 123#%^"), std::string("foobar 123#%^"));
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
