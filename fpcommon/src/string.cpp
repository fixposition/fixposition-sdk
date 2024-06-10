/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \033ndverbatim
 *
 * @file
 * @brief Fixposition SDK: string utilities
 */

/* LIBC/STL */
#include <algorithm>
#include <cctype>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unordered_map>

/* EXTERNAL */

/* PACKAGE */
#include "fpcommon/string.hpp"

namespace fp {
namespace common {
namespace string {
/* ****************************************************************************************************************** */

std::string Sprintf(const char* const fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    std::string str = Vsprintf(fmt, args);
    va_end(args);
    return str;
}

// ---------------------------------------------------------------------------------------------------------------------

std::string Vsprintf(const char* fmt, va_list args)
{
    // Copy list, interpolate format string without actually formatting a string in order
    // to determine the required string size
    va_list args_copy;
    va_copy(args_copy, args);
    const int len = std::vsnprintf(NULL, 0, fmt, args_copy);
    va_end(args_copy);

    // Allocate buffer of required size and format string into that
    std::vector<char> buf(len + 1);  // formatted length + 1 for nul termination
    std::vsnprintf(buf.data(), buf.size(), fmt, args);

    // Convert buffer to c++ string
    return std::string(buf.data(), len);
}

// ---------------------------------------------------------------------------------------------------------------------

std::string Strftime(const char* const fmt, const int64_t ts, const bool utc)
{
    std::vector<char> str(1000);
    struct tm now;
    const time_t t = (ts <= 0 ? time(NULL) : ts);
    bool ok = false;
    if (utc) {
        ok = gmtime_r(&t, &now) == &now;
    } else {
        ok = localtime_r(&t, &now) == &now;
    }
    int len = 0;
    if (ok) {
        len = strftime(str.data(), str.size(), fmt == NULL ? "%Y-%m-%d %H:%M:%S" : fmt, &now);
    }
    return std::string(str.data(), len);
}

// ---------------------------------------------------------------------------------------------------------------------

int StrReplace(std::string& str, const std::string& search, const std::string& replace, const int max)
{
    int count = 0;
    std::size_t pos = 0;
    while (((max <= 0) || (count < max)) && ((pos = str.find(search, pos)) != std::string::npos)) {
        str.replace(pos, search.size(), replace);
        pos += replace.size();
        count++;
    }
    return count;
}

// ---------------------------------------------------------------------------------------------------------------------

void StrTrimLeft(std::string& str)
{
    str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](const int c) { return std::isspace(c) == 0; }));
}

void StrTrimRight(std::string& str)
{
    str.erase(
        std::find_if(str.rbegin(), str.rend(), [](const int c) { return std::isspace(c) == 0; }).base(), str.end());
}

void StrTrim(std::string& str)
{
    StrTrimLeft(str);
    StrTrimRight(str);
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<std::string> StrSplit(const std::string& str, const std::string& sep, const int maxNum)
{
    std::vector<std::string> out;
    if (str.empty()) {
        return out;
    }
    if (sep.empty()) {
        out.push_back(str);
        return out;
    }

    std::string strCopy = str;
    std::size_t pos = 0;
    int num = 0;
    while ((pos = strCopy.find(sep)) != std::string::npos) {
        std::string part = strCopy.substr(0, pos);
        strCopy.erase(0, pos + sep.length());
        out.push_back(part);
        num++;
        if ((maxNum > 0) && (num >= (maxNum - 1))) {
            break;
        }
    }
    out.push_back(strCopy);
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------

std::string StrJoin(const std::vector<std::string>& strs, const std::string& sep)
{
    std::string res = "";
    for (auto& str : strs) {
        res += sep;
        res += str;
    }
    return res.empty() ? res : res.substr(sep.size());
}

// ---------------------------------------------------------------------------------------------------------------------

void MakeUnique(std::vector<std::string>& strs)
{
    std::unordered_map<std::string, bool> seen;
    for (auto iter = strs.begin(); iter != strs.end();) {
        if (seen.find(*iter) != seen.end()) {
            iter = strs.erase(iter);
        } else {
            seen[*iter] = true;
            iter++;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::vector<std::string> HexDump(const std::vector<uint8_t> data)
{
    return HexDump(data.data(), data.size());
}

std::vector<std::string> HexDump(const uint8_t* data, const std::size_t size)
{
    std::vector<std::string> hexdump;
    const char i2hex[] = "0123456789abcdef";
    const uint8_t* pData = data;
    for (std::size_t ix = 0; ix < size;) {
        char str[70];
        std::memset(str, ' ', sizeof(str));
        str[50] = '|';
        str[67] = '|';
        str[68] = '\0';
        for (int ix2 = 0; (ix2 < 16) && ((ix + ix2) < size); ix2++) {
            //           1         2         3         4         5         6
            // 012345678901234567890123456789012345678901234567890123456789012345678
            // xx xx xx xx xx xx xx xx  xx xx xx xx xx xx xx xx  |................|\0
            // 0  1  2  3  4  5  6  7   8  9  10 11 12 13 14 15
            const uint8_t c = pData[ix + ix2];
            int pos1 = 3 * ix2;
            int pos2 = 51 + ix2;
            if (ix2 > 7) {
                pos1++;
            }
            // clang-format off
            str[pos1    ] = i2hex[ (c >> 4) & 0xf ];
            str[pos1 + 1] = i2hex[  c       & 0xf ];
            // clang-format on
            str[pos2] = isprint((int)c) ? c : '.';
        }
        char buf[1024];
        std::snprintf(buf, sizeof(buf), "0x%04" PRIx64 " %05" PRIu64 "  %s", ix, ix, str);
        hexdump.push_back(buf);
        ix += 16;
    }
    return hexdump;
}

// ---------------------------------------------------------------------------------------------------------------------

bool StrStartsWith(const std::string& str, const std::string& prefix)
{
    const auto str_len = str.size();
    const auto prefix_len = prefix.size();
    if ((str_len >= prefix_len) && (prefix_len > 0)) {
        return str.substr(0, prefix_len) == prefix;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool StrEndsWith(const std::string& str, const std::string& suffix)
{
    const auto str_len = str.size();
    const auto suffix_len = suffix.size();
    if ((str_len >= suffix_len) && (suffix_len > 0)) {
        return str.substr(str_len - suffix_len, suffix_len) == suffix;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool StrContains(const std::string& str, const std::string& sub)
{
    if (!str.empty() && !sub.empty()) {
        return str.find(sub) != std::string::npos;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

static bool StrToValueSigned(const std::string& str, int64_t& value)
{
    // No empty string, no leading whitespace
    if (str.empty() || (std::isspace(str[0]) != 0)) {
        return false;
    }

    // Parse
    int num = 0;
    int count = std::sscanf(str.data(), "%" SCNi64 "%n", &value, &num);

    // Number of values found must be 1 and the entire string must have been used (no trailing stuff)
    return ((count == 1) && ((std::size_t)num == str.size()));
}

static bool StrToValueUnsigned(const std::string& str, uint64_t& value)
{
    // No empty string, no leading whitespace, not starting with a "-" (sscanf("%u") will happy give us the
    // two's complement number, which is not what we want)
    if (str.empty() || (std::isspace(str[0]) != 0) || (str[0] == '-')) {
        return false;
    }

    // Parse
    int num = 0;
    int count = 0;
    // Hex
    if ((str.size() > 2) && (str[0] == '0') && ((str[1] == 'x') || (str[1] == 'X'))) {
        count = std::sscanf(str.data(), "%" SCNx64 "%n", &value, &num);
    }
    // Octal
    else if (str[0] == '0') {
        count = std::sscanf(str.data(), "%" SCNo64 "%n", &value, &num);
    }
    // Decimal
    else {
        count = std::sscanf(str.data(), "%" SCNu64 "%n", &value, &num);
    }

    // Number of values found must be 1 and the entire string must have been used (no trailing stuff)
    return ((count == 1) && ((std::size_t)num == str.size()));
}

bool StrToValue(const std::string& str, int32_t& value)
{
    int64_t value_tmp = 0;
    if (StrToValueSigned(str, value_tmp) && (value_tmp >= INT32_MIN) && (value_tmp <= INT32_MAX)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

bool StrToValue(const std::string& str, uint32_t& value)
{
    uint64_t value_tmp = 0;
    if (StrToValueUnsigned(str, value_tmp) && (value_tmp <= UINT32_MAX)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

bool StrToValue(const std::string& str, int64_t& value)
{
    int64_t value_tmp = 0;
    // Note that there is no good way to check the actual allowed range (>= and <=), so we reduce the allowed range by 1
    if (StrToValueSigned(str, value_tmp) && (value_tmp > INT64_MIN) && (value_tmp < INT64_MAX)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

bool StrToValue(const std::string& str, uint64_t& value)
{
    uint64_t value_tmp = 0;
    // Note that there is no good way to check the actual allowed range (<=), so we reduce the allowed range by 1
    if (StrToValueUnsigned(str, value_tmp) && (value_tmp < UINT64_MAX)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

bool StrToValue(const std::string& str, float& value)
{
    if (str.empty() || (std::isspace(str[0]) != 0)) {
        return false;
    }

    float value_tmp = 0.0f;
    int num = 0;
    int count = std::sscanf(str.data(), "%f%n", &value_tmp, &num);

    if ((count == 1) && ((std::size_t)num == str.size()) && std::isnormal(value_tmp)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

bool StrToValue(const std::string& str, double& value)
{
    if (str.empty() || (std::isspace(str[0]) != 0)) {
        return false;
    }

    double value_tmp = 0.0f;
    int num = 0;
    int count = std::sscanf(str.data(), "%lf%n", &value_tmp, &num);

    if ((count == 1) && ((std::size_t)num == str.size()) && std::isnormal(value_tmp)) {
        value = value_tmp;
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::string StrToUpper(const std::string& str)
{
    std::string res = str;
    std::transform(res.begin(), res.end(), res.begin(), ::toupper);
    return res;
}

std::string StrToLower(const std::string& str)
{
    std::string res = str;
    std::transform(res.begin(), res.end(), res.begin(), ::tolower);
    return res;
}

/* ****************************************************************************************************************** */
}  // namespace string
}  // namespace common
}  // namespace fp
