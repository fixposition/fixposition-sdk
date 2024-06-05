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
 * @brief Fixposition SDK: string utilities
 */
#ifndef __FPCOMMON_STRING_HPP__
#define __FPCOMMON_STRING_HPP__

/* LIBC/STL */
#include <cinttypes>
#include <cstdarg>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fp {
namespace common {
namespace string {
/* ****************************************************************************************************************** */

/**
 * @brief Helper macro for marking functions as taking printf() style formatting strings
 *
 * This helps the compiler to check that the number and types of the variable arguments match the format string.
 */
#ifndef PRINTF_ATTR
#  define PRINTF_ATTR(n) __attribute__((format(printf, n, n + 1)))
#endif

/**
 * @brief Format string
 *
 * @param[in]  fmt  printf() style format string
 * @param[in]  ...  Arguments to the format string
 *
 * @returns the formatted string
 */
std::string Sprintf(const char* const fmt, ...) PRINTF_ATTR(1);

/**
 * @brief Format string
 *
 * @param[in]  fmt   printf() style format string
 * @param[in]  args  Arguments list to the format string
 *
 * @returns the formatted string
 */
std::string Vsprintf(const char* fmt, va_list args);

/**
 * @brief Format time
 *
 * @param[in]  fmt  Format, see strftime(3)
 * @param[in]  ts   Posix timestamp [s] (time_t), or 0 for "now"
 * @param[in]  utc  Format as UTC (true) or localtime (false, default)
 *
 * @returns a string with the formatted time
 */
std::string Strftime(const char* const fmt, const int64_t ts = 0, const bool utc = false);

/**
 * @brief Search and replace
 *
 * @param[in,out]  str      The string to search and replace in
 * @param[in]      search   The search term
 * @param[in]      replace  The replacement
 * @param[in]      max      Maximum number of replacements to do (or <= 0 for unlimited)
 *
 * @returns the number of matches
 */
int StrReplace(std::string& str, const std::string& search, const std::string& replace, const int max = 0);

/**
 * @brief Trim string left
 *
 * @param[in,out]  str  The string with all whitespace (" ", \\t, \\r, \\n) removed on the left
 */
void StrTrimLeft(std::string& str);

/**
 * @brief Trim string right
 *
 * @param[in,out]  str  The string with all whitespace (" ", \\t, \\r, \\n) removed on the right
 */
void StrTrimRight(std::string& str);

/**
 * @brief Trim string left and right
 *
 * @param[in,out]  str  The string with all whitespace (" ", \\t, \\r, \\n) removed on the left and the right
 */
void StrTrim(std::string& str);

/**
 * @brief Split string
 *
 * Splits a string using a separator string. All parts, including empty ones, are returned. An empty separator
 * leads to only one part, that is equal to the input string.
 *
 * Examples:
 *    "foo,bar,baz" separated by ","         --> [ "foo", "bar", "baz" ]
 *    "foo,bar,baz" separated by "," (max=2) --> [ "foo", "bar,baz" ]
 *    "foo,,baz,,," separated by ","         --> [ "foo", "", "baz", "", "", "" ]
 *
 * @param[in]  str    The string
 * @param[in]  sep    The separator, empty string is allowed
 * @param[in]  maxNum The maximum number of parts, or 0 (default) for as many as necessary
 *
 * @returns a vector with all parts
 */
std::vector<std::string> StrSplit(const std::string& str, const std::string& sep, const int maxNum = 0);

/**
 * @brief Join strings
 *
 * @param[in]  strs  List of strings to join
 * @param[in]  sep   Separator
 *
 * @returns a string of all given parts separated by the given separator
 */
std::string StrJoin(const std::vector<std::string>& strs, const std::string& sep);

/**
 * @brief Remove duplicates
 *
 * @param[in]  strs  List of strings
 */
void MakeUnique(std::vector<std::string>& strs);

/**
 * @brief Format hexdump of data
 *
 * @param[in]  data  The data
 *
 * @returns one or more lines (strings) with a hexdump of the data
 */
std::vector<std::string> HexDump(const std::vector<uint8_t> data);

/**
 * @brief Format hexdump of data
 *
 * @param[in]  data  The data
 * @param[in]  size  The size of the data
 *
 * @returns one or more lines (strings) with a hexdump of the data
 */
std::vector<std::string> HexDump(const uint8_t* data, const std::size_t size);

/**
 * @brief Check if one string starts with another string
 *
 * @param[in]  str     String to check
 * @param[in]  prefix  Prefix to check, length must be >= length of str
 *
 * @returns true if string starts with the prefix, false otherwise (no match or empty string(s))
 */
bool StrStartsWith(const std::string& str, const std::string& prefix);

/**
 * @brief Check if one string ends with another string
 *
 * @param[in]  str     String to check
 * @param[in]  suffix  Prefix to check, length must be >= length of str
 *
 * @returns true if string ends with the suffix, false otherwise (no match or empty string(s))
 */
bool StrEndsWith(const std::string& str, const std::string& suffix);

/**
 * @brief Check if one string is contained within another string string
 *
 * @param[in]  str  String to check
 * @param[in]  sub  String to find
 *
 * @returns true if string contains the substriong, false otherwise(no match or empty string(s))
 */
bool StrContains(const std::string& str, const std::string& sub);

/**
 * @brief Convert string to value (int32_t, int)
 *
 * @note White-space or other spurious characters in the string or valid number strings that are out of range of the
 *       target value type result in a failure (return false). The output value is only modified on success.
 *
 * @param[in]  str    The string, decimal, hex ("0x...") or octal ("0..."), valid range: INT32_MIN..INT32_MAX
 * @param[out] value  The value
 *
 * @returns true if the string could be converted, false otherwise
 */
bool StrToValue(const std::string& str, int32_t& value);

/**
 * @brief Convert string to value (uint32_t, unsigned int)
 *
 * @note White-space or other spurious characters in the string or valid number strings that are out of range of the
 *       target value type result in a failure (return false). The output value is only modified on success.
 *
 * @param[in]  str    The string, decimal, hex ("0x...") or octal ("0..."), valid range: 0..UINT32_MAX
 * @param[out] value  The value
 *
 * @returns true if the string could be converted, false otherwise
 */
bool StrToValue(const std::string& str, uint32_t& value);

/**
 * @brief Convert string to value (int64_t, long)
 *
 * @note White-space or other spurious characters in the string or valid number strings that are out of range of the
 *       target value type result in a failure (return false). The output value is only modified on success.
 *
 * @param[in]  str    The string, decimal, hex ("0x...") or octal ("0..."), valid range: (INT64_MIN-1)..(INT64_MAX-1)
 * @param[out] value  The value
 *
 * @returns true if the string could be converted, false otherwise
 */
bool StrToValue(const std::string& str, int64_t& value);

/**
 * @brief Convert string to value (uint64_t, unsigned long)
 *
 * @note White-space or other spurious characters in the string or valid number strings that are out of range of the
 *       target value type result in a failure (return false). The output value is only modified on success.
 *
 * @param[in]  str    The string, decimal, hex ("0x...") or octal ("0..."), valid range: 0..(INT64_MAX-1)
 * @param[out] value  The value
 *
 * @returns true if the string could be converted, false otherwise
 */
bool StrToValue(const std::string& str, uint64_t& value);

/**
 * @brief Convert string to value (float)
 *
 * @note White-space or other spurious characters in the string or valid number strings that are out of range of the
 *       target value type result in a failure (return false). The output value is only modified on success.
 *
 * @param[in]  str    The string, anything %f understands
 * @param[out] value  The value
 *
 * @returns true if the string could be converted, false otherwise
 */
bool StrToValue(const std::string& str, float& value);

/**
 * @brief Convert string to value (double)
 *
 * @note White-space or other spurious characters in the string or valid number strings that are out of range of the
 *       target value type result in a failure (return false). The output value is only modified on success.
 *
 * @param[in]  str    The string, anything %f understands
 * @param[out] value  The value
 *
 * @returns true if the string could be converted, false otherwise
 */
bool StrToValue(const std::string& str, double& value);

/**
 * @brief Convert string to all upper case
 *
 * @param[in]  str  The string
 *
 * @returns the string converted to all upper case
 */
std::string StrToUpper(const std::string& str);

/**
 * @brief Convert string to all lower case
 *
 * @param[in]  str  The string
 *
 * @returns the string converted to all lower case
 */
std::string StrToLower(const std::string& str);

/* ****************************************************************************************************************** */
}  // namespace string
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_STRING_HPP__
