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
 * @brief Fixposition SDK: Utilities
 *
 * @page FPCOMMON_UTILS Utilities
 *
 * @todo add documentation
 *
 */
#ifndef __FPCOMMON_UTILS_HPP__
#define __FPCOMMON_UTILS_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* PACKAGE */

namespace fp {
namespace common {
/**
 * @brief Utilities
 */
namespace utils {
/* ****************************************************************************************************************** */

/**
 * @brief Get version string
 *
 * @returns the version string (e.g. "0.0.0-heads/feature/something-0-gf61ae50-dirty")
 */
const char* GetVersionString();

/**
 * @brief Get copyright string
 *
 * @returns the copyright string
 */
const char* GetCopyrightString();

/**
 * @brief Get license string
 *
 * @returns the license string
 */
const char* GetLicenseString();

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_UTILS_HPP__
