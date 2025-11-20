/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Based on work by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Parser CRC routines
 *
 * @page FPSDK_COMMON_PARSER_CRC Parser CRC routines
 *
 * **API**: fpsdk_common/parser/crc.hpp and fpsdk::common::parser::crc
 *
 */
#ifndef __FPSDK_COMMON_PARSER_CRC_HPP__
#define __FPSDK_COMMON_PARSER_CRC_HPP__

/* LIBC/STL */
#include <cstdint>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser NOV_B routines and types
 */
namespace crc {
/* ****************************************************************************************************************** */

/**
 * @brief Calculate 32-bit CRC as used by FP_B
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC
 */
uint32_t Crc32fpb(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 24-bit CRC as used by RTCM3 and SPARTN
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC (32-bit value masked with 0x00ffffff)
 */
uint32_t Crc24rtcm3(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 32-bit CRC as used by NOV_B (and UNI_B)
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC
 */
uint32_t Crc32novb(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 4-bit CRC as used by SPARTN
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data

    \returns CRC (8-bit value masked with 0x0f)
*/
uint8_t Crc4spartn(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 8-bit CRC as used by SPARTN
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC
 */
uint8_t Crc8spartn(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 16-bit CRC as used by SPARTN
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC (result undefined if size <= 0 or data == NULL)
 */
uint16_t Crc16spartn(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 32-bit CRC as used by SPARTN
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC
 */
uint32_t Crc32spartn(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 16-bit CRC as used by SBF
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the CRC (result undefined if size <= 0 or data == NULL)
 */
uint16_t Crc16sbf(const uint8_t* data, const std::size_t size);

/**
 * @brief Calculate 16-bit checksum as used by UBX
 *
 * @param[in]  data  Pointer to the data to calculate the CRC for
 * @param[in]  size  Size of data
 *
 * @returns the checksum
 */
uint16_t ChecksumUbx(const uint8_t* data, const std::size_t size);

/* ****************************************************************************************************************** */
}  // namespace crc
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_CRC_HPP__
