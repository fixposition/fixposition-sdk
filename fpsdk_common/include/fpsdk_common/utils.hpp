/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 *
 * Partially based on work by flipflip (https://github.com/phkehl)
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Utilities
 *
 * @page FPSDK_COMMON_UTILS Utilities
 *
 * **API**: fpsdk_common/utils.hpp and fpsdk::common::utils
 *
 */
#ifndef __FPSDK_COMMON_UTILS_HPP__
#define __FPSDK_COMMON_UTILS_HPP__

/* LIBC/STL */
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
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

/**
 * @brief Get a HTTP User-Agent string
 *
 * @returns a string suitable as the value for the HTTP User-Agent header
 */
std::string GetUserAgentStr();

/**
 * Circular buffer for chunks of data (bytes, memory). For objects use use boost::circular_buffer or std::deque.
 */
class CircularBuffer
{
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  size  Size of buffer [bytes]
     */
    CircularBuffer(const std::size_t size);

    /**
     * @brief Reset buffer, discard all data
     */
    void Reset();

    /**
     * @brief Get size (capacity) of buffer
     *
     * @returns the size (capacity) of the buffer
     */
    std::size_t Size() const;

    /**
     * @brief Check if empty
     *
     * @returns true if the buffer is empty, false otherwise
     */
    bool Empty() const;

    /**
     * @brief Check if full
     *
     * @returns true if the buffer is completely full
     */
    bool Full() const;

    /**
     * @brief Get size used
     *
     * @returns the size of the used part of the buffer
     */
    std::size_t Used() const;

    /**
     * @brief Get size available
     *
     * @returns the size of the unused part of the buffer
     */
    std::size_t Avail() const;

    /**
     * @brief Write chunk of data to buffer
     *
     * @param[in]  data  Pointer to the data
     * @param[in]  size  Size of the data (> 0)
     *
     * @returns true if data was copied to the buffer, false otherwise (not enough free space, bad params)
     */
    bool Write(const uint8_t* data, const std::size_t size);

    /**
     * @brief Read chunk of data from buffer
     *
     * @param[in]   data  Pointer to the data
     * @param[out]  size  Size of the data (> 0)
     *
     * @returns true if data was copied from the buffer, false otherwise (not enough available data, bad params)
     */
    bool Read(uint8_t* data, const std::size_t size);

    /**
     * @brief Read chunk of data from buffer (but don't remove it from the buffer)
     *
     * @param[in]   data  Pointer to the data
     * @param[out]  size  Size of the data (> 0)
     *
     * @returns true if data was copied from the buffer, false otherwise (not enough available data, bad params)
     */
    bool Peek(uint8_t* data, const std::size_t size);

   private:
    std::vector<uint8_t> buf_;  //!< Buffer/memory
    std::size_t read_;          //!< Read pointer (index)
    std::size_t write_;         //!< Write pointer (index)
    bool full_;                 //!< Buffer full flag
};

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_UTILS_HPP__
