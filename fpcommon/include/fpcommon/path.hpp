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
 * @brief Fixposition SDK: path utilities
 */
#ifndef __FPCOMMON_PATH_HPP__
#define __FPCOMMON_PATH_HPP__

/* LIBC/STL */
#include <iostream>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */

namespace fp {
namespace common {
namespace path {
/* ****************************************************************************************************************** */

/**
 * @brief Check if path exists
 *
 * @param[in]  path  Path (file or directory) to check
 *
 * @returns true if path exists, false otherwise
 */
bool PathExists(const std::string& path);

/**
 * @brief Get file size
 *
 * @param[in]  path  Path to file
 *
 * @returns the size of the file in bytes, 0 is only valid if file is readable
 */
std::size_t FileSize(const std::string& path);

/**
 * @brief Output file handle
 */
class OutputFile
{
   public:
    OutputFile();
    ~OutputFile();

    /**
     * @brief Open output file for writing
     *
     * @param[in]  path  The path / filename, can end in ".gz" for compressed output
     *
     * @returns true on success, failure otherwise
     */
    bool Open(const std::string& path);

    /**
     * @brief Close output file
     */
    void Close();

    /**
     * @brief Write data to file
     *
     * @param[in]  data  The data to be written
     *
     * @returns true on success, false otherwise
     */
    bool Write(const std::vector<uint8_t>& data);
    /**
     * @brief Write data to file
     *
     * @param[in]  data  The data to be written
     * @param[in]  size  The size of the data to be written
     *
     * @returns true on success, false otherwise
     */
    bool Write(const uint8_t* data, const std::size_t size);

   private:
    std::string path_;                  //!< File path
    std::unique_ptr<std::ostream> fh_;  //!< File handle
};

/* ****************************************************************************************************************** */
}  // namespace path
}  // namespace common
}  // namespace fp
#endif  // __FPCOMMON_PATH_HPP__
