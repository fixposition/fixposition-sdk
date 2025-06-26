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
 * @brief Fixposition SDK: File and path utilities
 *
 * @page FPSDK_COMMON_PATH File and path utilities
 *
 * **API**: fpsdk_common/path.hpp and fpsdk::common::path
 *
 */
#ifndef __FPSDK_COMMON_PATH_HPP__
#define __FPSDK_COMMON_PATH_HPP__

/* LIBC/STL */
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */

/* PACKAGE */
#include "types.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief File and path utilities
 */
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
 * @brief Check if path is a directory
 *
 * @param[in]  path  Path to check
 *
 * @returns true if path is a directory, false otherwise
 */
bool PathIsDirectory(const std::string& path);

/**
 * @brief Check if path is a regular file
 *
 * @param[in]  path  Path to check
 *
 * @returns true if path is a regular file, false otherwise
 */
bool PathIsFile(const std::string& path);

/**
 * @brief Check if path is a symlink
 *
 * @param[in]  path  Path to check
 *
 * @returns true if path is a symlink, false otherwise
 */
bool PathIsSymlink(const std::string& path);

/**
 * @brief Check if path is readable
 *
 * @param[in]  path  Path (file or directory) to check
 *
 * @returns true if path is readable, false otherwise
 */
bool PathIsReadable(const std::string& path);

/**
 * @brief Check if path is writable
 *
 * @param[in]  path  Path (file or directory) to check
 *
 * @returns true if path is writable, false otherwise
 */
bool PathIsWritable(const std::string& path);

/**
 * @brief Check if path is executable
 *
 * @param[in]  path  Path (file or directory) to check
 *
 * @returns true if path is executable, false otherwise
 */
bool PathIsExecutable(const std::string& path);

/**
 * @brief Get file size
 *
 * @param[in]  path  Path to file
 *
 * @returns the size of the file in bytes, 0 is only valid if file is readable
 */
std::size_t FileSize(const std::string& path);

/**
 * @brief Get directory size
 *
 * @param[in]  path  Path to directory
 *
 * @returns the size of all the files in the directory in bytes, 0 is only valid if file is readable
 */
std::size_t DirSize(const std::string& path);

/**
 * @brief Remove, recursively
 *
 * Removes the path, including all its subdirectories, recursively.
 *
 * @param[in]  path  The path to remove
 */
void RemoveAll(const std::string& path);

/**
 * @brief Output file handle
 */
class OutputFile : private types::NoCopyNoMove
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

    /**
     * @brief Write data to file
     *
     * @param[in]  data  The data to be written
     *
     * @returns true on success, false otherwise
     */
    bool Write(const std::string& data);

    /**
     * @brief Get file path
     *
     * @returns the file path if the file has been opened before, the empty string otherwise
     */
    const std::string& Path() const;

    /**
     * @brief Check if output file is open
     *
     * @returns true if the file is open, false otherwise
     */
    bool IsOpen() const;

    /**
     * @brief Get file size
     *
     * @returns the size written to the file so far
     */
    std::size_t Size() const;

    /**
     * @brief Get error
     *
     * @returns an error string in case of error (e.g. after Write() failed), the empty string if there's no error
     */
    const std::string& Error() const;

   private:
    std::string path_;                  //!< File path
    std::unique_ptr<std::ostream> fh_;  //!< File handle
    std::size_t size_ = 0;              //!< Size written
    std::string error_;                 //!< Last error
};

/**
 * @brief Input file handle
 */
class InputFile
{
   public:
    InputFile();
    ~InputFile();

    /**
     * @brief Open input file for reading
     *
     * @param[in]  path  The path / filename, can end in ".gz" for compressed output
     *
     * @returns true on success, failure otherwise
     */
    bool Open(const std::string& path);

    /**
     * @brief Close input file
     */
    void Close();

    /**
     * @brief Read data from file
     *
     * @param[out]  data  Buffer to read into
     * @param[in]   size  Size of buffer (> 0)
     *
     * @returns the size read, 0 if at end of file
     */
    std::size_t Read(uint8_t* data, const std::size_t size);

    /**
     * @brief Get file path
     *
     * @returns the file path if the file has been opened before, the empty string otherwise
     */
    const std::string& Path() const;

    /**
     * @brief Check if output file is open
     *
     * @returns true if the file is open, false otherwise
     */
    bool IsOpen() const;

    /**
     * @brief Get file size
     *
     * @returns the file size
     */
    std::size_t Size() const;

    /**
     * @brief Get file position
     *
     * @returns the current file position
     */
    std::size_t Tell() const;

    /**
     * @brief Seek to position
     *
     * @param[in]  pos  Position to seek to
     *
     * @returns true on success, false otherwise (our of range, cannot seek)
     */
    bool Seek(const std::size_t pos);

    /**
     * @brief Check if handle can seek
     *
     * @returns true if Seek() is possible, false if not (e.g. compressed file)
     */
    bool CanSeek() const;

    /**
     * @brief Get error
     *
     * @returns an error string in case of error (e.g. after Open() failed), the empty string if there's no error
     */
    const std::string& Error() const;

   private:
    std::string path_;                  //!< File path
    std::unique_ptr<std::istream> fh_;  //!< File handle
    std::size_t size_ = 0;              //!< File size
    std::size_t pos_ = 0;               //!< File position
    std::string error_;                 //!< Last error
    bool can_seek_ = false;             //!< Seek possible?
};

/**
 * @brief Read entire file into a string
 *
 * @param[in]  path  File path
 * @param[out] data  The data
 *
 * @returns true if data was successfully read from file, false otherwise
 */
bool FileSlurp(const std::string& path, std::vector<uint8_t>& data);

/**
 * @brief Write string to file
 *
 * @param[in]  path  File path
 * @param[out] data  The data
 *
 * @returns true if data was successfully written to file, false otherwise
 */
bool FileSpew(const std::string& path, const std::vector<uint8_t>& data);

/* ****************************************************************************************************************** */
}  // namespace path
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PATH_HPP__
