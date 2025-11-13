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
 * @brief Fixposition SDK: path utilities
 */

/* LIBC/STL */
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>

/* EXTERNAL */
#include <sys/stat.h>
#include <unistd.h>
#include "zfstream.hpp"  // Shipped with package

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/path.hpp"
#include "fpsdk_common/string.hpp"

namespace fpsdk {
namespace common {
namespace path {
/* ****************************************************************************************************************** */

bool PathExists(const std::string& path)
{
    return access(path.c_str(), F_OK) == 0;
}

// ---------------------------------------------------------------------------------------------------------------------

bool PathIsDirectory(const std::string& path)
{
    struct stat st;
    return (stat(path.c_str(), &st) == 0) && S_ISDIR(st.st_mode);
}

// ---------------------------------------------------------------------------------------------------------------------

bool PathIsFile(const std::string& path)
{
    struct stat st;
    return (stat(path.c_str(), &st) == 0) && S_ISREG(st.st_mode);
}

// ---------------------------------------------------------------------------------------------------------------------

bool PathIsSymlink(const std::string& path)
{
    struct stat st;
    return (stat(path.c_str(), &st) == 0) && S_ISLNK(st.st_mode);
}

// ---------------------------------------------------------------------------------------------------------------------

bool PathIsReadable(const std::string& path)
{
    return access(path.c_str(), R_OK) == 0;
}

// ---------------------------------------------------------------------------------------------------------------------

bool PathIsWritable(const std::string& path)
{
    return access(path.c_str(), W_OK) == 0;
}

// ---------------------------------------------------------------------------------------------------------------------

bool PathIsExecutable(const std::string& path)
{
    return access(path.c_str(), X_OK) == 0;
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t FileSize(const std::string& path)
{
    uint64_t size = 0;
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        size = st.st_size;
    }
    return size;
}

// ---------------------------------------------------------------------------------------------------------------------

static std::size_t DirSizeEx(const std::string& path, std::size_t depth)
{
    if (depth > 100) {
        WARNING("DirSize() max depth exceeded");
        return 0;
    }
    std::size_t size = 0;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(path)) {
        if (entry.is_directory()) {
            size += DirSizeEx(entry.path(), depth + 1);
        } else {
            size += entry.file_size();
        }
    }
    return size;
}

std::size_t DirSize(const std::string& path)
{
    return DirSizeEx(path, 0);
}

// ---------------------------------------------------------------------------------------------------------------------

void RemoveAll(const std::string& path)
{
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    if (ec) {
        WARNING("Failed to remove %s: %s", path.c_str(), ec.message().c_str());
    }
}

/* ****************************************************************************************************************** */

OutputFile::OutputFile()
{
}

OutputFile::~OutputFile()
{
    Close();
}

// ---------------------------------------------------------------------------------------------------------------------

bool OutputFile::Open(const std::string& path)
{
    if (IsOpen()) {
        return false;
    }

    // Open file
    path_ = path;
    if (fpsdk::common::string::StrEndsWith(path, ".gz")) {
        fh_ = std::make_unique<gzofstream>(path.c_str());
    } else {
        fh_ = std::make_unique<std::ofstream>(path, std::ios::binary);
    }
    if (fh_->fail()) {
        error_ = string::StrError(errno);
        WARNING("OutputFile: open fail %s: %s", path.c_str(), error_.c_str());
        fh_.reset();
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void OutputFile::Close()
{
    if (fh_) {
        fh_.reset();
        path_.clear();
        size_ = 0;
        error_.clear();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool OutputFile::Write(const std::vector<uint8_t>& data)
{
    return Write(data.data(), data.size());
}

bool OutputFile::Write(const uint8_t* data, const std::size_t size)
{
    if (!fh_ || (data == NULL) || (size == 0)) {
        return false;
    }
    bool ok = true;
    fh_->write((const char*)data, size);
    if (fh_->fail()) {
        error_ = string::StrError(errno);
        WARNING("OutputFile: write fail %s: %s", path_.c_str(), error_.c_str());
        ok = false;
    }
    return ok;
}

bool OutputFile::Write(const std::string& data)
{
    return Write((const uint8_t*)data.data(), data.size());
}

// ---------------------------------------------------------------------------------------------------------------------

const std::string& OutputFile::Path() const
{
    return path_;
}

bool OutputFile::IsOpen() const
{
    return fh_ ? true : false;
}

std::size_t OutputFile::Size() const
{
    return size_;
}

const std::string& OutputFile::Error() const
{
    return error_;
}

/* ****************************************************************************************************************** */

InputFile::InputFile()
{
}

InputFile::~InputFile()
{
    Close();
}

// ---------------------------------------------------------------------------------------------------------------------

bool InputFile::Open(const std::string& path)
{
    if (IsOpen()) {
        return false;
    }

    // Open file
    path_ = path;
    const bool is_gz = fpsdk::common::string::StrEndsWith(path, ".gz");
    if (is_gz) {
        fh_ = std::make_unique<gzifstream>(path.c_str());

    } else {
        fh_ = std::make_unique<std::ifstream>(path, std::ios::binary);
    }
    if (fh_->fail()) {
        error_ = string::StrError(errno);
        WARNING("InputFile: open fail %s: %s", path.c_str(), error_.c_str());
        fh_.reset();
        return false;
    }
    size_ = FileSize(path_);
    can_seek_ = !is_gz;

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void InputFile::Close()
{
    if (fh_) {
        fh_.reset();
        path_.clear();
        size_ = 0;
        pos_ = 0;
        error_.clear();
        can_seek_ = false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t InputFile::Read(uint8_t* data, const std::size_t size)
{
    if (!fh_ || (data == NULL) || (size == 0)) {
        return false;
    }
    fh_->read((char*)data, size);
    const std::size_t n = fh_->gcount();
    pos_ += n;
    return n;
}

// ---------------------------------------------------------------------------------------------------------------------

const std::string& InputFile::Path() const
{
    return path_;
}

bool InputFile::IsOpen() const
{
    return fh_ ? true : false;
}

std::size_t InputFile::Size() const
{
    return size_;
}

std::size_t InputFile::Tell() const
{
    return pos_;
}

const std::string& InputFile::Error() const
{
    return error_;
}

bool InputFile::CanSeek() const
{
    return can_seek_;
}

bool InputFile::Seek(const std::size_t pos)
{
    if (!can_seek_ || !fh_ || (pos > size_)) {
        return false;
    }
    fh_->clear();  // seems to be neccessary or seekg() sometimes fails
    fh_->seekg(pos, std::ios::beg);
    if (fh_->fail()) {
        error_ = string::StrError(errno);
        WARNING("InputFile: seek %s %" PRIuMAX " fail: %s", path_.c_str(), pos, error_.c_str());
        return false;
    }
    return true;
}

/* ****************************************************************************************************************** */

bool FileSlurp(const std::string& path, std::vector<uint8_t>& data)
{
    std::ifstream fh(path, std::ios::binary);
    if (fh.fail()) {
        WARNING("FileSlurp: fail open %s: %s", path.c_str(), string::StrError(errno).c_str());
        return false;
    }

    while (!fh.eof() && !fh.fail()) {
        uint8_t buf[100 * 1024];
        fh.read((char*)buf, sizeof(buf));
        data.insert(data.end(), buf, buf + fh.gcount());
    }
    bool ok = true;
    if (!fh.eof() && fh.fail()) {
        WARNING("FileSlurp: fail read %s: %s", path.c_str(), string::StrError(errno).c_str());
        ok = false;
    }

    fh.close();
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FileSpew(const std::string& path, const std::vector<uint8_t>& data)
{
    std::ofstream fh(path, std::ios::binary);
    if (fh.fail()) {
        WARNING("FileSpew: fail open %s: %s", path.c_str(), string::StrError(errno).c_str());
        return false;
    }

    fh.write((const char*)data.data(), data.size());
    bool ok = true;
    if (fh.fail()) {
        WARNING("FileSlurp: fail write %s: %s", path.c_str(), string::StrError(errno).c_str());
        ok = false;
    }

    fh.close();
    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace path
}  // namespace common
}  // namespace fpsdk
