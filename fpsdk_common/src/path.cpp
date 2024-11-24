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

std::size_t FileSize(const std::string& path)
{
    uint64_t size = 0;
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        size = st.st_size;
    }
    return size;
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
    Close();

    // Open file
    if (fpsdk::common::string::StrEndsWith(path, ".gz")) {
        fh_ = std::make_unique<gzofstream>(path.c_str());
    } else {
        fh_ = std::make_unique<std::ofstream>(path, std::ios::binary);
    }
    if (fh_->fail()) {
        WARNING("OutputFile: open fail %s: %s", path.c_str(), std::strerror(errno));
        fh_.reset();
        return false;
    }

    path_ = path;
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void OutputFile::Close()
{
    if (fh_) {
        fh_.reset();
        path_.clear();
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
        WARNING("OutputFile: write fail %s: %s", path_.c_str(), std::strerror(errno));
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

bool FileSlurp(const std::string& path, std::vector<uint8_t>& data)
{
    std::ifstream fh(path, std::ios::binary);
    if (fh.fail()) {
        WARNING("FileSlurp: fail open %s: %s", path.c_str(), std::strerror(errno));
        return false;
    }

    while (!fh.eof() && !fh.fail()) {
        uint8_t buf[100 * 1024];
        fh.read((char*)buf, sizeof(buf));
        data.insert(data.end(), buf, buf + fh.gcount());
    }
    bool ok = true;
    if (!fh.eof() && fh.fail()) {
        WARNING("FileSlurp: fail read %s: %s", path.c_str(), std::strerror(errno));
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
        WARNING("FileSpew: fail open %s: %s", path.c_str(), std::strerror(errno));
        return false;
    }

    fh.write((const char*)data.data(), data.size());
    bool ok = true;
    if (fh.fail()) {
        WARNING("FileSlurp: fail write %s: %s", path.c_str(), std::strerror(errno));
        ok = false;
    }

    fh.close();
    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace path
}  // namespace common
}  // namespace fpsdk
