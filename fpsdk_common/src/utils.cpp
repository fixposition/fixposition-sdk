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
 * @brief Fixposition SDK: Utilities
 */

/* LIBC/STL */
#include <cstring>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/string.hpp"
#include "fpsdk_common/utils.hpp"

namespace fpsdk {
namespace common {
namespace utils {
/* ****************************************************************************************************************** */

const char* GetVersionString()
{
    return FPSDK_VERSION_STRING;  // "0.0.0", "0.0.0-heads/feature/bla-g123456-dirty"
}

// ---------------------------------------------------------------------------------------------------------------------

const char* GetCopyrightString()
{
    return "Copyright (c) Fixposition AG (www.fixposition.com) and contributors";
}

// ---------------------------------------------------------------------------------------------------------------------

const char* GetLicenseString()
{
    return "License: see the LICENSE files included in the source distribution";
}

// ---------------------------------------------------------------------------------------------------------------------

std::string GetUserAgentStr()
{
    return "FixpositionSDK/" + string::StrSplit(GetVersionString(), "-")[0];
}

/* ****************************************************************************************************************** */

CircularBuffer::CircularBuffer(const std::size_t size)
{
    buf_.resize(size);
    Reset();
}

// ---------------------------------------------------------------------------------------------------------------------

void CircularBuffer::Reset()
{
    read_ = 0;
    write_ = 0;
    full_ = false;
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t CircularBuffer::Size() const
{
    return buf_.size();
}

// ---------------------------------------------------------------------------------------------------------------------

bool CircularBuffer::Empty() const
{
    return !full_ && (read_ == write_);
}

// ---------------------------------------------------------------------------------------------------------------------

bool CircularBuffer::Full() const
{
    return full_;
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t CircularBuffer::Used() const
{
    return full_ ? buf_.size() : (write_ >= read_ ? write_ - read_ : buf_.size() + write_ - read_);
}

// ---------------------------------------------------------------------------------------------------------------------

std::size_t CircularBuffer::Avail() const
{
    return buf_.size() - Used();
}

// ---------------------------------------------------------------------------------------------------------------------

bool CircularBuffer::Write(const uint8_t* data, const std::size_t size)
{
    if ((data == NULL) || (Avail() < size) || (size == 0)) {
        return false;
    }

    const std::size_t size1 = std::min(size, buf_.size() - write_);
    std::memcpy(&buf_[write_], data, size1);

    if (size1 < size) {
        const std::size_t size2 = size - size1;
        std::memcpy(&buf_[0], &data[size1], size2);
    }

    write_ = (write_ + size) % buf_.size();
    full_ = (read_ == write_);
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool CircularBuffer::Read(uint8_t* data, const std::size_t size)
{
    if (Peek(data, size)) {
        read_ = (read_ + size) % buf_.size();
        full_ = false;
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool CircularBuffer::Peek(uint8_t* data, const std::size_t size)
{
    if ((data == NULL) || (Used() < size) || (size == 0)) {
        return false;
    }

    const std::size_t size1 = std::min(size, buf_.size() - read_);
    std::memcpy(&data[0], &buf_[read_], size1);
    if (size1 < size) {
        const std::size_t size2 = size - size1;
        std::memcpy(&data[size1], &buf_[0], size2);
    }

    return true;
}

/* ****************************************************************************************************************** */
}  // namespace utils
}  // namespace common
}  // namespace fpsdk
