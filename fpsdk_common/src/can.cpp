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
 * @brief Fixposition SDK: CAN bus utilities
 */

/* LIBC/STL */
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

/* EXTERNAL */
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

/* PACKAGE */
#include "fpsdk_common/can.hpp"
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/math.hpp"
#include "fpsdk_common/string.hpp"

namespace fpsdk {
namespace common {
namespace can {
/* ****************************************************************************************************************** */

void CanFrame::SetData(const uint8_t* data, const std::size_t size)
{
    if ((data != NULL) && (size <= sizeof(fd_frame_.data))) {
        data_len_ = size;
        std::memcpy(fd_frame_.data, data, data_len_);
    } else {
        data_len_ = 0;
    }
    data_ = fd_frame_.data;
}

// ---------------------------------------------------------------------------------------------------------------------

bool CanFrame::RawToFrame(const uint8_t* data, const std::size_t size)
{
    if ((size != CAN_MTU) && (size != CANFD_MTU)) {
        return false;
    }

    // Copy frame
    std::memcpy(&fd_frame_, data, size);

    // Extract flags, mask ID, etc.
    // clang-format off
    is_eff_   = ( (fd_frame_.can_id & CAN_EFF_FLAG) == CAN_EFF_FLAG );
    is_fd_    = ( size == CANFD_MTU );
    rtr_flag_ = ( (fd_frame_.can_id & CAN_RTR_FLAG) == CAN_RTR_FLAG );
    err_flag_ = ( (fd_frame_.can_id & CAN_ERR_FLAG) == CAN_ERR_FLAG );
    fd_brs_   = ( is_fd_ && ((fd_frame_.flags & CANFD_BRS) == CANFD_BRS) );
    data_len_ = fd_frame_.len;  // = can_frame_.can_dlc
    data_     = fd_frame_.data; // = can_frame_.data
    can_id_   = (is_eff_ ? (fd_frame_.can_id & CAN_EFF_MASK) : (fd_frame_.can_id & CAN_SFF_MASK));
    // clang-format on

    // DEBUG_HEXDUMP(data_, data_len_, NULL, "CAN%s%s%s%s %cFF 0x%0*x (%" PRIuMAX ")", is_fd_ ? " FD" : "",
    //     rtr_flag_ ? " RTR" : "", err_flag_ ? " ERR" : "", fd_brs_ ? " BRS" : "", is_eff_ ? 'E' : 'S',
    //     is_eff_ ? 8 : 3, can_id_, data_len_);

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void CanFrame::FrameToRaw()
{
    if (is_eff_) {
        fd_frame_.can_id = can_id_ & CAN_EFF_MASK;
        math::SetBits(fd_frame_.can_id, CAN_EFF_FLAG);
    } else {
        fd_frame_.can_id = can_id_ & CAN_SFF_MASK;
    }

    fd_frame_.flags = 0;
    if (is_fd_ && fd_brs_) {
        math::SetBits(fd_frame_.flags, (uint8_t)CANFD_BRS);
    }

    if (rtr_flag_) {
        math::SetBits(fd_frame_.can_id, CAN_RTR_FLAG);
    }

    if (err_flag_) {
        math::SetBits(fd_frame_.can_id, CAN_ERR_FLAG);
    }

    fd_frame_.len = std::clamp(data_len_, (std::size_t)0, sizeof(fd_frame_.data));

    // DEBUG_HEXDUMP(data_, data_len_, NULL, "CAN%s%s%s%s %cFF 0x%0*x (%" PRIuMAX ")", is_fd_ ? " FD" : "",
    //     rtr_flag_ ? " RTR" : "", err_flag_ ? " ERR" : "", fd_brs_ ? " BRS" : "", is_eff_ ? 'E' : 'S', is_eff_ ? 8 :
    //     3, can_id_, data_len_);
}

/* ****************************************************************************************************************** */

RawCan::RawCan(const std::string& device) /* clang-format off */ :
    device_        { device },
    sock_          { -1 },
    saved_errno_   { 0 }  // clang-format on
{
}

// ---------------------------------------------------------------------------------------------------------------------

RawCan::~RawCan()
{
    Close();
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::Open()
{
    // Create socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        saved_errno_ = errno;
        WARNING("%s: socket() error: %s", device_.c_str(), string::StrError(errno).c_str());
        return false;
    }

    // Bind to CAN device
    struct ifreq ifr;
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", device_.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        saved_errno_ = errno;
        WARNING("%s: bind() error: %s", device_.c_str(), string::StrError(errno).c_str());
        close(s);
        return false;
    }

    // Make handle non-blocking
    const int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0) {
        saved_errno_ = errno;
        WARNING("%s: fcntl(F_GETFL) error: %s", device_.c_str(), string::StrError(errno).c_str());
        close(s);
        return false;
    }
    int res = fcntl(s, F_SETFL, flags | O_NONBLOCK);
    if (res < 0) {
        saved_errno_ = errno;
        WARNING("%s: fcntl(F_SETFL) error: %s", device_.c_str(), string::StrError(errno).c_str());
        close(s);
        return false;
    }

    // Check that the interface can do FD
    if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
        saved_errno_ = errno;
        WARNING("%s: ioctl(SIOCGIFMTU) error: %s", device_.c_str(), string::StrError(errno).c_str());
        close(s);
        return false;
    }
    DEBUG("%s: mtu=%d (CAN_MTU=%d, CANFD_MTU=%d)", device_.c_str(), ifr.ifr_mtu, (int)CAN_MTU, (int)CANFD_MTU);

    // Enable FD frames
    const int enable_can_fd = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_can_fd, sizeof(enable_can_fd)) < 0) {
        saved_errno_ = errno;
        WARNING("%s: setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES) error: %s", device_.c_str(),
            string::StrError(errno).c_str());
        close(s);
        return false;
    }

    sock_ = s;
    saved_errno_ = 0;
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::SetFilters(const std::vector<struct can_filter>& filters)
{
    if (filters.empty()) {
        return false;
    }

    if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(struct can_filter)) <
        0) {
        saved_errno_ = errno;
        WARNING(
            "%s: setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER) error: %s", device_.c_str(), string::StrError(errno).c_str());
        return false;
    }

    saved_errno_ = 0;
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void RawCan::Close()
{
    if (sock_ < 0) {
        return;
    }
    close(sock_);
    sock_ = -1;
    saved_errno_ = 0;
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::IsOpen() const
{
    return sock_ >= 0;
}

// ---------------------------------------------------------------------------------------------------------------------

int RawCan::GetSocket() const
{
    return sock_;
}

// ---------------------------------------------------------------------------------------------------------------------

int RawCan::GetErrno() const
{
    return saved_errno_;
}

// ---------------------------------------------------------------------------------------------------------------------

std::string RawCan::GetStrerror()
{
    return string::StrError(saved_errno_);
}

// ---------------------------------------------------------------------------------------------------------------------

void RawCan::FlushInput()
{
    saved_errno_ = 0;
    uint8_t buf[sizeof(struct can_frame) * 10];
    while (read(sock_, buf, sizeof(buf)) > 0) {
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::SendFrame(CanFrame& frame)
{
    frame.FrameToRaw();
    if (frame.is_fd_) {
        return SendFrame(frame.fd_frame_);
    } else {
        return SendFrame(frame.can_frame_);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::ReadFrame(CanFrame& frame, const int timeout)
{
    saved_errno_ = 0;
    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout) != 0) {
        uint8_t buf[sizeof(frame)];
        const ssize_t n_read = read(sock_, buf, sizeof(buf));
        if (n_read < 0) {
            saved_errno_ = errno;
            return false;
        } else if (!frame.RawToFrame(buf, n_read)) {
            saved_errno_ = ERANGE;  // unexpected frame size
            return false;
        }
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::SendFrame(const struct can_frame& frame)
{
    saved_errno_ = 0;
    if (write(sock_, &frame, sizeof(frame)) == sizeof(frame)) {
        return true;
    } else {
        saved_errno_ = errno;
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::SendFrame(const struct canfd_frame& frame)
{
    saved_errno_ = 0;
    if (write(sock_, &frame, sizeof(frame)) == sizeof(frame)) {
        return true;
    } else {
        saved_errno_ = errno;
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::ReadFrame(struct can_frame& frame, const int timeout)
{
    saved_errno_ = 0;
    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout) != 0) {
        if (read(sock_, &frame, sizeof(frame)) == sizeof(frame)) {
            return true;
        } else {
            saved_errno_ = errno;
            return false;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool RawCan::ReadFrame(struct canfd_frame& frame, const int timeout)
{
    saved_errno_ = 0;
    struct pollfd pfd;
    pfd.fd = sock_;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout) != 0) {
        if (read(sock_, &frame, sizeof(frame)) == sizeof(frame)) {
            return true;
        } else {
            saved_errno_ = errno;
            return false;
        }
    }
    return false;
}

/* ****************************************************************************************************************** */
}  // namespace can
}  // namespace common
}  // namespace fpsdk
