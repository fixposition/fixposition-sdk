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
 *
 * @page FPSDK_COMMON_CAN CAN bus utilities
 *
 * **API**: fpsdk_common/can.hpp and fpsdk::common::utils
 *
 * @section FPSDK_COMMON_CAN_FRAMES CAN frames (messages)
 *
 * - Standard frame format (SFF): 11 bits CAN ID
 * - Extended frame format (EFF): 29 bits CAN ID
 * - Classical CAN: 8 bytes payload
 * - CAN FD: up to 72 bytes payload, with optional switching to a second bitrate for the payload (BRS)
 *
 * The cansend tool (from can-utils):
 *
 *     cansend vcan0 123#0102030405060708                      # SFF frame, 8 bytes payload
 *     cansend vcan0 12345678#0102030405060708                 # EFF frame, 8 bytes payload
 *
 *     cansend vcan0 123##0112233445566778899aabbccddeeff      # SFF FD 16 bytes, no bit rate switch ("0")
 *     cansend vcan0 12345678##0112233445566778899aabbccddeeff # EFF FD 16 bytes, no bit rate switch ("0")
 *
 *     cansend vcan0 123##1112233445566778899aabbccddeeff      # SFF FD 16 bytes, with bit rate switch ("1" = BRS)
 *     cansend vcan0 12345678##1112233445566778899aabbccddeeff # EFF FD 16 bytes, with bit rate switch ("1" = BRS)
 *
 *     cansend vcan0 123#01020304                              # SFF frame, only 4 bytes payload
 *
 * @section FP_COMMON_CAN_TESTING Testing
 *
 * For testing on a normal Linux box once can use a virtual CAN device:
 *
 *     sudo modprobe vcan
 *     sudo ip link add dev vcan0 type vcan
 *     sudo ip link set up vcan0
 *
 * In different terminals:
 *
 *     candump vcan0
 *
 *     cansend vcan0 ... # see above
 *
 * Setup a real can device:
 *
 *     ip link set can0 down
 *     ip link set can0 type can bitrate 125000 dbitrate 500000 fd on fd-non-iso on restart-ms 100
 *     ip link set can0 up
 *     ip link set can0 txqueuelen 1000
 *
 * @section FP_COMMON_CAN_REFS References
 *
 * - linux/can.h, a very good read!
 * - <https://www.kernel.org/doc/html/latest/networking/can.html>
 *
 */
#ifndef __FPSDK_COMMON_CAN_HPP__
#define __FPSDK_COMMON_CAN_HPP__

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */
#include <linux/can.h>

/* PACKAGE */
#include "types.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief CAN bus utilities
 */
namespace can {
/* ****************************************************************************************************************** */

/**
 * @brief CAN frame abstraction for standard, extended and FD (and combinations thereof) CAN frames
 */
struct CanFrame : private types::NoCopyNoMove
{
    // clang-format off
    uint32_t       can_id_    = 0;      //!< CAN ID (masked by CAN_SFF_MASK resp CAN_EFF_MASK), i.e. flags removed)
    bool           is_eff_    = false;  //!< true = extended frame (29 bits ID, CAN_EFF_FLAG), false = standard frame (11 bits ID)
    bool           is_fd_     = false;  //!< true = CAN FD frame (up to CANFD_MAX_DLEN bytes), false = classical frame (up to CAN_MAX_DLEN bytes)
    bool           rtr_flag_  = false;  //!< Remote transmission request flag (CAN_RTR_FLAG)
    bool           err_flag_  = false;  //!< Error message frame flag (CAN_ERR_FLAG)
    bool           fd_brs_    = false;  //!< Use bit rate switch for a FD frame (only valid if is_fd_ = true)
    std::size_t    data_len_  = false;  //!< Number of bytes in payload data: up to CAN_MAX_DLEN for classical frames (is_fd_ = false), up to CANFD_MAX_DLEN for FD frames (is_fd_ = true)
    const uint8_t* data_      = { can_frame_.data };  //!< Pointer to data (can_frame_.data resp. fd_frame_.data)
    // clang-format on

    /**
     * @brief Set data (data_len_ and data_ fields)
     *
     * @param[in]  data  The data
     * @param[in]  size  Size of the data
     */
    void SetData(const uint8_t* data, const std::size_t size);

    /**
     * @brief Populate frame (this struct) from raw data
     *
     * On success, all the fields of the struct are set from the data.
     *
     * @param[in]  data  The data (struct can_frame or struct canfd_frame)
     * @param[in]  size  Size of the data (CAN_MTU or CANFD_MTU)
     *
     * @returns true on success, false otherwise
     */
    bool RawToFrame(const uint8_t* data, const std::size_t size);

    /**
     * @brief Set raw CAN frame from frame (this struct)
     *
     * Updates the raw CAN frame data (below) from the flags and data (above).
     */
    void FrameToRaw();

    /**
     * @brief Raw CAN frame
     */
    union
    {
        struct can_frame can_frame_;                              //!< Classical CAN frame (is_fd_ = false)
        struct canfd_frame fd_frame_ = { 0, 0, 0, 0, 0, { 0 } };  //!< FD CAN frame (is_fd_ = true)
    };
};

/**
 * @brief Raw CAN helper class using SocketCAN
 */
class RawCan : private types::NoCopyNoMove
{
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  device   Device name (e.g. "can0")
     */
    RawCan(const std::string& device);

    /**
     * @brief Destructor
     */
    ~RawCan();

    /**
     * @brief Open CAN interface
     *
     * @returns true if device was successfully opened, false otherwise
     */
    bool Open();

    /**
     * @brief Set filters
     *
     * @param[in]  filters   List of filters to apply to the CAN interface
     *
     * @returns true if filters were applied successfully, false othwise
     *
     * See https://www.kernel.org/doc/html/latest/networking/can.html#raw-socket-option-can-raw-filter and linux/can.h
     * for the can_filter struct documentation (for example, the filter flags).
     */
    bool SetFilters(const std::vector<struct can_filter>& filters);

    /**
     * @brief Close CAN interface
     */
    void Close();

    /**
     * @brief Check if CAN is open
     *
     * @returns true if socket is open, false otherwise
     */
    bool IsOpen() const;

    /**
     * @brief Get socket
     *
     * @returns the socket, -1 if interface is not opened
     */
    int GetSocket() const;

    /**
     * @brief Get last error (errno)
     *
     * @returns the last error (errno), 0 = no error
     */
    int GetErrno() const;

    /**
     * @brief Get last error string (strerror)
     *
     * @returns the last error string
     */
    std::string GetStrerror();

    /**
     * @brief Flush all pending input data
     */
    void FlushInput();

    /**
     * @brief Send raw CAN frame (standard, extended and FD, and combinations thereof)
     *
     * @param[in,out]  frame  The CAN frame to send
     *
     * @returns true if sent successfully, false otherwise
     */
    bool SendFrame(CanFrame& frame);

    /**
     * @brief Read raw classical CAN frame
     *
     * @param[out]  frame    The received frame
     * @param[in]   timeout  Timeout to wait for a frame [ms], use 0 for no wait, < 0 for infinite wait (like poll())
     *
     * @returns true if a frame was received, false otherwise
     */
    bool ReadFrame(struct can_frame& frame, const int timeout = 0);

    /**
     * @brief Send raw classical CAN frame
     *
     * @param[in]  frame  The CAN frame to send
     *
     * @returns true if sent successfully, false otherwise
     */
    bool SendFrame(const struct can_frame& frame);

    /**
     * @brief Send raw CAN FD frame
     *
     * @param[in]  frame  The CAN frame to send
     *
     * @returns true if sent successfully, false otherwise
     */
    bool SendFrame(const struct canfd_frame& frame);

    /**
     * @brief Read raw CAN FD frame
     *
     * @param[out]  frame    The received frame
     * @param[in]   timeout  Timeout to wait for a frame [ms], use 0 for no wait, < 0 for infinite wait (like poll())
     *
     * @returns true if a frame was received, false otherwise
     */
    bool ReadFrame(struct canfd_frame& frame, const int timeout = 0);

    /**
     * @brief Read raw CAN frame (standard, extended and FD, and combinations thereof)
     *
     * @param[out]  frame    The received frame
     * @param[in]   timeout  Timeout to wait for a frame [ms], use 0 for no wait, < 0 for infinite wait (like poll())
     *
     * @returns true if a frame was received, false otherwise
     */
    bool ReadFrame(CanFrame& frame, const int timeout = 0);

   protected:
    std::string device_;  //!< Device name
    int sock_;            //!< The socket to the CAN inferface
    int saved_errno_;     //!< Saved errno
};

/* ****************************************************************************************************************** */
}  // namespace can
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_CAN_HPP__
