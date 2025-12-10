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
 * @brief Fixposition SDK: Parser
 */

/* LIBC/STL */
#include <array>
#include <cstdio>
#include <cstring>
#include <functional>

/* EXTERNAL */

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/parser.hpp"
#include "fpsdk_common/parser/crc.hpp"
#include "fpsdk_common/parser/fpa.hpp"
#include "fpsdk_common/parser/fpb.hpp"
#include "fpsdk_common/parser/nmea.hpp"
#include "fpsdk_common/parser/novb.hpp"
#include "fpsdk_common/parser/qgc.hpp"
#include "fpsdk_common/parser/rtcm3.hpp"
#include "fpsdk_common/parser/sbf.hpp"
#include "fpsdk_common/parser/spartn.hpp"
#include "fpsdk_common/parser/ubx.hpp"
#include "fpsdk_common/parser/unib.hpp"

namespace fpsdk {
namespace common {
namespace parser {
/* ****************************************************************************************************************** */

using namespace fpsdk::common::parser::crc;
using namespace fpsdk::common::parser::fpa;
using namespace fpsdk::common::parser::fpb;
using namespace fpsdk::common::parser::nmea;
using namespace fpsdk::common::parser::novb;
using namespace fpsdk::common::parser::qgc;
using namespace fpsdk::common::parser::rtcm3;
using namespace fpsdk::common::parser::sbf;
using namespace fpsdk::common::parser::spartn;
using namespace fpsdk::common::parser::ubx;
using namespace fpsdk::common::parser::unib;

// ---------------------------------------------------------------------------------------------------------------------

Parser::Parser()
{
    Reset();
}

// ---------------------------------------------------------------------------------------------------------------------

void Parser::Reset()
{
    size_ = 0;
    offs_ = 0;
    stats_ = ParserStats();
}

// ---------------------------------------------------------------------------------------------------------------------

bool Parser::Add(const uint8_t* data, const std::size_t size)
{
    if (data == NULL) {
        return false;
    }
    if (size == 0) {
        return true;
    }

    // Overflow (no space for data)
    if ((offs_ + size_ + size) > sizeof(buf_)) {
        return false;
    }

    // Add to buffer
    std::memcpy(&buf_[offs_ + size_], data, size);
    size_ += size;
    return true;
}

bool Parser::Add(const std::vector<uint8_t>& data)
{
    return Add(data.data(), data.size());
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Detector functions to test the presence of a message in the buffer
 *
 * @param[in]  buf   Pointer to the buffer to check for presence of a message
 * @param[in]  size  Size of data in buffer
 *
 * @returns either the special values NADA or WAIT, or the the message size
 */
using IsMessageFunc = std::function<int(const uint8_t* buf, const std::size_t size)>;
static constexpr int NADA = 0;   //!< Indicates that there is no message in the buffer
static constexpr int WAIT = -1;  //!< Indicates that more data is needed to decide (e.g., partial message in buffer)

//! Check for presence of a UBX message?, in the buffer (see IsMessageFunc)
static int IsUbxMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a NMEA (or FP_A) message in the buffer (see IsMessageFunc)
static int IsNmeaMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a RTCM3 message in the buffer (see IsMessageFunc)
static int IsRtcm3Message(const uint8_t* buf, const std::size_t size);
//! Check for presence of a SPARTN message in the buffer (see IsMessageFunc)
static int IsSpartnMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a NOV_B message in the buffer (see IsMessageFunc)
static int IsNovbMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a FP_B message in the buffer (see IsMessageFunc)
static int IsFpbMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a UNI_B message in the buffer (see IsMessageFunc)
static int IsUnibMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a SBF message in the buffer (see IsMessageFunc)
static int IsSbfMessage(const uint8_t* buf, const std::size_t size);
//! Check for presence of a QGC message in the buffer (see IsMessageFunc)
static int IsQgcMessage(const uint8_t* buf, const std::size_t size);

//! Helper for iterating through the different message frame detectors
struct ParserFunc
{
    IsMessageFunc is_message_func_;
    Protocol proto_;
    const char* proto_name_;
};

//! List of different message frame detectors
static const std::array<ParserFunc, 9> PARSER_FUNCS = { {
    // clang-format off
    { IsFpbMessage,    Protocol::FP_B,   PROTOCOL_NAME_FP_B },
    { IsUbxMessage,    Protocol::UBX,    PROTOCOL_NAME_UBX },
    { IsNmeaMessage,   Protocol::NMEA,   PROTOCOL_NAME_NMEA },
    { IsRtcm3Message,  Protocol::RTCM3,  PROTOCOL_NAME_RTCM3 },
    { IsUnibMessage,   Protocol::UNI_B,  PROTOCOL_NAME_UNI_B },
    { IsNovbMessage,   Protocol::NOV_B,  PROTOCOL_NAME_NOV_B },
    { IsSbfMessage,    Protocol::SBF,    PROTOCOL_NAME_SBF },
    { IsQgcMessage,    Protocol::QGC,    PROTOCOL_NAME_QGC },
    { IsSpartnMessage, Protocol::SPARTN, PROTOCOL_NAME_SPARTN },
}};  // clang-format on

// ---------------------------------------------------------------------------------------------------------------------

#if 0
#  define PARSER_TRACE(...) TRACE(__VA_ARGS__)
#  define PARSER_TRACE_HEXDUMP(...) TRACE_HEXDUMP(__VA_ARGS__)
#else
#  define PARSER_TRACE(...)         /* nothing */
#  define PARSER_TRACE_HEXDUMP(...) /* nothing */
#endif

bool Parser::Process(ParserMsg& msg)
{
    while (size_ > 0) {
        PARSER_TRACE_HEXDUMP(
            &buf_[offs_], std::min(size_, (std::size_t)16), "    ", "Parser: offs_=%d size_%d", (int)offs_, (int)size_);

        // Run parser functions
        int msg_size = 0;
        Protocol proto = Protocol::OTHER;
        for (auto& parser : PARSER_FUNCS) {
            msg_size = parser.is_message_func_(&buf_[offs_], size_);
            PARSER_TRACE("process: try %s, msg_size=%d", parser.proto_name_, (int)msg_size);

            // Parser said: Wait, need more data
            if (msg_size == WAIT) {
                break;
            }
            // Parser said: I have a message
            else if (msg_size > 0) {
                proto = parser.proto_;
                break;
            }
            // else (msg_size == NADA) // Parser said: Not my message
        }

        // Waiting for more data...
        if (msg_size == WAIT) {
            PARSER_TRACE("process: need more data");
            return false;
        }

        // No known message in buffer, move first byte to garbage
        else if (msg_size == NADA) {
            //     buf: GGGG?????????????................ (offs_ >= 0, size_ > 0)
            // --> buf: GGGGG????????????................ (offs_ > 0, size_ >= 0)
            offs_++;
            size_--;
            PARSER_TRACE("process: collect garbage");

            // Garbage bin full
            if (offs_ >= MAX_OTHER_SIZE) {
                EmitGarbage(msg);
                return true;
            }
        }

        // We have a message (and come back to the same message in the next iteration)
        else {  // msgLen > 0
            // Return garbage first
            if (offs_ > 0) {
                EmitGarbage(msg);
                return true;
            } else {  // offs_ == 0: Return message
                EmitMessage(msg, msg_size, proto);
                return true;
            }
        }
    }

    // All data consumed, return garbage immediately if there is any
    if (offs_ > 0) {
        EmitGarbage(msg);
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Parser::Flush(ParserMsg& msg)
{
    // buf: GGGGGGGGGGGGG???????????????........
    //      --- offs_ --><--  size_  -->
    //      <---------- rem ----------->
    const std::size_t rem = offs_ + size_;
    if (rem > 0) {
        if (rem > MAX_OTHER_SIZE) {
            offs_ = MAX_OTHER_SIZE;
            size_ = rem - offs_;
        } else {
            offs_ += size_;
            size_ = 0;
        }
        EmitGarbage(msg);
        return true;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void Parser::EmitMessage(ParserMsg& msg, const std::size_t size, const Protocol proto)
{
    // Copy message to tmp, move remaining data to beginning of buf
    //     buf: MMMMMMMMMMMMMMM????????............. (offs_ = 0)
    //          <-- msg_size ->
    //          <------- size_ ------->
    // --> tmp: MMMMMMMMMMMMMMM.....
    // --> buf: ????????............................ (offs_ = 0, size_ >= 0)

    // PARSER_TRACE("msg copy tmp %d", size);
    msg.data_ = { buf_, buf_ + size };
    // PARSER_TRACE("msg move 0 <- %d (%d)", size_ - size, size);
    size_ -= size;
    if (size_ > 0) {
        std::memmove(&buf_[0], &buf_[size], size_);
    }
    // Make message
    msg.proto_ = proto;
    msg.info_.clear();
    char sname[MAX_NAME_SIZE];
    const uint8_t* mdata = msg.Data();
    const std::size_t msize = msg.Size();
    switch (proto) {  // clang-format off
        case Protocol::FP_A:  // Handled in case Protocol::NMEA below                                                   // GCOVR_EXCL_LINE
            break;                                                                                                      // GCOVR_EXCL_LINE
        case Protocol::FP_B:
            msg.name_ = (FpbGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "FP_B-?");                     // GCOVR_EXCL_LINE
            break;
        case Protocol::NMEA: // resp. FP_A
            if ((msg.data_[1] == 'F') && (msg.data_[2] == 'P') && (msg.data_[3] == ',')) {
                msg.name_ =  (FpaGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "FP_A-?");                // GCOVR_EXCL_LINE
                msg.proto_ = Protocol::FP_A;
            } else {
                msg.name_ = (NmeaGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "NMEA-?-?");              // GCOVR_EXCL_LINE
            }
            break;
        case Protocol::UBX:
            msg.name_ = (UbxGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "UBX-?-?");                    // GCOVR_EXCL_LINE
            break;
        case Protocol::RTCM3:
            msg.name_ = (Rtcm3GetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "RTCM3-?");                  // GCOVR_EXCL_LINE
            break;
        case Protocol::UNI_B:
            msg.name_ = (UnibGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "UNI_B-?");                   // GCOVR_EXCL_LINE
            break;
        case Protocol::NOV_B:
            msg.name_ = (NovbGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "NOV_B-?");                   // GCOVR_EXCL_LINE
            break;
        case Protocol::SBF:
            msg.name_ = (SbfGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "SBF-?");                      // GCOVR_EXCL_LINE
            break;
        case Protocol::QGC:
            msg.name_ = (QgcGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "Qgc-?");                      // GCOVR_EXCL_LINE
            break;
        case Protocol::SPARTN:
            msg.name_ = (SpartnGetMessageName(sname, sizeof(sname), mdata, msize) ? sname : "SPARTN-?-?");              // GCOVR_EXCL_LINE
            break;
        case Protocol::OTHER:  // Handled in EmitGarbage()                                                              // GCOVR_EXCL_LINE
            break;                                                                                                      // GCOVR_EXCL_LINE
    }  // clang-format on

    stats_.Update(msg);
    msg.seq_ = stats_.n_msgs_;
    PARSER_TRACE("process: emit %s, size %d", msg.name_.c_str(), (int)msg.Size());
}

// ---------------------------------------------------------------------------------------------------------------------

void Parser::EmitGarbage(ParserMsg& msg)
{
    // Copy garbage to msg buf and move data in parser buf
    //     buf: GGGGGGGGGGGGG???????????????........ (offs_ > 0, size_ >= 0)
    //          --- offs_ --><--  size_  -->
    // --> tmp: GGGGGGGGGGGG......
    // --> buf  ???????????????..................... (offs_ = 0, size_ >= 0)
    const std::size_t size = offs_;
    // PARSER_TRACE("garb copy tmp %d ", size);
    msg.data_ = { buf_, buf_ + size };
    // PARSER_TRACE("garb move 0 <- %d (%d) ", size, size_);
    std::memmove(&buf_[0], &buf_[size], size_);
    offs_ = 0;

    // Make message
    msg.proto_ = Protocol::OTHER;
    msg.name_ = PROTOCOL_NAME_OTHER;
    msg.info_.clear();
    stats_.Update(msg);
    msg.seq_ = stats_.n_msgs_;

    PARSER_TRACE("process: emit %s, size %d ", msg.name_.c_str(), (int)msg.Size());
}

// ---------------------------------------------------------------------------------------------------------------------

ParserStats Parser::GetStats() const
{
    return stats_;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsFpbMessage(const uint8_t* buf, const std::size_t size)
{
    // We need the two sync chars
    if (buf[0] != FP_B_SYNC_1) {
        return NADA;
    }
    if (size < 2) {
        return WAIT;
    }
    if (buf[1] != FP_B_SYNC_2) {
        return NADA;
    }

    // Wait for full header
    if (size < FP_B_HEAD_SIZE) {
        return WAIT;
    }

    // Limit payload size
    const uint16_t payload_size = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    if (payload_size > (MAX_FP_B_SIZE - FP_B_FRAME_SIZE)) {
        return NADA;
    }

    // Wait for entire message
    if (size < (payload_size + FP_B_FRAME_SIZE)) {
        return WAIT;
    }

    // Verify using CRC
    const uint32_t crc = *((uint32_t*)&buf[payload_size + FP_B_HEAD_SIZE]);
    if (crc == Crc32fpb(&buf[0], payload_size + FP_B_HEAD_SIZE)) {
        return payload_size + FP_B_FRAME_SIZE;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsUbxMessage(const uint8_t* buf, const std::size_t size)
{
    // We need the two sync chars
    if (buf[0] != UBX_SYNC_1) {
        return NADA;
    }
    if (size < 2) {
        return WAIT;
    }
    if (buf[1] != UBX_SYNC_2) {
        return NADA;
    }

    // Wait for full header
    if (size < UBX_HEAD_SIZE) {
        return WAIT;
    }

    // Limit payload size
    const std::size_t payload_size = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    if (payload_size > (MAX_UBX_SIZE - UBX_FRAME_SIZE)) {
        return NADA;
    }

    // Wait for entire message
    if (size < (payload_size + UBX_FRAME_SIZE)) {
        return WAIT;
    }

    // Verify using checksum
    const uint16_t ck = *((uint16_t*)&buf[payload_size + UBX_HEAD_SIZE]);
    if (ck == ChecksumUbx(&buf[2], payload_size + (UBX_HEAD_SIZE - 2))) {
        return payload_size + UBX_FRAME_SIZE;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsNmeaMessage(const uint8_t* buf, const std::size_t size)
{
    // Start of sentence
    if (buf[0] != NMEA_PREAMBLE) {
        return NADA;
    }

    // Don't mistake it for a SBF message
    if ((size > 1) && (buf[1] == SBF_SYNC_1)) {
        return NADA;
    }

    // Find end of sentence, calculate checksum along the way
    std::size_t len = 1;  // Length of sentence excl. "$"
    uint8_t ck = 0;
    while (true) {
        if (len > MAX_NMEA_SIZE) {
            return NADA;
        }
        if (len >= size) {  // len doesn't include '$'
            return WAIT;
        }
        if ((buf[len] == '\r') || (buf[len] == '\n') || (buf[len] == '*')) {
            break;
        }
        // clang-format off
        if ( // ((buf[len] & 0x80) != 0) || // 7-bit only
             (buf[len] < 0x20) || (buf[len] > 0x7e) || // outside valid range
             (buf[len] == '$') || (buf[len] == '\\') || (buf[len] == '!') || (buf[len] == '~') ) { // reserved

            return NADA;
        }  // clang-format on
        ck ^= buf[len];
        len++;
    }

    // Not nough data for sentence end (star + checksum + \r\n)?
    if (size < (len + 1 + 2 + 2)) {
        return WAIT;
    }

    // Properly terminated sentence?
    if ((buf[len] == '*') && (buf[len + 3] == '\r') && (buf[len + 4] == '\n')) {
        uint8_t n1 = buf[len + 1];
        uint8_t n2 = buf[len + 2];
        uint8_t c1 = '0' + ((ck >> 4) & 0x0f);
        uint8_t c2 = '0' + (ck & 0x0f);
        if (c2 > '9') {
            c2 += 'A' - '9' - 1;
        }
        // Checksum valid?
        if ((n1 == c1) && (n2 == c2)) {
            return len + 5;
        }
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsRtcm3Message(const uint8_t* buf, const std::size_t size)
{
    // Not RTCM3 preamble?
    if (buf[0] != RTCM3_PREAMBLE) {
        return NADA;
    }

    // Wait for enough data to look at header
    if (size < RTCM3_HEAD_SIZE) {
        return WAIT;
    }

    // Check header
    const uint16_t head = (uint16_t)buf[2] | ((uint16_t)buf[1] << 8);
    const std::size_t payload_size = head & 0x03ff;  // 10 bits
    // const uint16_t empty = head & 0xfc00; // 6 bits

    // Too large?
    const std::size_t msg_size = payload_size + RTCM3_FRAME_SIZE;
    if ((msg_size > MAX_RTCM3_SIZE) /*|| (empty != 0x0000)*/) {
        return NADA;
    }

    // Wait for full message
    if (size < msg_size) {
        return WAIT;
    }

    // CRC okay?
    const uint32_t crc =
        ((uint32_t)buf[msg_size - 3] << 16) | ((uint32_t)buf[msg_size - 2] << 8) | ((uint32_t)buf[msg_size - 1]);
    if (crc == Crc24rtcm3(&buf[0], msg_size - 3)) {
        return msg_size;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsSpartnMessage(const uint8_t* buf, const std::size_t size)
{
    // Not RTCM3 preamble?
    if (buf[0] != SPARTN_PREAMBLE) {
        return NADA;
    }

    // Wait for enough data to look at header
    if (size < SPARTN_MIN_HEAD_SIZE) {
        return WAIT;
    }

    // byte 0    0b11111111   8 bits  preamble
    // byte 1    0b1111111.   7 bits  message type
    // const int proto = (buf[1] & 0xfe) >> 1;
    //           0b.......1 .
    // byte 2    0b11111111  10 bits  payload length
    // byte 3    0b1....... .
    const std::size_t payload_size =
        ((uint16_t)(buf[1] & 0x01) << 9) | ((uint16_t)buf[2] << 1) | (((uint16_t)buf[3] & 0x80) >> 7);
    //           0b.1......   1 bit   encryption and authentication flag
    const int enc_auth_flag = (buf[3] & 0x40) >> 6;
    //           0b..11....   2 bits  crc type
    const int crc_type = (buf[3] & 0x30) >> 4;
    //           0b....1111   4 bits  frame crc
    const uint8_t frame_crc = (buf[3] & 0x0f);
    uint8_t tmp[3] = { buf[1], buf[2], (uint8_t)(buf[3] & 0xf0) };
    if (frame_crc != Crc4spartn(tmp, sizeof(tmp))) {
        return NADA;
    }
    // byte 4    0b1111....   4 bits  message subtype
    // const int msgSubType = (buf[4] & 0xf0) >> 4;
    //           0b....1...   1 bit   time tag type
    const int time_tag_type = (buf[4] & 0x08) >> 3;
    //           0b.....111 .
    // byte 5    0b11111111  16 bits  gnss time tag
    //         6 0b11111111  or 32
    //         7 0b11111111
    // byte 6  8 0b11111... .
    //           0b.....111 .
    // byte 7  9 0b1111.... . 7 bits  solution id
    //           0b....1111   4 bits  slution processor id
    std::size_t msg_size = (time_tag_type ? 10 : 8);
    if (size < msg_size) {
        return WAIT;
    }
    // if ea flag is set
    // byte s    0b1111....   4 bits  encryption id
    // byte s+1  0b....1111 .
    //           0b11...... - 6 bits  encryption sequence number
    //           0b..111... - 3 bits  authentication indicator
    const int auth_ind = (buf[msg_size + 1] & 38) >> 3;
    //           0b.....111 - 3 bits  embedded authentication length
    int ea_length = -1;
    if (enc_auth_flag) {
        if (auth_ind > 1) {
            /*const int*/ ea_length = (buf[msg_size + 1] & 0x07);
            const int ea_lengths[8] = { 8, 12, 16, 32, 64, 0, 0, 0 };
            msg_size += ea_lengths[ea_length];
        }
        msg_size += 2;
    }
    // byte s+2...  payload
    msg_size += payload_size;
    // message crc
    const int crc_sizes[4] = { 1, 2, 3, 4 };
    msg_size += crc_sizes[crc_type];

    if (size < msg_size) {
        return WAIT;
    }

    uint32_t msg_crc = 0;
    uint32_t chk_crc = 0;
    switch (crc_type) {
        case 0:  // FIXME: untested
            msg_crc = (uint32_t)buf[msg_size - 1];
            chk_crc = Crc8spartn(&buf[1], msg_size - 1 - crc_sizes[crc_type]);
            break;
        case 1:  // FIXME: untested
            msg_crc = ((uint32_t)buf[msg_size - 2] << 8) | ((uint32_t)buf[msg_size - 1]);
            chk_crc = Crc16spartn(&buf[1], msg_size - 1 - crc_sizes[crc_type]);
            break;
        case 2:  // OK
            msg_crc = ((uint32_t)buf[msg_size - 3] << 16) | ((uint32_t)buf[msg_size - 2] << 8) |
                      ((uint32_t)buf[msg_size - 1]);
            chk_crc = Crc24rtcm3(&buf[1], msg_size - 1 - crc_sizes[crc_type]);
            break;
        case 3:  // FIXME: untested
            msg_crc = ((uint32_t)buf[msg_size - 4] << 24) | ((uint32_t)buf[msg_size - 3] << 16) |
                      ((uint32_t)buf[msg_size - 2] << 8) | ((uint32_t)buf[msg_size - 1]);
            chk_crc = Crc32spartn(&buf[1], msg_size - 1 - crc_sizes[crc_type]);
            break;
    }

    if (msg_crc == chk_crc) {
        return msg_size;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsNovbMessage(const uint8_t* buf, const std::size_t size)
{
    // Need the correct three sync chars
    if (buf[0] != NOV_B_SYNC_1) {
        return NADA;
    }
    if (size < 3) {
        return WAIT;
    }
    if ((buf[1] != NOV_B_SYNC_2) || ((buf[2] != NOV_B_SYNC_3_LONG) && (buf[2] != NOV_B_SYNC_3_SHORT))) {
        return NADA;
    }

    // https://docs.novatel.com/OEM7/Content/Messages/Binary.htm
    // https://docs.novatel.com/OEM7/Content/Messages/Description_of_Short_Headers.htm
    // Offset  Type      Long header   Short header
    // 0       uint8_t   0xaa         uint8_t   0xaa
    // 1       uint8_t   0x44         uint8_t   0x44
    // 2       uint8_t   0x12         uint8_t   0x13
    // 3       uint8_t   header len   uint8_t   msg len
    // 4       uint16_t  msg ID       uint16_t  msg ID
    // 6       uint8_t   msg type     uint16_t  wno
    // 7       uint8_t   port addr
    // 8       uint16_t  msg len      int32_t   tow
    // ...
    // 3+hlen  payload             offs 13 payload

    if (size < 12) {
        return WAIT;
    }

    std::size_t len = 0;

    // Long header
    if (buf[2] == NOV_B_SYNC_3_LONG) {
        const uint8_t header_len = buf[3];
        const uint16_t msg_len = ((uint16_t)buf[9] << 8) | (uint16_t)buf[8];
        len = header_len + msg_len + sizeof(uint32_t);
    }
    // Short header
    else {
        const uint8_t header_len = 12;
        const uint8_t msg_len = buf[3];
        len = header_len + msg_len + sizeof(uint32_t);
    }

    // Discard too long messages
    if (len > MAX_NOV_B_SIZE) {
        return NADA;
    }

    // Wait for full message
    if (size < len) {
        return WAIT;
    }

    // Verify using CRC
    const uint32_t crc = (buf[len - 1] << 24) | (buf[len - 2] << 16) | (buf[len - 3] << 8) | (buf[len - 4]);
    if (crc == Crc32novb(buf, len - sizeof(uint32_t))) {
        return len;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsUnibMessage(const uint8_t* buf, const std::size_t size)
{
    // Need the correct three sync chars
    if (buf[0] != UNI_B_SYNC_1) {
        return NADA;
    }
    if (size < 3) {
        return WAIT;
    }
    if ((buf[1] != UNI_B_SYNC_2) || (buf[2] != UNI_B_SYNC_3)) {
        return NADA;
    }

    // Wait for UnibHeader.size
    if (size < 8) {
        return WAIT;
    }

    // Wait for complete message
    const std::size_t len = sizeof(UnibHeader) + UnibMsgSize(buf) + sizeof(uint32_t);

    // Discard too long messages
    if (len > MAX_UNI_B_SIZE) {
        return NADA;
    }

    // Wait for full message
    if (size < len) {
        return WAIT;
    }

    // Verify using CRC
    const uint32_t crc = (buf[len - 1] << 24) | (buf[len - 2] << 16) | (buf[len - 3] << 8) | (buf[len - 4]);
    if (crc == Crc32novb(buf, len - sizeof(uint32_t))) {
        return len;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

static int IsSbfMessage(const uint8_t* buf, const std::size_t size)
{
    // We need the two sync chars
    if (buf[0] != SBF_SYNC_1) {
        return NADA;
    }
    if (size < 2) {
        return WAIT;
    }
    if (buf[1] != SBF_SYNC_2) {
        return NADA;
    }

    // Wait for full header
    if (size < SBF_HEAD_SIZE) {
        return WAIT;
    }

    // // Limit message size
    const std::size_t message_size = ((uint16_t)buf[6] | ((uint16_t)buf[7] << 8));
    if (message_size > MAX_SBF_SIZE) {
        return NADA;
    }

    // Wait for entire message
    if (size < message_size) {
        return WAIT;
    }

    // Verify using checksum
    const uint16_t ck = ((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    if (ck == Crc16sbf(&buf[4], message_size - 4)) {
        return message_size;
    }

    return NADA;
}

// ---------------------------------------------------------------------------------------------------------------------

// Like IsUbxMessage(), but with different sync chars
static int IsQgcMessage(const uint8_t* buf, const std::size_t size)
{
    // We need the two sync chars
    if (buf[0] != QGC_SYNC_1) {
        return NADA;
    }
    if (size < 2) {
        return WAIT;
    }
    if (buf[1] != QGC_SYNC_2) {
        return NADA;
    }

    // Wait for full header
    if (size < QGC_HEAD_SIZE) {
        return WAIT;
    }

    // Limit payload size
    const std::size_t payload_size = ((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    if (payload_size > (MAX_QGC_SIZE - QGC_FRAME_SIZE)) {
        return NADA;
    }

    // Wait for entire message
    if (size < (payload_size + QGC_FRAME_SIZE)) {
        return WAIT;
    }

    // Verify using checksum
    const uint16_t ck = *((uint16_t*)&buf[payload_size + QGC_HEAD_SIZE]);
    if (ck == ChecksumUbx(&buf[2], payload_size + (QGC_HEAD_SIZE - 2))) {
        return payload_size + QGC_FRAME_SIZE;
    }

    return NADA;
}

/* ****************************************************************************************************************** */
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
