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
 * @brief Fixposition SDK: Parser FP_B routines and types
 *
 * @page FPSDK_COMMON_PARSER_FPB Parser FP_B routines and types
 *
 * **API**: fpsdk_common/parser/fpb.hpp and fpsdk::common::parser::fpb
 *
 * @fp_msgspec_begin{FP_B-Protocol}
 *
 * **Framing:**
 *
 * The structure of a FP_B message (frame) is:
 *
 * | Offs | Size | Type     | Content         |
 * |-----:|:----:|----------|-----------------|
 * |    0 |    1 | uint8_t  | Sync 1: `f` = 102 = 0x66 = 0b0110'0110   |
 * |    1 |    1 | uint8_t  | Sync 1: `!` =  33 = 0x21 = 0b0010'0001   |
 * |    2 |    2 | uint16_t | Message ID (`message_id`), 1...65534  |
 * |    4 |    2 | uint16_t | Payload size (`payload_size`)  |
 * |    6 |    2 | uint16_t | Monotonic time [ms] (`message_time`), overflows/wraps around at ~65s  |
 * |   8… |    … | …        | Payload   |
 * | 8 + `payload_size` | 4 | uint32_t | Checksum  |
 *
 * The checksum is a 32 bit cyclic redundancy check (CRC) with the polynomial 0x32c00699. See Koopman's *CRC polynomial
 * zoo* at <http://users.ece.cmu.edu/~koopman/crc/crc32.html> for details.
 *
 * **Payload:**
 *
 * The payload encoding can be:
 *
 * - Fixed structure (for example, FP_B-MEASUREMENTS) using little-endian signed and unsigned integers of 8, 16, 32 or
 *   64 bits, as well as IEEE 754 single (32 bits) or double (64 bits) precision.
 * - Serialised data of a t.b.d. interface description language (e.g. Protobuf, Capn' Proto, Flatbuffers, ....)
 *
 * **Example:**
 *
 * A short FP_B frame with the bytes in order they appear on the wire:
 *
 *     First byte                                   Last byte
 *     |                                            |
 *     66 21 34 12 04 00 21 43 01 02 03 04 61 c4 c5 9c
 *     ^^ ^^ ^^^^^ ^^^^^ ^^^^^ ^^^^^^^^^^^ ^^^^^^^^^^^
 *     |  |  |     |     |     |           |
 *     |  |  |     |     |     |           CRC (= 0x9cc5c461)
 *     |  |  |     |     |     Payload (= [ 0x01, 0x02, 0x03, 0x04 ])
 *     |  |  |     |     Message time (= 0x4321 = 17185 = 17.185s)
 *     |  |  |     Payload size = (=x0004 = 4)
 *     |  |  Message ID (= 0x1234)
 *     |  Sync 2
 *     Sync 1
 *
 * @fp_msgspec_end
 *
 * @fp_msgspec_begin{FP_B-Payload-Rules}
 *
 * **Fixed payload definition:**
 *
 * A short and incomplete guideline:
 *
 * - All fields must be aligned
 * - Padding must be explicitly added as reserved fields: e.g. `uint8_t reserved[3] //!< Reserved for future use. Set to
 * 0.`
 * - Fields can only be basic types (no enums!)
 * - Decoding must be doable with a single pass. I.e. repeated parts must follow static parts.
 * - Repeated parts of the payload mus have a `num_foo` field in the static part.
 * - For enums the value 0 must always be "unspecified" or "unknown"
 * - Don't use bitfields. Waste some space (e.g. `uint8_t` for a bool, instead of a bit in a bitfield)
 * - Values that may or may not be valid must be accompanied by a `foo_valid` flag field.
 * - No strings! Not ever. With some exceptions, probably... :-)
 * - ...and more...
 *
 * @fp_msgspec_end
 *
 */
#ifndef __FPSDK_COMMON_PARSER_FPB_HPP__
#define __FPSDK_COMMON_PARSER_FPB_HPP__

/* LIBC/STL */
#include <cstdint>
#include <cstring>
#include <vector>

/* EXTERNAL */

/* PACKAGE */
#include "../types.hpp"

namespace fpsdk {
namespace common {
namespace parser {
/**
 * @brief Parser FP_B routines and types
 */
namespace fpb {
/* ****************************************************************************************************************** */

static constexpr std::size_t FP_B_FRAME_SIZE = 12;  //!< Size (in bytes) of FP_B frame
static constexpr std::size_t FP_B_HEAD_SIZE = 8;    //!< Size of FP_B frame header
static constexpr uint8_t FP_B_SYNC_1 = 0x66;        //!< FP_B frame sync char 1 ('f', 102, 0b0110'0110)
static constexpr uint8_t FP_B_SYNC_2 = 0x21;        //!< FP_B frame sync char 2 ('!',  33, 0b0010'0001)

/**
 * @brief Get message ID
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid FP_B message.
 *
 * @returns message ID
 */
constexpr uint16_t FpbMsgId(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[3] << 8) | (uint16_t)((uint8_t*)msg)[2]);
}

/**
 * @brief Get message time
 *
 * @param[in]  msg  Pointer to the start of the message
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid FP_B message.
 *
 * @returns message time
 */
constexpr uint16_t FpbMsgTime(const uint8_t* msg)
{
    return (((uint16_t)((uint8_t*)msg)[7] << 8) | (uint16_t)((uint8_t*)msg)[6]);
}

/**
 * @brief Get FP_B message name
 *
 * Generates a name (string) in the form "FP_B-NAME", where NAME is a suitable stringifications of the message ID if
 * known (for example, "FP_B-SYSTEMSTATUS", respectively "%05u" formatted message ID if unknown (for example,
 * "FP_B-MSG01234").
 *
 * @param[out] name      String to write the name to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the FP_B message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid FP_B message.
 *
 * @returns true if message name was generated, false if \c name buffer was too small
 */
bool FpbGetMessageName(char* name, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Get FP_B message info
 *
 * This stringifies the content of some FP_B messages, for debugging.
 *
 * @param[out] info      String to write the info to
 * @param[in]  size      Size of \c name (incl. nul termination)
 * @param[in]  msg       Pointer to the FP_B message
 * @param[in]  msg_size  Size of the \c msg
 *
 * @note No check on the data provided is done. The caller must ensure that the data is a valid FP_B message.
 *
 * @returns true if message info was generated (even if info is empty), false if \c name buffer was too small
 */
bool FpbGetMessageInfo(char* info, const std::size_t size, const uint8_t* msg, const std::size_t msg_size);

/**
 * @brief Make a FP_B message
 *
 * @param[out]  msg           The message frame
 * @param[in]   msg_id        Message ID
 * @param[in]   msg_time      Message time [ms]
 * @param[in]   payload       The message payload (up to MAX_FP_B_SIZE - FP_B_FRAME_SIZE bytes, can be empty)
 *
 * @returns true if the message was successfully constructed (\c msg now contains the message),
 *          false if failed contructing the message (payload too large)
 */
bool FpbMakeMessage(
    std::vector<uint8_t>& msg, const uint16_t msg_id, const uint16_t msg_time, const std::vector<uint8_t>& payload);

/**
 * @brief Make a FP_B message
 *
 * @param[out]  msg           The message frame
 * @param[in]   msg_id        Message ID
 * @param[in]   msg_time      Message time [ms]
 * @param[in]   payload       The message payload (up to MAX_FP_B_SIZE - FP_B_FRAME_SIZE bytes, can be NULL)
 * @param[in]   payload_size  The message payload (up to MAX_FP_B_SIZE - FP_B_FRAME_SIZE, can be 0, even if payload is
 *                            not NULL)
 *
 * @returns true if the message was successfully constructed (\c msg now contains the message),
 *          false if failed contructing the message (payload too large, bad arguments)
 */
bool FpbMakeMessage(std::vector<uint8_t>& msg, const uint16_t msg_id, const uint16_t msg_time, const uint8_t* payload,
    const std::size_t payload_size);

/**
 * @name FP_B messages (names and IDs)
 *
 * -      1- 999: unused
 * - 1000-1999: messages with payload binary serialised data
 *   - 1101-1199: Fusion
 *   - 1201-1299: GNSS
 *   - 1301-1399: System
 *   - 1401-1499: Configuration
 *   - 1501-1599: Sensor data
 *   - 1601-1699: Reserved
 *   - 1701-1699: Reserved
 *   - 1801-1699: Reserved
 *   - 1901-1999: Reserved
 * - 2000-3000: messages with static payload
 *   - 2001-2099: Input data messages (measurements, ...)
 *   - 2101-2199: Input command message
 *   - 2201-2299: Input config message
 *   - 2301-2399: System
 *   - 2401-2499: Reserved
 *   - 2501-2599: Reserved
 *   - 2601-2699: Reserved
 *   - 2701-2799: Reserved
 *   - 2801-2899: Reserved
 *   - 2901-2999: Reserved
 * - 3000-65000: Reserved
 * - 65001-...:  Test messages
 * @{
 */
// clang-format off
// @fp_codegen_begin{FPSDK_COMMON_PARSER_FPB_MESSAGES}
static constexpr uint16_t    FP_B_GNSSSTATUS_MSGID          =  1201;                    //!< FP_B-GNSSSTATUS message ID
static constexpr const char* FP_B_GNSSSTATUS_STRID          = "FP_B-GNSSSTATUS";        //!< FP_B-GNSSSTATUS message name
static constexpr uint16_t    FP_B_SYSTEMSTATUS_MSGID        =  1301;                    //!< FP_B-SYSTEMSTATUS message ID
static constexpr const char* FP_B_SYSTEMSTATUS_STRID        = "FP_B-SYSTEMSTATUS";      //!< FP_B-SYSTEMSTATUS message name
static constexpr uint16_t    FP_B_MEASUREMENTS_MSGID        =  2001;                    //!< FP_B-MEASUREMENTS message ID
static constexpr const char* FP_B_MEASUREMENTS_STRID        = "FP_B-MEASUREMENTS";      //!< FP_B-MEASUREMENTS message name
static constexpr uint16_t    FP_B_VERSION_MSGID             =  2301;                    //!< FP_B-VERSION message ID
static constexpr const char* FP_B_VERSION_STRID             = "FP_B-VERSION";           //!< FP_B-VERSION message name
static constexpr uint16_t    FP_B_UNITTEST1_MSGID           = 65001;                    //!< FP_B-UNITTEST1 message ID
static constexpr const char* FP_B_UNITTEST1_STRID           = "FP_B-UNITTEST1";         //!< FP_B-UNITTEST1 message name
static constexpr uint16_t    FP_B_UNITTEST2_MSGID           = 65002;                    //!< FP_B-UNITTEST2 message ID
static constexpr const char* FP_B_UNITTEST2_STRID           = "FP_B-UNITTEST2";         //!< FP_B-UNITTEST2 message name
// @fp_codegen_end{FPSDK_COMMON_PARSER_FPB_MESSAGES}
// clang-format off
///@}

// ---------------------------------------------------------------------------------------------------------------------
// clang-format off
// Note: the formatting here is optimised for the wiki, not for Doxygen!
/**
 * @name FP_B-MEASUREMENTS
 *
 * @fp_msgspec_begin{FP_B_MEASUREMENTS}
 *
 * **Description:**
 *
 * This message is used to input measurements, such as wheelspeeds, to the sensor.
 *
 * **Notes:**
 *
 * - The supported `meas_type`, `meas_loc` and `timestamp_type` values depend on the sensor software and configuration
 *   used. Refer to the sensor documentation for valid combinations of values for these fields.
 *
 * **Payload fields:**
 *
 * |         # | Offset    | Field             | Type           | Unit | Description                                                            |
 * |----------:|----------:|-------------------|----------------|------|------------------------------------------------------------------------|
 * |         1 |  0        | `version`         | uint8_t        | -    | Version of the FP_B_MEASUREMENTS message (currently 1)                 |
 * |         2 |  1        | `num_meas`        | uint8_t        | -    | Number of measurements present in the body of the message (1…10)       |
 * |         3 |  2        | `reserved0`       | uint8_t[6]     | -    | Reserved for future use. Set to 0.                                     |
 * |           |           |                   |                |      | *The following fields are repeated `num_meas` times (i = 0…`num_meas`-1):* |
 * |  4 + i·12 |  8 + i·28 | `meas_x`          | int32_t        | *    | Measurement x axis (for example, [mm/s])                               |
 * |  5 + i·12 | 12 + i·28 | `meas_y`          | int32_t        | *    | Measurement y axis (for example, [mm/s])                               |
 * |  6 + i·12 | 16 + i·28 | `meas_z`          | int32_t        | *    | Measurement z axis (for example, [mm/s])                               |
 * |  7 + i·12 | 20 + i·28 | `meas_x_valid`    | uint8_t        | -    | Validity of `meas_x` (1 = valid data, 0 = invalid data or n/a)         |
 * |  8 + i·12 | 21 + i·28 | `meas_y_valid`    | uint8_t        | -    | Validity of `meas_y` (1 = valid data, 0 = invalid data or n/a)         |
 * |  9 + i·12 | 22 + i·28 | `meas_z_valid`    | uint8_t        | -    | Validity of `meas_z` (1 = valid data, 0 = invalid data or n/a)         |
 * | 10 + i·12 | 23 + i·28 | `meas_type`       | uint8_t        | -    | Type of measurement (see below)                                        |
 * | 11 + i·12 | 24 + i·28 | `meas_loc`        | uint8_t        | -    | Location of measurement (see below)                                    |
 * | 12 + i·12 | 25 + i·28 | `reserved1`       | uint8_t[4]     | -    | Reserved for future use. Set to 0.                                     |
 * | 13 + i·12 | 29 + i·28 | `timestamp_type`  | uint8_t        | -    | Type of timestamp (see below)                                          |
 * | 14 + i·12 | 30 + i·28 | `gps_wno`         | uint16_t       | -    | GPS week number                                                        |
 * | 15 + i·12 | 32 + i·28 | `gps_tow`         | uint32_t       | *    | GPS time of week [ms] or monotonic time [-]                            |
 *
 * Valid `meas_type` values are:
 *
 * | Value | Description              |
 * |:-----:|--------------------------|
 * |  `0`  | Unspecified              |
 * |  `1`  | Velocity (wheel speed)   |
 *
 * Valid `meas_loc` values are:
 *
 * | Value | Description                                       |
 * |:-----:|---------------------------------------------------|
 * |  `0`  | Unspecified                                       |
 * |  `1`  | Measurement of a sensor at the rear-center (RC)   |
 * |  `2`  | Measurement of a sensor at the front-right (FR)   |
 * |  `3`  | Measurement of a sensor at the front-left (FL)    |
 * |  `4`  | Measurement of a sensor at the rear-right (RR)    |
 * |  `5`  | Measurement of a sensor at the rear-left (RL)     |
 *
 * Valid `timestamp_type` values are:
 *
 * | Value | Description                                                             |
 * |:-----:|-------------------------------------------------------------------------|
 * |  `0`  | Unspecified                                                             |
 * |  `1`  | Use time of arrival of the measurement (ignore `gps_wno` and `gps_tow`) |
 * |  `2`  | Use monotonic time [any] (stored in the `gps_tow` field)                |
 * |  `3`  | Use GPS time (stored in `gps_wno` [-] and `gps_tow` [ms] fields)        |
 *
 * @fp_msgspec_end
 *
 * @{
 */
// clang-format on

//! FP_B-MEASUREMENTS.version value
static constexpr uint8_t FP_B_MEASUREMENTS_V1 = 0x01;

//! FP_B-MEASUREMENTS payload: head
struct FpbMeasurementsHead
{
    // clang-format off
    uint8_t  version = FP_B_MEASUREMENTS_V1;  //!< Message version (= FP_B_MEASUREMENTS_V1 for this version of the message)
    uint8_t  num_meas = 0;                    //!< Number of measurements in the body (1..FP_B_MEASUREMENTS_MAX_NUM_MEAS)
    uint8_t  reserved0[6] = { 0 };            //!< Reserved for future use. Set to 0.
    // clang-format on
};

//! FP_B-MEASUREMENTS payload head size
static constexpr std::size_t FP_B_MEASUREMENTS_HEAD_SIZE = 8;

//! FP_B-MEASUREMENTS measurement type
enum class FpbMeasurementsMeasType : uint8_t  // clang-format off
{
    UNSPECIFIED = 0,  //!< Unspecified
    VELOCITY    = 1,  //!< Velocity measuement (wheel speed)
    // ....
};  // clang-format on

//! FP_B-MEASUREMENTS measurement location
enum class FpbMeasurementsMeasLoc : uint8_t  // clang-format off
{
    UNSPECIFIED = 0,  //!< Unspecified
    RC          = 1,  //!< Measurement of a sensor at the rear-center (RC)
    FR          = 2,  //!< Measurement of a sensor at the front-right (FR)
    FL          = 3,  //!< Measurement of a sensor at the front-left (FL)
    RR          = 4,  //!< Measurement of a sensor at the rear-right (RR)
    RL          = 5,  //!< Measurement of a sensor at the rear-left (RL)
};  // clang-format on

//! FP_B-MEASUREMENTS timestamp type
enum class FpbMeasurementsTimestampType : uint8_t  // clang-format off
{
    UNSPECIFIED   = 0,  //!< Unspecified
    TIMEOFARRIVAL = 1,  //!< Use time of arrival of the measurement (ignore gps_wno and gps_tow)
    MONOTONIC     = 2,  //!< Use monotonic time [any] (stored in the gps_tow field)
    GPS           = 3,  //!< Use GPS time (stored in gps_wno [-] and gps_tow [ms] fields)
};  // clang-format on

//! FP_B-MEASUREMENTS payload: measurement
struct FpbMeasurementsMeas
{
    // clang-format off
    int32_t  meas_x = 0 ;          //!< Measurement x
    int32_t  meas_y = 0 ;          //!< Measurement y
    int32_t  meas_z = 0 ;          //!< Measurement z
    uint8_t  meas_x_valid = 0;     //!< Validity of measurement x (1 = meas_x contains valid data, 0 = data invalid or n/a)
    uint8_t  meas_y_valid = 0;     //!< Validity of measurement y (1 = meas_x contains valid data, 0 = data invalid or n/a)
    uint8_t  meas_z_valid = 0;     //!< Validity of measurement z (1 = meas_x contains valid data, 0 = data invalid or n/a)
    uint8_t  meas_type             //! See #FpbMeasurementsMeasType
         = types::EnumToVal(FpbMeasurementsMeasType::UNSPECIFIED);
    uint8_t  meas_loc              //! See #FpbMeasurementsMeasLoc
        = types::EnumToVal(FpbMeasurementsMeasLoc::UNSPECIFIED);
    uint8_t  reserved1[4] = { 0 }; //!< Reserved for future use. Set to 0.
    uint8_t  timestamp_type        //! See #FpbMeasurementsTimestampType
        = types::EnumToVal(FpbMeasurementsTimestampType::UNSPECIFIED);
    uint16_t gps_wno = 0;          //!< GPS week number [-]
    uint32_t gps_tow = 0;          //!< GPS time of week [ms] or monotonic time [-]
    // clang-format on
};

//! Size of FpbMeasurementsMeas
static constexpr std::size_t FP_B_MEASUREMENTS_MEAS_SIZE = 28;

//! Maximum number of measurements
static constexpr std::size_t FP_B_MEASUREMENTS_MAX_NUM_MEAS = 10;  // Keep in sync with the docu above!

static_assert(sizeof(FpbMeasurementsHead) == FP_B_MEASUREMENTS_HEAD_SIZE, "");
static_assert(sizeof(FpbMeasurementsMeas) == FP_B_MEASUREMENTS_MEAS_SIZE, "");

///@}
// ---------------------------------------------------------------------------------------------------------------------
// clang-format off
/**
 * @name FP_B-VERSION
 *
 * @fp_msgspec_begin{FP_B-VERSION}
 *
 * **Description:**
 *
 * This message contains version strings.
 *
 * **Payload fields:**
 *
 * |  # | Offset  | Field             | Type           | Unit | Description                                          |
 * |---:|--------:|-------------------|----------------|------|------------------------------------------------------|
 * |  1 |    0    | `version`         | uint8_t        | -    | Version of the FP_B_VERSION message (currently 1)    |
 * |  2 |    1    | `reserved0`       | uint8_t[7]     | -    | Reserved for future use. Set to 0.                   |
 * |  3 |    8    | `sw_version`      | uint8_t[64]    | -    | Software version (nul-terminated string)             |
 * |  4 |   72    | `hw_name`         | uint8_t[32]    | -    | Hardware name string (nul-terminated string)         |
 * |  5 |  104    | `hw_ver`          | uint8_t[32]    | -    | Hardware version (nul-terminated string)             |
 * |  6 |  136    | `hw_uid`          | uint8_t[32]    | -    | Hardware UID (nul-terminated string)                 |
 * |  7 |  172    | `product_model`   | uint8_t[32]    | -    | Product model (nul-terminated string)                |
 * |  8 |  200    | `reserved1`       | uint8_t[32]    | -    | Reserved for future use. Set to 0.                   |
 * @fp_msgspec_end
 *
 * @{
 */
// clang-format on

//! FP_B-VERSION.version value
static constexpr uint8_t FP_B_VERSION_V1 = 0x01;

//! FP_B-VERSION payload: head
struct FpbVersionPayload
{  // clang-format off
    uint8_t  version = FP_B_VERSION_V1;  //!< Message version (= FP_B_VERSION_V1 for this version of the message)
    uint8_t  reserved0[7]      = { 0 };  //!< Reserved for future use. Set to 0.
    uint8_t  sw_version[64]    = { 0 };  //!< Software version (nul-terminated string)
    uint8_t  hw_name[32]       = { 0 };  //!< Hardware name string (nul-terminated string)
    uint8_t  hw_ver[32]        = { 0 };  //!< Hardware version (nul-terminated string)
    uint8_t  hw_uid[32]        = { 0 };  //!< Hardware UID (nul-terminated string)
    uint8_t  product_model[32] = { 0 };  //!< Product model (nul-terminated string)
    uint8_t  reserved1[32]     = { 0 };  //!< Reserved for future use. Set to 0.
};  // clang-format on

//! Size of FpbVersionPayload
static constexpr std::size_t FP_B_VERSION_PAYLOAD_SIZE = 232;
static_assert(sizeof(FpbVersionPayload) == FP_B_VERSION_PAYLOAD_SIZE, "");

///@}

/* ****************************************************************************************************************** */
}  // namespace fpb
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_FPB_HPP__
