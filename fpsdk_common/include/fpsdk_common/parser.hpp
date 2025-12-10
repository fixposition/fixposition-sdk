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
 *
 * @page FPSDK_COMMON_PARSER Parser
 *
 * **API**: fpsdk_common/parser.hpp and fpsdk::common::parser
 *
 * @section FPSDK_COMMON_PARSER_INTRO Introduction
 *
 * This implements a message parser for various protocols (see #fpsdk::common::parser::Protocol). A
 * fpsdk::common::parser::Parser object is fed with data and it emits one fpsdk::common::parser::ParserMsg after
 * another. No data is discarded, any data input into the parser is also output as a message. Unknown parts of the input
 * data (other protocols not recognised by this parser, incorrect or incomplete messages, spurious data, line noise,
 * etc.) are output as one or multiple messages of type fpsdk::common::parser::Protocol::OTHER.
 *
 * Note that the Parser does not *decode* messages. It only *extracts* messages (frames) from a stream of bytes.
 * The *decoding* of messages, i.e. decoding the data or fields from the message payload must be done by the
 * application. However, a number of types and utility functions to facilitate this are included.
 *
 * The Parser code is organised into the following parts:
 *
 * - @subpage FPSDK_COMMON_PARSER_TYPES
 * - @subpage FPSDK_COMMON_PARSER_CRC
 * - @subpage FPSDK_COMMON_PARSER_FPA
 * - @subpage FPSDK_COMMON_PARSER_FPB
 * - @subpage FPSDK_COMMON_PARSER_NMEA
 * - @subpage FPSDK_COMMON_PARSER_NOVB
 * - @subpage FPSDK_COMMON_PARSER_QGC
 * - @subpage FPSDK_COMMON_PARSER_RTCM3
 * - @subpage FPSDK_COMMON_PARSER_SBF
 * - @subpage FPSDK_COMMON_PARSER_SPARTN
 * - @subpage FPSDK_COMMON_PARSER_UBX
 * - @subpage FPSDK_COMMON_PARSER_UNIB
 *
 * @section FPSDK_COMMON_PARSER_USAGE Usage
 *
 * The usage of the parser is best explained in source code:
 *
 * @code{cpp}
 * #include <cstdint>
 * #include <ifstream>
 *
 * #include <fpsdk_common/logging.hpp>
 * #include <fpsdk_common/parser.hpp>
 *
 * using namespace fpsdk::common::parser;
 *
 * std::ifstream input("some_logfile.bin", std::ios::binary);
 * Parser parser;
 * ParserMsg msg;
 *
 * while (input.good()) {
 *
 *     // Read a chunk of data from the logfile
 *     uint8_t data[MAX_ADD_SIZE];
 *     const std::size_t size = input.readsome((char*)data, sizeof(data));
 *
 *     // No more data from file
 *     if (size <= 0) {
 *         break;
 *     }
 *
 *     // Add the chunk of data to the parser
 *     if (!parser.Add(data, size)) {
 *         // We added too much data! This will not happen if we don't add more than MAX_ANY_SIZE at a time.
 *         WARNING(("Parser overflow!");
 *         parser.Reset();
 *         continue;
 *     }
 *
 *     // Run parser and print message name and size to the screen
 *     while (parser.Process(msg)) {
 *         msg.MakeInfo();  // Try to populate msg.info_
 *         INFO("Message %s, size %d (%s)", msg.name_.c_str(), (int)msg.Size(), msg.info_.c_str());
 *     }
 * }
 *
 * // There may be some remaining data in the parser.
 * // If there is anytghing, we should see only type OTHER "messages" here.
 * while (parser.Flush(msg)) {
 *     msg.MakeInfo();
 *     INFO("Message %s, size %d", msg.name_.c_str(), (int)msg.Size(), msg.info_.c_str());
 * }
 * @endcode
 *
 * For a more sophisticated example app see the @ref FPSDK_APPS_PARSERTOOL (fpsdk_apps/parsertool/parsertool.cpp).
 *
 * @section FPSDK_COMMON_PARSER_NAMING Protocol and message naming
 *
 * The protocols names are defined in #fpsdk::common::parser::Protocol and can be stringified using
 * fpsdk::common::parser::ProtocolStr(). The names must match `/^[A-Z][A-Z0-9_]{2,5}$/.
 *
 * The message naming scheme consists of words separated by dashes. The first word is always the protocol name.
 * Depending on the protocol one or two more words are added. All words must match `/^[A-Z][A-Z0-9]{2,9}$/`.
 *
 * Examples:
 *
 * - **FP_A**-NAME, e.g. `FP_A-ODOMETRY`, `FP_A-RAWIMU`. See fpsdk::common::parser::nmea::NmeaGetMessageName() for
 * details.
 * - **FP_B**-NAME, e.g. `FP_B-SYSTEMSTATUS`, `FP_B-GNSSSTATUS`. See fpsdk::common::parser::fpb::FpbGetMessageName() for
 * details.
 * - **NMEA**-TALKER-FORMATTER, e.g. `NMEA-GN-GGA`, `NMEA-GN-RMC`. See fpsdk::common::parser::nmea::NmeaGetMessageName()
 * for details.
 * - **UBX**-CLASS-MESSAGE, e.g. `UBX-NAV-PVT`, `UBX-RXM-RAWX`. See fpsdk::common::parser::ubx::UbxGetMessageName() for
 * details.
 * - **RTCM3**-TYPENNNN, e.g. `RTCM3-TYPE1234`. See fpsdk::common::parser::rtcm3::Rtcm3GetMessageName() for details.
 * - **UNI_B**-NAME, .e.g. `UNI_B-VERSION`, `UNI_B-BESTNAV`. See fpsdk::common::parser::unib::UnibGetMessageName() for
 * details.
 * - **NOV_B**-NAME, .e.g. `NOV_B-BESTGNSSPOS`, `NOV_B-RAWDMI`. See fpsdk::common::parser::novb::NovbGetMessageName()
 * for details.
 * - **SPARTN**-TYPE-SUBTYPE, e.g. `SPARTN-OCB-GPS`. See fpsdk::common::parser::spartn::SpartnGetMessageName() for
 * details.
 * - **OTHER** (there are no individual messages in this "protocol")
 *
 * @section FPSDK_COMMON_PARSER_DESIGN Design
 *
 * @image html parser.drawio.svg ""
 *
 * The parser implementation does not use any STL containers, templates or other c++ conveniences. Instead, low-level
 * c-like types are used, in order to prevent any expensive heap (re)allocation and too much copying of data. For
 * example, the returned ParserMsg contains a pointer to message rather than a std::vector<uint8_t>, and a const
 * char* message name instead of a std::string. Applications can convert (copy) these easily to std::vector resp.
 * std::string for further processing. Some of the utilities, such as fpsdk::common::parser::ubx::UbxMakeMessage(), do
 * use STL containers for convenience.
 *
 * The maximum message size for each protocol is limited. For example, while the FP_B frame meta data would allow for
 * messages up to 65'536 bytes, the parser discards any message larger than #fpsdk::common::parser::MAX_FP_B_SIZE.
 * Messages larger than this are not practical and they do not (should not) exist. For example, it would take 5.6
 * seconds for such a message to transfer over a serial port at baudrate 115'200.
 *
 */
#ifndef __FPSDK_COMMON_PARSER_HPP__
#define __FPSDK_COMMON_PARSER_HPP__

/* LIBC/STL */
#include <cstdint>
#include <vector>

/* EXTERNAL */

/* PACKAGE */
#include "parser/types.hpp"

namespace fpsdk {
namespace common {
/**
 * @brief Parser
 */
namespace parser {
/* ****************************************************************************************************************** */

/**
 * @brief Message parser class
 */
class Parser
{
   public:
    //
    // ----- Constructor -----------------------------------------------------------------------------------------------

    /**
     * @brief Constructor, initialises parser and makes it ready to accept data and emit messages
     */
    Parser();

    //
    // ----- Input data ------------------------------------------------------------------------------------------------

    /**
     * @brief Add data to parser
     *
     * @param[in]  data  Pointer to data
     * @param[in]  size  Size of data (should be <= #MAX_ADD_SIZE)
     *
     * @returns true if data was added to the parser (or data was empty, that is, data != NULL and size = 0),
     *          false if there was not enough space left
     */
    bool Add(const uint8_t* data, const std::size_t size);

    /**
     * @brief Add data to parser
     *
     * @param[in]  data  data, can be empty (should be <= #MAX_ADD_SIZE)
     *
     * @returns true if data was added to the parser (or data was empty), false if there was not enough space left
     */
    bool Add(const std::vector<uint8_t>& data);

    /**
     * @brief Reset parser
     *
     * Resets the parser state and discards all collected data.
     */
    void Reset();

    //
    // ----- Process data (run parser) ---------------------------------------------------------------------------------

    /**
     * @brief Process data in parser, return message
     *
     * @param[out]  msg  The detected message frame
     *
     * @returns true if a message was detected, false otherwise (meaning: not enough data in parser)
     */
    bool Process(ParserMsg& msg);

    /**
     * @brief Get remaining data from parser as OTHER message(s)
     *
     * @param[out]  msg  A chunk of the remaining data as a OTHER message
     *
     * @returns true if there was remaining data, false if parser buffer was empty
     */
    bool Flush(ParserMsg& msg);

    //
    // ----- Utilities -------------------------------------------------------------------------------------------------

    /**
     * @brief Get parser statistics
     *
     * @returns a copy of the parser statistics
     */
    ParserStats GetStats() const;

    //
    // ----- Private ---------------------------------------------------------------------------------------------------
   private:
    // Parser state
    // clang-format off
    uint8_t     buf_[MAX_ADD_SIZE + (2 * MAX_ANY_SIZE)];  //!< Parser buffer
    std::size_t size_;   //!< Buffer size (size of data that is available for processing)
    std::size_t offs_;   //!< Buffer offset (points to start of data to be processed)
    ParserStats stats_;  //!< Statistics
    // clang-format on

    void EmitMessage(ParserMsg&, const std::size_t, const Protocol);  //!< Helper to emit a normal message
    void EmitGarbage(ParserMsg&);                                     //!< Helper to emit a OTHER message
};

/* ****************************************************************************************************************** */
}  // namespace parser
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_PARSER_HPP__
