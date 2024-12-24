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
 * @brief Fixposition SDK: common app utils
 */
#ifndef __FPSDK_APPS_PCOMMOM_HPP__
#define __FPSDK_APPS_PCOMMOM_HPP__

/* LIBC/STL */
#include <cstdint>
#include <string>
#include <vector>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/parser/types.hpp>

/* PACKAGE */

namespace fpsdk {
namespace apps {
/**
 * @brief common apps utils
 */
namespace common {
/* ****************************************************************************************************************** */

/**
 * @brief Print header for PrintMessageData() to stdout
 */
void PrintMessageHeader();

/**
 * @brief Print message summary to stdout
 *
 * @param[in]  msg      The message
 * @param[in]  offs     Offset [bytes] of message in data (stream, file)
 * @param[in]  hexdump  Print also hexdump
 */
void PrintMessageData(const ::fpsdk::common::parser::ParserMsg& msg, const std::size_t offs, const bool hexdump);

/**
 * @brief Print parser stats to stdout
 *
 * @param[in]  stats  The stats
 */
void PrintParserStats(const ::fpsdk::common::parser::ParserStats& stats);

/* ****************************************************************************************************************** */
}  // namespace common
}  // namespace apps
}  // namespace fpsdk
#endif  // __FPSDK_APPS_PCOMMOM_HPP__
