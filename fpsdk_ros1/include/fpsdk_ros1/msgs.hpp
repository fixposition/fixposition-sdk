/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: ROS1 messages
 *
 * @page FPSDK_ROS1_MSGS ROS1 messages
 *
 * **API**: fpsdk_ros1/msgs.hpp and fpsdk::ros1::msgs
 *
 */
#ifndef __FPSDK_ROS1_MSGS_HPP__
#define __FPSDK_ROS1_MSGS_HPP__

/* LIBC/STL */

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#include <fpsdk_ros1/ParserMsg.h>
#pragma GCC diagnostic pop

namespace fpsdk {
namespace ros1 {
/**
 * @brief ROS1 messages
 */
namespace msgs {
/* ****************************************************************************************************************** */

/**
 * @name ParserMsg
 *
 * @include{lineno} ParserMsg.msg
 *
 * @{
 */

//! ROS message (instance) similar to fpsdk::common::parser::ParserMsg
using ParserMsg = fpsdk_ros1::ParserMsg;

//! ROS message (shared pointer) similar to fpsdk::common::parser::ParserMsg
using ParserMsgPtr = fpsdk_ros1::ParserMsgPtr;

//! ROS message (shared const pointer) similar to fpsdk::common::parser::ParserMsg
using ParserMsgConstPtr = fpsdk_ros1::ParserMsgConstPtr;

///@}

/* ****************************************************************************************************************** */
}  // namespace msgs
}  // namespace ros1
}  // namespace fpsdk
#endif  // __FPSDK_ROS1_MSGS_HPP__
