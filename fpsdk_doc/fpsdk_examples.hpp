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
 * @brief Fixposition SDK: Documentation
 */

#ifndef __FPSDK_EXAMPLES_HPP__
#define __FPSDK_EXAMPLES_HPP__

namespace fpsdk {
/* ****************************************************************************************************************** */
// clang-format off

/*!
    @page FPSDK_EXAMPLES_DOC Fixposition SDK: Examples

    - @subpage FPSDK_EXAMPLES_PARSER_INTRO
    - @subpage FPSDK_EXAMPLES_FUSION_EPOCH
    - @subpage FPSDK_EXAMPLES_FPB_MEASUREMENTS
    - @subpage FPSDK_EXAMPLES_ROS1_DEMO
    - @subpage FPSDK_EXAMPLES_ROS2_DEMO


    @page FPSDK_EXAMPLES_PARSER_INTRO Parsing and decoding FP_A messages

    @include{lineno} parser_intro.cpp


    @page FPSDK_EXAMPLES_FUSION_EPOCH Collecting fusion epoch data from FP_A messages

    @include{lineno} fusion_epoch.cpp


    @page FPSDK_EXAMPLES_FPB_MEASUREMENTS Making a FP_B-MEASUREMENTS message

    @include{lineno} fpb_measurements.cpp


    @page FPSDK_EXAMPLES_ROS1_DEMO ROS 1 node

    This is a demonstratation of using the Fixposition SDK in a simple ROS 1 node. See the examples/ros1_fpsdk_demo
    directory in the source code repository for details.


    @page FPSDK_EXAMPLES_ROS2_DEMO ROS 2 node

    This is a demonstratation of using the Fixposition SDK in a simple ROS 2 node. See the examples/ros2_fpsdk_demo
    directory in the source code repository for details.
*/

// clang-format on
/* ****************************************************************************************************************** */
}  // namespace fpsdk
#endif  // __FPSDK_EXAMPLES_HPP__
