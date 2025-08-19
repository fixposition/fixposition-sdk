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

#ifndef __FPSDK_RUN_HPP__
#define __FPSDK_RUN_HPP__

namespace fpsdk {
/* ****************************************************************************************************************** */
// clang-format off

/*!

    @page FPSDK_RUN_DOC Fixposition SDK: Running pre-built apps

    @section FPSDK_RUN_DOC_OVERVIEW Overview

    Docker images with pre-built binaries of the @ref FPSDK_APPS_DOC are provided along with a stand-alone script to
    run those apps easily on a Linux machine (requires Docker).

    No @ref FPSDK_BUILD_DOC is required.

    @section FPSDK_RUN_DOC_SETUP Setup

    Download the `fpsdk.sh` script and make it executable:

        curl -o fpsdk.sh https://raw.githubusercontent.com/fixposition/fixposition-sdk/refs/heads/main/fpsdk.sh
        chmod +x fpsdk.sh

    To run the apps the script automatically pulls the Docker image with the pre-built binaries on first run from the
    package registry. The `-u` flag can be used to pull the image again in order to update it. Images for for ROS1, ROS2
    and "no ROS" are provided. Note that downloading the image may take some time.

        ./fpsdk.sh fpltool -V

    This should download the image and then execute `fpltool -V`, which should print the version information.


    @section FPSDK_RUN_HELP Command line help

    @include fpsdk_helpscreen.txt

*/

// clang-format on
/* ****************************************************************************************************************** */
}  // namespace fpsdk
#endif  // __FPSDK_RUN_HPP__
