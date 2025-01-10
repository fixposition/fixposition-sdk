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
 * @brief Fixposition SDK: ROS1 demo node
 */

/* LIBC/STL */
#include <cinttypes>
#include <functional>

/* EXTERNAL */
#include <fpsdk_ros1/ext/ros.hpp>
#include <fpsdk_ros1/ext/ros_callback_queue.hpp>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/thread.hpp>
#include <fpsdk_ros1/utils.hpp>

/* PACKAGE */
#include "ros1_fpsdk_demo/node.hpp"

/* ****************************************************************************************************************** */

int main(int argc, char** argv)
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
    WARNING("***** Running debug build *****");
#endif
    ROS_INFO("main() 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());

    bool ok = true;

    // Initialise
    ros::init(argc, argv, "ros1_fpsdk_demo_node");
    ros::NodeHandle nh("~");

    // Redirect Fixposition SDK logging to ROS console, all should use the "ros.ros1_fpsdk_demo" logger
    fpsdk::ros1::utils::RedirectLoggingToRosConsole();
    DEBUG("This is a message from fpsdk_common's logging, redirected to the ROS console");
    ROS_DEBUG("This is a proper ROS console message");

    // Handle CTRL-C / SIGINT ourselves
    fpsdk::common::app::SigIntHelper sigint;

    // Load params
    Ros1FpsdkDemo::DemoParams params;
    if (!params.LoadFromRos("~")) {
        ROS_ERROR("Failed loading params");
        ok = false;
    }

    // Start node
    if (ok) {
        Ros1FpsdkDemo::DemoNode node(params, nh);
        if (node.Start()) {
            ROS_INFO("main() spinning...");

#if 1
            // Do the same as ros::spin(), but also handle CTRL-C / SIGINT nicely
            // Callbacks execute in main thread
            auto cbq = ros::getGlobalCallbackQueue();
            while (ros::ok() && !sigint.ShouldAbort()) {
                cbq->callAvailable(ros::WallDuration(0.25));
            }
#else
            // Use multiple spinner threads. Callback execute in one of them.
            ros::AsyncSpinner spinner{ 4 };
            spinner.start();
            sigint.WaitAbort();
            spinner.stop();
#endif

            ROS_INFO("main() stopping");

        } else {
            ROS_ERROR("Failed starting node");
            ok = false;
        }

        // Cancel any running ros::Duration::sleep() (e.g. in DemoNode::Worker())
        // @todo Why does this not work?
        ros::Time::shutdown();

        // Stop node
        node.Stop();
    }

    // Are we happy?
    if (ok) {
        ROS_INFO("Done");
    } else {
        ROS_FATAL("Ouch!");
    }

    // As a very last thing, shutdown ROS
    ros::shutdown();

    exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}

/* ****************************************************************************************************************** */
