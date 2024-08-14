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
 * @brief Fixposition SDK: ROS2 demo node
 */

/* LIBC/STL */
#include <chrono>
#include <future>
#include <memory>

/* EXTERNAL */
#include <rclcpp/rclcpp.hpp>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/thread.hpp>
#include <fpsdk_ros2/utils.hpp>

/* PACKAGE */
#include "ros2_fpsdk_demo/node.hpp"

/* ****************************************************************************************************************** */

int main(int argc, char* argv[])
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
#endif
    auto logger = rclcpp::get_logger("node_main");
    RCLCPP_INFO(logger, "main() 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());

    bool ok = true;

    // Initialise
    rclcpp::init(argc, argv);

    // Redirect Fixposition SDK logging to ROS console
    fpsdk::ros2::utils::RedirectLoggingToRosConsole();
    DEBUG("This is a message from fpsdk_common's logging, redirected to the ROS console");  // logger: "fpsdk_ros2"
    RCLCPP_DEBUG(logger, "This is a proper ROS console message");                           // logger: "node_main"

    // Handle CTRL-C / SIGINT ourselves
    fpsdk::common::app::SigIntHelper sigint;

    // Create node. This doesn't do much yet.
    auto node = std::make_shared<Ros2FpsdkDemo::DemoNode>();

    // Load params
    if (node->LoadParams()) {
        // Start node
        if (node->Start()) {
            RCLCPP_INFO(logger, "main() spinning...");
#if 1
            // Do the same as rclpp::spin(), but also handle CTRL-C / SIGINT nicely
            // Callbacks execute in main thread
            while (rclcpp::ok() && !sigint.ShouldAbort()) {
                rclcpp::spin_until_future_complete(
                    node, std::promise<bool>().get_future(), std::chrono::milliseconds(345));
            }
#else
            // Use multiple spinner threads. Callback execute in one of them.
            // TODO: this doesn't seem to work
            rclcpp::executors::MultiThreadedExecutor executor{ rclcpp::ExecutorOptions(), 4 };
            executor.add_node(node);
            while (rclcpp::ok() && !sigint.ShouldAbort()) {
                executor.spin_once(std::chrono::milliseconds(345));
            }
#endif
            RCLCPP_INFO(logger, "main() stopping...");
        } else {
            RCLCPP_ERROR(logger, "Failed starting node");
            ok = false;
        }
    } else {
        RCLCPP_ERROR(logger, "Failed loading parameters");
        ok = false;
    }

    node->Stop();

    // Are we happy?
    if (ok) {
        RCLCPP_INFO(logger, "Done");
    } else {
        RCLCPP_ERROR(logger, "Ouch!");
    }

    // As a very last thing, shutdown ROS
    rclcpp::shutdown();

    exit(ok ? EXIT_SUCCESS : EXIT_FAILURE);
}

/* ****************************************************************************************************************** */
