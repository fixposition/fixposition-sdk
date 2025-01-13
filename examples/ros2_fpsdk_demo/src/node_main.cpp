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

using namespace Ros2FpsdkDemo;

int main(int argc, char* argv[])
{
#ifndef NDEBUG
    fpsdk::common::app::StacktraceHelper stacktrace;
    WARNING("***** Running debug build *****");
#endif
    INFO("main() 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());

    bool ok = true;

    // Initialise ROS, create node handle
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("ros2_fpsdk_demo");
    auto logger = nh->get_logger();

    // Redirect Fixposition SDK logging to ROS console, all should use the "ros2_fpsdk_demo" logger
    fpsdk::ros2::utils::RedirectLoggingToRosConsole(logger.get_name());
    DEBUG("This is a message from fpsdk_common's logging, redirected to the ROS console");
    RCLCPP_DEBUG(logger, "This is a proper ROS console message");

    // Load parameters
    RCLCPP_INFO(logger, "Loading parameters...");
    DemoParams params;
    if (!params.LoadFromRos(nh)) {
        RCLCPP_ERROR(logger, "Failed loading parameters");
        ok = false;
    }

    // Handle CTRL-C / SIGINT ourselves
    fpsdk::common::app::SigIntHelper sigint;

    // Start node
    std::unique_ptr<DemoNode> node;
    if (ok) {
        try {
            node = std::make_unique<DemoNode>(nh, params);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR(logger, "Failed creating node: %s", ex.what());
            ok = false;
        }
    }
    if (ok) {
        RCLCPP_INFO(logger, "Starting node...");
        if (node->Start()) {
            RCLCPP_INFO(logger, "main() spinning...");
#if 1
            // Do the same as rclpp::spin(nh), but also handle CTRL-C / SIGINT nicely
            // Callbacks execute in main thread
            while (rclcpp::ok() && !sigint.ShouldAbort()) {
                rclcpp::spin_until_future_complete(
                    nh, std::promise<bool>().get_future(), std::chrono::milliseconds(345));
            }
#else

            // Use multiple spinner threads. Callback execute in one of them.
            // TODO: this (executing in those threads) doesn't seem to work
            rclcpp::executors::MultiThreadedExecutor executor{ rclcpp::ExecutorOptions(), 4 };
            executor.add_node(nh);
            while (rclcpp::ok() && !sigint.ShouldAbort()) {
                executor.spin_once(std::chrono::milliseconds(345));
            }
#endif
            RCLCPP_INFO(logger, "main() stopping...");
        } else {
            RCLCPP_ERROR(logger, "Failed starting node");
            ok = false;
        }
        node->Stop();
        node.reset();
        nh.reset();
    }

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
