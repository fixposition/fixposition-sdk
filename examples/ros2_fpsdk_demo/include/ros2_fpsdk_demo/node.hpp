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
#ifndef __ROS2_FPSDK_DEMO_NODE_HPP__
#define __ROS2_FPSDK_DEMO_NODE_HPP__

/* LIBC/STL */
#include <memory>

/* EXTERNAL */
#include <fpsdk_ros2/ext/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/* Fixposition SDK */
#include <fpsdk_common/thread.hpp>

/* PACKAGE */

namespace Ros2FpsdkDemo {
/* ****************************************************************************************************************** */

class DemoParams
{
   public:
    DemoParams();
    bool LoadFromRos(std::shared_ptr<rclcpp::Node> nh, const std::string& ns = "");

    double worker1_interval_;
    double worker2_interval_;
    double timer1_interval_;
    double timer2_interval_;

    static constexpr double INTERVAL_MIN = 0.5;
    static constexpr double INTERVAL_MAX = 10.0;
};

class DemoNode
{
   public:
    DemoNode(std::shared_ptr<rclcpp::Node> nh, const DemoParams& params);
    ~DemoNode();
    bool Start();
    void Stop();

   private:
    std::shared_ptr<rclcpp::Node> nh_;
    DemoParams params_;
    rclcpp::Logger logger_;
    fpsdk::common::thread::Thread worker1_;
    fpsdk::common::thread::Thread worker2_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    bool Worker1();
    bool Worker2();
    void Timer1();
    void Timer2();
};

/* ****************************************************************************************************************** */
}  // namespace Ros2FpsdkDemo
#endif  // __ROS2_FPSDK_DEMO_NODE_HPP__
