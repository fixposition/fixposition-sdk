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
#include <cinttypes>
#include <functional>
#include <memory>

/* EXTERNAL */

/* Fixposition SDK */

/* PACKAGE */
#include "ros2_fpsdk_demo/node.hpp"

namespace Ros2FpsdkDemo {
/* ****************************************************************************************************************** */

DemoParams::DemoParams() /* clang-format off */ :
    worker1_interval_  { 1.0 },
    worker2_interval_  { 1.0 },
    timer1_interval_   { 1.0 },
    timer2_interval_   { 1.0 }  // clang-format on
{
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoParams::LoadFromRos(std::shared_ptr<rclcpp::Node> node)
{
    bool ok = true;

    // Declare used params, use as default whatever we the object has currently
    const std::string WORKER1_INTERVAL_NAME = "worker1_interval";
    const std::string WORKER2_INTERVAL_NAME = "worker2_interval";
    const std::string TIMER1_INTERVAL_NAME = "timer1_interval";
    const std::string TIMER2_INTERVAL_NAME = "timer2_interval";
    node->declare_parameter(WORKER1_INTERVAL_NAME, worker1_interval_);
    node->declare_parameter(WORKER2_INTERVAL_NAME, worker2_interval_);
    node->declare_parameter(TIMER1_INTERVAL_NAME, timer1_interval_);
    node->declare_parameter(TIMER2_INTERVAL_NAME, timer2_interval_);

    // Load
    if (!node->get_parameter(WORKER1_INTERVAL_NAME, worker1_interval_)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: worker1_interval param missing");
        ok = false;
    }
    if (!node->get_parameter(WORKER2_INTERVAL_NAME, worker2_interval_)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: worker2_interval param missing");
        ok = false;
    }
    if (!node->get_parameter(TIMER1_INTERVAL_NAME, timer1_interval_)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: timer1_interval param missing");
        ok = false;
    }
    if (!node->get_parameter(TIMER2_INTERVAL_NAME, timer2_interval_)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: timer2_interval param missing");
        ok = false;
    }

    // Check
    if ((worker1_interval_ < INTERVAL_MIN) || (worker1_interval_ > INTERVAL_MAX)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: Bad value for worker1_interval_ param");
        ok = false;
    }
    if ((worker2_interval_ < INTERVAL_MIN) || (worker2_interval_ > INTERVAL_MAX)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: Bad value for worker2_interval param");
        ok = false;
    }
    if ((timer1_interval_ < INTERVAL_MIN) || (timer1_interval_ > INTERVAL_MAX)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: Bad value for timer1_interval param");
        ok = false;
    }
    if ((timer2_interval_ < INTERVAL_MIN) || (timer2_interval_ > INTERVAL_MAX)) {
        RCLCPP_WARN(node->get_logger(), "DemoParams: Bad value for timer2_interval param");
        ok = false;
    }

    RCLCPP_INFO(node->get_logger(),
        "DemoParams: worker1_interval=%.1f worker2_interval=%.1f timer1_interval=%.1f timer2_interval=%.1f",
        worker1_interval_, worker2_interval_, timer1_interval_, timer2_interval_);

    return ok;
}

/* ****************************************************************************************************************** */

DemoNode::DemoNode() /* clang-format off */ :
    Node("demo_node"),
    worker1_  { "worker1", std::bind(&DemoNode::Worker1, this, std::placeholders::_1) },
    worker2_  { "worker2", std::bind(&DemoNode::Worker2, this, std::placeholders::_1) }  // clang-format on
{
    RCLCPP_DEBUG(this->get_logger(), "i am debug");
    RCLCPP_INFO(this->get_logger(), "i am info");
    RCLCPP_WARN(this->get_logger(), "i am warn");
    RCLCPP_ERROR(this->get_logger(), "i am error");
    RCLCPP_FATAL(this->get_logger(), "i am fatal");
}

// ---------------------------------------------------------------------------------------------------------------------

DemoNode::~DemoNode()
{
    Stop();
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoNode::LoadParams()
{
    return params_.LoadFromRos(shared_from_this());
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoNode::Start()
{
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Start() %s %s", get_name(), get_namespace());
    timer1_ = this->create_wall_timer(
        rclcpp::Duration::from_seconds(params_.timer1_interval_).to_chrono<std::chrono::milliseconds>(),
        std::bind(&DemoNode::Timer1, this));
    timer2_ = this->create_wall_timer(
        rclcpp::Duration::from_seconds(params_.timer2_interval_).to_chrono<std::chrono::milliseconds>(),
        std::bind(&DemoNode::Timer2, this));

    const std::string topic_name = std::string(get_namespace()) + std::string(get_name()) + "/string";
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 5);

    return worker1_.Start() && worker2_.Start();
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Stop()
{
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Stop()");
    // timer1_.stop();
    // timer2_.stop();
    worker1_.Stop();
    worker2_.Stop();
    // publisher_.shutdown();
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Worker1(void* /*arg*/)
{
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Worker1() start 0x%" PRIxMAX, fp::common::thread::ThisThreadId());
    while (!worker1_.ShouldAbort()) {
        RCLCPP_DEBUG(this->get_logger(), "DemoNode::Worker1() ...");
        auto msg = std_msgs::msg::String();
        msg.data = "worker1...";
        publisher_->publish(msg);
        rclcpp::sleep_for(
            rclcpp::Duration::from_seconds(params_.worker1_interval_).to_chrono<std::chrono::milliseconds>());
    }
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Worker1() done");
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Worker2(void* /*arg*/)
{
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Worker2() start 0x%" PRIxMAX, fp::common::thread::ThisThreadId());
    while (!worker2_.ShouldAbort()) {
        RCLCPP_DEBUG(this->get_logger(), "DemoNode::Worker2() ...");
        auto msg = std_msgs::msg::String();
        msg.data = "worker2...";
        publisher_->publish(msg);
        rclcpp::sleep_for(
            rclcpp::Duration::from_seconds(params_.worker2_interval_).to_chrono<std::chrono::milliseconds>());
    }
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Worker() done");
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Timer1()
{
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Timer1() 0x%" PRIxMAX, fp::common::thread::ThisThreadId());
    auto msg = std_msgs::msg::String();
    msg.data = "timer1...";
    publisher_->publish(msg);
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Timer2()
{
    RCLCPP_DEBUG(this->get_logger(), "DemoNode::Timer2() 0x%" PRIxMAX, fp::common::thread::ThisThreadId());
    auto msg = std_msgs::msg::String();
    msg.data = "timer2...";
    publisher_->publish(msg);
}

/* ****************************************************************************************************************** */
}  // namespace Ros2FpsdkDemo
