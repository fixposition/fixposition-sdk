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
#include <fpsdk_ros1/ext/ros_msgs.hpp>

/* Fixposition SDK */
#include <fpsdk_common/app.hpp>
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/thread.hpp>
#include <fpsdk_ros1/utils.hpp>

/* PACKAGE */
#include "ros1_fpsdk_demo/node.hpp"

namespace Ros1FpsdkDemo {
/* ****************************************************************************************************************** */

DemoParams::DemoParams() /* clang-format off */ :
    worker1_interval_   { 1.0 },
    worker2_interval_   { 1.0 },
    timer1_interval_    { 1.0 },
    timer2_interval_    { 1.0 }  // clang-format on
{
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoParams::LoadFromRos(const std::string& ns)
{
    bool ok = true;

    if (!fpsdk::ros1::utils::LoadRosParam(ns + "/worker1_interval", worker1_interval_)) {
        ROS_WARN("DemoParams: worker1_interval param missing");
        ok = false;
    }
    if (!fpsdk::ros1::utils::LoadRosParam(ns + "/worker2_interval", worker2_interval_)) {
        ROS_WARN("DemoParams: worker2_interval param missing");
        ok = false;
    }
    if (!fpsdk::ros1::utils::LoadRosParam(ns + "/timer1_interval", timer1_interval_)) {
        ROS_WARN("DemoParams: timer1_interval param missing");
        ok = false;
    }
    if (!fpsdk::ros1::utils::LoadRosParam(ns + "/timer2_interval", timer2_interval_)) {
        ROS_WARN("DemoParams: timer2_interval param missing");
        ok = false;
    }

    if ((worker1_interval_ < INTERVAL_MIN) || (worker1_interval_ > INTERVAL_MAX)) {
        ROS_WARN("DemoParams: Bad value for worker1_interval_ param");
        ok = false;
    }
    if ((worker2_interval_ < INTERVAL_MIN) || (worker2_interval_ > INTERVAL_MAX)) {
        ROS_WARN("DemoParams: Bad value for worker2_interval param");
        ok = false;
    }
    if ((timer1_interval_ < INTERVAL_MIN) || (timer1_interval_ > INTERVAL_MAX)) {
        ROS_WARN("DemoParams: Bad value for timer1_interval param");
        ok = false;
    }
    if ((timer2_interval_ < INTERVAL_MIN) || (timer2_interval_ > INTERVAL_MAX)) {
        ROS_WARN("DemoParams: Bad value for timer2_interval param");
        ok = false;
    }

    ROS_INFO("DemoParams: worker1_interval=%.1f worker2_interval=%.1f timer1_interval=%.1f timer2_interval=%.1f",
        worker1_interval_, worker2_interval_, timer1_interval_, timer2_interval_);

    return ok;
}

/* ****************************************************************************************************************** */

DemoNode::DemoNode(const DemoParams& params, ros::NodeHandle& nh) /* clang-format off */ :
    params_   { params },
    nh_       { nh },
    worker1_  { "worker1", std::bind(&DemoNode::Worker1, this) },
    worker2_  { "worker2", std::bind(&DemoNode::Worker2, this) }  // clang-format on
{
    ROS_DEBUG("DemoNode()");
}

// ---------------------------------------------------------------------------------------------------------------------

DemoNode::~DemoNode()
{
    ROS_DEBUG("~DemoNode()");
    Stop();
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoNode::Start()
{
    ROS_DEBUG("DemoNode::Start()");
    timer1_ = nh_.createTimer(ros::Duration(params_.timer1_interval_), &DemoNode::Timer1, this);
    timer2_ = nh_.createTimer(ros::Duration(params_.timer2_interval_), &DemoNode::Timer2, this);
    publisher_ = nh_.advertise<std_msgs::String>("string", 5);
    return worker1_.Start() && worker2_.Start();
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Stop()
{
    ROS_DEBUG("DemoNode::Stop()");
    timer1_.stop();
    timer2_.stop();
    if (worker1_.GetStatus() == worker1_.Status::RUNNING) {
        worker1_.Stop();
    }
    if (worker2_.GetStatus() == worker2_.Status::RUNNING) {
        worker2_.Stop();
    }
    publisher_.shutdown();
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoNode::Worker1()
{
    ROS_DEBUG("DemoNode::Worker1() start 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());
    while (!worker1_.ShouldAbort()) {
        ROS_DEBUG("DemoNode::Worker1() ...");
        std_msgs::String msg;
        msg.data = "worker1...";
        publisher_.publish(msg);
        ros::Duration(params_.worker1_interval_).sleep();
    }
    ROS_DEBUG("DemoNode::Worker1() done");
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

bool DemoNode::Worker2()
{
    ROS_DEBUG("DemoNode::Worker2() start 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());
    while (!worker2_.ShouldAbort()) {
        ROS_DEBUG("DemoNode::Worker2() ...");
        std_msgs::String msg;
        msg.data = "worker2...";
        publisher_.publish(msg);
        ros::Duration(params_.worker2_interval_).sleep();
    }
    ROS_DEBUG("DemoNode::Worker() done");
    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Timer1(const ros::TimerEvent& /*event*/)
{
    ROS_DEBUG("DemoNode::Timer1() 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());
    std_msgs::String msg;
    msg.data = "timer1...";
    publisher_.publish(msg);
}

// ---------------------------------------------------------------------------------------------------------------------

void DemoNode::Timer2(const ros::TimerEvent& /*event*/)
{
    ROS_DEBUG("DemoNode::Timer2() 0x%" PRIxMAX, fpsdk::common::thread::ThisThreadId());
    std_msgs::String msg;
    msg.data = "timer2...";
    publisher_.publish(msg);
}

/* ****************************************************************************************************************** */
}  // namespace Ros1FpsdkDemo
