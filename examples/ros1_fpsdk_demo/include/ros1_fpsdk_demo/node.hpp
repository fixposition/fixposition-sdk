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
#ifndef __ROS1_FPSDK_DEMO_NODE_HPP__
#define __ROS1_FPSDK_DEMO_NODE_HPP__

/* LIBC/STL */

/* EXTERNAL */
#include <fpsdk_ros1/ext/ros.hpp>

/* Fixposition SDK */
#include <fpsdk_common/thread.hpp>

/* PACKAGE */

namespace Ros1FpsdkDemo {
/* ****************************************************************************************************************** */

class DemoParams
{
   public:
    DemoParams();
    bool LoadFromRos(const std::string& ns);

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
    DemoNode(const DemoParams& params, ros::NodeHandle& nh);
    ~DemoNode();
    bool Start();
    void Stop();

   private:
    const DemoParams params_;
    ros::NodeHandle nh_;
    fpsdk::common::thread::Thread worker1_;
    fpsdk::common::thread::Thread worker2_;
    ros::Timer timer1_;
    ros::Timer timer2_;
    ros::Publisher publisher_;

    bool Worker1();
    bool Worker2();
    void Timer1(const ros::TimerEvent& event);
    void Timer2(const ros::TimerEvent& event);
};

/* ****************************************************************************************************************** */
}  // namespace Ros1FpsdkDemo
#endif  // __ROS1_FPSDK_DEMO_NODE_HPP__
