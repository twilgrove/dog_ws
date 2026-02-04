#pragma once

#include "state_estimation/StateEstimatorBase.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

namespace dog_controllers
{
    class TopicEstimator : public StateEstimatorBase
    {
    public:
        TopicEstimator(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
        virtual ~TopicEstimator() = default;

        // 覆盖基类接口
        const vector_t &estimate() override;

        // 核心计算逻辑
        void update();

    private:
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        inline size_t computeMpcMode(const std::array<bool, 4> &contactFlags)
        {
            return (contactFlags[0] ? 1 : 0) |
                   (contactFlags[1] ? 2 : 0) |
                   (contactFlags[2] ? 4 : 0) |
                   (contactFlags[3] ? 8 : 0);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

        // 线程安全相关
        nav_msgs::msg::Odometry lastOdomMsg_;
        std::mutex mutex_;
        bool msgReceived_ = false;
    };
}