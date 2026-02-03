#pragma once

#include "state_estimation/StateEstimatorBase.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

namespace dog_controllers
{
    class TopicEstimator : public StateEstimatorBase
    {
    public:
        // 构造函数需要传入节点指针来创建订阅者
        TopicEstimator(rclcpp::Node::SharedPtr node);
        virtual ~TopicEstimator() = default;

        // 实现基类的虚函数
        const vector_t &estimate() override;

        // 子类特有的更新函数，用于处理逻辑
        void update();

    private:
        // Odom 话题的回调函数
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // 辅助函数：将布尔数组转为 MPC Mode
        size_t computeMpcMode(const std::array<bool, 4> &contactFlags);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

        // 缓存最新的话题数据，用于线程安全
        nav_msgs::msg::Odometry lastOdomMsg_;
        std::mutex mutex_;
        bool msgReceived_ = false;
    };
}