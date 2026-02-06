#pragma once

#include "state_estimation/StateEstimatorBase.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

namespace dog_controllers
{
    class TopicEstimator final : public StateEstimatorBase
    {
    public:
        TopicEstimator(const LegData *legsPtr_,
                       const ImuData *imuPtr_,
                       PinocchioInterface pinocchioInterface,
                       CentroidalModelInfo info,
                       const PinocchioEndEffectorKinematics &eeKinematics,
                       rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        ~TopicEstimator() = default;

        const vector_t &estimate() override final;

    private:
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

        std::mutex mutex_;
        nav_msgs::msg::Odometry::SharedPtr lastOdomPtr_;
        bool msgReceived_ = false;

        nav_msgs::msg::Odometry odomSnapshot_;
    };
}