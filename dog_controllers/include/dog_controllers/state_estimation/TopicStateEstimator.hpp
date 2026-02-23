#pragma once

#include "state_estimation/StateEstimatorBase.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

namespace dog_controllers
{
    class TopicEstimator final : public StateEstimatorBase
    {
    public:
        TopicEstimator(
            const PinocchioInterface &pinocchioInterface,
            const CentroidalModelInfo &info,
            const PinocchioEndEffectorKinematics &eeKinematics,
            rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        ~TopicEstimator() override = default;

        const vector_t &estimate(const std::array<LegData, 4> &legsPtr, const ImuData & /*imuData*/, const rclcpp::Duration &period) override final;

    private:
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

        std::mutex mutex_;
        nav_msgs::msg::Odometry::SharedPtr lastOdomPtr_;
        bool msgReceived_ = false;

        nav_msgs::msg::Odometry odomSnapshot_;
    };
}