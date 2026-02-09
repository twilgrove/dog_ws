#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <mutex>
#include <memory>

#include "dog_bringup/msg/dog_state.hpp"
#include "common/dog_data_bridge.hpp"
#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{
    class DebugManager final
    {
    public:
        DebugManager(const ImuData *imuPtr_,
                     const EstimatorResults *results_,
                     rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        void publish();

    private:
        const ImuData *imuPtr_;
        const EstimatorResults *results_;

        int threshold = 10;
        int count = 0;
        std::chrono::steady_clock::time_point last_loop_time_;

        dog_bringup::msg::DogState stateMsg_;
        rclcpp::Publisher<dog_bringup::msg::DogState>::SharedPtr statePub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };
}
