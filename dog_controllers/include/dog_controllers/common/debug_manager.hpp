#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <mutex>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_mpc/CommandData.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include "dog_bringup/msg/dog_state.hpp"
#include "common/dog_data_bridge.hpp"
#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{
    using namespace ocs2;
    using namespace ocs2::legged_robot;
    class DebugManager final
    {
    public:
        DebugManager(const ImuData *imuPtr_,
                     const EstimatorResults *results_,
                     const PinocchioInterface &pinocchioInterface,
                     const CentroidalModelInfo &info,
                     const PinocchioEndEffectorKinematics &eeKinematics,
                     rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        void update_debug(const SystemObservation &observation,
                          const PrimalSolution &primalSolution = PrimalSolution(),
                          const CommandData &command = CommandData());
        void publishStateData();
        void publishTF();
        void publishObservation(const SystemObservation &observation);

    private:
        const ImuData *imuPtr_;
        const EstimatorResults *results_;

        scalar_t threshold = 0.01;
        scalar_t lastTime_{};
        scalar_t delt_Time_{};

        std::shared_ptr<rclcpp::Node> debugNode_;
        std::unique_ptr<ocs2::legged_robot::LeggedRobotVisualizer> visualizerPtr_;

        dog_bringup::msg::DogState stateMsg_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPub_;
        rclcpp::Publisher<dog_bringup::msg::DogState>::SharedPtr statePub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };
}
