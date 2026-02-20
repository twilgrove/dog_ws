#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <mutex>

namespace dog_controllers
{
  using namespace ocs2;

  class TargetTrajectoriesPublisher : public rclcpp::Node
  {
  public:
    explicit TargetTrajectoriesPublisher(const rclcpp::NodeOptions &options);

  private:
    TargetTrajectories goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation);
    TargetTrajectories cmdVelToTargetTrajectories(const vector_t &cmdVel, const SystemObservation &observation);
    TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose, const SystemObservation &observation, const scalar_t &targetReachingTime);
    scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement);

    rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;

    std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    std::mutex latestObservationMutex_;
    SystemObservation latestObservation_;

    scalar_t TARGET_DISPLACEMENT_VELOCITY_;
    scalar_t TARGET_ROTATION_VELOCITY_;
    scalar_t COM_HEIGHT_;
    vector_t DEFAULT_JOINT_STATE_;
    scalar_t TIME_TO_TARGET_;
  };

}