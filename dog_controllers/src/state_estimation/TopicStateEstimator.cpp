#include "state_estimation/TopicStateEstimator.hpp"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace dog_controllers
{
    TopicEstimator::TopicEstimator(const LegData *legsPtr_,
                                   const ImuData *imuPtr_,
                                   PinocchioInterface pinocchioInterface,
                                   CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics &eeKinematics,
                                   rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimatorBase(legsPtr_, imuPtr_, std::move(pinocchioInterface), std::move(info), eeKinematics, node)
    {
        sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth", rclcpp::SensorDataQoS().keep_last(1),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            { this->odomCallback(msg); });
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ TopicEstimator\033[0m");
    }

    void TopicEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastOdomPtr_ = msg;
        msgReceived_ = true;
    }

    const vector_t &TopicEstimator::estimate()
    {
        if (__builtin_expect(!msgReceived_, 0))
            return results.rbdState_36;

        nav_msgs::msg::Odometry::SharedPtr currentMsg;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            currentMsg = lastOdomPtr_;
        }

        const auto &pose = currentMsg->pose.pose;
        const auto &twist = currentMsg->twist.twist;

        results.rbdState_36.segment<3>(21) << twist.linear.x, twist.linear.y, twist.linear.z;
        results.rbdState_36.segment<3>(3) << pose.position.x, pose.position.y, pose.position.z;
        results.rbdState_36.segment<3>(18) << twist.angular.x, twist.angular.y, twist.angular.z;
        updateGenericResults(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

        return results.rbdState_36;
    }

}