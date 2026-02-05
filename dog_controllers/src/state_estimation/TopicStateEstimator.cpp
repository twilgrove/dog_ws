#include "state_estimation/TopicStateEstimator.hpp"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace dog_controllers
{
    TopicEstimator::TopicEstimator(const DogDataBridge *bridge,
                                   PinocchioInterface pinocchioInterface,
                                   CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics &eeKinematics,
                                   rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimatorBase(bridge, std::move(pinocchioInterface), std::move(info), eeKinematics, node)
    {
        sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth", rclcpp::SensorDataQoS().keep_last(1),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            { this->odomCallback(msg); });
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

        const quaternion_t quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        // 使用 Eigen 原生转换确保兼容性，(2,1,0) 对应 ZYX
        const vector3_t euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);

        results.rbdState_36.head<3>() = euler;
        results.rbdState_36.segment<3>(3) << pose.position.x, pose.position.y, pose.position.z;

        matrix3_t R_world_base = ocs2::getRotationMatrixFromZyxEulerAngles<scalar_t>(euler);

        results.rbdState_36.segment<3>(18) << twist.angular.x, twist.angular.y, twist.angular.z;
        results.rbdState_36.segment<3>(21).noalias() = R_world_base * vector3_t(twist.linear.x, twist.linear.y, twist.linear.z);

        updateGenericResults();

        return results.rbdState_36;
    }

}