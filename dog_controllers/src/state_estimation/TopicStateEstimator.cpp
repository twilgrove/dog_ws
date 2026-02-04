#include "topic_estimator.hpp"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace dog_controllers
{
    TopicEstimator::TopicEstimator(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {
        // 订阅真值话题，使用 SensorDataQoS 减少延迟
        sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth", rclcpp::SensorDataQoS(),
            std::bind(&TopicEstimator::odomCallback, this, std::placeholders::_1));
    }

    void TopicEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastOdomMsg_ = *msg;
        msgReceived_ = true;
    }

    const vector_t &TopicEstimator::estimate()
    {
        update();
        return results.rbdState_36;
    }

    void TopicEstimator::update()
    {
        if (!msgReceived_ || bridgePtr_ == nullptr)
            return;

        // 1. 局部快照：为了缩短锁的持有时间，先拷贝出来立刻解锁
        nav_msgs::msg::Odometry odom;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            odom = lastOdomMsg_;
        }

        // --- 开始计算 (在锁外进行，不阻塞回调线程) ---

        // 2. 时间戳同步
        results.time = rclcpp::Time(odom.header.stamp).seconds();

        // 3. 姿态处理 (0-2)
        const auto &q_msg = odom.pose.pose.orientation;
        const quaternion_t quat(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        const vector3_t euler = ocs2::quatToZyx(quat); // ZYX顺序: [Yaw, Pitch, Roll]
        results.rbdState_36.head<3>() = euler;

        // 4. 位置处理 (3-5)
        results.rbdState_36.segment<3>(3) << odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z;

        // 5. 关节角度 (6-17)
        results.rbdState_36.segment<12>(6) = bridgePtr_->joint_positions;

        // 6. 坐标系转换：重点！
        // Gazebo P3D 的 twist 通常在 base_link 坐标系下。
        // OCS2 要求的线速度通常是在世界系对齐坐标系（World-aligned）下。
        matrix3_t R_world_base = ocs2::getRotationMatrixFromZyxEulerAngles(euler);

        vector3_t v_base(odom.twist.twist.linear.x,
                         odom.twist.twist.linear.y,
                         odom.twist.twist.linear.z);
        vector3_t w_base(odom.twist.twist.angular.x,
                         odom.twist.twist.angular.y,
                         odom.twist.twist.angular.z);

        // 角速度 (18-20): 填入机体系角速度
        results.rbdState_36.segment<3>(18) = w_base;

        // 线速度 (21-23): 将机体系速度旋转到世界系
        results.rbdState_36.segment<3>(21) = R_world_base * v_base;

        // 7. 12个关节速度 (24-35)
        results.rbdState_36.segment<12>(24) = bridgePtr_->joint_velocities;

        // 8. 触地状态与模式映射
        results.contactFlags_WBC = bridgePtr_->contact_flags;
        results.contactFlags_MPC = computeMpcMode(results.contactFlags_WBC);
    }
}