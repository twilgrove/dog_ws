#include "topic_estimator.hpp"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace dog_controllers
{
    TopicEstimator::TopicEstimator(rclcpp::Node::SharedPtr node)
    {
        // 订阅 Gazebo 的 P3D 插件或其它真值话题
        sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth", 10,
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
        // 更新并返回结果变量中的 36 维向量
        update();
        return results.rbdState_36;
    }

    void TopicEstimator::update()
    {
        if (!msgReceived_ || bridgePtr_ == nullptr)
            return;

        std::lock_guard<std::mutex> lock(mutex_);

        // 1. 时间戳
        results.time = rclcpp::Time(lastOdomMsg_.header.stamp).seconds();

        // --- 填充 36 维向量 ---

        // 2. 姿态 (0-2): 四元数转 ZYX 欧拉角 (Roll, Pitch, Yaw)
        auto &q_msg = lastOdomMsg_.pose.pose.orientation;
        const quaternion_t quat(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        results.rbdState_36.head<3>() = ocs2::quatToZyx(quat);

        // 3. 位置 (3-5): 世界系 XYZ
        results.rbdState_36.segment<3>(3) << lastOdomMsg_.pose.pose.position.x,
            lastOdomMsg_.pose.pose.position.y,
            lastOdomMsg_.pose.pose.position.z;

        // 4. 12个关节角度 (6-17): 从 Bridge 拿
        results.rbdState_36.segment<12>(6) = bridgePtr_->joint_positions;

        // 5. 角速度 (18-20): 这里假设话题给出的是机体系角速度
        results.rbdState_36.segment<3>(18) << lastOdomMsg_.twist.twist.angular.x,
            lastOdomMsg_.twist.twist.angular.y,
            lastOdomMsg_.twist.twist.angular.z;

        // 6. 线速度 (21-23): 世界系线速度
        results.rbdState_36.segment<3>(21) << lastOdomMsg_.twist.twist.linear.x,
            lastOdomMsg_.twist.twist.linear.y,
            lastOdomMsg_.twist.twist.linear.z;

        // 7. 12个关节速度 (24-35): 从 Bridge 拿
        results.rbdState_36.segment<12>(24) = bridgePtr_->joint_velocities;

        // --- 处理触地状态 ---

        // 8. WBC 触地标志: 仿真中通常从 Bridge 的触地传感器读取
        results.contactFlags_WBC = bridgePtr_->contact_flags;

        // 9. MPC 模式索引: 将布尔数组转为整数
        results.contactFlags_MPC = computeMpcMode(results.contactFlags_WBC);
    }

    size_t TopicEstimator::computeMpcMode(const std::array<bool, 4> &contactFlags)
    {
        size_t mode = 0;
        if (contactFlags[0])
            mode += 1; // LF
        if (contactFlags[1])
            mode += 2; // RF
        if (contactFlags[2])
            mode += 4; // LH
        if (contactFlags[3])
            mode += 8; // RH
        return mode;
    }
}