#include "common/debug_manager.hpp"
#include <cstring>

namespace dog_controllers
{
    DebugManager::DebugManager(const ImuData *imuPtr_, const EstimatorResults *results_, rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : imuPtr_(imuPtr_), results_(results_), node_(node)
    {
        statePub_ = node_->create_publisher<dog_bringup::msg::DogState>(
            "/dog_robot_state", rclcpp::SensorDataQoS());

        stateMsg_.header.frame_id = "world";
    }

    void DebugManager::publish()
    {
        if (count++ >= hz)
        {
            count = 0;

            const auto &s = results_->rbdState_36;
            auto &m = stateMsg_;

            // 1. Header 更新
            m.header.stamp = node_->get_clock()->now();

            // 2. 基座姿态
            m.orientation_zyx.x = s(2); // Yaw
            m.orientation_zyx.y = s(1); // Pitch
            m.orientation_zyx.z = s(0); // Roll

            // 3. 基座位置
            m.position.x = s(3);
            m.position.y = s(4);
            m.position.z = s(5);

            // 4. 运动学数据
            // 基座角速度
            m.base_angular_velocity.x = s(18);
            m.base_angular_velocity.y = s(19);
            m.base_angular_velocity.z = s(20);

            // 世界系线速度
            m.world_linear_velocity.x = s(21);
            m.world_linear_velocity.y = s(22);
            m.world_linear_velocity.z = s(23);

            // 基座线加速度
            m.base_linear_acceleration.x = imuPtr_->lin_acc[0];
            m.base_linear_acceleration.y = imuPtr_->lin_acc[1];
            m.base_linear_acceleration.z = imuPtr_->lin_acc[2];

            // 5. 关节角度 (rbdState 6-17)
            std::memcpy(m.q_lf.data(), s.data() + 6, 3 * sizeof(double));
            std::memcpy(m.q_lh.data(), s.data() + 9, 3 * sizeof(double));
            std::memcpy(m.q_rf.data(), s.data() + 12, 3 * sizeof(double));
            std::memcpy(m.q_rh.data(), s.data() + 15, 3 * sizeof(double));

            // 6. 关节速度 (rbdState 24-35)
            std::memcpy(m.dq_lf.data(), s.data() + 24, 3 * sizeof(double));
            std::memcpy(m.dq_lh.data(), s.data() + 27, 3 * sizeof(double));
            std::memcpy(m.dq_rf.data(), s.data() + 30, 3 * sizeof(double));
            std::memcpy(m.dq_rh.data(), s.data() + 33, 3 * sizeof(double));

            // 7. 触地状态 (WBC Flags)
            // 映射顺序根据结果数组定义: [LF, LH, RF, RH]
            m.contact_state[0] = results_->contactFlags_WBC[0];
            m.contact_state[1] = results_->contactFlags_WBC[2];
            m.contact_state[2] = results_->contactFlags_WBC[1];
            m.contact_state[3] = results_->contactFlags_WBC[3];

            statePub_->publish(m);
        }
    }
}