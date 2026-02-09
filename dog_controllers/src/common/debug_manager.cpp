#include "common/debug_manager.hpp"
#include <cstring>

namespace dog_controllers
{
    DebugManager::DebugManager(const ImuData *imuPtr_,
                               const EstimatorResults *results_,
                               rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : imuPtr_(imuPtr_), results_(results_), node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 DebugManager\033[0m");

        node_->get_parameter_or<int>("threshold", threshold, 100);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m📊 [PARAM] 已加载配置清单:\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  └─ 发布间隔: \033[0m%dms", threshold);

        statePub_ = node_->create_publisher<dog_bringup::msg::DogState>(
            "/dog_robot_state", rclcpp::SensorDataQoS());

        stateMsg_.header.frame_id = "world";
        last_loop_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ DebugManager\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m====================================================\033[0m");
    }

    void DebugManager::publish()
    {

        if (count++ >= threshold)
        {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - last_loop_time_;
            double total_dt = elapsed.count();
            last_loop_time_ = now;

            auto &m = stateMsg_;
            const auto &s = results_->rbdState_36;

            // 1. Header 更新
            m.header.stamp = node_->get_clock()->now();
            m.system_monitor.time = (total_dt * 1000.0) / count;
            count = 0;
            // 2. 基座姿态
            m.base_state.orientation.x = s(2); // Yaw
            m.base_state.orientation.y = s(1); // Pitch
            m.base_state.orientation.z = s(0); // Roll

            // 3. 基座位置
            m.base_state.position.x = s(3);
            m.base_state.position.y = s(4);
            m.base_state.position.z = s(5);

            // 4. 运动学数据
            // 基座角速度
            m.base_state.world_angular_velocity.x = s(18);
            m.base_state.world_angular_velocity.y = s(19);
            m.base_state.world_angular_velocity.z = s(20);

            // 世界系线速度
            m.base_state.world_linear_velocity.x = s(21);
            m.base_state.world_linear_velocity.y = s(22);
            m.base_state.world_linear_velocity.z = s(23);

            // 基座线加速度
            m.base_state.imu_linear_acceleration.x = imuPtr_->lin_acc[0];
            m.base_state.imu_linear_acceleration.y = imuPtr_->lin_acc[1];
            m.base_state.imu_linear_acceleration.z = imuPtr_->lin_acc[2];

            for (int i = 0; i < 4; ++i)
            {
                // 5. 关节位置
                std::memcpy(m.leg_state[i].position.data(), s.data() + 6 + i * 3, 3 * sizeof(double));

                // 6. 关节速度
                std::memcpy(m.leg_state[i].velocity.data(), s.data() + 24 + i * 3, 3 * sizeof(double));

                // 7. 触地状态
                m.leg_state[i].contact_state = results_->contactFlags_WBC[i];
            }

            statePub_->publish(m);
        }
    }
}