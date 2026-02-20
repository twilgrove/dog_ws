#include "common/debug_manager.hpp"
#include <cstring>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace dog_controllers
{
    DebugManager::DebugManager(const ImuData *imuPtr_,
                               const EstimatorResults *results_,
                               const PinocchioInterface &pinocchioInterface,
                               const CentroidalModelInfo &info,
                               const PinocchioEndEffectorKinematics &eeKinematics,
                               rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : imuPtr_(imuPtr_), results_(results_), node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 DebugManager\033[0m");

        node_->get_parameter_or<double>("threshold", threshold, 0.01);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m📊 [PARAM] 已加载配置清单:\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  └─ 发布间隔: \033[0m%.3fs", threshold);

        debugNode_ = std::make_shared<rclcpp::Node>(
            std::string(node_->get_name()) + "_debug_sub",
            node_->get_namespace(),
            node_->get_node_options());

        visualizerPtr_ = std::make_unique<ocs2::legged_robot::LeggedRobotVisualizer>(
            pinocchioInterface,
            info,
            eeKinematics,
            debugNode_,
            1.0 / threshold);

        statePub_ = debugNode_->create_publisher<dog_bringup::msg::DogState>(
            "dog_robot_state", rclcpp::SensorDataQoS());
        stateMsg_.header.frame_id = "world";

        tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(debugNode_);

        observationPub_ = debugNode_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            "dog_robot_mpc_observation", 10);

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ DebugManager\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m====================================================\033[0m");
    }

    void DebugManager::update_debug(const SystemObservation &observation,
                                    const PrimalSolution &primalSolution,
                                    const CommandData &command)
    {
        delt_Time_ = observation.time - lastTime_;
        // if (delt_Time_ >= threshold)
        // {
        //     publishStateData();
        //     lastTime_ = observation.time;
        // }
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "运行中... ");
        if (delt_Time_ >= 0.002) // 500Hz
        {
            publishTF();
            publishObservation(observation);
            lastTime_ = observation.time;
        }
    }
    void DebugManager::publishTF()
    {
        const auto &s = results_->rbdState_36;
        auto now = node_->get_clock()->now();

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = s(3);
        t.transform.translation.y = s(4);
        t.transform.translation.z = s(5);

        tf2::Quaternion q;
        q.setRPY(s(2), s(1), s(0));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tfBroadcaster_->sendTransform(t);
    }

    void DebugManager::publishObservation(const SystemObservation &observation)
    {
        auto msg = ocs2::ros_msg_conversions::createObservationMsg(observation);
        observationPub_->publish(msg);
    }

    void DebugManager::publishStateData()
    {

        auto &m = stateMsg_;
        const auto &s = results_->rbdState_36;

        // 1. Header 更新
        m.header.stamp = node_->get_clock()->now();
        m.system_monitor.time = delt_Time_;
        // 2. 基座姿态
        m.base_state.orientation.x = s(2);
        m.base_state.orientation.y = s(1);
        m.base_state.orientation.z = s(0);

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