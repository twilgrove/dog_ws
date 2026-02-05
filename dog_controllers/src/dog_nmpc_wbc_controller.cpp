#include "dog_nmpc_wbc_controller.hpp"

namespace dog_controllers
{
    CallbackReturn DogNmpcWbcController::on_init()
    {
        node_ = get_node();
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("dog_bringup");
        taskFile = pkg_share_path + "/config/description/task.info";
        urdfFile = pkg_share_path + "/config/description/dog.urdf";
        referenceFile = pkg_share_path + "/config/description/reference.info";
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn DogNmpcWbcController::on_configure(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DogNmpcWbcController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::ALL;
        return conf;
    }

    controller_interface::InterfaceConfiguration DogNmpcWbcController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::ALL;
        return conf;
    }

    CallbackReturn DogNmpcWbcController::on_activate(const rclcpp_lifecycle::State &)
    {
        bridge_ = std::make_unique<DogDataBridge>(state_interfaces_, command_interfaces_, node_);

        dog_interface_ = std::make_unique<DogInterface>(taskFile, urdfFile, referenceFile);

        state_estimator_ = std::make_unique<TopicEstimator>(
            bridge_.get(),
            dog_interface_->getPinocchioInterface(),
            dog_interface_->getCentroidalModelInfo(),
            dog_interface_->getEndEffectorKinematics(), node_);

        debug_manager_ = std::make_unique<DebugManager>(node_);

        if (!bridge_ || !dog_interface_ || !state_estimator_ || !debug_manager_)
        {
            RCLCPP_ERROR(node_->get_logger(), "核心类初始化错误！！");
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DogNmpcWbcController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        bridge_->read_from_hw();

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                bridge_->legs[i].joints[j]->cmd_pos = 0.0;
                bridge_->legs[i].joints[j]->cmd_kp = 40.0;
                bridge_->legs[i].joints[j]->cmd_kd = 2.0;
                bridge_->legs[i].joints[j]->cmd_ff = 0.0;
                bridge_->legs[i].joints[j]->cmd_vel = 0.0;
            }
        }

        static int print_count = 0;
        if (++print_count >= 100)
        {
            print_count = 0;
            RCLCPP_INFO(node_->get_logger(), "==================== DOG STATE SNAPSHOT ====================");

            // --- 关节数据 (Joints) ---
            const char *leg_names[] = {"FL", "FR", "HL", "HR"};
            const char *joint_names[] = {"Hip", "Thigh", "Calf"};

            for (int i = 0; i < 4; ++i)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[%s Leg] | Pos(rad): [%.3f, %.3f, %.3f] | Contact: %s",
                            leg_names[i],
                            bridge_->legs[i].hip.pos, bridge_->legs[i].thigh.pos, bridge_->legs[i].calf.pos,
                            bridge_->legs[i].contact > 0.5 ? "GROUND" : "AIR");
            }

            // --- IMU 数据 ---
            RCLCPP_INFO(node_->get_logger(), "------------------------------------------------------------");
            RCLCPP_INFO(node_->get_logger(),
                        "[IMU Ori] | W:%.3f X:%.3f Y:%.3f Z:%.3f",
                        bridge_->imu.ori[3], bridge_->imu.ori[0], bridge_->imu.ori[1], bridge_->imu.ori[2]);
            RCLCPP_INFO(node_->get_logger(),
                        "[IMU Acc] | X:%.3f Y:%.3f Z:%.3f m/s^2",
                        bridge_->imu.lin_acc[0], bridge_->imu.lin_acc[1], bridge_->imu.lin_acc[2]);
            RCLCPP_INFO(node_->get_logger(),
                        "[IMU Ang] | X:%.3f Y:%.3f Z:%.3f rad/s",
                        bridge_->imu.ang_vel[0], bridge_->imu.ang_vel[1], bridge_->imu.ang_vel[2]);

            RCLCPP_INFO(node_->get_logger(), "============================================================");
        }

        bridge_->write_to_hw();
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController, controller_interface::ControllerInterface)