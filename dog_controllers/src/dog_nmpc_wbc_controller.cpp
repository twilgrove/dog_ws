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

    void DogNmpcWbcController::init_real_or_god()
    {
        state_estimator_ = std::make_unique<KalmanFilterEstimator>(
            taskFile,
            robot_interface_->getPinocchioInterface(),
            robot_interface_->getCentroidalModelInfo(),
            robot_interface_->getEndEffectorKinematics(), node_);
    }
    void DogNmpcWbcController_God::init_real_or_god()
    {
        state_estimator_ = std::make_unique<TopicEstimator>(
            robot_interface_->getPinocchioInterface(),
            robot_interface_->getCentroidalModelInfo(),
            robot_interface_->getEndEffectorKinematics(), node_);
    }

    CallbackReturn DogNmpcWbcController::on_activate(const rclcpp_lifecycle::State &)
    {
        bridge_ = std::make_unique<DogDataBridge>(state_interfaces_, command_interfaces_, node_);

        robot_interface_ = std::make_unique<LeggedRobotInterface>(taskFile, urdfFile, referenceFile);

        init_real_or_god();

        debug_manager_ = std::make_unique<DebugManager>(
            &(bridge_->imu),
            &(state_estimator_->results),
            robot_interface_->getPinocchioInterface(),
            robot_interface_->getCentroidalModelInfo(),
            robot_interface_->getEndEffectorKinematics(),
            node_);

        wbc_ = std::make_unique<WeightedWbc>(
            taskFile,
            robot_interface_->getPinocchioInterface(),
            robot_interface_->getCentroidalModelInfo(),
            robot_interface_->getEndEffectorKinematics(),
            node_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DogNmpcWbcController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        bridge_->read_from_hw();

        state_estimator_->estimate(bridge_->legs, bridge_->imu, period);

        // vector_t dummyState = vector_t::Zero(24);
        // vector_t dummyInput = vector_t::Zero(24);
        // vector_t qpResult = wbc_->update(dummyState,
        //                                  dummyInput,
        //                                  state_estimator_->results.rbdState_36,
        //                                  state_estimator_->results.contactFlags_MPC,
        //                                  period.seconds());
        // Eigen::Vector3d qNom;
        // qNom << 0.0, -0.8, 1.5;
        // for (int i = 0; i < 4; ++i)
        // {
        //     for (int j = 0; j < 3; ++j)
        //     {
        //         int jointIdx = i * 3 + j;

        //         bridge_->legs[i].joints[j]->cmd_pos = qNom(j); // 目标角度 (0, -0.8, 1.5)
        //         bridge_->legs[i].joints[j]->cmd_vel = 0.0;     // 目标速度设为 0
        //         bridge_->legs[i].joints[j]->cmd_kp = 10.0;     // 低增益 Kp，提供基础刚度
        //         bridge_->legs[i].joints[j]->cmd_kd = 1.0;      // 阻尼，防止震荡

        //         bridge_->legs[i].joints[j]->cmd_ff = qpResult(30 + jointIdx);
        //     }
        // }

        bridge_->write_to_hw();
        debug_manager_->update_debug(state_estimator_->currentObservation_);
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController_God, controller_interface::ControllerInterface)