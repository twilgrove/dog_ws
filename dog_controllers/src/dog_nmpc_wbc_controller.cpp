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
            dog_interface_->getPinocchioInterface(),
            dog_interface_->getEndEffectorKinematics(), node_);
    }
    void DogNmpcWbcController_God::init_real_or_god()
    {
        state_estimator_ = std::make_unique<TopicEstimator>(
            dog_interface_->getPinocchioInterface(),
            dog_interface_->getEndEffectorKinematics(), node_);
    }

    CallbackReturn DogNmpcWbcController::on_activate(const rclcpp_lifecycle::State &)
    {
        bridge_ = std::make_unique<DogDataBridge>(state_interfaces_, command_interfaces_, node_);

        dog_interface_ = std::make_unique<DogInterface>(taskFile, urdfFile, referenceFile);

        init_real_or_god();

        debug_manager_ = std::make_unique<DebugManager>(
            &(bridge_->imu),
            &(state_estimator_->results),
            node_);

        wbc_ = std::make_unique<WbcBase>(
            dog_interface_->getPinocchioInterface(),
            dog_interface_->getCentroidalModelInfo(),
            dog_interface_->getEndEffectorKinematics());

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DogNmpcWbcController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        bridge_->read_from_hw();

        // for (int i = 0; i < 4; ++i)
        // {
        //     for (int j = 0; j < 3; ++j)
        //     {
        //         bridge_->legs[i].joints[j]->cmd_pos = 0.0;
        //         bridge_->legs[i].joints[j]->cmd_kp = 40.0;
        //         bridge_->legs[i].joints[j]->cmd_kd = 2.0;
        //         bridge_->legs[i].joints[j]->cmd_ff = 0.0;
        //         bridge_->legs[i].joints[j]->cmd_vel = 0.0;
        //     }
        // }
        state_estimator_->estimate(bridge_->legs, bridge_->imu, period);
        bridge_->write_to_hw();
        debug_manager_->publish();
        return controller_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController_God, controller_interface::ControllerInterface)