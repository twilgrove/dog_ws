#include "dog_nmpc_wbc_controller.hpp"

namespace dog_controllers
{
    CallbackReturn DogNmpcWbcController::on_init()
    {
        node_ = get_node();
        ros_interface_node_ = std::make_shared<rclcpp::Node>(
            std::string(node_->get_name()) + "_ros_interface",
            node_->get_namespace(),
            node_->get_node_options());
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
        ros_interface_thread_ = std::make_unique<std::thread>([this]()
                                                              { rclcpp::spin(ros_interface_node_); });
        setThreadPriority(20, *ros_interface_thread_);

        bridge_ = std::make_unique<DogDataBridge>(state_interfaces_, command_interfaces_, node_);

        robot_interface_ = std::make_shared<LeggedRobotInterface>(taskFile, urdfFile, referenceFile);

        init_real_or_god();

        debug_manager_ = std::make_unique<DebugManager>(
            &(bridge_->imu),
            &(state_estimator_->results),
            robot_interface_->getPinocchioInterface(),
            robot_interface_->getCentroidalModelInfo(),
            robot_interface_->getEndEffectorKinematics(),
            ros_interface_node_);

        nmpc_controller_ = std::make_unique<NmpcController>(
            ros_interface_node_,
            robot_interface_);

        wbc_ = std::make_unique<WeightedWbc>(
            taskFile,
            robot_interface_->getPinocchioInterface(),
            robot_interface_->getCentroidalModelInfo(),
            robot_interface_->getEndEffectorKinematics(),
            node_);

        nmpc_controller_->start(state_estimator_->currentObservation_);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DogNmpcWbcController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        mainLoopTimer_.startTimer();
        bridge_->read_from_hw();

        state_estimator_->estimate(bridge_->legs, bridge_->imu, period);

        vector_t optimizedState, optimizedInput;
        size_t plannedMode;
        nmpc_controller_->update(state_estimator_->currentObservation_, optimizedState, optimizedInput, plannedMode);

        vector_t qpResult = wbc_->update(optimizedState,
                                         optimizedInput,
                                         state_estimator_->results.rbdState_36,
                                         plannedMode,
                                         period.seconds());
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
        debug_manager_->update_debug(state_estimator_->currentObservation_,
                                     nmpc_controller_->mpcMrtInterface_->getPolicy(),
                                     nmpc_controller_->mpcMrtInterface_->getCommand());
        mainLoopTimer_.endTimer();

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            10000,
            "\n\033[1;33m====================================================\033[0m"
            "\n\033[1;33m[ 主循环实时性能报告 ]\033[0m 🔄"
            "\n\033[1;33m----------------------------------------------------\033[0m"
            "\n  运行总数   : %d 次"
            "\n  平均耗时   : \033[1;32m%.3f\033[0m ms"
            "\n  最大耗时   : \033[1;31m%.3f\033[0m ms"
            "\n  实时要求   : < 1.000 ms"
            "\n\033[1;33m====================================================\033[0m",
            mainLoopTimer_.getNumTimedIntervals(),
            mainLoopTimer_.getAverageInMilliseconds(),
            mainLoopTimer_.getMaxIntervalInMilliseconds());

        return controller_interface::return_type::OK;
    }
    DogNmpcWbcController::~DogNmpcWbcController()
    {
        if (ros_interface_node_)
        {
            rclcpp::shutdown();
        }

        if (ros_interface_thread_ && ros_interface_thread_->joinable())
        {
            ros_interface_thread_->join();
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController_God, controller_interface::ControllerInterface)