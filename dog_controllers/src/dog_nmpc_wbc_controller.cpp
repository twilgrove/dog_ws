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

        ros_interface_thread_ = std::make_unique<std::thread>([this]()
                                                              { rclcpp::spin(ros_interface_node_); });
        setThreadPriority(20, *ros_interface_thread_);

        return CallbackReturn::SUCCESS;
    }
    controller_interface::return_type DogNmpcWbcController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        mainLoopTimer_.startTimer();
        bridge_->read_from_hw();
        state_estimator_->estimate(bridge_->legs, bridge_->imu, period);

        // --- 初始化临时变量，防止仪表盘读取随机内存 ---
        vector_t optimizedState = vector_t::Zero(24);
        vector_t optimizedInput = vector_t::Zero(24);
        size_t plannedMode = 0;

        // --- 状态 1：关节空间 PD 插值起立 ---
        if (currentState_ == ControlState::JOINT_STANDUP)
        {
            standUpTimer_ += period.seconds();
            scalar_t phase = std::min(standUpTimer_ / standUpDuration_, 1.0);
            scalar_t s = (1.0 - std::cos(M_PI * phase)) / 2.0;

            for (int i = 0; i < 4; ++i)
            {
                scalar_t q_haa_start = (i == 0 || i == 2) ? -0.4 : 0.4;
                scalar_t q_hfe_start = -1.2;
                scalar_t q_kfe_start = 2.8;

                scalar_t q_haa_goal = 0.0;
                scalar_t q_hfe_goal = -0.8;
                scalar_t q_kfe_goal = 1.5;

                bridge_->legs[i].joints[0]->cmd_pos = q_haa_start + s * (q_haa_goal - q_haa_start);
                bridge_->legs[i].joints[1]->cmd_pos = q_hfe_start + s * (q_hfe_goal - q_hfe_start);
                bridge_->legs[i].joints[2]->cmd_pos = q_kfe_start + s * (q_kfe_goal - q_kfe_start);

                bridge_->legs[i].joints[0]->cmd_kp = 50.0;
                bridge_->legs[i].joints[1]->cmd_kp = 50.0;
                bridge_->legs[i].joints[2]->cmd_kp = 50.0;
                bridge_->legs[i].joints[0]->cmd_kd = 3.0;
                bridge_->legs[i].joints[1]->cmd_kd = 3.0;
                bridge_->legs[i].joints[2]->cmd_kd = 3.0;
                bridge_->legs[i].joints[0]->cmd_ff = 0.0;
                bridge_->legs[i].joints[1]->cmd_ff = 0.0;
                bridge_->legs[i].joints[2]->cmd_ff = 0.0;
            }

            if (phase >= 1.0 && state_estimator_->currentObservation_.state(8) > 0.1)
            {
                nmpc_controller_->start(state_estimator_->currentObservation_);

                // ocs2::TargetTrajectories target;
                // scalar_t t = state_estimator_->currentObservation_.time;
                // target.timeTrajectory = {t, t + 3.0, t + 10.0, t + 15.0, t + 20.0};

                // vector_t goalState = vector_t::Zero(24);
                // goalState(8) = 0.32;
                // for (int k = 0; k < 4; k++)
                // {
                //     goalState(12 + k * 3 + 0) = 0.0;
                //     goalState(12 + k * 3 + 1) = -0.8;
                //     goalState(12 + k * 3 + 2) = 1.5;
                // }
                // target.stateTrajectory = {goalState, goalState, goalState, goalState, goalState};
                // target.inputTrajectory = {vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24)};

                // nmpc_controller_->mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target);
                currentState_ = ControlState::NMPC_ACTIVE;
                RCLCPP_INFO(node_->get_logger(), "\033[1;32m[状态切换] 关节起立完成，NMPC 接管！\033[0m");
            }
        }
        // --- 状态 2：NMPC + WBC 激活阶段 ---
        else if (currentState_ == ControlState::NMPC_ACTIVE)
        {
            // {
            //     ocs2::TargetTrajectories target;
            //     scalar_t t = state_estimator_->currentObservation_.time;
            //     // 设定从现在到未来 2 秒的参考轨迹（保持 0.32m）
            //     target.timeTrajectory = {t, t + 1.0, t + 2.0};

            //     vector_t goalState = vector_t::Zero(24);
            //     goalState(8) = 0.32; // 目标高度
            //     for (int k = 0; k < 4; k++)
            //     {
            //         goalState(12 + k * 3 + 0) = 0.0;  // HAA
            //         goalState(12 + k * 3 + 1) = -0.8; // HFE
            //         goalState(12 + k * 3 + 2) = 1.5;  // KFE
            //     }
            //     target.stateTrajectory = {goalState, goalState, goalState};
            //     target.inputTrajectory = {vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24)};

            //     // 持续同步给 NMPC
            //     nmpc_controller_->mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target);
            // }
            nmpc_controller_->update(state_estimator_->currentObservation_, optimizedState, optimizedInput, plannedMode);

            // NaN 检查，防止错误指令发给电机
            if (optimizedState.hasNaN() || optimizedInput.hasNaN())
            {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "NMPC 产生无效值(NaN)，跳过本次更新");
            }
            else
            {
                vector_t qpResult = wbc_->update(optimizedState, optimizedInput, state_estimator_->results.rbdState_36, plannedMode, period.seconds());
                vector_t torque = qpResult.tail(12);
                vector_t posDes = optimizedState.segment(12, 12);
                vector_t velDes = optimizedInput.segment(12, 12);

                for (int i = 0; i < 4; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        int jointIdx = i * 3 + j;
                        bridge_->legs[i].joints[j]->cmd_pos = posDes(jointIdx);
                        bridge_->legs[i].joints[j]->cmd_vel = velDes(jointIdx);
                        bridge_->legs[i].joints[j]->cmd_kp = 0.0;
                        bridge_->legs[i].joints[j]->cmd_kd = 3.0;
                        bridge_->legs[i].joints[j]->cmd_ff = torque(jointIdx);
                    }
                }
            }
            debug_manager_->update_debug(state_estimator_->currentObservation_, nmpc_controller_->mpcMrtInterface_->getPolicy(), nmpc_controller_->mpcMrtInterface_->getCommand());
        }

        bridge_->write_to_hw();
        mainLoopTimer_.endTimer();

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 300,
                             "\n\033[1;36m[ 🤖 机器人综合诊断报告 ] >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\033[0m"
                             "\n\033[1;33m[ 1. 物理状态 ]\033[0m"
                             "\n  实测高度 (Z) : \033[1;32m%.3f\033[0m m  |  实测速度 (Vz) : %.3f m/s"
                             "\n  当前模式     : %s"
                             "\n\033[1;33m[ 2. NMPC 规划 ]\033[0m"
                             "\n  最终目标高度 : \033[1;35m%.3f\033[0m m  |  当前规划高度 : %.3f m" // 修改这里
                             "\n  步态模式     : %zu (15=全支撑)"
                             "\n  期望力(Fz)   : LF:%.1f, RF:%.1f, LH:%.1f, RH:%.1f N"
                             "\n  LF 关节参考   : HAA:%.2f, HFE:%.2f, KFE:%.2f rad"
                             "\n\033[1;33m[ 3. 控制性能 ]\033[0m"
                             "\n  主循环平均耗时 : \033[1;32m%.3f\033[0m ms  |  运行总数 : %d 次"
                             "\n\033[1;36m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\033[0m",
                             state_estimator_->currentObservation_.state(8),
                             state_estimator_->currentObservation_.state(2),
                             (currentState_ == ControlState::JOINT_STANDUP ? "\033[1;35m关节起立中\033[0m" : "\033[1;32mNMPC激活\033[0m"),
                             0.32,              // 打印固定的 0.32
                             optimizedState(8), // 打印 NMPC 实时规划的过渡值
                             plannedMode,
                             optimizedInput(2), optimizedInput(5), optimizedInput(8), optimizedInput(11),
                             optimizedState(12), optimizedState(13), optimizedState(14),
                             mainLoopTimer_.getAverageInMilliseconds(),
                             mainLoopTimer_.getNumTimedIntervals());

        return controller_interface::return_type::OK;
    }
    DogNmpcWbcController::~DogNmpcWbcController()
    {
        if (ros_interface_thread_ && ros_interface_thread_->joinable())
        {
            ros_interface_thread_->join();
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController, controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(dog_controllers::DogNmpcWbcController_God, controller_interface::ControllerInterface)