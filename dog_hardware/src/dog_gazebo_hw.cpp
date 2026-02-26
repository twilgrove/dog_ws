#include "dog_hardware/dog_gazebo_hw.hpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>

// 0:前趴；1：后趴；2：站立；
#define JointPosition 0
namespace dog_hardware
{
    CallbackReturn DogGazeboHW::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (GazeboSystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::FAILURE;
        return CallbackReturn::SUCCESS;
    }

    bool DogGazeboHW::initSim(
        rclcpp::Node::SharedPtr &node,
        gazebo::physics::ModelPtr model,
        const hardware_interface::HardwareInfo &hardware_info,
        sdf::ElementPtr /*sdf*/)
    {
        this->node_ = node;
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 DogGazeboHW\033[0m");
        // 提取参数
        delay_ = std::stod(hardware_info.hardware_parameters.at("delay"));
        double a_var = std::stod(hardware_info.hardware_parameters.at("imu_accel_var"));
        double g_var = std::stod(hardware_info.hardware_parameters.at("imu_gyro_var"));
        double o_var = std::stod(hardware_info.hardware_parameters.at("imu_ori_var"));
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m📊 [PARAM] 已加载配置清单:\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 命令延迟      : \033[0m%.3f 秒", delay_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 加速度协方差  : \033[0m%.6f", a_var);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 角速度协方差  : \033[0m%.6f", g_var);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  └─ 姿态协方差    : \033[0m%.6f", o_var);

        // --- 初始化关节 ---
        for (const auto &joint_info : hardware_info.joints)
        {
            JointData jd;
            jd.name = joint_info.name;
            jd.gz_joint = model->GetJoint(joint_info.name);

            if (!jd.gz_joint)
            {
                RCLCPP_ERROR(node_->get_logger(), "在 Gazebo 模型中找不到关节: %s", jd.name.c_str());
                return false;
            }

            double initial_pos = 0.0;

#if JointPosition == 0 // 前趴
            double HAA = 0.4;
            double HFE = -1.2;
            double KFE = 2.79;
#elif JointPosition == 1 // 后趴
            double HAA = 0.0;
            double HFE = -2.67;
            double KFE = 2.79;
#elif JointPosition == 2 // 站立
            double HAA = 0.0;
            double HFE = -0.8;
            double KFE = 1.5;
#endif
            if (jd.name.find("HAA") != std::string::npos)
                initial_pos = (jd.name.find("L") != std::string::npos) ? -HAA : HAA;

            else if (jd.name.find("HFE") != std::string::npos)
                initial_pos = HFE;

            else if (jd.name.find("KFE") != std::string::npos)
                initial_pos = KFE;

            jd.gz_joint->SetPosition(0, initial_pos);

            jd.pos = initial_pos;
            jd.cmd.pos_des = initial_pos;

            joints_.push_back(jd);
        }

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[配置成功] 🔗 关节映射完成，共绑定 %zu 关节指针\033[0m",
                    joints_.size());

        // --- 初始化传感器 ---

        for (const auto &sensor : hardware_info.sensors)
        {
            // 触地传感器
            if (sensor.name.find("contact") != std::string::npos)
            {
                ContactSensorData data;
                data.name = sensor.name;
                auto gz_s = gazebo::sensors::SensorManager::Instance()->GetSensor(sensor.name);
                data.gazebo_sensor = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(gz_s);
                data.contact_value = 0.0;
                contact_sensors_.push_back(data);
            }
            // IMU 传感器
            if (sensor.name.find("imu") != std::string::npos)
            {
                imu_data_.name = sensor.name;
                auto imu_sensor = gazebo::sensors::SensorManager::Instance()->GetSensor(imu_data_.name);
                imu_data_.gazebo_sensor = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(imu_sensor);
                imu_data_.lin_acc_cov.fill(0.0);
                imu_data_.lin_acc_cov[0] = imu_data_.lin_acc_cov[4] = imu_data_.lin_acc_cov[8] = a_var;

                imu_data_.ang_vel_cov.fill(0.0);
                imu_data_.ang_vel_cov[0] = imu_data_.ang_vel_cov[4] = imu_data_.ang_vel_cov[8] = g_var;

                imu_data_.ori_cov.fill(0.0);
                imu_data_.ori_cov[0] = imu_data_.ori_cov[4] = imu_data_.ori_cov[8] = o_var;
            }
        }
        if (contact_sensors_.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "未找到任何触地传感器！");
            return false;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "\033[1;32m[配置成功] 🔗 触地传感器映射完成，共绑定 %zu 个传感器指针\033[0m",
                        contact_sensors_.size());
        }

        if (!imu_data_.gazebo_sensor)
        {
            RCLCPP_ERROR(node_->get_logger(), "未找到 IMU 传感器！");
            return false;
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "\033[1;32m[配置成功] 🔗 成功绑定了 IMU 传感器指针\033[0m");
        }
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ DogGazeboHW\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m====================================================\033[0m");
        return true;
    }

    std::vector<hardware_interface::StateInterface> DogGazeboHW::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> states;

        // --- 导出位置、速度、力矩反馈 ---
        for (auto &j : joints_)
        {
            states.emplace_back(j.name, "position", &j.pos);
            states.emplace_back(j.name, "velocity", &j.vel);
            states.emplace_back(j.name, "effort", &j.eff);
        }

        // --- 导出触地检测槽位 ---
        for (auto &sensor : contact_sensors_)
        {
            states.emplace_back(sensor.name, "contact", &sensor.contact_value);
        }

        // --- 导出 IMU 状态接口 ---
        // 导出原始数据
        states.emplace_back(imu_data_.name, "orientation.x", &imu_data_.ori[0]);
        states.emplace_back(imu_data_.name, "orientation.y", &imu_data_.ori[1]);
        states.emplace_back(imu_data_.name, "orientation.z", &imu_data_.ori[2]);
        states.emplace_back(imu_data_.name, "orientation.w", &imu_data_.ori[3]);
        states.emplace_back(imu_data_.name, "angular_velocity.x", &imu_data_.ang_vel[0]);
        states.emplace_back(imu_data_.name, "angular_velocity.y", &imu_data_.ang_vel[1]);
        states.emplace_back(imu_data_.name, "angular_velocity.z", &imu_data_.ang_vel[2]);

        states.emplace_back(imu_data_.name, "linear_acceleration.x", &imu_data_.lin_acc[0]);
        states.emplace_back(imu_data_.name, "linear_acceleration.y", &imu_data_.lin_acc[1]);
        states.emplace_back(imu_data_.name, "linear_acceleration.z", &imu_data_.lin_acc[2]);

        // 导出协方差地址
        states.emplace_back(imu_data_.name, "orientation_covariance", &imu_data_.ori_cov[0]);
        states.emplace_back(imu_data_.name, "angular_velocity_covariance", &imu_data_.ang_vel_cov[0]);
        states.emplace_back(imu_data_.name, "linear_acceleration_covariance", &imu_data_.lin_acc_cov[0]);

        return states;
    }

    std::vector<hardware_interface::CommandInterface> DogGazeboHW::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> commands;

        for (auto &j : joints_)
        {
            commands.emplace_back(j.name, "position", &j.cmd.pos_des);
            commands.emplace_back(j.name, "velocity", &j.cmd.vel_des);
            commands.emplace_back(j.name, "kp", &j.cmd.kp);
            commands.emplace_back(j.name, "kd", &j.cmd.kd);
            commands.emplace_back(j.name, "effort", &j.cmd.ff);
        }
        return commands;
    }

    hardware_interface::return_type DogGazeboHW::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Read 循环正在运行中...");
        // 1. 获取关节物理状态
        for (auto &j : joints_)
        {
            if (j.gz_joint)
            {
                j.pos = j.gz_joint->Position(0);
                j.vel = j.gz_joint->GetVelocity(0);
                j.eff = j.gz_joint->GetForce(0u);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "关节 %s 指针未绑定，无法获取数据！", j.name.c_str());
            }
        }
        // 2. 获取 IMU 数据
        if (imu_data_.gazebo_sensor)
        {
            // 获取姿态 (四元数)
            ignition::math::Quaterniond q = imu_data_.gazebo_sensor->Orientation();
            imu_data_.ori[0] = q.X();
            imu_data_.ori[1] = q.Y();
            imu_data_.ori[2] = q.Z();
            imu_data_.ori[3] = q.W();

            // 获取角速度
            ignition::math::Vector3d ang_vel = imu_data_.gazebo_sensor->AngularVelocity();
            imu_data_.ang_vel[0] = ang_vel.X();
            imu_data_.ang_vel[1] = ang_vel.Y();
            imu_data_.ang_vel[2] = ang_vel.Z();

            // 获取原始线加速度 (加速度计读数：包含重力)
            ignition::math::Vector3d raw_acc = imu_data_.gazebo_sensor->LinearAcceleration();
            imu_data_.lin_acc[0] = raw_acc.X();
            imu_data_.lin_acc[1] = raw_acc.Y();
            imu_data_.lin_acc[2] = raw_acc.Z();
            // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "IMU 数据：ori = [%.3f, %.3f, %.3f, %.3f], ang_vel = [%.3f, %.3f, %.3f], lin_acc = [%.3f, %.3f, %.3f]",
            //                      imu_data_.ori[0], imu_data_.ori[1], imu_data_.ori[2], imu_data_.ori[3],
            //                      imu_data_.ang_vel[0], imu_data_.ang_vel[1], imu_data_.ang_vel[2],
            //                      imu_data_.lin_acc[0], imu_data_.lin_acc[1], imu_data_.lin_acc[2]);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "IMU 传感器指针未绑定，无法获取数据！");
        }
        // 3. 获取触地状态
        for (auto &sensor : contact_sensors_)
        {
            if (sensor.gazebo_sensor)
            {
                gazebo::msgs::Contacts contacts = sensor.gazebo_sensor->Contacts();
                sensor.contact_value = (contacts.contact_size() > 0) ? 1.0 : 0.0;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "触地传感器 %s 指针未绑定，无法获取数据！", sensor.name.c_str());
            }
        }
        // RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "触地状态：%s,%s", (contact_sensors_[0].contact_value > 0.5 ? "已触地" : "未触地"), (contact_sensors_[1].contact_value > 0.5 ? "已触地" : "未触地"));
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type DogGazeboHW::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        const rclcpp::Duration delay_duration = rclcpp::Duration::from_seconds(delay_);

        for (auto &j : joints_)
        {
            // 存入当前命令
            j.cmd_buffer.push_back({j.cmd.pos_des, j.cmd.vel_des, j.cmd.kp, j.cmd.kd, j.cmd.ff, time});

            // 查找最新的符合延迟条件的指令
            bool has_valid_cmd = false;
            HybridCommand effective_cmd;

            while (!j.cmd_buffer.empty() && (j.cmd_buffer.front().stamp + delay_duration) < time)
            {
                effective_cmd = j.cmd_buffer.front();
                j.cmd_buffer.pop_front();
                has_valid_cmd = true;
            }

            if (has_valid_cmd)
            {
                // 控制律：$\tau = k_p(q_{des} - q) + k_d(\dot{q}_{des} - \dot{q}) + \tau_{ff}$
                double torque = effective_cmd.kp * (effective_cmd.pos_des - j.pos) +
                                effective_cmd.kd * (effective_cmd.vel_des - j.vel) +
                                effective_cmd.ff;

                j.gz_joint->SetForce(0u, torque);
            }
        }
        return hardware_interface::return_type::OK;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_hardware::DogGazeboHW, gazebo_ros2_control::GazeboSystemInterface)