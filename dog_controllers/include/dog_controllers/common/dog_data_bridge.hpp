#pragma once
#include <vector>
#include <string>
#include <array>
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dog_controllers
{
    struct JointData
    {
        std::string name;
        double pos = 0.0, vel = 0.0, eff = 0.0;
        double cmd_pos = 0.0, cmd_vel = 0.0, cmd_kp = 0.0, cmd_kd = 0.0, cmd_ff = 0.0;
    };

    struct LegData
    {
        std::string name;
        JointData hip, thigh, calf;
        double contact = 0.0;
        JointData *joints[3] = {&hip, &thigh, &calf};
    };

    struct ImuData
    {
        std::string name;
        double ori[4] = {0, 0, 0, 1};
        double ang_vel[3] = {0, 0, 0};
        double lin_acc[3] = {0, 0, 0};
    };

    class DogDataBridge
    {
    public:
        DogDataBridge(
            std::vector<hardware_interface::LoanedStateInterface> &state_interfaces,
            std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
            rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        ~DogDataBridge() = default;

        void read_from_hw();
        void write_to_hw();

        std::array<LegData, 4> legs;
        ImuData imu;

    private:
        struct ReadTask
        {
            hardware_interface::LoanedStateInterface *hw_handle;
            double *local_var;
        };

        struct WriteTask
        {
            hardware_interface::LoanedCommandInterface *hw_handle;
            double *local_var;
        };

        std::vector<ReadTask> read_tasks_;
        std::vector<WriteTask> write_tasks_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> contact_names_;
        std::string imu_name_;
    };
}
