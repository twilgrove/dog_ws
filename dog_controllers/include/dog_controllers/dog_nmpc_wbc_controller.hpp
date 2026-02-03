#pragma once
#include "controller_interface/controller_interface.hpp"
#include "dog_data_bridge.hpp"
#include "state_estimation/StateEstimatorBase.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dog_controllers
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class DogNmpcWbcController : public controller_interface::ControllerInterface
    {
    public:
        DogNmpcWbcController() = default;
        ~DogNmpcWbcController() = default;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        DogDataBridge bridge_;
        std::shared_ptr<StateEstimatorBase> state_estimator_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> contact_names_;
        std::string imu_name_;
    };
}
