#pragma once
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nmpc_ocs2_legged_robot/LeggedRobotInterface.h>

#include "common/dog_data_bridge.hpp"
#include "common/dog_interface.hpp"
#include "common/debug_manager.hpp"
#include "state_estimation/KalmanFilterEstimator.hpp"
#include "state_estimation/StateEstimatorBase.hpp"
#include "state_estimation/TopicStateEstimator.hpp"
#include "wbc/WbcBase.hpp"
#include "wbc/WeightedWbc.hpp"

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

        virtual void init_real_or_god();

    protected:
        std::unique_ptr<DogDataBridge> bridge_;
        std::unique_ptr<DebugManager> debug_manager_;
        std::unique_ptr<DogInterface> dog_interface_;
        std::unique_ptr<StateEstimatorBase> state_estimator_;
        std::unique_ptr<WbcBase> wbc_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::string urdfFile;
        std::string taskFile;
        std::string referenceFile;
    };
    class DogNmpcWbcController_God : public DogNmpcWbcController
    {
        void init_real_or_god() override;
    };
}
