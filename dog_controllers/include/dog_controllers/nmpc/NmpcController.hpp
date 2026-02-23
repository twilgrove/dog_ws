#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <atomic>
#include <mutex>
#include <thread>

// OCS2 核心依赖
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_legged_robot/gait/GaitReceiver.h"
#include "ocs2_legged_robot/LeggedRobotInterface.h"

namespace dog_controllers
{

    using namespace ocs2;
    using namespace ocs2::legged_robot;

    class NmpcController final
    {
    public:
        NmpcController(rclcpp::Node::SharedPtr &node,
                       std::shared_ptr<LeggedRobotInterface> leggedInterface);

        ~NmpcController();

        void start(const SystemObservation &initObservation);

        void update(SystemObservation &observation,
                    vector_t &optimizedState,
                    vector_t &optimizedInput,
                    size_t &plannedMode);
        std::unique_ptr<MPC_MRT_Interface> mpcMrtInterface_;

    private:
        void mpcThreadTask();

        std::shared_ptr<LeggedRobotInterface> leggedInterface_;

        // OCS2 求解组件
        std::unique_ptr<MPC_BASE> mpcPtr_;

        // 线程与生命周期控制
        std::shared_ptr<rclcpp::Node> node_;
        std::atomic_bool controllerRunning_{false};
        std::atomic_bool mpcRunning_{false};
        std::thread mpcThread_;

        // 性能统计
        benchmark::RepeatedTimer mpcTimer_;
    };

}