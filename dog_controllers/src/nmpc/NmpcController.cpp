#include "nmpc/NmpcController.hpp"

namespace dog_controllers
{

    NmpcController::NmpcController(rclcpp::Node::SharedPtr &node,
                                   std::shared_ptr<LeggedRobotInterface> leggedInterface)
        : leggedInterface_(leggedInterface),
          node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 NmpcController\033[0m");

        mpcPtr_ = std::make_unique<SqpMpc>(
            leggedInterface_->mpcSettings(),
            leggedInterface_->sqpSettings(),
            leggedInterface_->getOptimalControlProblem(),
            leggedInterface_->getInitializer());

        const std::string robotName = "dog_robot";

        auto gaitReceiverPtr = std::make_shared<GaitReceiver>(
            node_,
            leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
            robotName);

        auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(
            robotName,
            leggedInterface_->getReferenceManagerPtr());

        // rosReferenceManagerPtr->subscribe(node_);

        mpcPtr_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
        mpcPtr_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

        mpcMrtInterface_ = std::make_unique<MPC_MRT_Interface>(*mpcPtr_);
        mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ NmpcController\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m====================================================\033[0m");
    }

    void NmpcController::start(const SystemObservation &initObservation)
    {
        TargetTrajectories target;
        scalar_t t = initObservation.time;

        target.timeTrajectory = {t, t + 1.0, t + 10.0, t + 50.0, t + 100.0};

        vector_t goalState = vector_t::Zero(24);
        goalState(8) = 0.306; // 目标高度
        for (int k = 0; k < 4; k++)
        {
            goalState(12 + k * 3 + 0) = 0.0;  // HAA
            goalState(12 + k * 3 + 1) = -0.8; // HFE
            goalState(12 + k * 3 + 2) = 1.5;  // KFE
        }
        target.stateTrajectory = {goalState, goalState, goalState, goalState, goalState};
        target.inputTrajectory = {vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24), vector_t::Zero(24)};

        // // 设置初始目标轨迹（原地静止）
        // TargetTrajectories targetTrajectories({initObservation.time},
        //                                       {initObservation.state},
        //                                       {initObservation.input});

        mpcMrtInterface_->setCurrentObservation(initObservation);
        mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target);
        while (rclcpp::ok() && !mpcMrtInterface_->initialPolicyReceived())
        {
            mpcMrtInterface_->advanceMpc();
            rclcpp::Rate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m已收到初始策略，NMPC控制器启动。\033[0m");

        controllerRunning_ = true;
        mpcThread_ = std::thread(&NmpcController::mpcThreadTask, this);
        setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);

        mpcRunning_ = true;
    }

    void NmpcController::mpcThreadTask()
    {
        while (controllerRunning_)
        {
            executeAndSleep(
                [&]()
                {
                    if (mpcRunning_)
                    {
                        mpcTimer_.startTimer();
                        mpcMrtInterface_->advanceMpc();
                        mpcTimer_.endTimer();
                    }
                },
                leggedInterface_->mpcSettings().mpcDesiredFrequency_);
        }
    }

    void NmpcController::update(SystemObservation &observation,
                                vector_t &optimizedState,
                                vector_t &optimizedInput,
                                size_t &plannedMode)
    {
        mpcMrtInterface_->setCurrentObservation(observation);
        mpcMrtInterface_->updatePolicy();

        mpcMrtInterface_->evaluatePolicy(observation.time,
                                         observation.state,
                                         optimizedState,
                                         optimizedInput,
                                         plannedMode);
        observation.input = optimizedInput;

        // RCLCPP_INFO_THROTTLE(
        //     node_->get_logger(),
        //     *node_->get_clock(),
        //     1000,
        //     "\n\033[1;36m====================================================\033[0m"
        //     "\n\033[1;36m[ NMPC 实时性能报告 ]\033[0m 🚀"
        //     "\n\033[1;36m----------------------------------------------------\033[0m"
        //     "\n  求解总数   : %d 次"
        //     "\n  平均耗时   : \033[1;32m%.3f\033[0m ms"
        //     "\n  最大耗时   : \033[1;31m%.3f\033[0m ms"
        //     "\n  实时要求   : < %.3f ms"
        //     "\n\033[1;36m====================================================\033[0m",
        //     mpcTimer_.getNumTimedIntervals(),
        //     mpcTimer_.getAverageInMilliseconds(),
        //     mpcTimer_.getMaxIntervalInMilliseconds(),
        //     1000.0 / leggedInterface_->mpcSettings().mpcDesiredFrequency_);
    }

    NmpcController::~NmpcController()
    {
        controllerRunning_ = false;
        if (mpcThread_.joinable())
        {
            mpcThread_.join();
        }
    }

}