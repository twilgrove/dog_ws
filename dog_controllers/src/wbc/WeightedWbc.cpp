#include "wbc/WeightedWbc.hpp"
#include <ocs2_core/misc/LoadData.h>
#include <qpOASES.hpp>
namespace dog_controllers
{
    WeightedWbc::WeightedWbc(
        const std::string &taskFile,
        const PinocchioInterface &pinocchioInterface,
        const CentroidalModelInfo &info,
        const PinocchioEndEffectorKinematics &eeKinematics,
        rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : WbcBase(pinocchioInterface, info, eeKinematics, node)
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 WeightedWbc\033[0m");
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        loadData::loadPtreeValue(pt, wbcnWSR_, "weight.nWSR", false);
        loadData::loadPtreeValue(pt, weightSwingLeg_, "weight.swingLeg", false);
        loadData::loadPtreeValue(pt, weightBaseAccel_, "weight.baseAccel", false);
        loadData::loadPtreeValue(pt, weightContactForce_, "weight.contactForce", false);
        torqueLimits_.resize(3);
        loadData::loadPtreeValue(pt, torqueLimits_(0), "torqueLimitsTask.HAA", false);
        loadData::loadPtreeValue(pt, torqueLimits_(1), "torqueLimitsTask.HFE", false);
        loadData::loadPtreeValue(pt, torqueLimits_(2), "torqueLimitsTask.KFE", false);
        loadData::loadPtreeValue(pt, frictionCoeff_, "frictionConeTask.frictionCoefficient", false);
        loadData::loadPtreeValue(pt, swingKp_, "swingLegTask.kp", false);
        loadData::loadPtreeValue(pt, swingKd_, "swingLegTask.kd", false);

        RCLCPP_INFO(node_->get_logger(), "\033[1;33m📊 [PARAM] 已加载 WBC 权重配置清单:\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ QP 最大迭代次数       : \033[0m%d", wbcnWSR_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 关节力矩限制 (H,H,K): \033[0m[%.1f, %.1f, %.1f] N.m",
                    torqueLimits_(0), torqueLimits_(1), torqueLimits_(2));
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 地面摩擦系数        : \033[0m%.2f", frictionCoeff_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 摆动腿 PD 增益      : \033[0mKp=%.1f, Kd=%.1f", swingKp_, swingKd_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 摆动腿跟踪权重       : \033[0m%.3f", weightSwingLeg_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 机身加速度权重       : \033[0m%.3f", weightBaseAccel_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  └─ 接触力正则化         : \033[0m%.3f", weightContactForce_);

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ WeightedWbc\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m====================================================\033[0m");
    }

    /**
     * WeightedWbc 求解主循环
     * 这里的逻辑是将所有 Task 转化为标准 QP 形式：min 1/2*x^T*H*x + g^T*x
     */
    vector_t WeightedWbc::update(const vector_t &stateDesired,
                                 const vector_t &inputDesired,
                                 const vector_t &rbdStateMeasured,
                                 size_t mode,
                                 scalar_t period)
    {
        wbcTimer_.startTimer();
        // 1. 基类更新基础动力学数据（J, M, nle 等）
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        // 2. 构造硬约束
        Task constraints = formulateConstraints();
        size_t numConstraints = constraints.b_.size() + constraints.f_.size();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, numDecisionVars_);
        vector_t lbA(numConstraints), ubA(numConstraints);

        A << constraints.a_, constraints.d_;

        lbA << constraints.b_, -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
        ubA << constraints.b_, constraints.f_;

        // 3. 构造加权目标函数
        Task weighedTask = formulateWeightedTasks(rbdStateMeasured, stateDesired, inputDesired, period);

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
        vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

        // 4. 调用qpOASES求解器求解
        auto qpProblem = qpOASES::QProblem(numDecisionVars_, numConstraints);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_LOW;
        options.enableEqualities = qpOASES::BT_TRUE;
        qpProblem.setOptions(options);

        // 执行 QP 初始化和求解
        int nWSR = wbcnWSR_;
        qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR);

        // 提取原始解
        vector_t qpSol(numDecisionVars_);
        qpProblem.getPrimalSolution(qpSol.data());

        float total_fz = qpSol(18 + 2) + qpSol(18 + 5) + qpSol(18 + 8) + qpSol(18 + 11);
        wbcTimer_.endTimer();
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "\n\033[1;35m====================================================\033[0m"
            "\n\033[1;35m[ WBC 实时性能报告 ]\033[0m 🛡️"
            "\n\033[1;35m----------------------------------------------------\033[0m"
            "\n  求解总数   : %d 次"
            "\n  平均耗时   : \033[1;32m%.3f\033[0m ms"
            "\n  最大耗时   : \033[1;31m%.3f\033[0m ms"
            "\n\033[1;35m====================================================\033[0m",
            wbcTimer_.getNumTimedIntervals(),
            wbcTimer_.getAverageInMilliseconds(),
            wbcTimer_.getMaxIntervalInMilliseconds());

        return qpSol;
    }

    /**
     * 汇总所有硬约束任务
     */
    Task WeightedWbc::formulateConstraints()
    {
        return formulateFloatingBaseEomTask() +
               formulateTorqueLimitsTask() +
               formulateFrictionConeTask() +
               formulateNoContactMotionTask();
    }

    /**
     * 汇总并加权所有目标任务
     */
    Task WeightedWbc::formulateWeightedTasks(const vector_t &rbdStateMeasured, const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
        return formulateSwingLegTask() * weightSwingLeg_ +
               formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
               formulateContactForceTask(inputDesired) * weightContactForce_;
    }

}