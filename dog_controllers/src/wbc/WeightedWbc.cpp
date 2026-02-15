#include "wbc/WeightedWbc.hpp"
#include <ocs2_core/misc/LoadData.h>
#include <qpOASES.hpp>
namespace dog_controllers
{
    WeightedWbc::WeightedWbc(
        const std::string &taskFile,
        const PinocchioInterface &pinocchioInterface,
        const CentroidalModelInfo &info,
        const PinocchioEndEffectorKinematics &eeKinematics)
        : WbcBase(pinocchioInterface, info, eeKinematics)
    {
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;36m[ 初始化开始 ] 🚀 WeightedWbc\033[0m");
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        loadData::loadPtreeValue(pt, weightSwingLeg_, "weight.swingLeg", false);
        loadData::loadPtreeValue(pt, weightBaseAccel_, "weight.baseAccel", false);
        loadData::loadPtreeValue(pt, weightContactForce_, "weight.contactForce", false);
        torqueLimits_.resize(3);
        loadData::loadPtreeValue(pt, torqueLimits_(0), "torqueLimitsTask.(0,0)", false);
        loadData::loadPtreeValue(pt, torqueLimits_(1), "torqueLimitsTask.(1,0)", false);
        loadData::loadPtreeValue(pt, torqueLimits_(2), "torqueLimitsTask.(2,0)", false);
        loadData::loadPtreeValue(pt, frictionCoeff_, "frictionConeTask.frictionCoefficient", true);
        loadData::loadPtreeValue(pt, swingKp_, "swingLegTask.kp", true);
        loadData::loadPtreeValue(pt, swingKd_, "swingLegTask.kd", true);

        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m📊 [PARAM] 已加载 WBC 权重配置清单:\033[0m");
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 关节力矩限制 (H,H,K): \033[0m[%.1f, %.1f, %.1f] N.m",
                    torqueLimits_(0), torqueLimits_(1), torqueLimits_(2));
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 地面摩擦系数        : \033[0m%.2f", frictionCoeff_);
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 摆动腿 PD 增益      : \033[0mKp=%.1f, Kd=%.1f", swingKp_, swingKd_);
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 摆动腿跟踪权重       : \033[0m%.3f", weightSwingLeg_);
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 机身加速度权重       : \033[0m%.3f", weightBaseAccel_);
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  └─ 接触力正则化         : \033[0m%.3f", weightContactForce_);

        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;32m[ 初始化完成 ] ✅ WeightedWbc\033[0m");
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;32m====================================================\033[0m");
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
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);

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
        int nWSR = 20;
        qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR);

        // 提取原始解
        vector_t qpSol(numDecisionVars_);
        qpProblem.getPrimalSolution(qpSol.data());

        return qpSol;
    }

    /**
     * 汇总所有硬约束任务
     */
    Task WeightedWbc::formulateConstraints()
    {
        return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
    }

    /**
     * 汇总并加权所有目标任务
     */
    Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
        //  formulateSwingLegTask() * weightSwingLeg_ +
        //  formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
        return formulateContactForceTask(inputDesired) * weightContactForce_;
    }

}