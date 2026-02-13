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
        std::string prefix = "weight.";
        loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", false);
        loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", false);
        loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", false);

        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m📊 [PARAM] 已加载 WBC 权重配置清单:\033[0m");
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 摆动腿跟踪权重  : \033[0m%.3f", weightSwingLeg_);
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  ├─ 机身加速度权重  : \033[0m%.3f", weightBaseAccel_);
        RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;33m  └─ 接触力正则化    : \033[0m%.3f", weightContactForce_);

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
        // 1. 首先调用基类更新基础动力学数据（J, M, nle 等）
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        // 2. 构造硬约束 (Constraints)
        // 包含：浮基座动力学方程、力矩限制、摩擦锥、支撑腿不动
        Task constraints = formulateConstraints();
        size_t numConstraints = constraints.b_.size() + constraints.f_.size();

        // 准备 qpOASES 所需的矩阵格式 (RowMajor 行优先)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, numDecisionVars_);
        vector_t lbA(numConstraints), ubA(numConstraints);

        // 将等式约束 (Ax = b) 和不等式约束 (Dx <= f) 合并到 A 矩阵中
        A << constraints.a_,
            constraints.d_;

        // 设置约束的上下限
        // 对于等式约束 b，上下限相等 [b, b]；对于不等式约束 f，下限为负无穷 [-inf, f]
        lbA << constraints.b_,
            -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
        ubA << constraints.b_,
            constraints.f_;

        // 3. 构造加权目标函数 (Cost)
        // 包含：摆动腿跟踪、机身加速度、期望接触力跟踪，分别乘以对应的权重参数
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);

        // 将最小二乘形式 ||Ax - b||^2 展开为标准 QP 形式
        // H = A^T * A,  g = -A^T * b
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
        vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

        // 4. 调用求解器求解 (Solve)
        auto qpProblem = qpOASES::QProblem(numDecisionVars_, numConstraints);
        qpOASES::Options options;
        options.setToMPC(); // 针对模型预测控制优化设置
        options.printLevel = qpOASES::PL_LOW;
        options.enableEqualities = qpOASES::BT_TRUE; // 开启等式约束处理
        qpProblem.setOptions(options);

        int nWsr = 20; // 最大迭代次数 (Working Set Recalculations)

        // 执行 QP 初始化和求解
        qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);

        // 提取原始解（包含 q_acc, f, tau）
        vector_t qpSol(numDecisionVars_);
        qpProblem.getPrimalSolution(qpSol.data());

        return qpSol;
    }

    /**
     * 汇总所有硬约束任务
     * 这些约束是“必须满足”的，否则 QP 无解
     */
    Task WeightedWbc::formulateConstraints()
    {
        return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
    }

    /**
     * 汇总并加权所有目标任务
     * 权重（weightXXX_）决定了当多个任务冲突时，优先牺牲哪一个
     */
    Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
        return formulateSwingLegTask() * weightSwingLeg_ +
               formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
               formulateContactForceTask(inputDesired) * weightContactForce_;
    }

}