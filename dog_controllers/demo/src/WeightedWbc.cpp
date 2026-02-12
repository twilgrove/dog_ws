//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace legged {

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  // 调用基类更新函数，初始化WBC所需数据
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // 构建约束条件
  // 约束任务包括：浮基动力学方程、扭矩限制、摩擦锥约束、无接触运动约束
  Task constraints = formulateConstraints();
  // 计算总约束数量：等式约束 + 不等式约束
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  // 构建QP问题的约束矩阵和边界
  // A矩阵：包含等式约束和不等式约束的系数矩阵
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  // 组合等式约束和不等式约束的系数矩阵
  A << constraints.a_,
       constraints.d_;

  // 设置约束下界：等式约束b_必须严格满足，不等式约束下界为负无穷
  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  // 设置约束上界：等式约束上界等于b_，不等式约束上界为f_
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // 构建加权任务作为优化目标
  // 加权任务包括：摆动腿任务、基座加速度任务、接触力任务
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  // 构建QP问题的Hessian矩阵：A^T * A
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  // 构建QP问题的梯度向量：-A^T * b
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // 使用qpOASES求解QP问题
  // 创建QP问题实例，变量数为决策变量数，约束数为总约束数
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  // 设置求解器选项
  qpOASES::Options options;
  options.setToMPC();  // 设置为MPC模式
  options.printLevel = qpOASES::PL_LOW;  // 低输出级别
  options.enableEqualities = qpOASES::BT_TRUE;  // 启用等式约束
  qpProblem.setOptions(options);
  int nWsr = 20;  // 最大工作集更新次数

  // 初始化并求解QP问题
  // 参数：Hessian矩阵、梯度向量、约束矩阵、变量边界、约束边界
  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());

  // 获取QP问题的最优解
  qpProblem.getPrimalSolution(qpSol.data());
  return qpSol;
}

// 构建约束任务：组合所有约束条件
Task WeightedWbc::formulateConstraints() {
  // 返回：浮基动力学方程 + 扭矩限制 + 摩擦锥约束 + 无接触运动约束
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}

// 构建加权任务：组合所有加权优化目标
Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  // 返回：摆动腿任务×权重 + 基座加速度任务×权重 + 接触力任务×权重
  return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}

// 从配置文件加载任务权重参数
void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // 首先加载基类的任务设置
  WbcBase::loadTasksSetting(taskFile, verbose);

  // 使用boost属性树读取配置文件
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";  // 权重参数前缀
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  // 加载各任务的权重参数
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged